"""
Reliable CAN Bus communication library for Raspberry Pi Zero 2W
Prevents hardware buffer overflow by managing messages in application layer

KISS Design:
- Detect link failures immediately
- Queue messages in RAM (not hardware buffer)
- Flush hardware buffers when link fails
- Simple retry logic
- Background thread for continuous ACK reception
"""

import can
import time
import threading
from typing import Optional, List, Tuple
from collections import deque


class CANBus:
    """
    Reliable CAN Bus interface - prevents hardware buffer overflow
    
    Args:
        interface: 'can0' for Pi or 'COM6' for PC USB adapter
        bitrate: CAN bitrate in bps (default: 250000)
        auto_setup: Auto setup CAN interface on Pi (default: True)
        queue_size: Max messages in RAM queue (default: 1000, 0=unlimited)
    """
    
    def __init__(self, interface: str = 'can0', bitrate: int = 250000, 
                 auto_setup: bool = True, queue_size: int = 1000):
        self.interface = interface
        self.bitrate = bitrate
        self.bus = None
        self._is_pi = not interface.startswith('COM')
        
        # Link state tracking (simple)
        self._link_ok = True
        self._fail_count = 0
        
        # Application-level queue (RAM, not hardware buffer)
        self._queue_size = queue_size
        self._msg_queue = deque(maxlen=queue_size if queue_size > 0 else None)
        
        # RX buffer for ACKs (accessed by background thread)
        self._rx_buffer = deque(maxlen=2000)  # Large buffer for Windows/SLCAN bursts
        self._rx_lock = threading.Lock()
        self._rx_thread = None
        self._rx_running = False
        
        # Statistics
        self._queued_total = 0
        self._dropped_total = 0
        self._sent_total = 0
        
        if self._is_pi and auto_setup:
            self._setup_socketcan()
        
        self._connect()
        self._start_rx_thread()
        self._start_maintenance_thread()
    
    def _setup_socketcan(self):
        """Setup SocketCAN with minimal TX queue to prevent buffer buildup"""
        try:
            import os
            # Bring down interface
            os.system(f'sudo ip link set {self.interface} down 2>/dev/null')
            
            # Configure with small TX queue (10 messages max)
            # This prevents hardware buffer overflow
            os.system(f'sudo ip link set {self.interface} type can bitrate {self.bitrate} restart-ms 100')
            os.system(f'sudo ip link set {self.interface} txqueuelen 10')
            
            # Bring up interface
            os.system(f'sudo ip link set {self.interface} up')
            time.sleep(0.1)
        except Exception as e:
            print(f"[CAN] Warning: Setup failed ({e}) - attempting to continue...")
    
    def _connect(self):
        """Connect to CAN bus with minimal buffering"""
        try:
            if self._is_pi:
                self.bus = can.interface.Bus(
                    channel=self.interface,
                    bustype='socketcan'
                )
            else:
                self.bus = can.interface.Bus(
                    channel=self.interface,
                    bustype='slcan',
                    bitrate=self.bitrate,
                    rtscts=False
                )
            print(f"[CAN] Connected to {self.interface} @ {self.bitrate} bps")
        except Exception as e:
            if "serial" in str(e).lower():
                raise ConnectionError(f"Missing pyserial: pip install pyserial")
            raise ConnectionError(f"Failed to connect: {e}")
    
    def _start_rx_thread(self):
        """Start background thread to continuously receive messages"""
        self._rx_running = True
        self._rx_thread = threading.Thread(target=self._rx_worker, daemon=True)
        self._rx_thread.start()
    
    def _rx_worker(self):
        """Background thread: continuously receive and buffer messages"""
        while self._rx_running:
            try:
                msg = self.bus.recv(timeout=0.1)
                if msg:
                    with self._rx_lock:
                        self._rx_buffer.append((msg.arbitration_id, list(msg.data)))
            except Exception:
                pass  # Ignore errors, keep receiving
    
    def _start_maintenance_thread(self):
        """Start background thread for automatic queue draining and link recovery"""
        maintenance_thread = threading.Thread(target=self._maintenance_worker, daemon=True)
        maintenance_thread.start()
    
    def _maintenance_worker(self):
        """Background thread: automatically drain queue when link recovers"""
        while self._rx_running:
            try:
                # Every 0.5 seconds, try to drain queue if link is down
                if not self._link_ok and len(self._msg_queue) > 0:
                    self._drain_queue()
                
                time.sleep(0.5)
            except Exception:
                pass  # Keep running even if error occurs
    
    def send(self, can_id: int, data: List[int], extended: bool = False) -> bool:
        """
        Send CAN message - just call this and forget about buffering!
        
        Library automatically:
        - Sends immediately if link is up
        - Queues in RAM if link is down
        - Drains queue when link recovers (background thread)
        - Detects link failures automatically
        
        User doesn't need to handle exceptions or manage buffers - all errors caught internally
        
        Args:
            can_id: CAN arbitration ID (0x000-0x7FF standard, 0x1FFFFFFF extended)
            data: List of bytes (0-8 bytes)
            extended: Use extended ID format (default: False)
            
        Returns:
            True if sent or queued successfully, False only if queue full
        """
        try:
            # Validate input
            if not 0 <= len(data) <= 8:
                print(f"[CAN] Warning: Data must be 0-8 bytes (got {len(data)}), truncating...")
                data = data[:8] if len(data) > 8 else data
            
            # Step 1: Try to drain queue if we have pending messages and link is OK
            if self._link_ok and len(self._msg_queue) > 0:
                self._drain_queue()
            
            # Step 2: If link is down, queue immediately (don't touch hardware buffer)
            if not self._link_ok:
                return self._queue_message(can_id, data, extended)
            
            # Step 3: Try to send directly (ONE attempt only - KISS)
            msg = can.Message(arbitration_id=can_id, data=data, is_extended_id=extended)
            
            try:
                # Ultra-short timeout to avoid blocking
                self.bus.send(msg, timeout=0.2)
                self._fail_count = 0
                self._sent_total += 1
                return True
                
            except Exception as e:
                # First failure - mark link down and flush hardware buffer
                self._fail_count += 1
                
                if self._fail_count >= 3:
                    if self._link_ok:
                        print(f"[CAN] Link DOWN - switching to queue mode")
                        self._link_ok = False
                        self._flush_hardware_buffer()
                
                # Queue the failed message
                return self._queue_message(can_id, data, extended)
        
        except Exception as e:
            # Catch ALL exceptions - library should never crash user's application
            print(f"[CAN] Unexpected error in send(): {e} - queueing message")
            return self._queue_message(can_id, data, extended)
    
    def _queue_message(self, can_id: int, data: List[int], extended: bool) -> bool:
        """Queue message in RAM (not hardware buffer) - never throws exceptions"""
        try:
            old_len = len(self._msg_queue)
            self._msg_queue.append((can_id, data.copy(), extended))
            
            # Check if message was dropped due to queue limit
            if self._queue_size > 0 and len(self._msg_queue) <= old_len:
                self._dropped_total += 1
                print(f"[CAN] DROPPED (queue full: {self._queue_size}) - total: {self._dropped_total}")
                return False
            
            self._queued_total += 1
            if len(self._msg_queue) % 50 == 1:  # Print every 50 messages
                print(f"[CAN] Queued: {len(self._msg_queue)} msgs")
            
            return True
        except Exception as e:
            print(f"[CAN] Queue error: {e}")
            return False
    
    def _drain_queue(self):
        """Send queued messages when link recovers (fast drain with small timeout) - never throws"""
        try:
            if len(self._msg_queue) == 0:
                return
            
            # Drain ALL queued messages as fast as possible
            # Use small timeout to avoid blocking but still allow hardware to accept messages
            initial_count = len(self._msg_queue)
            
            if initial_count > 10:
                print(f"[CAN] Draining {initial_count} queued messages...")
            
            sent = 0
            failed = False
            
            # Drain EVERYTHING in the queue at maximum speed
            while self._msg_queue:
                can_id, data, extended = self._msg_queue[0]
                msg = can.Message(arbitration_id=can_id, data=data, is_extended_id=extended)
                
                try:
                    # Small timeout = fast but won't fail if buffer temporarily full
                    self.bus.send(msg, timeout=0.01)  # 10ms timeout
                    self._msg_queue.popleft()
                    sent += 1
                    self._sent_total += 1
                        
                except Exception:
                    # Failed during drain - stop immediately
                    failed = True
                    if len(self._msg_queue) > 10:
                        print(f"[CAN] Drain failed - {len(self._msg_queue)} msgs remain in queue")
                    self._link_ok = False  # Keep link marked as DOWN
                    break
            
            if not failed and len(self._msg_queue) == 0:
                print(f"[CAN] Queue drained successfully ({sent} msgs) - link UP")
                self._link_ok = True
                self._fail_count = 0
            elif not failed and sent > 0:
                print(f"[CAN] Drained {sent} msgs - {len(self._msg_queue)} remain")
        
        except Exception as e:
            print(f"[CAN] Drain error: {e}")
            self._link_ok = False

    
    def _flush_hardware_buffer(self):
        """Flush hardware TX/RX buffers to prevent overflow"""
        try:
            # Drain RX buffer
            while self.bus.recv(timeout=0):
                pass
            
            # Reset interface on Pi to clear TX buffer
            if self._is_pi:
                import os
                print(f"[CAN] Flushing hardware buffers...")
                os.system(f'sudo ip link set {self.interface} down 2>/dev/null')
                time.sleep(0.05)
                os.system(f'sudo ip link set {self.interface} up')
                time.sleep(0.1)
        except Exception as e:
            print(f"[CAN] Flush error: {e}")
    
    def receive(self, timeout: float = 1.0) -> Optional[Tuple[int, List[int]]]:
        """
        Receive one CAN message - just call this and forget about threading!
        
        Background thread continuously receives and buffers all messages.
        You just pull from the buffer whenever you want.
        
        Never throws exceptions - returns None on timeout
        
        Args:
            timeout: Timeout in seconds (waits for message in buffer)
            
        Returns:
            (can_id, data) or None if timeout
        """
        try:
            start_time = time.time()
            while time.time() - start_time < timeout:
                with self._rx_lock:
                    if self._rx_buffer:
                        return self._rx_buffer.popleft()
                time.sleep(0.01)  # Small sleep to avoid busy-waiting
            return None
        except Exception as e:
            print(f"[CAN] Receive error: {e}")
            return None
    
    def receive_all(self, timeout: float = 0.01, max_msgs: int = 100) -> List[Tuple[int, List[int]]]:
        """
        Receive all buffered messages at once - efficient bulk receive!
        
        Background thread continuously receives and buffers all messages.
        You just pull all of them whenever you want.
        
        Never throws exceptions - returns empty list if none available
        
        Args:
            timeout: Not used (returns immediately with buffered messages)
            max_msgs: Max messages to return per call
            
        Returns:
            List of (can_id, data) tuples (empty list if none available)
        """
        try:
            messages = []
            
            # Read from RX buffer (populated by background thread)
            with self._rx_lock:
                # Get up to max_msgs from buffer
                count = min(len(self._rx_buffer), max_msgs)
                for _ in range(count):
                    if self._rx_buffer:
                        messages.append(self._rx_buffer.popleft())
            
            return messages
        except Exception as e:
            print(f"[CAN] Receive all error: {e}")
            return []
    
    def send_with_ack(self, can_id: int, data: List[int], ack_id: Optional[int] = None, 
                      timeout: float = 0.5, max_retries: int = 1, extended: bool = False) -> bool:
        """
        Send message and wait for ACK (guarantees delivery to destination) - never throws exceptions
        Uses background RX thread for non-blocking ACK reception
        
        Args:
            can_id: CAN ID for data message
            data: Data bytes (0-8 bytes)
            ack_id: Expected ACK CAN ID (default: can_id + 1)
            timeout: Timeout waiting for ACK in seconds (default: 0.5)
            max_retries: Number of retries if no ACK (default: 1)
            extended: Use extended ID format
            
        Returns:
            True if ACK received, False if failed after retries or error
        """
        try:
            if ack_id is None:
                ack_id = can_id + 1
            
            for attempt in range(max_retries):
                # IMPORTANT: Clear any stale ACKs with matching ID from buffer
                # This prevents false positives when cable is disconnected
                with self._rx_lock:
                    self._rx_buffer = deque(
                        [(rx_id, rx_data) for rx_id, rx_data in self._rx_buffer 
                         if not (rx_id == ack_id and len(rx_data) > 0 and rx_data[0] == 0xFF)],
                        maxlen=self._rx_buffer.maxlen
                    )
                
                # Send the message
                if not self.send(can_id, data, extended):
                    # Send failed (queue full)
                    return False
                
                # Wait for ACK - check RX buffer instead of blocking on recv()
                start_time = time.time()
                while time.time() - start_time < timeout:
                    # Check RX buffer for ACK
                    with self._rx_lock:
                        for i, (rx_id, rx_data) in enumerate(self._rx_buffer):
                            # Check if this is our ACK (ID matches and first byte is 0xFF)
                            if rx_id == ack_id and len(rx_data) > 0 and rx_data[0] == 0xFF:
                                # Remove ACK from buffer and return success
                                del self._rx_buffer[i]
                                return True
                    
                    # Small sleep to avoid busy-waiting
                    time.sleep(0.01)
                
                # No ACK received
                if attempt < max_retries - 1:
                    time.sleep(0.05)  # Brief pause before retry
            
            return False
        except Exception as e:
            print(f"[CAN] Send with ACK error: {e}")
            return False
    
    def send_ack(self, original_id: int, original_data: List[int], extended: bool = False) -> bool:
        """
        Send ACK for received message (non-blocking, fire-and-forget) - never throws exceptions
        
        Args:
            original_id: CAN ID of message being acknowledged
            original_data: Data of original message
            extended: Use extended ID format
            
        Returns:
            True if ACK sent successfully, False on error (safe to ignore)
        """
        try:
            ack_id = original_id + 1
            ack_data = [0xFF, original_data[0] if original_data else 0x00]
            
            # Use direct bus.send() with zero timeout for non-blocking ACK
            msg = can.Message(arbitration_id=ack_id, data=ack_data, is_extended_id=extended)
            self.bus.send(msg, timeout=0)  # Non-blocking
            return True
        except:
            return False  # ACK failed, but don't block or crash

    
    def check_link(self) -> bool:
        """
        Manually check link health (attempts to send/drain) - never throws exceptions
        Call this periodically from your main loop
        
        Returns:
            True if link is up, False if down or error
        """
        try:
            if not self._link_ok:
                # Try to drain queue (this will mark link up if successful)
                self._drain_queue()
            
            return self._link_ok
        except Exception as e:
            print(f"[CAN] Check link error: {e}")
            return False
    
    def get_stats(self) -> dict:
        """
        Get statistics - never throws exceptions
        
        Returns:
            Dictionary with queue and transmission stats (safe defaults on error)
        """
        try:
            return {
                'link_up': self._link_ok,
                'queue_length': len(self._msg_queue),
                'queue_max': self._queue_size,
                'total_sent': self._sent_total,
                'total_queued': self._queued_total,
                'total_dropped': self._dropped_total,
            }
        except Exception as e:
            print(f"[CAN] Get stats error: {e}")
            return {
                'link_up': False,
                'queue_length': 0,
                'queue_max': 0,
                'total_sent': 0,
                'total_queued': 0,
                'total_dropped': 0,
            }
    
    def clear_queue(self):
        """Clear all queued messages (use with caution - data loss!) - never throws exceptions"""
        try:
            cleared = len(self._msg_queue)
            self._msg_queue.clear()
            if cleared > 0:
                print(f"[CAN] Cleared {cleared} queued messages")
            return cleared
        except Exception as e:
            print(f"[CAN] Clear queue error: {e}")
            return 0
    
    def close(self):
        """Close CAN bus and report final statistics - never throws exceptions"""
        try:
            # Stop RX thread
            self._rx_running = False
            if self._rx_thread:
                self._rx_thread.join(timeout=1.0)
            
            stats = self.get_stats()
            print(f"[CAN] Closing - Stats: {stats}")
            
            if self.bus:
                self.bus.shutdown()
            
            if self._is_pi:
                import os
                os.system(f'sudo ip link set {self.interface} down 2>/dev/null')
        except Exception as e:
            print(f"[CAN] Close error: {e} (continuing anyway)")
    
    def __enter__(self):
        return self
    
    def __exit__(self, exc_type, exc_val, exc_tb):
        self.close()
