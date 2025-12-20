"""
Reliable CAN Bus communication library for Raspberry Pi Zero 2W
Prevents hardware buffer overflow by managing messages in application layer

KISS Design:
- Detect link failures immediately
- Queue messages in RAM (not hardware buffer)
- Flush hardware buffers when link fails
- Simple retry logic
"""

import can
import time
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
        
        # Statistics
        self._queued_total = 0
        self._dropped_total = 0
        self._sent_total = 0
        
        if self._is_pi and auto_setup:
            self._setup_socketcan()
        
        self._connect()
    
    def _setup_socketcan(self):
        """Setup SocketCAN with minimal TX queue to prevent buffer buildup"""
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
    
    def send(self, can_id: int, data: List[int], extended: bool = False) -> bool:
        """
        Send CAN message reliably (queues if link down, no data loss)
        
        Args:
            can_id: CAN arbitration ID (0x000-0x7FF standard, 0x1FFFFFFF extended)
            data: List of bytes (0-8 bytes)
            extended: Use extended ID format
            
        Returns:
            True if sent or queued successfully, False only if queue full
        """
        if not 0 <= len(data) <= 8:
            raise ValueError("Data must be 0-8 bytes")
        
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
            self.bus.send(msg, timeout=0.05)
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
    
    def _queue_message(self, can_id: int, data: List[int], extended: bool) -> bool:
        """Queue message in RAM (not hardware buffer)"""
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
    
    def _drain_queue(self):
        """Send queued messages when link recovers"""
        if len(self._msg_queue) == 0:
            return
        
        print(f"[CAN] Draining {len(self._msg_queue)} queued messages...")
        sent = 0
        failed = False
        
        while self._msg_queue and sent < 100:  # Limit drain rate
            can_id, data, extended = self._msg_queue[0]
            msg = can.Message(arbitration_id=can_id, data=data, is_extended_id=extended)
            
            try:
                self.bus.send(msg, timeout=0.05)
                self._msg_queue.popleft()
                sent += 1
                self._sent_total += 1
            except Exception:
                # Failed during drain - stop immediately
                failed = True
                print(f"[CAN] Drain failed - {len(self._msg_queue)} msgs remain in queue")
                self._link_ok = False  # Keep link marked as DOWN
                break
        
        if not failed and len(self._msg_queue) == 0:
            print(f"[CAN] Queue drained successfully - link UP")
            self._link_ok = True
            self._fail_count = 0
        elif not failed:
            print(f"[CAN] Drained {sent} msgs - {len(self._msg_queue)} remain")

    
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
        Receive CAN message
        
        Args:
            timeout: Timeout in seconds
            
        Returns:
            (can_id, data) or None if timeout
        """
        try:
            msg = self.bus.recv(timeout=timeout)
            if msg is None:
                return None
            return (msg.arbitration_id, list(msg.data))
        except Exception as e:
            print(f"[CAN] RX error: {e}")
            return None
    
    def receive_all(self, timeout: float = 0.01, max_msgs: int = 100) -> List[Tuple[int, List[int]]]:
        """
        Receive all available messages (bulk read)
        
        Args:
            timeout: Timeout for first message
            max_msgs: Max messages per call
            
        Returns:
            List of (can_id, data) tuples
        """
        messages = []
        
        try:
            msg = self.bus.recv(timeout=timeout)
            if msg:
                messages.append((msg.arbitration_id, list(msg.data)))
                
                # Get remaining buffered messages
                for _ in range(max_msgs - 1):
                    msg = self.bus.recv(timeout=0)
                    if not msg:
                        break
                    messages.append((msg.arbitration_id, list(msg.data)))
        except Exception as e:
            print(f"[CAN] RX error: {e}")
        
        return messages
    
    def send_with_ack(self, can_id: int, data: List[int], ack_id: Optional[int] = None, 
                      timeout: float = 2.0, max_retries: int = 3, extended: bool = False) -> bool:
        """
        Send message and wait for ACK (guarantees delivery to destination)
        
        Args:
            can_id: CAN ID for data message
            data: Data bytes (0-8 bytes)
            ack_id: Expected ACK CAN ID (default: can_id + 1)
            timeout: Timeout waiting for ACK in seconds
            max_retries: Number of retries if no ACK
            extended: Use extended ID format
            
        Returns:
            True if ACK received, False if failed after retries
        """
        if ack_id is None:
            ack_id = can_id + 1
        
        for attempt in range(max_retries):
            # Send the message
            self.send(can_id, data, extended)
            
            # Wait for ACK
            start_time = time.time()
            while time.time() - start_time < timeout:
                result = self.receive(timeout=0.1)
                if result:
                    rx_id, rx_data = result
                    # Check if this is our ACK (ID matches and first byte is 0xFF)
                    if rx_id == ack_id and len(rx_data) > 0 and rx_data[0] == 0xFF:
                        return True
            
            # No ACK received, retry
            if attempt < max_retries - 1:
                print(f"[NO ACK] Retry {attempt + 2}/{max_retries}")
        
        print(f"[ACK FAIL] No ACK after {max_retries} attempts")
        return False
    
    def send_ack(self, original_id: int, original_data: List[int], extended: bool = False) -> bool:
        """
        Send ACK for received message
        
        Args:
            original_id: CAN ID of message being acknowledged
            original_data: Data of original message
            extended: Use extended ID format
            
        Returns:
            True if ACK sent successfully
        """
        ack_id = original_id + 1
        ack_data = [0xFF, original_data[0] if original_data else 0x00]
        return self.send(ack_id, ack_data, extended)
    
    def check_link(self) -> bool:
        """
        Manually check link health (attempts to send/drain)
        Call this periodically from your main loop
        
        Returns:
            True if link is up
        """
        if not self._link_ok:
            # Try to drain queue (this will mark link up if successful)
            self._drain_queue()
        
        return self._link_ok
    
    def get_stats(self) -> dict:
        """
        Get statistics
        
        Returns:
            Dictionary with queue and transmission stats
        """
        return {
            'link_up': self._link_ok,
            'queue_length': len(self._msg_queue),
            'queue_max': self._queue_size,
            'total_sent': self._sent_total,
            'total_queued': self._queued_total,
            'total_dropped': self._dropped_total,
        }
    
    def clear_queue(self):
        """Clear all queued messages (use with caution - data loss!)"""
        cleared = len(self._msg_queue)
        self._msg_queue.clear()
        if cleared > 0:
            print(f"[CAN] Cleared {cleared} queued messages")
        return cleared
    
    def close(self):
        """Close CAN bus and report final statistics"""
        stats = self.get_stats()
        print(f"[CAN] Closing - Stats: {stats}")
        
        if self.bus:
            self.bus.shutdown()
        
        if self._is_pi:
            import os
            os.system(f'sudo ip link set {self.interface} down 2>/dev/null')
    
    def __enter__(self):
        return self
    
    def __exit__(self, exc_type, exc_val, exc_tb):
        self.close()
