"""
Simple CAN Bus communication library
Supports SocketCAN (can0 on Pi) and USB CAN adapters (COM port on PC)
"""

import can
import time
import struct
from typing import Optional, List, Tuple
from collections import deque


class CANBus:
    """
    Simple CAN Bus interface with reliable transmission
    
    Args:
        interface: 'can0' for Pi SocketCAN or 'COM6' (or other COM port) for USB CAN
        bitrate: CAN bitrate in bps (default: 100000)
        auto_setup: Auto setup CAN interface on Pi (default: True)
        queue_size: Max messages to queue when link is down (default: 100, 0=unlimited)
    """
    
    def __init__(self, interface: str = 'can0', bitrate: int = 100000, auto_setup: bool = True, queue_size: int = 100):
        self.interface = interface
        self.bitrate = bitrate
        self.bus = None
        self._is_pi = not interface.startswith('COM')
        self._consecutive_failures = 0
        self._link_down = False
        self._queue_size = queue_size
        self._message_queue = deque(maxlen=queue_size if queue_size > 0 else None)
        self._queued_count = 0
        self._dropped_count = 0
        
        if self._is_pi and auto_setup:
            self._setup_socketcan()
        
        self._connect()
    
    def _setup_socketcan(self):
        """Setup SocketCAN interface on Pi"""
        import os
        os.system(f'sudo ip link set {self.interface} down 2>/dev/null')
        os.system(f'sudo ip link set {self.interface} type can bitrate {self.bitrate}')
        os.system(f'sudo ip link set {self.interface} up')
        time.sleep(0.1)  # Give it time to come up
    
    def _connect(self):
        """Connect to CAN bus"""
        try:
            if self._is_pi:
                # Pi with SocketCAN
                self.bus = can.interface.Bus(
                    channel=self.interface,
                    bustype='socketcan'
                )
            else:
                # PC with USB CAN adapter
                self.bus = can.interface.Bus(
                    channel=self.interface,
                    bustype='slcan',
                    bitrate=self.bitrate
                )
            print(f"[CAN] Connected to {self.interface}")
        except Exception as e:
            error_msg = str(e)
            if "serial_for_url" in error_msg or "serial" in error_msg:
                raise ConnectionError(
                    f"Failed to connect to {self.interface}: Missing pyserial package.\n"
                    "Install it with: pip install pyserial"
                )
            raise ConnectionError(f"Failed to connect to {self.interface}: {e}")
    
    def send(self, can_id: int, data: List[int], extended: bool = False, max_retries: int = 3, queue_on_fail: bool = True) -> bool:
        """
        Send CAN message with retry logic and link monitoring
        
        Args:
            can_id: CAN arbitration ID (0x000 to 0x7FF for standard, up to 0x1FFFFFFF for extended)
            data: List of bytes (0-8 bytes)
            extended: Use extended ID format
            max_retries: Number of retries on failure
            queue_on_fail: Queue message if link is down (default: True)
            
        Returns:
            True if sent successfully or queued, False if failed and not queued
        """
        if not 0 <= len(data) <= 8:
            raise ValueError("Data must be 0-8 bytes")
        
        # Try to drain queue first if link appears up
        if not self._link_down and len(self._message_queue) > 0:
            self._drain_queue()
        
        # If link is down, queue the message
        if self._link_down:
            if queue_on_fail:
                old_len = len(self._message_queue)
                self._message_queue.append((can_id, data.copy(), extended))
                if len(self._message_queue) < old_len:
                    # Message was dropped due to queue size limit
                    self._dropped_count += 1
                    print(f"[CAN] Message dropped (queue full: {self._queue_size}), total dropped: {self._dropped_count}")
                else:
                    self._queued_count += 1
                    if self._queued_count % 10 == 0:
                        print(f"[CAN] Messages queued: {len(self._message_queue)}")
                return True  # Queued = success from caller's perspective
            return False
        
        msg = can.Message(
            arbitration_id=can_id,
            data=data,
            is_extended_id=extended
        )
        
        for attempt in range(max_retries):
            try:
                # Use shorter timeout to avoid blocking
                self.bus.send(msg, timeout=0.1)
                # Reset failure counter on success
                if self._consecutive_failures > 0:
                    print(f"[CAN] Link recovered after {self._consecutive_failures} failures")
                    self._consecutive_failures = 0
                    self._link_down = False
                    # Drain queued messages now that link is back
                    self._drain_queue()
                return True
            except (can.CanError, OSError) as e:
                if attempt == max_retries - 1:
                    self._consecutive_failures += 1
                    
                    # Detect persistent link issues
                    if self._consecutive_failures >= 5:
                        if not self._link_down:
                            print(f"[CAN] Link down detected, queuing enabled (queued: {len(self._message_queue)})")
                            self._link_down = True
                            self.flush_buffers()
                            # Queue this message since we just detected link down
                            if queue_on_fail:
                                self._message_queue.append((can_id, data.copy(), extended))
                                self._queued_count += 1
                                return True
                    elif self._consecutive_failures % 10 == 0:
                        print(f"[CAN] Warning: {self._consecutive_failures} consecutive send failures")
                    
                    return False
                time.sleep(0.01)  # Brief delay before retry
        
        return False
    
    def receive(self, timeout: float = 1.0) -> Optional[Tuple[int, List[int]]]:
        """
        Receive CAN message
        
        Args:
            timeout: Timeout in seconds (None for blocking)
            
        Returns:
            Tuple of (can_id, data) or None if timeout
        """
        try:
            msg = self.bus.recv(timeout=timeout)
            if msg is None:
                return None
            return (msg.arbitration_id, list(msg.data))
        except Exception as e:
            print(f"[CAN] Receive error: {e}")
            return None
    
    def send_string(self, can_id: int, text: str, extended: bool = False) -> bool:
        """
        Send string as CAN message(s)
        Automatically splits into multiple frames if needed
        
        Args:
            can_id: CAN arbitration ID
            text: String to send (will be encoded as UTF-8)
            extended: Use extended ID format
            
        Returns:
            True if all frames sent successfully
        """
        data = text.encode('utf-8')
        chunks = [data[i:i+7] for i in range(0, len(data), 7)]  # 7 bytes per frame (1 byte for sequence)
        
        for idx, chunk in enumerate(chunks):
            frame = [idx] + list(chunk)  # First byte is sequence number
            if not self.send(can_id, frame, extended):
                return False
            time.sleep(0.001)  # Small delay between frames
        
        return True
    
    def receive_string(self, timeout: float = 5.0, max_frames: int = 100) -> Optional[str]:
        """
        Receive multi-frame string message
        
        Args:
            timeout: Total timeout in seconds
            max_frames: Maximum number of frames to collect
            
        Returns:
            Reconstructed string or None if timeout/error
        """
        frames = {}
        start_time = time.time()
        
        while time.time() - start_time < timeout:
            result = self.receive(timeout=0.1)
            if result is None:
                continue
            
            _, data = result
            if len(data) < 1:
                continue
            
            seq = data[0]
            payload = bytes(data[1:])
            frames[seq] = payload
            
            # Check if we got the last frame (empty or short frame)
            if len(payload) < 7:
                break
            
            if len(frames) >= max_frames:
                break
        
        if not frames:
            return None
        
        # Reconstruct message
        result = b''
        for i in sorted(frames.keys()):
            result += frames[i]
        
        try:
            return result.decode('utf-8')
        except UnicodeDecodeError:
            return None
    
    def _drain_queue(self):
        """Send all queued messages when link recovers"""
        if len(self._message_queue) == 0:
            return
        
        print(f"[CAN] Draining message queue ({len(self._message_queue)} messages)...")
        sent_count = 0
        failed_messages = []
        
        while self._message_queue:
            can_id, data, extended = self._message_queue.popleft()
            
            msg = can.Message(
                arbitration_id=can_id,
                data=data,
                is_extended_id=extended
            )
            
            try:
                self.bus.send(msg, timeout=0.1)
                sent_count += 1
            except (can.CanError, OSError) as e:
                # If we fail during drain, re-queue and stop
                print(f"[CAN] Failed to drain queue, re-queuing {len(self._message_queue) + 1} messages")
                failed_messages.append((can_id, data, extended))
                break
        
        # Re-add failed messages back to front of queue
        for msg in reversed(failed_messages):
            self._message_queue.appendleft(msg)
        
        # Add remaining messages back
        if sent_count > 0:
            print(f"[CAN] Queue drained: {sent_count} messages sent, {len(self._message_queue)} remaining")
            self._queued_count = max(0, self._queued_count - sent_count)
    
    def flush_buffers(self):
        """Flush TX/RX buffers to prevent blocking"""
        try:
            # Drain receive buffer
            while True:
                msg = self.bus.recv(timeout=0)
                if msg is None:
                    break
            
            # On Pi, reset the interface to clear buffers
            if self._is_pi:
                import os
                print(f"[CAN] Resetting interface {self.interface}...")
                os.system(f'sudo ip link set {self.interface} down 2>/dev/null')
                time.sleep(0.1)
                os.system(f'sudo ip link set {self.interface} type can bitrate {self.bitrate} restart-ms 100')
                os.system(f'sudo ip link set {self.interface} up')
                time.sleep(0.2)
        except Exception as e:
            print(f"[CAN] Buffer flush error: {e}")
    
    def reset_link(self):
        """Manually reset link state (call after fixing connection issues)"""
        self._consecutive_failures = 0
        self._link_down = False
        print(f"[CAN] Link state reset")
        # Try to drain queue after manual reset
        self._drain_queue()
    
    def is_link_up(self) -> bool:
        """Check if link appears to be up"""
        return not self._link_down
    
    def get_queue_stats(self) -> dict:
        """Get queue statistics
        
        Returns:
            Dictionary with queue_size, queued_count, dropped_count
        """
        return {
            'queue_length': len(self._message_queue),
            'total_queued': self._queued_count,
            'total_dropped': self._dropped_count,
            'queue_max': self._queue_size
        }
    
    def clear_queue(self):
        """Clear all queued messages (use with caution!)"""
        cleared = len(self._message_queue)
        self._message_queue.clear()
        if cleared > 0:
            print(f"[CAN] Cleared {cleared} queued messages")
        return cleared
    
    def close(self):
        """Close CAN bus connection"""
        if self.bus:
            self.bus.shutdown()
            print(f"[CAN] Disconnected from {self.interface}")
        
        if self._is_pi:
            import os
            os.system(f'sudo ip link set {self.interface} down 2>/dev/null')
    
    def __enter__(self):
        """Context manager entry"""
        return self
    
    def __exit__(self, exc_type, exc_val, exc_tb):
        """Context manager exit"""
        self.close()
