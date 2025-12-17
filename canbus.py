"""
Simple CAN Bus communication library
Supports SocketCAN (can0 on Pi) and USB CAN adapters (COM port on PC)
"""

import can
import time
import struct
from typing import Optional, List, Tuple


class CANBus:
    """
    Simple CAN Bus interface with reliable transmission
    
    Args:
        interface: 'can0' for Pi SocketCAN or 'COM6' (or other COM port) for USB CAN
        bitrate: CAN bitrate in bps (default: 100000)
        auto_setup: Auto setup CAN interface on Pi (default: True)
    """
    
    def __init__(self, interface: str = 'can0', bitrate: int = 100000, auto_setup: bool = True):
        self.interface = interface
        self.bitrate = bitrate
        self.bus = None
        self._is_pi = not interface.startswith('COM')
        
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
    
    def send(self, can_id: int, data: List[int], extended: bool = False, max_retries: int = 3) -> bool:
        """
        Send CAN message with retry logic
        
        Args:
            can_id: CAN arbitration ID (0x000 to 0x7FF for standard, up to 0x1FFFFFFF for extended)
            data: List of bytes (0-8 bytes)
            extended: Use extended ID format
            max_retries: Number of retries on failure
            
        Returns:
            True if sent successfully, False otherwise
        """
        if not 0 <= len(data) <= 8:
            raise ValueError("Data must be 0-8 bytes")
        
        msg = can.Message(
            arbitration_id=can_id,
            data=data,
            is_extended_id=extended
        )
        
        for attempt in range(max_retries):
            try:
                self.bus.send(msg, timeout=1.0)
                return True
            except can.CanError as e:
                if attempt == max_retries - 1:
                    print(f"[CAN] Send failed after {max_retries} attempts: {e}")
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
