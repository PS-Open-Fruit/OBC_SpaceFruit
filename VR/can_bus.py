"""
Simple CAN Bus Module for Python
Easy-to-use module for CAN Bus communication with simple function calls.
"""

import can
import threading
from typing import Callable, Optional, List

# Global CAN bus instance
_bus: Optional[can.Bus] = None
_receive_thread: Optional[threading.Thread] = None
_is_receiving = False
_callbacks: List[Callable] = []


def init(channel='can0', bustype='socketcan', bitrate=500000):
    """
    Initialize the CAN bus.
    
    Args:
        channel: CAN interface name (e.g., 'can0', 'vcan0')
        bustype: Interface type ('socketcan', 'pcan', 'vector', 'virtual')
        bitrate: Bus speed in bits/second (default: 500000)
    
    Example:
        can_bus.init('can0', 'socketcan', 500000)
    """
    global _bus
    try:
        _bus = can.Bus(channel=channel, bustype=bustype, bitrate=bitrate)
        print(f"CAN bus connected: {channel}")
        return True
    except Exception as e:
        print(f"Error connecting to CAN bus: {e}")
        return False


def send(msg_id, data, timeout: Optional[float] = None):
    """
    Send a CAN message.
    
    Args:
        msg_id: Message ID (e.g., 0x123)
        data: Data bytes (list or bytes, up to 8 bytes)
    
    Example:
        can_bus.send(0x123, [0x01, 0x02, 0x03, 0x04])
        can_bus.send(0x456, b'\\x11\\x22\\x33')
    """
    if _bus is None:
        print("Error: CAN bus not initialized. Call init() first.")
        return False
    
    try:
        # If backend exposes bus state and it's bus-off, refuse to send
        try:
            state = getattr(_bus, "state", None)
            if state is not None and str(state).upper().endswith("BUS_OFF"):
                print("Bus state is BUS_OFF; send skipped")
                return False
        except Exception:
            pass
        if isinstance(data, list):
            data = bytes(data)
        msg = can.Message(arbitration_id=msg_id, data=data)
        # Pass through timeout when available; backend may raise on timeout
        _bus.send(msg, timeout=timeout)
        return True
    except Exception as e:
        print(f"Error sending message: {e}")
        return False


def get_state():
    """
    Get current bus state if supported by backend.

    Returns:
        Backend-specific state enum/string, or None if unsupported.
    """
    if _bus is None:
        print("Error: CAN bus not initialized. Call init() first.")
        return None
    try:
        return getattr(_bus, "state", None)
    except Exception:
        return None


def receive(timeout=1.0):
    """
    Receive one CAN message (blocking).
    
    Args:
        timeout: Maximum wait time in seconds
    
    Returns:
        Dictionary with 'id' and 'data' keys, or None if timeout
    
    Example:
        msg = can_bus.receive(timeout=2.0)
        if msg:
            print(f"ID: {msg['id']}, Data: {msg['data']}")
    """
    if _bus is None:
        print("Error: CAN bus not initialized. Call init() first.")
        return None
    
    try:
        msg = _bus.recv(timeout=timeout)
        if msg:
            return {
                'id': msg.arbitration_id,
                'data': list(msg.data),
                'timestamp': msg.timestamp
            }
    except Exception as e:
        print(f"Error receiving message: {e}")
    return None


def on_receive(callback):
    """
    Register a callback function for incoming messages.
    The callback will receive a dictionary with 'id' and 'data'.
    
    Args:
        callback: Function to call when message is received
    
    Example:
        def my_handler(msg):
            print(f"Received ID: {msg['id']}, Data: {msg['data']}")
        
        can_bus.on_receive(my_handler)
        can_bus.start()
    """
    global _callbacks
    _callbacks.append(callback)


def _receive_loop():
    """Internal function for background receiving."""
    global _is_receiving
    while _is_receiving:
        try:
            msg = _bus.recv(timeout=0.1)
            if msg:
                msg_dict = {
                    'id': msg.arbitration_id,
                    'data': list(msg.data),
                    'timestamp': msg.timestamp
                }
                for callback in _callbacks:
                    try:
                        callback(msg_dict)
                    except Exception as e:
                        print(f"Error in callback: {e}")
        except Exception as e:
            if _is_receiving:
                print(f"Error in receive loop: {e}")


def start():
    """
    Start receiving messages in background.
    Messages will be passed to registered callbacks.
    
    Example:
        can_bus.on_receive(my_handler)
        can_bus.start()
    """
    global _receive_thread, _is_receiving
    
    if _bus is None:
        print("Error: CAN bus not initialized. Call init() first.")
        return
    
    if _is_receiving:
        print("Already receiving messages.")
        return
    
    _is_receiving = True
    _receive_thread = threading.Thread(target=_receive_loop, daemon=True)
    _receive_thread.start()
    print("Started receiving CAN messages")


def stop():
    """
    Stop receiving messages.
    
    Example:
        can_bus.stop()
    """
    global _is_receiving, _receive_thread
    
    if _is_receiving:
        _is_receiving = False
        if _receive_thread:
            _receive_thread.join(timeout=2.0)
        print("Stopped receiving messages")


def close():
    """
    Close the CAN bus connection.
    
    Example:
        can_bus.close()
    """
    global _bus
    
    stop()
    if _bus:
        _bus.shutdown()
        _bus = None
        print("CAN bus disconnected")


def set_filter(msg_id, mask=0x7FF):
    """
    Filter messages by ID.
    
    Args:
        msg_id: The CAN ID to filter for
        mask: Bit mask for filtering (default: 0x7FF for 11-bit IDs)
    
    Example:
        can_bus.set_filter(0x100)  # Only receive ID 0x100
    """
    if _bus is None:
        print("Error: CAN bus not initialized. Call init() first.")
        return
    
    try:
        _bus.set_filters([{"can_id": msg_id, "can_mask": mask}])
        print(f"Filter set: ID=0x{msg_id:X}, Mask=0x{mask:X}")
    except Exception as e:
        print(f"Error setting filter: {e}")
