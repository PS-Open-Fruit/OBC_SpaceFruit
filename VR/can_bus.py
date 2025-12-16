"""
Simple CAN Bus Module for Python
Easy-to-use module for CAN Bus communication with simple function calls.
Includes reliable send with buffering and persistence for space-grade reliability.
"""

import can
import threading
import json
import os
import time
from typing import Callable, Optional, List, Dict, Any

# Global CAN bus instance
_bus: Optional[can.Bus] = None
_receive_thread: Optional[threading.Thread] = None
_is_receiving = False
_callbacks: List[Callable] = []

# Reliable send queue (space-grade buffering)
_send_queue: List[Dict[str, Any]] = []  # [{'id': ..., 'data': ..., 'attempts': ..., 'last_attempt': ...}, ...]
_send_queue_lock = threading.Lock()
_retry_thread: Optional[threading.Thread] = None
_is_retrying = False
_queue_file = "can_send_queue.json"  # Persistent buffer on disk
_retry_config = {
    'max_attempts': 10,
    'initial_delay': 0.1,  # seconds
    'max_delay': 5.0,
    'backoff_factor': 1.5,
}


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
    stop_reliable_send()  # Stop retry worker before closing
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


# ============================================================================
# Reliable Send with Buffering & Persistence (for CubeSat & Space-Grade Apps)
# ============================================================================

def _load_queue_from_disk():
    """Load persisted send queue from disk at startup."""
    global _send_queue
    if os.path.exists(_queue_file):
        try:
            with open(_queue_file, 'r') as f:
                _send_queue = json.load(f)
            print(f"Loaded {len(_send_queue)} buffered messages from disk")
        except Exception as e:
            print(f"Error loading queue from disk: {e}")
            _send_queue = []
    else:
        _send_queue = []


def _save_queue_to_disk():
    """Persist send queue to disk for recovery on restart."""
    try:
        with open(_queue_file, 'w') as f:
            json.dump(_send_queue, f)
    except Exception as e:
        print(f"Error saving queue to disk: {e}")


def _calculate_backoff(attempt: int) -> float:
    """Calculate exponential backoff delay."""
    delay = _retry_config['initial_delay'] * (_retry_config['backoff_factor'] ** attempt)
    return min(delay, _retry_config['max_delay'])


def send_reliable(msg_id, data, label: str = ""):
    """
    Send a CAN message with reliable buffering and automatic retry.
    
    If send fails, message is buffered to disk and retried automatically
    with exponential backoff. Unsent messages survive app restart.
    
    Args:
        msg_id: Message ID (e.g., 0x123)
        data: Data bytes (list or bytes, up to 8 bytes)
        label: Optional label for debugging (e.g., "telemetry_1")
    
    Returns:
        True if queued for send (not necessarily sent yet)
    
    Example:
        can_bus.send_reliable(0x123, [0x01, 0x02, 0x03])
    """
    if _bus is None:
        print("Error: CAN bus not initialized. Call init() first.")
        return False
    
    if isinstance(data, list):
        data = bytes(data)
    
    with _send_queue_lock:
        msg_entry = {
            'id': msg_id,
            'data': list(data),  # JSON-safe
            'label': label,
            'attempts': 0,
            'last_attempt': time.time(),
            'queued_at': time.time(),
        }
        _send_queue.append(msg_entry)
        print(f"Queued message (label={label}, id=0x{msg_id:X}). Queue size: {len(_send_queue)}")
        _save_queue_to_disk()
    
    return True


def _retry_loop():
    """Background worker: retry queued messages with exponential backoff."""
    global _is_retrying
    while _is_retrying:
        time.sleep(0.1)  # Check queue every 100ms
        
        with _send_queue_lock:
            if not _send_queue:
                continue
            
            now = time.time()
            for i, msg_entry in enumerate(_send_queue[:]):  # Copy to iterate safely
                attempts = msg_entry.get('attempts', 0)
                if attempts >= _retry_config['max_attempts']:
                    print(f"FAIL: Message {msg_entry.get('label', msg_entry['id'])} exhausted retries")
                    _send_queue.remove(msg_entry)
                    _save_queue_to_disk()
                    continue
                
                last_attempt = msg_entry.get('last_attempt', 0)
                backoff = _calculate_backoff(attempts)
                
                if now - last_attempt < backoff:
                    continue  # Not ready to retry yet
                
                # Try to send
                msg_entry['attempts'] += 1
                msg_entry['last_attempt'] = now
                
                try:
                    if isinstance(msg_entry['data'], list):
                        data = bytes(msg_entry['data'])
                    else:
                        data = msg_entry['data']
                    msg = can.Message(arbitration_id=msg_entry['id'], data=data)
                    _bus.send(msg, timeout=0.1)
                    
                    # Success: remove from queue
                    print(f"SUCCESS: Message {msg_entry.get('label', msg_entry['id'])} sent after {msg_entry['attempts']} attempt(s)")
                    _send_queue.remove(msg_entry)
                    _save_queue_to_disk()
                    
                except Exception as e:
                    if msg_entry['attempts'] < 3:  # Only log first few failures
                        print(f"Retry {msg_entry['attempts']}/{_retry_config['max_attempts']} for {msg_entry.get('label', msg_entry['id'])}: {e}")


def start_reliable_send():
    """Start the background retry worker for reliable send queue."""
    global _retry_thread, _is_retrying
    
    if _bus is None:
        print("Error: CAN bus not initialized. Call init() first.")
        return
    
    if _is_retrying:
        print("Reliable send already running.")
        return
    
    _load_queue_from_disk()  # Load any persisted messages
    _is_retrying = True
    _retry_thread = threading.Thread(target=_retry_loop, daemon=True)
    _retry_thread.start()
    print("Started reliable send worker")


def stop_reliable_send():
    """Stop the background retry worker."""
    global _is_retrying, _retry_thread
    
    if _is_retrying:
        _is_retrying = False
        if _retry_thread:
            _retry_thread.join(timeout=2.0)
        print("Stopped reliable send worker")


def get_queue_status():
    """
    Get current status of the send queue.
    
    Returns:
        Dict with 'pending_count', 'total_attempts', 'oldest_msg_age_s'
    """
    with _send_queue_lock:
        if not _send_queue:
            return {'pending_count': 0, 'total_attempts': 0, 'oldest_msg_age_s': None}
        
        now = time.time()
        oldest = min(msg.get('queued_at', now) for msg in _send_queue)
        total_attempts = sum(msg.get('attempts', 0) for msg in _send_queue)
        
        return {
            'pending_count': len(_send_queue),
            'total_attempts': total_attempts,
            'oldest_msg_age_s': now - oldest,
        }


def clear_queue():
    """Clear the send queue and delete persisted buffer. Use with caution."""
    global _send_queue
    with _send_queue_lock:
        _send_queue = []
        if os.path.exists(_queue_file):
            os.remove(_queue_file)
        print("Send queue cleared")
