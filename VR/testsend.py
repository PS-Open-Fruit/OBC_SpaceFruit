import time
import can_bus
import platform

# Platform-aware: use slcan on Windows (USB adapter), socketcan on Linux (native)
SYSTEM = platform.system()
if SYSTEM == 'Windows':
    CHANNEL = 'COM6'      # WeAct USB2CANFD V1 on Windows
    BUSTYPE = 'slcan'
else:
    CHANNEL = 'can0'      # Native CAN on Linux (RS485 HAT)
    BUSTYPE = 'socketcan'
BITRATE = 250000

# Heartbeat gating: only send when a heartbeat is seen recently (reliable link detection)
USE_HEARTBEAT = True
HEARTBEAT_ID = 0x700
HEARTBEAT_TIMEOUT_S = 1.0

# Sending parameters
SEND_ID = 0x123
SEND_PERIOD_S = 0.1
SEND_TIMEOUT_S = 0.2  # Fail fast when adapter/backend can't accept frame

_last_heartbeat = 0.0


def _on_rx(msg):
    """Track heartbeat frames to know when peer is online."""
    global _last_heartbeat
    if msg.get('id') == HEARTBEAT_ID:
        _last_heartbeat = time.monotonic()


def _peer_online():
    return (time.monotonic() - _last_heartbeat) < HEARTBEAT_TIMEOUT_S


def main():
    if not can_bus.init(CHANNEL, BUSTYPE, BITRATE):
        return

    can_bus.on_receive(_on_rx)
    can_bus.start()
    can_bus.start_reliable_send()  # Start the reliable send worker

    print("Starting reliable sender. Waiting for heartbeat to begin...")

    next_send = time.monotonic()
    next_status = time.monotonic()
    counter = 0
    try:
        while True:
            now = time.monotonic()
            
            # Print queue status every 2 seconds
            if now >= next_status:
                next_status += 2.0
                status = can_bus.get_queue_status()
                if status['pending_count'] > 0:
                    print(f"[QUEUE] Pending: {status['pending_count']}, Attempts: {status['total_attempts']}, Age: {status['oldest_msg_age_s']:.1f}s")
            
            if now >= next_send:
                next_send += SEND_PERIOD_S

                # If backend exposes bus state, pause when bus-off
                state = getattr(can_bus, "get_state", lambda: None)()
                if state is not None and str(state).upper().endswith("BUS_OFF"):
                    print("Bus OFF; pausing 1s")
                    time.sleep(1.0)
                    continue

                if (not USE_HEARTBEAT) or _peer_online():
                    payload = [counter & 0xFF, (counter >> 8) & 0xFF, 0x01, 0x02]
                    # Use send_reliable: buffers & retries automatically if send fails
                    can_bus.send_reliable(SEND_ID, payload, label=f"data_{counter}")
                    print(f"TX id=0x{SEND_ID:X} data={payload}")
                    counter = (counter + 1) % 65536
                else:
                    print("Peer offline (no heartbeat); queuing paused")

            time.sleep(0.01)
    except KeyboardInterrupt:
        pass
    finally:
        can_bus.stop()
        can_bus.close()


if __name__ == "__main__":
    main()