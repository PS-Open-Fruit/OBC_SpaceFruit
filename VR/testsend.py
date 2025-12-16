import time
import can_bus

# Adapter configuration
CHANNEL = 'COM6'      # WeAct USB2CANFD V1 on Windows
BUSTYPE = 'slcan'
BITRATE = 250000

# Heartbeat gating: if enabled, only send when a heartbeat is seen recently.
# If you are just running `candump` on the Pi (no heartbeat), set USE_HEARTBEAT=False.
USE_HEARTBEAT = False
HEARTBEAT_ID = 0x700
HEARTBEAT_TIMEOUT_S = 1.0

# Sending parameters
SEND_ID = 0x123
SEND_PERIOD_S = 0.1
SEND_TIMEOUT_S = 0.2  # Fail fast when adapter/backend can't accept frame

_last_heartbeat = 0.0


def _on_rx(msg):
    """Track heartbeat frames and optionally print others for debugging."""
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

    print("Starting gated sender. Waiting for heartbeat to begin...")

    next_send = time.monotonic()
    counter = 0
    try:
        while True:
            now = time.monotonic()
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
                    ok = can_bus.send(SEND_ID, payload, timeout=SEND_TIMEOUT_S)
                    if ok:
                        print(f"TX id=0x{SEND_ID:X} data={payload}")
                        counter = (counter + 1) % 65536
                    else:
                        print("Send failed; backing off 200 ms")
                        time.sleep(0.2)
                else:
                    print("Peer offline (no heartbeat); skipping send")

            time.sleep(0.01)
    except KeyboardInterrupt:
        pass
    finally:
        can_bus.stop()
        can_bus.close()


if __name__ == "__main__":
    main()