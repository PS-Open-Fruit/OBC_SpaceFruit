import time
import can_bus
import platform

# Platform-aware: use slcan on Windows (USB adapter), socketcan on Linux (native)
SYSTEM = platform.system()
if SYSTEM == 'Windows':
    CHANNEL = 'COM6'
    BUSTYPE = 'slcan'
else:
    CHANNEL = 'can0'
    BUSTYPE = 'socketcan'
BITRATE = 250000

# Send a heartbeat periodically so the sender knows we're alive
HEARTBEAT_ID = 0x700
HEARTBEAT_PERIOD_S = 0.1


def _print_msg(msg):
	ts = msg.get('timestamp', time.time())
	print(f"RX t={ts:.3f} id=0x{msg['id']:X} data={msg['data']}")


def main():
	if not can_bus.init(CHANNEL, BUSTYPE, BITRATE):
		return
	can_bus.on_receive(_print_msg)
	can_bus.start()
	print("Receiving and sending heartbeat...")
	
	next_hb = time.monotonic()
	try:
		while True:
			now = time.monotonic()
			if now >= next_hb:
				next_hb += HEARTBEAT_PERIOD_S
				can_bus.send(HEARTBEAT_ID, [0x01], timeout=0.1)
			time.sleep(0.01)
	except KeyboardInterrupt:
		pass
	finally:
		can_bus.stop()
		can_bus.close()


if __name__ == "__main__":
	main()