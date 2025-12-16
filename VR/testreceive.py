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


def _print_msg(msg):
	ts = msg.get('timestamp', time.time())
	print(f"RX t={ts:.3f} id=0x{msg['id']:X} data={msg['data']}")


def main():
	if not can_bus.init(CHANNEL, BUSTYPE, BITRATE):
		return
	can_bus.on_receive(_print_msg)
	can_bus.start()
	print("Receiving for 10 seconds...")
	try:
		time.sleep(10)
	finally:
		can_bus.stop()
		can_bus.close()


if __name__ == "__main__":
	main()