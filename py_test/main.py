import serial
import serial.tools.list_ports
import questionary

def select_com_port():
    ports = [
        port for port in serial.tools.list_ports.comports()
        if port.description
        and port.description.strip()
        and port.description.strip().lower() not in ("n/a", "na")
    ]

    if not ports:
        print("No valid serial ports found.")
        return None

    choices = [
        f"{port.device} — {port.description}"
        for port in ports
    ]

    selected = questionary.select(
        "Select a serial port:",
        choices=choices
    ).ask()

    if selected is None:
        return None

    return selected.split(" — ")[0]

port = select_com_port()

if port:
    ser = serial.Serial(port, baudrate=115200, timeout=1)
    print(f"Connected to {port}")
    ser.write(b"abcdefghijklmno")
    ser.close()