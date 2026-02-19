
import serial
import serial.tools.list_ports
import time
import os

def list_ports():
    ports = serial.tools.list_ports.comports()
    for port in ports:
        print(f"Found port: {port.device}")

def read_serial_hex(port='COM5', baud=209700):
    output_file = os.path.join(os.path.dirname(os.path.abspath(__file__)), "text.txt")
    
    # Clear file at startup
    with open(output_file, "w") as f:
        f.write("")
    print(f"Output will be saved to: {output_file}")

    try:
        ser = serial.Serial(port, baud, timeout=1)
        print(f"Connected to {port} at {baud} baud.")
        
        while True:
            if ser.in_waiting > 0:
                # Read available bytes
                data = ser.read(ser.in_waiting)
                
                # Format as space-separated hex values
                hex_output = " ".join([f"{b:02X}" for b in data])
                
                print(hex_output)
                
                with open(output_file, "a") as f:
                    f.write(hex_output + "\n")
                
            time.sleep(0.01)
            
    except serial.SerialException as e:
        print(f"Error opening serial port {port}: {e}")
        list_ports()
    except KeyboardInterrupt:
        print("\nExiting...")
        if 'ser' in locals() and ser.is_open:
            ser.close()

if __name__ == "__main__":
    # You can change the COM port here if needed
    PORT = 'COM5' 
    BAUDRATE = 209700
    
    read_serial_hex(PORT, BAUDRATE)
