import argparse
import sys

def calculate_crc(data: bytes) -> int:
    """
    Calculates CRC32 using the polynomial:
    X^32 + X^26 + X^23 + X^22 + X^16 + X^12 + X^11 + X^10 + X^8 + X^7 + X^5 + X^4 + X^2 + X + 1
    Hex: 0x04C11DB7

    Configuration (Matches STM32 Hardware CRC / MPEG-2):
    - Poly: 0x04C11DB7
    - Init: 0xFFFFFFFF
    - RefIn: False (MSB first)
    - RefOut: False
    - XorOut: 0
    """
    crc = 0xFFFFFFFF
    poly = 0x04C11DB7

    for byte in data:
        # Align byte to the top 8 bits of the register (MSB-first)
        crc ^= (byte << 24)
        for _ in range(8):
            if crc & 0x80000000:
                crc = ((crc << 1) ^ poly) & 0xFFFFFFFF
            else:
                crc = (crc << 1) & 0xFFFFFFFF

    return crc

def process_hex_string(hex_input: str) -> str:
    """
    Takes a space-separated hex string, calculates the CRC32,
    and returns the CRC as a space-separated hex string (02X format).
    """
    try:
        data_bytes = bytes.fromhex(hex_input)
    except ValueError as e:
        sys.exit(f"Error parsing hex input: {e}\nMake sure you are only using valid hex characters (0-9, A-F).")

    crc_int = calculate_crc(data_bytes)
    
    # Using byteorder='big' (MSB first). Change to 'little' if your system needs it reversed.
    crc_bytes = crc_int.to_bytes(4, byteorder='big')
    crc_hex_string = " ".join(f"{b:02X}" for b in crc_bytes)
    
    return crc_hex_string

def main():
    parser = argparse.ArgumentParser(
        description="Calculate STM32/MPEG-2 CRC32 from a space-separated hex string."
    )
    
    # nargs='+' allows passing multiple space-separated bytes without quotes
    parser.add_argument(
        "hex_data", 
        nargs="+", 
        help="The hex data to process (e.g., 01 0A 2B 3C)"
    )

    args = parser.parse_args()

    # Join the terminal arguments back into a single string
    combined_hex_string = " ".join(args.hex_data)
    
    # Calculate and print
    result = process_hex_string(combined_hex_string)
    print(result)

if __name__ == "__main__":
    main()
