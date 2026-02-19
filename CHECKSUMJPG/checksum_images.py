import os
import zlib
import struct

def crc32_stm32(data):
    """
    Calculates CRC32 using the standard STM32 hardware CRC default parameters:
    - Poly: 0x04C11DB7
    - Init: 0xFFFFFFFF
    - RefIn: False
    - RefOut: False
    - XorOut: 0x00000000
    """
    crc = 0xFFFFFFFF
    for byte in data:
        crc ^= (byte << 24)
        for _ in range(8):
            if crc & 0x80000000:
                crc = (crc << 1) ^ 0x04C11DB7
            else:
                crc <<= 1
            crc &= 0xFFFFFFFF
    return crc

def calculate_accuracy_bit_by_bit(ref_content, target_content):
    """
    Compares two byte arrays bit-by-bit (byte-by-byte in Python) and returns the accuracy percentage.
    Accuracy is defined as: (matching_bytes / max_len) * 100
    """
    len1 = len(ref_content)
    len2 = len(target_content)
    max_len = max(len1, len2)
    
    if max_len == 0: return 100.0 if len1 == len2 else 0.0
    
    # Compare up to the length of the shorter file
    min_len = min(len1, len2)
    match_count = 0
    for i in range(min_len):
        if ref_content[i] == target_content[i]:
            match_count += 1
            
    # Accuracy is based on the maximum length (penalizes missing or extra bytes)
    return (match_count / max_len) * 100.0

def process_files():
    base_dir = os.path.dirname(os.path.abspath(__file__))
    
    # 1. Process Panorama File
    panorama_file = os.path.join(base_dir, "02191444.jpg")
    panorama_crc = 0
    panorama_content = b""
    
    if os.path.exists(panorama_file):
        print(f"Processing Panorama: {os.path.basename(panorama_file)}")
        try:
            with open(panorama_file, "rb") as f:
                panorama_content = f.read()
                panorama_crc = crc32_stm32(panorama_content)
                print(f"  CRC32 (STM32): 0x{panorama_crc:08X}")
                print(f"  Size         : {len(panorama_content):,} bytes")
        except Exception as e:
            print(f"  Error reading file: {e}")
            return
    else:
        print(f"Panorama file not found: {panorama_file}")
        return
        
    print("-" * 80)
    print(f"{'Filename':<25} | {'CRC32':<12} | {'Result':<8} | {'Accuracy %':<10}")
    print("-" * 80)

    # 2. Process Data Directory
    data_dir = os.path.join(base_dir, "data")
    if os.path.exists(data_dir) and os.path.isdir(data_dir):
        files = sorted(os.listdir(data_dir))
        match_count = 0
        total_count = 0
        
        for filename in files:
            file_path = os.path.join(data_dir, filename)
            if os.path.isfile(file_path): # Only process files
                total_count += 1
                try:
                    with open(file_path, "rb") as f:
                        content = f.read()
                        checksum = crc32_stm32(content)
                        result = "MATCH" if checksum == panorama_crc else "MISMATCH"
                        
                        accuracy_pct = 100.0
                        if result == "MATCH":
                            match_count += 1
                        else:
                            # Calculate Accuracy Percentage bit-by-bit
                            accuracy_pct = calculate_accuracy_bit_by_bit(panorama_content, content)
                            
                        print(f"{filename:<25} | 0x{checksum:08X}   | {result:<8} | {accuracy_pct:6.2f}%")
                except Exception as e:
                    print(f"{filename:<25} | ERROR          | {e}")
        
        print("-" * 80)
        print(f"Summary: {match_count}/{total_count} files matched.")
        
    else:
        print(f"Data directory not found: {data_dir}")

if __name__ == "__main__":
    process_files()
