import struct

class Transport_Layer_Data:
    def __init__(self,payload_id,pid,data,data_len,status):
        self.payload_id = payload_id 
        self.pid = pid 
        self.data = data 
        self.status = status
        self.data_len = data_len
    

class OSI_Payload_Protocol:

    PID_GS_VR_REQUEST_PING = 0
    PID_GS_VR_REQUEST_PI_STATUS = 1
    PID_GS_VR_REQUEST_CAPTURE = 2
    PID_GS_VR_REQUEST_COPY_IMAGE_TO_SD = 3
    PID_GS_VR_REQUEST_SYSTEM_STATUS = 4
    PID_GS_VR_REQUEST_SHUTDOWN = 0x90, 

    @staticmethod
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

    @classmethod
    def encode_layer_transport(cls,payload_id : int, pid : int,data : bytes) -> bytes:
        out = bytes([payload_id,pid,len(data).to_bytes(2,"big")])
        out += data
        return data

    @classmethod

    def encode_application_status(
                                    cls,
                                    boot_count: int, 
                                    timestamp: int, 
                                    uptime: int, 
                                    cpu_load: int, 
                                    cpu_temp: int, 
                                    ram_usage: int, 
                                    disk_usage: int, 
                                    camera_status: int
                                ) -> bytes:
        # Using a single pack call is more efficient and clearer
        # > : Big-endian
        # I : uint32 (Boot Count)
        # I : uint32 (Timestamp)
        # B : uint8  (Uptime - based on 1B size)
        # I : uint32 (CPU Load - based on 4B size)
        # i : int32  (CPU Temp - signed integer)
        # B : uint8  (RAM Usage)
        # B : uint8  (Disk Usage)
        # B : uint8  (Camera Status)
    
        return struct.pack(
            ">IIBiIBBB", 
            boot_count, 
            timestamp, 
            uptime, 
            cpu_load, 
            cpu_temp, 
            ram_usage, 
            disk_usage, 
            camera_status
        )

    @classmethod
    def encode_application_file_content(cls,data : bytes):
        pass

    @classmethod
    def decode_layer_transport(cls,data : bytes) -> list[bytes,bytes,bytes,bytes,int]:
        crc = cls.calculate_crc(data[:-4])
        content_crc = data[-4:]
        if (crc != content_crc):
            status = 0
            return [0,0,0,0,0]
        payload_id = int.from_bytes(data[0],"big")
        pid = int.from_bytes(data[1],"big")
        data_len = int.from_bytes(data[2:4],"big")
        return [payload_id,pid,data_len,data[4:-4],1]
