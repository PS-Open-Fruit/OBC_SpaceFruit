import zlib


class KISSProtocol:
    FEND = 0xC0
    FESC = 0xDB
    TFEND = 0xDC
    TFESC = 0xDD

    PID_ACK = 0xAC

    CMD_DATA = 0x00

    # Example payload IDs
    PAYLOAD_GENERIC = 0x00
    PAYLOAD_IMAGE = 0x01
    PID_TRANSFER_IMAGE = 0x02


    PAYLOAD_ID_VR               = 0x01

    VR_PID_GET_STATUS           = 0x00
    VR_PID_GET_IMAGE_CAPTURE    = 0x01
    VR_PID_GET_IMAGE_REQUEST    = 0x02
    VR_PID_GET_IMAGE_DOWNLOAD   = 0x03

    PID_ACK                     = 0xAC


    # @staticmethod
    # def calculate_crc(data: bytes) -> int:
    #     return zlib.crc32(data) & 0xFFFFFFFF

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
    def escape(cls, data: bytes) -> bytes:
        output = bytearray()
        for b in data:
            if b == cls.FEND:
                output.extend([cls.FESC, cls.TFEND])
            elif b == cls.FESC:
                output.extend([cls.FESC, cls.TFESC])
            else:
                output.append(b)
        return bytes(output)

    @classmethod
    def unescape(cls, data: bytes) -> bytes:
        output = bytearray()
        i = 0
        while i < len(data):
            if data[i] == cls.FESC:
                i += 1
                if i >= len(data):
                    break
                if data[i] == cls.TFEND:
                    output.append(cls.FEND)
                elif data[i] == cls.TFESC:
                    output.append(cls.FESC)
            else:
                output.append(data[i])
            i += 1
        return bytes(output)
    
    # -------------------------------------------------
    # Frame Builder
    # -------------------------------------------------
    @classmethod
    def wrap_frame(cls, payload_id: int, pid: int, data: bytes,
                   command: int = CMD_DATA) -> bytes:
        """
        Frame format:
        [FEND][CMD][PayloadID][PID][Data...][CRC32][FEND]

        CRC32 is calculated over: PayloadID + PID + Data
        """

        structured_payload = bytes([payload_id, pid]) + data
        crc = cls.calculate_crc(structured_payload)

        payload_with_crc = structured_payload + crc.to_bytes(4, "big")

        frame = bytearray([cls.FEND, command])
        frame.extend(cls.escape(payload_with_crc))
        frame.append(cls.FEND)

        return bytes(frame)

    # -------------------------------------------------
    # IMAGE TRANSFER FRAME
    # -------------------------------------------------
    @classmethod
    def wrap_image_chunk(cls,
                         file_id: int,
                         chunk_id: int,
                         content: bytes,
                         pid: int = 0x02,
                         command: int = CMD_DATA) -> bytes:
        """
        Frame format:
        [FEND][CMD][PayloadID][PID][FileID][ChunkID(4)][Content][CRC32][FEND]
        """
        # _payload_id = int.to_bytes(cls.PAYLOAD_IMAGE)
        # _pid = int.to_bytes(pid)
        # _file_id = int.to_bytes(file_id)
        # structured = (
        #     bytes([_payload_id + _pid + _file_id]) +
        #     chunk_id.to_bytes(4, "big") +
        #     content
        # )
        # print(bytes([cls.PAYLOAD_IMAGE, pid, file_id]))
        # print(type(chunk_id.to_bytes(4, "big")))
        # print(type(content))
        structured = (
            bytes([cls.PAYLOAD_IMAGE, pid, file_id]) +
            chunk_id.to_bytes(4, "big") +
            content
        )

        crc = cls.calculate_crc(structured)
        payload_with_crc = structured + crc.to_bytes(4, "big")

        frame = bytearray([cls.FEND, command])
        frame.extend(cls.escape(payload_with_crc))
        frame.append(cls.FEND)

        return bytes(frame)

    # -------------------------------------------------
    # UNWRAP (Auto detect payload type)
    # -------------------------------------------------
    @classmethod
    def unwrap_frame(cls, frame: bytes):

        if len(frame) < 9:
            print("len less than 9 return None")
            return None

        if frame[0] != cls.FEND or frame[-1] != cls.FEND:
            print("no proper FEND return None")
            return None

        inner = frame[1:-1]
        unescaped = cls.unescape(inner)
        command = unescaped[0]
        payload_with_crc = unescaped[1:]

        if len(payload_with_crc) < 6:
            print("len less than 6 return None")
            return None

        payload = payload_with_crc[:-4]
        recv_crc = int.from_bytes(payload_with_crc[-4:], "big")

        if cls.calculate_crc(payload) != recv_crc:
            print("crc err return None")
            return None

        payload_id = payload[0]
        pid = payload[1]

        # -------- IMAGE TRANSFER --------
        if (payload_id == cls.PAYLOAD_ID_VR) and (pid == cls.VR_PID_GET_IMAGE_DOWNLOAD):
            file_id = payload[2]
            chunk_id = int.from_bytes(payload[3:7], "big")
            content = payload[7:]

            return {
                "type": "image",
                "command": command,
                "payload_id": payload_id,
                "pid": pid,
                "file_id": file_id,
                "chunk_id": chunk_id,
                "content": content
            }

        # -------- GENERIC --------
        # elif payload_id == cls.PAYLOAD_GENERIC:
        #     return {
        #         "type": "generic",
        #         "command": command,
        #         "payload_id": payload_id,
        #         "pid": pid,
        #         "data": payload[2:]
        #     }
        else:
            return {
                "type": "generic",
                "command": command,
                "payload_id": payload_id,
                "pid": pid,
                "data": payload[2:]
            }
        return None