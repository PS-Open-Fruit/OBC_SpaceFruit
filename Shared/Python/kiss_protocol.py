class KISSProtocol:
    """
    Implements the KISS (Keep It Simple, Stupid) Protocol for serial communication.
    Reference: https://en.wikipedia.org/wiki/KISS_(TNC)
    """
    FEND = 0xC0
    FESC = 0xDB
    TFEND = 0xDC
    TFESC = 0xDD
    
    # Standard KISS Command Codes (for Port 0)
    CMD_DATA = 0x00
    CMD_TXDELAY = 0x01
    CMD_P = 0x02
    CMD_SLOTTIME = 0x03
    CMD_TXTAIL = 0x04
    CMD_FULLDUPLEX = 0x05
    CMD_SETHARDWARE = 0x06
    CMD_RETURN = 0xFF

    @classmethod
    def escape(cls, data: bytes) -> bytes:
        """
        Escapes special characters in the data stream.
        FEND -> FESC, TFEND
        FESC -> FESC, TFESC
        """
        output = bytearray()
        for byte in data:
            if byte == cls.FEND:
                output.extend([cls.FESC, cls.TFEND])
            elif byte == cls.FESC:
                output.extend([cls.FESC, cls.TFESC])
            else:
                output.append(byte)
        return bytes(output)

    @classmethod
    def unescape(cls, data: bytes) -> bytes:
        """
        Reverses the escaping process.
        """
        output = bytearray()
        i = 0
        while i < len(data):
            byte = data[i]
            if byte == cls.FESC:
                i += 1
                if i >= len(data): break
                if data[i] == cls.TFEND:
                    output.append(cls.FEND)
                elif data[i] == cls.TFESC:
                    output.append(cls.FESC)
            else:
                output.append(byte)
            i += 1
        return bytes(output)

    @classmethod
    def wrap_frame(cls, payload: bytes, command: int = 0x00) -> bytes:
        """
        Wraps a payload into a KISS frame.
        Format: [FEND] [COMMAND] [ESCAPED_PAYLOAD] [FEND]
        """
        frame = bytearray([cls.FEND, command])
        frame.extend(cls.escape(payload))
        frame.append(cls.FEND)
        return bytes(frame)

    @classmethod
    def unwrap_frame(cls, frame: bytes) -> tuple:
        """
        Unwraps a KISS frame.
        Returns tuple: (command_byte, payload_bytes) or None if invalid.
        """
        if len(frame) < 3: return None
        
        # Validate FENDs
        if frame[0] != cls.FEND or frame[-1] != cls.FEND:
            return None

        # Extract inner content (removing FENDs)
        inner = frame[1:-1]
        
        if len(inner) < 1:
            return None
            
        command_byte = inner[0]
        # The rest is the escaped payload
        payload = cls.unescape(inner[1:])
        
        return command_byte, payload
