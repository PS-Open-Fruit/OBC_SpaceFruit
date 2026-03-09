import struct

# ==========================================
# CONFIGURATION: Update these with your C macros
# ==========================================
EPS_NUM_VI_CHANNEL = 8      # Replace with actual macro value
EPS_NUM_OUTPUT_CHANNEL = 6  # Replace with actual macro value
EPS_NUM_TEMP_BATT = 2       # Replace with actual macro value

# Scale factor for pack_float_fixed (e.g., if C code does temp * 100)
TEMP_SCALE_FACTOR = 100.0   

# Struct packing based on eps_pack_sensor_data
# '>' = Big-Endian (from pack_i16_be)
# 'h' = int16_t (2 bytes)
# 'B' = uint8_t (1 byte)
# 'i' = int32_t (Assuming pack_float_fixed uses 4 bytes. If it uses 2 bytes, change to 'h')
FMT_VI_SENSOR = '>hhBB' 
FMT_OUTPUT_SENSOR = '>hhBB'
FMT_OUTPUT_STATE = '>BBB'
FMT_BATTERY_TEMP = '>hBB'

# Calculate offsets
EPS_BEACON_LEN = (
    (struct.calcsize(FMT_VI_SENSOR) * EPS_NUM_VI_CHANNEL) +
    (struct.calcsize(FMT_OUTPUT_SENSOR) * EPS_NUM_OUTPUT_CHANNEL) +
    (struct.calcsize(FMT_OUTPUT_STATE) * EPS_NUM_OUTPUT_CHANNEL) +
    (struct.calcsize(FMT_BATTERY_TEMP) * EPS_NUM_TEMP_BATT)
)

PACKET_SIZE = EPS_BEACON_LEN + 7 + 4  # EPS + RTC(7) + TMP1075(4)

# ==========================================
# HELPER FUNCTIONS
# ==========================================
def bcd_to_dec(bcd_val):
    return (bcd_val & 0x0F) + ((bcd_val >> 4) * 10)

def decode_beacon_packet(buffer):
    print(EPS_BEACON_LEN)
    if len(buffer) < PACKET_SIZE:
        print(f"Buffer too short! Expected {PACKET_SIZE}, got {len(buffer)}")
        return None

    data = {
        "eps": {
            "vi_sensors": [],
            "output_sensors": [],
            "output_states": [],
            "battery_temps": []
        },
        "rtc": {},
        "tmp1075": {}
    }

    offset = 0

    # 1. Decode EPS VI Sensors
    for _ in range(EPS_NUM_VI_CHANNEL):
        v, i, ch, state = struct.unpack_from(FMT_VI_SENSOR, buffer, offset)
        data["eps"]["vi_sensors"].append({"voltage": v, "current": i, "channel": ch, "data_state": state})
        offset += struct.calcsize(FMT_VI_SENSOR)

    # 2. Decode EPS Output Sensors
    for _ in range(EPS_NUM_OUTPUT_CHANNEL):
        v, i, ch, state = struct.unpack_from(FMT_OUTPUT_SENSOR, buffer, offset)
        data["eps"]["output_sensors"].append({"voltage": v, "current": i, "channel": ch, "data_state": state})
        offset += struct.calcsize(FMT_OUTPUT_SENSOR)

    # 3. Decode EPS Output States
    for _ in range(EPS_NUM_OUTPUT_CHANNEL):
        status, ch, state = struct.unpack_from(FMT_OUTPUT_STATE, buffer, offset)
        data["eps"]["output_states"].append({"status": status, "channel": ch, "data_state": state})
        offset += struct.calcsize(FMT_OUTPUT_STATE)

    # 4. Decode EPS Battery Temps (handling pack_float_fixed)
    for _ in range(EPS_NUM_TEMP_BATT):
        t_raw, ch, state = struct.unpack_from(FMT_BATTERY_TEMP, buffer, offset)
        t_float = t_raw / TEMP_SCALE_FACTOR
        data["eps"]["battery_temps"].append({"temperature": t_float, "channel": ch, "data_state": state})
        offset += struct.calcsize(FMT_BATTERY_TEMP)

    # 5. Decode RTC Data (7 Bytes)
    rtc_buf = buffer[offset:offset+7]
    data["rtc"] = {
        "sec": bcd_to_dec(rtc_buf[0] & 0x7F),
        "min": bcd_to_dec(rtc_buf[1] & 0x7F),
        "hour": bcd_to_dec(rtc_buf[2] & 0x3F),
        "wday": rtc_buf[3], 
        "day": bcd_to_dec(rtc_buf[4] & 0x3F),
        "month": bcd_to_dec(rtc_buf[5] & 0x1F),
        "year": bcd_to_dec(rtc_buf[6])
    }
    offset += 7

    # 6. Decode TMP1075 Temperature (4 Bytes, big-endian)
    temp_buf = buffer[offset:offset+4]
    data["tmp1075"]["raw_temp"] = struct.unpack('>i', temp_buf)[0]
    
    return data

def print_decoded_beacon_data(decoded_data):
    if not decoded_data:
        return
        
    print("\n" + "="*40)
    print("--- EPS SENSOR DATA ---")
    for idx, vi in enumerate(decoded_data["eps"]["vi_sensors"]):
        print(f"VI Sensor {idx}   | V: {vi['voltage']} mV, I: {vi['current']} mA, Ch: {vi['channel']}, State: {vi['data_state']}")
    
    for idx, out in enumerate(decoded_data["eps"]["output_sensors"]):
        print(f"Out Sensor {idx}  | V: {out['voltage']} mV, I: {out['current']} mA, Ch: {out['channel']}, State: {out['data_state']}")

    for idx, out_state in enumerate(decoded_data["eps"]["output_states"]):
        print(f"Out State {idx}   | Status: {out_state['status']}, Ch: {out_state['channel']}, State: {out_state['data_state']}")

    for idx, temp in enumerate(decoded_data["eps"]["battery_temps"]):
        print(f"Batt Temp {idx}   | Temp: {temp['temperature']:.2f} °C, Ch: {temp['channel']}, State: {temp['data_state']}")

    print("\n--- RTC DATETIME ---")
    r = decoded_data["rtc"]
    print(f"20{r['year']:02d}-{r['month']:02d}-{r['day']:02d} {r['hour']:02d}:{r['min']:02d}:{r['sec']:02d} (WDay: {r['wday']})")

    print("\n--- TMP1075 SENSOR ---")
    # You might want to apply the TMP1075 conversion math here later
    print(f"Raw Temp Value: {decoded_data['tmp1075']['raw_temp']}")
    print("="*40 + "\n")