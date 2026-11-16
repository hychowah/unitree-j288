"""Unitree J288 / S288 serial frames.

Command: 20 bytes, header FE EE included in the CRC.
Reply: 26 bytes, header FC EE excluded from the CRC.
6 Mbps, 8N1, half duplex. Send one command, then read one reply.
"""

import math
import struct

RATIO = 70070.0 / 243.0
M_PI = math.pi
BAUD = 6000000
# Output-end limits from the Unitree parameter table.
TOR_MIN = -36.9093
TOR_LIM = 36.908174
SPD_LIM = 278.90143
POS_LIM = 1428.018898
KP_LIM = 2128.523254
KD_LIM = 21.285233

# Polynomial 0x04C11DB7, init 0xFFFFFFFF. Each word is fed little-endian.
CRC32_TABLE = [
    0x00000000, 0x04C11DB7, 0x09823B6E, 0x0D4326D9, 0x130476DC, 0x17C56B6B, 0x1A864DB2, 0x1E475005,
    0x2608EDB8, 0x22C9F00F, 0x2F8AD6D6, 0x2B4BCB61, 0x350C9B64, 0x31CD86D3, 0x3C8EA00A, 0x384FBDBD,
    0x4C11DB70, 0x48D0C6C7, 0x4593E01E, 0x4152FDA9, 0x5F15ADAC, 0x5BD4B01B, 0x569796C2, 0x52568B75,
    0x6A1936C8, 0x6ED82B7F, 0x639B0DA6, 0x675A1011, 0x791D4014, 0x7DDC5DA3, 0x709F7B7A, 0x745E66CD,
    0x9823B6E0, 0x9CE2AB57, 0x91A18D8E, 0x95609039, 0x8B27C03C, 0x8FE6DD8B, 0x82A5FB52, 0x8664E6E5,
    0xBE2B5B58, 0xBAEA46EF, 0xB7A96036, 0xB3687D81, 0xAD2F2D84, 0xA9EE3033, 0xA4AD16EA, 0xA06C0B5D,
    0xD4326D90, 0xD0F37027, 0xDDB056FE, 0xD9714B49, 0xC7361B4C, 0xC3F706FB, 0xCEB42022, 0xCA753D95,
    0xF23A8028, 0xF6FB9D9F, 0xFBB8BB46, 0xFF79A6F1, 0xE13EF6F4, 0xE5FFEB43, 0xE8BCCD9A, 0xEC7DD02D,
    0x34867077, 0x30476DC0, 0x3D044B19, 0x39C556AE, 0x278206AB, 0x23431B1C, 0x2E003DC5, 0x2AC12072,
    0x128E9DCF, 0x164F8078, 0x1B0CA6A1, 0x1FCDBB16, 0x018AEB13, 0x054BF6A4, 0x0808D07D, 0x0CC9CDCA,
    0x7897AB07, 0x7C56B6B0, 0x71159069, 0x75D48DDE, 0x6B93DDDB, 0x6F52C06C, 0x6211E6B5, 0x66D0FB02,
    0x5E9F46BF, 0x5A5E5B08, 0x571D7DD1, 0x53DC6066, 0x4D9B3063, 0x495A2DD4, 0x44190B0D, 0x40D816BA,
    0xACA5C697, 0xA864DB20, 0xA527FDF9, 0xA1E6E04E, 0xBFA1B04B, 0xBB60ADFC, 0xB6238B25, 0xB2E29692,
    0x8AAD2B2F, 0x8E6C3698, 0x832F1041, 0x87EE0DF6, 0x99A95DF3, 0x9D684044, 0x902B669D, 0x94EA7B2A,
    0xE0B41DE7, 0xE4750050, 0xE9362689, 0xEDF73B3E, 0xF3B06B3B, 0xF771768C, 0xFA325055, 0xFEF34DE2,
    0xC6BCF05F, 0xC27DEDE8, 0xCF3ECB31, 0xCBFFD686, 0xD5B88683, 0xD1799B34, 0xDC3ABDED, 0xD8FBA05A,
    0x690CE0EE, 0x6DCDFD59, 0x608EDB80, 0x644FC637, 0x7A089632, 0x7EC98B85, 0x738AAD5C, 0x774BB0EB,
    0x4F040D56, 0x4BC510E1, 0x46863638, 0x42472B8F, 0x5C007B8A, 0x58C1663D, 0x558240E4, 0x51435D53,
    0x251D3B9E, 0x21DC2629, 0x2C9F00F0, 0x285E1D47, 0x36194D42, 0x32D850F5, 0x3F9B762C, 0x3B5A6B9B,
    0x0315D626, 0x07D4CB91, 0x0A97ED48, 0x0E56F0FF, 0x1011A0FA, 0x14D0BD4D, 0x19939B94, 0x1D528623,
    0xF12F560E, 0xF5EE4BB9, 0xF8AD6D60, 0xFC6C70D7, 0xE22B20D2, 0xE6EA3D65, 0xEBA91BBC, 0xEF68060B,
    0xD727BBB6, 0xD3E6A601, 0xDEA580D8, 0xDA649D6F, 0xC423CD6A, 0xC0E2D0DD, 0xCDA1F604, 0xC960EBB3,
    0xBD3E8D7E, 0xB9FF90C9, 0xB4BCB610, 0xB07DABA7, 0xAE3AFBA2, 0xAAFBE615, 0xA7B8C0CC, 0xA379DD7B,
    0x9B3660C6, 0x9FF77D71, 0x92B45BA8, 0x9675461F, 0x8832161A, 0x8CF30BAD, 0x81B02D74, 0x857130C3,
    0x5D8A9099, 0x594B8D2E, 0x5408ABF7, 0x50C9B640, 0x4E8EE645, 0x4A4FFBF2, 0x470CDD2B, 0x43CDC09C,
    0x7B827D21, 0x7F436096, 0x7200464F, 0x76C15BF8, 0x68860BFD, 0x6C47164A, 0x61043093, 0x65C52D24,
    0x119B4BE9, 0x155A565E, 0x18197087, 0x1CD86D30, 0x029F3D35, 0x065E2082, 0x0B1D065B, 0x0FDC1BEC,
    0x3793A651, 0x3352BBE6, 0x3E119D3F, 0x3AD08088, 0x2497D08D, 0x2056CD3A, 0x2D15EBE3, 0x29D4F654,
    0xC5A92679, 0xC1683BCE, 0xCC2B1D17, 0xC8EA00A0, 0xD6AD50A5, 0xD26C4D12, 0xDF2F6BCB, 0xDBEE767C,
    0xE3A1CBC1, 0xE760D676, 0xEA23F0AF, 0xEEE2ED18, 0xF0A5BD1D, 0xF464A0AA, 0xF9278673, 0xFDE69BC4,
    0x89B8FD09, 0x8D79E0BE, 0x803AC667, 0x84FBDBD0, 0x9ABC8BD5, 0x9E7D9662, 0x933EB0BB, 0x97FFAD0C,
    0xAFB010B1, 0xAB710D06, 0xA6322BDF, 0xA2F33668, 0xBCB4666D, 0xB8757BDA, 0xB5365D03, 0xB1F740B4
]

def crc32_lookup(data: bytes) -> int:
    crc = 0xFFFFFFFF
    n = len(data)
    i = 0
    while i + 3 < n:
        b0 = data[i + 3]
        b1 = data[i + 2]
        b2 = data[i + 1]
        b3 = data[i]
        crc = CRC32_TABLE[(crc >> 24) ^ b0] ^ ((crc << 8) & 0xFFFFFFFF)
        crc = CRC32_TABLE[(crc >> 24) ^ b1] ^ ((crc << 8) & 0xFFFFFFFF)
        crc = CRC32_TABLE[(crc >> 24) ^ b2] ^ ((crc << 8) & 0xFFFFFFFF)
        crc = CRC32_TABLE[(crc >> 24) ^ b3] ^ ((crc << 8) & 0xFFFFFFFF)
        i += 4
    return crc


# Motor Error Table. Bits that are not listed need a power cycle or reset.
ERROR_BITS = {
    0: "overcurrent",
    1: "transient overvoltage",
    2: "continuous overvoltage",
    3: "transient undervoltage",
    4: "chip overheating",
    5: "MOS overheating",
    6: "MOS temperature abnormal",
    8: "housing temperature abnormal",
    9: "winding overheating",
    10: "rotor encoder 1",
    12: "output encoder",
    13: "stored data",
    14: "abnormal reset",
    16: "chip verification",
    17: "calibration mode",
    19: "driver version",
    20: "firmware model",
    22: "hardware overcurrent",
}
# Clear-command column of that table. The others ignore status 6.
CLEARABLE_BITS = {1, 2, 3, 4, 5, 6, 8, 9, 12, 14}
WARN_BITS = {0: "low-voltage supply"}

def clamp(value, lo, hi):
    return max(lo, min(hi, value))


def clamp_pos(value):
    return clamp(float(value), -POS_LIM, POS_LIM)


def _i16(value):
    return max(-32768, min(32767, int(value)))


def _i32(value):
    return max(-2147483648, min(2147483647, int(value)))


def error_names(code):
    code = int(code) & 0xFFFFFFFF
    names = [ERROR_BITS[bit] for bit in sorted(ERROR_BITS) if code & (1 << bit)]
    known = 0
    for bit in ERROR_BITS:
        known |= 1 << bit
    extra = code & ~known
    if extra:
        names.append("0x%X" % extra)
    return names


def warn_names(code):
    code = int(code) & 0x7
    names = [WARN_BITS[bit] for bit in sorted(WARN_BITS) if code & (1 << bit)]
    known = 0
    for bit in WARN_BITS:
        known |= 1 << bit
    extra = code & ~known
    if extra:
        names.append("0x%X" % extra)
    return names


def _scale(tor, spd, pos, kp, kd):
    """Output-end floats to the rotor counts in ControlData_t."""
    kp = clamp(float(kp), 0.0, KP_LIM)
    kd = clamp(float(kd), 0.0, KD_LIM)
    tor = clamp(float(tor), TOR_MIN, TOR_LIM)
    spd = clamp(float(spd), -SPD_LIM, SPD_LIM)
    pos = clamp(float(pos), -POS_LIM, POS_LIM)
    k_pos = _i16(round(kp / (RATIO * RATIO) * 1280000.0))
    k_spd = _i16(round(kd / (RATIO * RATIO) * 128000000.0))
    pos_des = _i32(round(pos * RATIO * 32768.0 / M_PI / 2.0))
    spd_des = _i16(round(spd * RATIO * 2.560 / M_PI / 2.0))
    tor_des = _i16(round(tor / RATIO * 256000.0))
    return tor_des, spd_des, pos_des, k_pos, k_spd


def build_packet(motor_id, mode, timeout, tor, spd, pos, kp, kd, raw=False):
    """20-byte control frame. head 0xFE 0xEE is inside the CRC.

    raw=True sends the integer fields as they are. The clear command is
    status 6 and tor_des -256, which is not an output-end torque.
    """
    motor_id = max(0, min(15, int(motor_id)))
    mode = max(0, min(7, int(mode)))
    timeout = 1 if timeout else 0
    if raw:
        tor_des, spd_des, pos_des = _i16(tor), _i16(spd), _i32(pos)
        k_pos, k_spd = _i16(kp), _i16(kd)
    else:
        tor_des, spd_des, pos_des, k_pos, k_spd = _scale(tor, spd, pos, kp, kd)
    mode_byte = (motor_id & 0x0F) | ((mode & 0x07) << 4) | ((timeout & 0x01) << 7)
    # `I` rejects a negative position. The wire wants the same bits.
    body = bytes([0xFE, 0xEE, mode_byte, 0x00]) + struct.pack(
        "<hhIhh", tor_des, spd_des, pos_des & 0xFFFFFFFF, k_pos, k_spd
    )
    crc = crc32_lookup(body)
    packet = body + struct.pack("<I", crc)
    if len(packet) != 20:
        raise RuntimeError("control frame is %d bytes" % len(packet))
    return packet


def parse_feedback(data):
    """26-byte reply. head 0xFC 0xEE is not inside the CRC.

    sensor and vol are unsigned. The demo unpacks them as signed bytes,
    which wraps a winding temperature or a bus above 63.5 V.
    """
    if data is None or len(data) < 26:
        return None
    data = bytes(data[:26])
    if data[0] != 0xFC or data[1] != 0xEE:
        return None
    crc_rx = struct.unpack_from("<I", data, 22)[0]
    if crc_rx != crc32_lookup(data[2:22]):
        return None
    mode_byte = data[2]
    temp, sensor, vol = struct.unpack_from("<bBB", data, 3)
    torque, speed, pos, merror, mixed = struct.unpack_from("<hhiIH", data, 6)
    out_pos = mixed & 0x1FFF
    ex_flag = (mixed >> 13) & 0x07
    return {
        "motor_id": mode_byte & 0x0F,
        "mode": (mode_byte >> 4) & 0x07,
        "timeout": (mode_byte >> 7) & 0x01,
        "temp": int(temp),
        "sensor": int(sensor),
        "vol": vol / 2.0,
        "merror": int(merror),
        "mwarn": int(ex_flag),
        "ex_pos": 2 * M_PI * out_pos / 8192.0,
        "output_tor": (torque / 256000.0) * RATIO,
        "output_spd": (speed / 2.56) * 2 * M_PI / RATIO,
        "output_pos": 2 * M_PI * pos / 32768.0 / RATIO,
    }

