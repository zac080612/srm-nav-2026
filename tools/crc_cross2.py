# 重新按协议文档长度（比赛状态 20 字节）验证 CRC
def crc8(d, poly=0x8C, init=0xFF):
    crc = init
    for b in d:
        crc ^= b
        for _ in range(8):
            crc = (crc >> 1) ^ (poly if crc & 1 else 0)
    return crc & 0xFF


def crc16(d, poly=0x8408, init=0xFFFF):
    crc = init
    for b in d:
        crc ^= b
        for _ in range(8):
            crc = (crc >> 1) ^ (poly if crc & 1 else 0)
    return crc & 0xFFFF


# 按文档：比赛状态帧 20 字节 = 5头 + 2cmd + 11数据 + 2crc
frames = [
    bytes([0xa5, 0x0b, 0x00, 0xfb, 0x56, 0x01, 0x00, 0x00, 0x00, 0x00,
           0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0d, 0x53]),
    bytes([0xa5, 0x0b, 0x00, 0xfe, 0x69, 0x01, 0x00, 0x00, 0x00, 0x00,
           0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x12, 0xdf]),
    bytes([0xa5, 0x0b, 0x00, 0x01, 0x5c, 0x01, 0x00, 0x00, 0x00, 0x00,
           0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x66, 0xcb]),
    bytes([0xa5, 0x0b, 0x00, 0x07, 0x81, 0x01, 0x00, 0x00, 0x00, 0x00,
           0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xdd, 0x02]),
]

print("=== 比赛状态帧（20字节，文档格式）crc16 覆盖 0-17 ===")
for f in frames:
    got = crc16(f[:18])
    tgt = f[18] | (f[19] << 8)
    print("  seq={:#04x} crc8={} 计算crc16={:#06x} 帧内={:#06x} {}".format(
        f[3], "PASS" if crc8(f[:4]) == f[4] else "FAIL", got, tgt,
        "PASS" if got == tgt else "FAIL"))

# 如果还不过，暴力试覆盖范围
print("\n=== 若不过：暴力搜索 ===")
targets = [f[18] | (f[19] << 8) for f in frames]
for start in range(0, 7):
    for end in range(start + 2, 20):
        for init in [0x0000, 0xFFFF]:
            for poly in [0x8408, 0x1021, 0xA001]:
                if all(crc16(f[start:end], poly, init) == t
                       for f, t in zip(frames, targets)):
                    print("MATCH: start={} end={} init={:#06x} poly={:#06x}".format(
                        start, end, init, poly))
