# 用抓包中的多个比赛状态帧样本交叉验证 CRC 算法
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


# 从抓包提取的 4 帧比赛状态（cmd_id=0x0001，18 字节）
frames = [
    bytes([0xa5, 0x0b, 0x00, 0xfb, 0x56, 0x01, 0x00, 0x00, 0x00, 0x00,
           0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0d, 0x53]),
    bytes([0xa5, 0x0b, 0x00, 0xfe, 0x69, 0x01, 0x00, 0x00, 0x00, 0x00,
           0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x12, 0xdf]),
    bytes([0xa5, 0x0b, 0x00, 0x01, 0x5c, 0x01, 0x00, 0x00, 0x00, 0x00,
           0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x66, 0xcb]),
    bytes([0xa5, 0x0b, 0x00, 0x07, 0x81, 0x01, 0x00, 0x00, 0x00, 0x00,
           0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xdd, 0x02]),
]

print("=== crc8（覆盖 0-3）===")
for f in frames:
    got = crc8(f[:4])
    print("  seq={:#04x} 计算={:#04x} 帧内={:#04x} {}".format(
        f[3], got, f[4], "PASS" if got == f[4] else "FAIL"))

print("\n=== crc16 暴力搜索（要求 4 帧全部匹配）===")
targets = [f[-2] | (f[-1] << 8) for f in frames]
print("  目标: " + ", ".join("{:#06x}".format(t) for t in targets))
found = False
for start in range(0, 7):
    for end in range(start + 2, 18):
        for init in [0x0000, 0xFFFF, 0x1D0F, 0xFFFF0]:
            for poly in [0x8408, 0x1021, 0xA001, 0x8005, 0x0589, 0x3D65, 0x8C, 0x8D95]:
                ok = all(crc16(f[start:end], poly, init) == t
                         for f, t in zip(frames, targets))
                if ok:
                    print("  MATCH: start={} end={} init={:#06x} poly={:#06x}".format(
                        start, end, init, poly))
                    found = True
if not found:
    print("  无匹配")
    # 打印最可能组合（覆盖 0-16）的数值
    for f, t in zip(frames, targets):
        print("  覆盖0-16: 计算={:#06x} 帧内={:#06x} 覆盖0-15: {:#06x}".format(
            crc16(f[:16]), t, crc16(f[:15])))
