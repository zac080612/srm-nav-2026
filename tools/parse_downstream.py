# 分析下位机抓包：按 cmd_id 统计帧结构、crc8/crc16 通过率
import sys


def crc8(d, init=0xFF):
    crc = init
    for b in d:
        crc ^= b
        for _ in range(8):
            crc = (crc >> 1) ^ (0x8C if crc & 1 else 0)
    return crc & 0xFF


def crc16(d, init=0xFFFF):
    crc = init
    for b in d:
        crc ^= b
        for _ in range(8):
            crc = (crc >> 1) ^ (0x8408 if crc & 1 else 0)
    return crc & 0xFFFF


data = open(sys.argv[1], 'rb').read()
n = len(data)
i = 0
stats = {}
samples = {}
while i < n:
    if data[i] != 0xA5:
        i += 1
        continue
    if i + 5 > n:
        break
    ln = data[i + 1] | (data[i + 2] << 8)
    total = 9 + ln
    if i + total > n:
        break
    f = data[i:i + total]
    cmd = f[5] | (f[6] << 8)
    c8_ok = crc8(f[:4]) == f[4]
    c16_ok = crc16(f[:7 + ln]) == (f[total - 2] | (f[total - 1] << 8))
    key = (cmd, ln)
    s = stats.setdefault(key, [0, 0, 0])
    s[0] += 1
    s[1] += c8_ok
    s[2] += c16_ok
    samples.setdefault(key, []).append(f)
    i += total

print("=== 帧结构统计 ===")
for (cmd, ln), (cnt, c8, c16) in sorted(stats.items()):
    print("cmd={:#06x} 字段len={} 总长={} 帧数={:4d}  crc8通过={:4d}  crc16通过={:4d}".format(
        cmd, ln, 9 + ln, cnt, c8, c16))

# 对 crc16 不通过的帧类型，用多个样本暴力试参数
for (cmd, ln), s in sorted(stats.items()):
    if s[2] == s[0]:
        continue
    fs = samples[(cmd, ln)][:3]
    total = 9 + ln
    targets = [(f[total - 2] | (f[total - 1] << 8)) for f in fs]
    print("\n=== cmd={:#06x} crc16 不通过，暴力搜索 ===".format(cmd))
    print("目标 crc16: " + ", ".join("{:#06x}".format(t) for t in targets))
    for start in range(0, 6):
        for end in range(start + 2, total):
            for init in [0x0000, 0xFFFF, 0x1D0F]:
                for poly in [0x8408, 0x1021, 0xA001, 0x8005, 0x0589, 0x3D65]:
                    ok = all(crc16(f[start:end], init) == t
                             for f, t in zip(fs, targets))
                    if ok:
                        print("MATCH: start={} end={} init={:#06x} poly={:#06x}".format(
                            start, end, init, poly))
