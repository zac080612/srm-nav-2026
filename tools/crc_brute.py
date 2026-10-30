frame = bytes([0xa5, 0x0b, 0x00, 0x86, 0x53, 0x01, 0x00, 0x00, 0x00,
               0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x30, 0x9b])
target = 0x9B30


def crc16(data, poly, init):
    crc = init
    for b in data:
        crc ^= b
        for _ in range(8):
            crc = (crc >> 1) ^ (poly if crc & 1 else 0)
    return crc & 0xFFFF


found = []
for start in range(0, 6):
    for end in range(start + 2, 18):
        for init in [0x0000, 0xFFFF]:
            for poly in [0x8408, 0x1021, 0xA001, 0x8005, 0x0589]:
                if crc16(frame[start:end], poly, init) == target:
                    found.append((start, end, init, poly))

if found:
    for f in found:
        print("CRC16 MATCH: start={} end={} init={:#06x} poly={:#06x}".format(*f))
else:
    print("CRC16 brute force: no match")
    for start, end in [(0, 16), (5, 16), (7, 16), (1, 16), (0, 17)]:
        for init in [0x0000, 0xFFFF]:
            v = crc16(frame[start:end], 0x8408, init)
            print("  crc16[{}:{}] init={:#06x} = {:#06x} (target {:#06x})".format(
                start, end, init, v, target))
