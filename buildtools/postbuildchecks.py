from pathlib import Path

import intelhex
def check_hex_overlap():
    f1 = str(Path(r'C:\Users\dan\Documents\OpenRoverFirmware-dan\Power_Board\bin\PowerBoard.hex'))
    f2 = str(Path(r'C:\Users\dan\Documents\OpenRoverFirmware-dan\bootypic\bin\bootypic.hex'))

    h1 = intelhex.IntelHex16bit(f1)
    h2 = intelhex.IntelHex16bit(f2)

    print(f'hex1 segments: {h1.segments()}')
    print(f'hex2 segments: {h2.segments()}')

    overlap_segments = []
    for i in h1.segments():
        for j in h2.segments():
            lo = max(i[0], j[0])
            hi = min(i[1], j[1])
            if lo < hi:
                overlap_segments.append((lo, hi))
    print(f'overlapping segments: {overlap_segments}')

    for lo, hi in overlap_segments:
        for i in range(lo, hi):
            if (h1[i] != h2[i]):
                print(f'conflict at {i:06x}: {h1[i]:04x} vs {h2[i]:04x}')


if __name__ == '__main__':
    check_hex_overlap()


