#!/usr/bin/env python3
"""Check every UBX frame list in src/main/io/gps.cpp ( gps-spi-i2c task 4 ).

Run from the repo root:  python3 docs/fw-development-reference/active-development/gps-spi-i2c/tests/ubx_frames_check.py

For each `static const uint8_t <name> [] = { ... };` array ( and each SBAS message ), the bytes
must split exactly into UBX frames: B5 62, class, id, little-endian payload length, payload,
and a correct 8-bit Fletcher checksum. Prints one line per frame and exits 1 on any error.
"""
import re
import sys

import os

# repo root is five levels up from this file
SRC = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..", "..", "..", "..", "src", "main", "io", "gps.cpp")

def frames_of(data):
    pos, out = 0, []
    while pos < len(data):
        if data[pos:pos + 2] != [0xB5, 0x62]:
            raise ValueError(f"no sync at byte {pos}")
        if pos + 6 > len(data):
            raise ValueError(f"truncated header at byte {pos}")
        length = data[pos + 4] | (data[pos + 5] << 8)
        end = pos + 6 + length + 2
        if end > len(data):
            raise ValueError(f"frame at byte {pos} runs past the list ( length {length} )")
        a = b = 0
        for x in data[pos + 2:pos + 6 + length]:
            a = (a + x) & 0xFF
            b = (b + a) & 0xFF
        if [a, b] != data[end - 2:end]:
            raise ValueError(f"bad checksum at byte {pos}: have {data[end-2]:02X} {data[end-1]:02X}, want {a:02X} {b:02X}")
        out.append((data[pos + 2], data[pos + 3], length))
        pos = end
    return out

def strip_comments(text):
    text = re.sub(r"/\*.*?\*/", "", text, flags=re.S)
    return re.sub(r"//[^\n]*", "", text)

def main():
    src = open(SRC).read()
    lists = []
    for m in re.finditer(r"static const uint8_t (\w+) \[\] = \{(.*?)\};", src, flags=re.S):
        lists.append((m.group(1), m.group(2)))
    for i, m in enumerate(re.finditer(r"\{ SBAS_\w+, \{(.*?)\} \}", src)):
        lists.append((f"ubloxSbas[{i}]", m.group(1)))

    errors = 0
    for name, body in lists:
        data = [int(x, 16) for x in re.findall(r"0x[0-9A-Fa-f]{2}", strip_comments(body))]
        if not data or data[:2] != [0xB5, 0x62]:
            continue    # not a UBX list
        try:
            fr = frames_of(data)
            print(f"OK   {name}: {len(data)} B, {len(fr)} frames: " + ", ".join(f"{c:02X}-{i:02X}({l})" for c, i, l in fr))
        except ValueError as e:
            errors += 1
            print(f"FAIL {name}: {e}")
    print("all frames valid" if not errors else f"{errors} list(s) failed")
    return 1 if errors else 0

if __name__ == "__main__":
    sys.exit(main())
