#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
"""sasi.asm を組み立て、4 KB の全バイトの和（下位 8 ビット）が 00h になるように最後のバイトを合わせる（仕様書 1.2）。

使い方: python3 sasibios/mkrom.py [出力のファイル名]（省略すると sasibios/sasi.rom）
"""

import os
import subprocess
import sys
import tempfile

HERE = os.path.dirname(os.path.abspath(__file__))


def build(out=None):
    with tempfile.TemporaryDirectory() as tmp:
        raw = os.path.join(tmp, "sasi.bin")
        subprocess.run(["nasm", "-f", "bin", "-w+all", "-o", raw, os.path.join(HERE, "sasi.asm")], check=True)
        with open(raw, "rb") as f:
            rom = bytearray(f.read())
    if len(rom) != 0x1000:
        raise SystemExit(f"大きさが 4 KB でない: {len(rom)} バイト")
    rom[0xFFF] = (-sum(rom[:0xFFF])) & 0xFF
    assert sum(rom) & 0xFF == 0
    if out:
        with open(out, "wb") as f:
            f.write(rom)
    return bytes(rom)


if __name__ == "__main__":
    out = sys.argv[1] if len(sys.argv) > 1 else os.path.join(HERE, "sasi.rom")
    rom = build(out)
    used = 0xFFE - len(rom[:0xFFE].rstrip(b"\xff"))
    print(f"{out}: 4096 バイト（空き {used} バイト）")
