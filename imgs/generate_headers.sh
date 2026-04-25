#!/bin/bash
# Converts rear.png and side.png to RGB565 C headers for LovyanGFX sprites.
# Usage: ./generate_headers.sh
# Output: ../img_rear.h and ../img_side.h

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
OUT_DIR="$SCRIPT_DIR/.."

python3 -c "
import struct, sys, os
from PIL import Image

imgs = [('rear', 'REAR'), ('side', 'SIDE')]

for name, upper in imgs:
    src = os.path.join('$SCRIPT_DIR', f'{name}.png')
    if not os.path.exists(src):
        print(f'WARNING: {src} not found, skipping')
        continue

    img = Image.open(src).convert('RGB')
    w, h = img.size

    pixels = []
    for y in range(h):
        for x in range(w):
            r, g, b = img.getpixel((x, y))
            rgb565 = ((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3)
            pixels.append(rgb565)

    out = os.path.join('$OUT_DIR', f'img_{name}.h')
    with open(out, 'w') as f:
        f.write(f'// Auto-generated from imgs/{name}.png ({w}x{h})\n')
        f.write(f'// Run imgs/generate_headers.sh to regenerate\n')
        f.write(f'#pragma once\n')
        f.write(f'#include <pgmspace.h>\n')
        f.write(f'#define IMG_{upper}_W {w}\n')
        f.write(f'#define IMG_{upper}_H {h}\n')
        f.write(f'const uint16_t img_{name}[{w*h}] PROGMEM = {{\n')
        for i, val in enumerate(pixels):
            f.write(f'0x{val:04X},')
            if (i + 1) % 16 == 0: f.write('\n')
        f.write(f'\n}};\n')
    print(f'{name}.png ({w}x{h}) -> {out} ({w*h*2} bytes)')
"

echo "Done."
