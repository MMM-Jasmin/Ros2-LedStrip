# SPDX-FileCopyrightText: 2021 ladyada for Adafruit Industries
# SPDX-License-Identifier: MIT

import time
import board
import neopixel_spi as neopixel

NUM_PIXELS = 38
PIXEL_ORDER = neopixel.GRB
COLORS = (0xFF0000, 0x00FF00, 0x0000FF)
DELAY = 0.05

spi = board.SPI()

#pixels = neopixel.NeoPixel_SPI(spi, NUM_PIXELS, pixel_order=PIXEL_ORDER, auto_write=False)
pixels = neopixel.NeoPixel_SPI(spi, NUM_PIXELS, auto_write=False)

while True:
  for color in COLORS:
    for i in range(NUM_PIXELS):
      pixels[i] = color
      pixels.show()
      time.sleep(DELAY)
      pixels.fill(0)
  print("next color")

#while True:
#  pixels.fill(0xFFDEAD)
#  pixels.show()
#  time.sleep(0.1)

#while True:
#  for in in range(NUM_PIXELS):
#    pixels[i] = 0xFF0000
#    pixels.show()
#    time.sleep(1)

