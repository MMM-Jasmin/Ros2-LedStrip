# SPDX-FileCopyrightText: 2021 ladyada for Adafruit Industries
# SPDX-License-Identifier: MIT

# pip install numpy adafruit-circuitpython-neopixel-spi

import time
import numpy
import board
import neopixel_spi as neopixel

NUM_PIXELS = 38
PIXEL_ORDER = neopixel.GRB
COLORS = (0xFF0000, 0x00FF00, 0x0000FF)
DELAY = 0.05

spi = board.SPI()

#pixels = neopixel.NeoPixel_SPI(spi, NUM_PIXELS, pixel_order=PIXEL_ORDER, auto_write=False)
pixels = neopixel.NeoPixel_SPI(spi, NUM_PIXELS, auto_write=False)

# while True:
#   for color in COLORS:
#     for i in range(NUM_PIXELS):
#       pixels[i] = color
#       pixels.show()
#       time.sleep(DELAY)
#       pixels.fill(0)
#   print("next color")

#while True:
#  pixels.fill(0xFFDEAD)
#  pixels.show()
#  time.sleep(0.1)

#while True:
#  for in in range(NUM_PIXELS):
#    pixels[i] = 0xFF0000
#    pixels.show()
#    time.sleep(1)

NUM_LEDS_HORIZ = 7
NUM_LEDS_VERT = 12

def map_horiz_led(width_frac):
    return int((NUM_LEDS_HORIZ - 1) * width_frac)

def map_vert_led(width_frac):
    return int((NUM_LEDS_VERT - 1) * width_frac)

def map_edge_bottom_to_led(width_frac):
    return map_horiz_led(width_frac)

def map_edge_right_to_led(width_frac):
    return NUM_LEDS_HORIZ + map_vert_led(1 - width_frac)

def map_edge_top_to_led(width_frac):
    return NUM_LEDS_HORIZ + NUM_LEDS_VERT + map_horiz_led(1 - width_frac)

def map_edge_left_to_led(width_frac):
    return NUM_LEDS_HORIZ + NUM_LEDS_VERT + NUM_LEDS_HORIZ + map_vert_led(width_frac)

def map_width_to_leds(width_frac):
    led_top_idx = map_edge_top_to_led(width_frac)
    led_bottom_idx = map_edge_bottom_to_led(width_frac)
    return (led_bottom_idx, led_top_idx)

def map_height_to_leds(width_frac):
    led_right_idx = map_edge_right_to_led(width_frac)
    led_left_idx = map_edge_left_to_led(width_frac)
    return (led_right_idx, led_left_idx)

def map_width_range_to_leds(width_frac_start, width_frac_end):
    (led_bottom_idx_start, led_top_idx_start) = map_width_to_leds(width_frac_start)
    (led_bottom_idx_end, led_top_idx_end) = map_width_to_leds(width_frac_end)
    led_bottom_idx_min = min(led_bottom_idx_start, led_bottom_idx_end)
    led_bottom_idx_max = max(led_bottom_idx_start, led_bottom_idx_end)
    led_top_idx_min = min(led_top_idx_start, led_top_idx_end)
    led_top_idx_max = max(led_top_idx_start, led_top_idx_end)
    return [*range(led_bottom_idx_min, led_bottom_idx_max + 1)] + [*range(led_top_idx_min, led_top_idx_max + 1)]

def map_height_range_to_leds(height_frac_start, height_frac_end):
    (led_right_idx_start, led_left_idx_start) = map_height_to_leds(height_frac_start)
    (led_right_idx_end, led_left_idx_end) = map_height_to_leds(height_frac_end)
    led_right_idx_min = min(led_right_idx_start, led_right_idx_end)
    led_right_idx_max = max(led_right_idx_start, led_right_idx_end)
    led_left_idx_min = min(led_left_idx_start, led_left_idx_end)
    led_left_idx_max = max(led_left_idx_start, led_left_idx_end)
    return [*range(led_right_idx_min, led_right_idx_max + 1)] + [*range(led_left_idx_min, led_left_idx_max + 1)]

def map_bbox_to_leds(top_left, bottom_right):
    (top_left_x, top_left_y) = top_left
    (bottom_right_x, bottom_right_y) = bottom_right
    led_width_range = map_width_range_to_leds(top_left_x, bottom_right_x)
    led_height_range = map_height_range_to_leds(top_left_y, bottom_right_y)
    led_range = led_width_range + led_height_range
    led_range.sort()
    return led_range

def run_bbox():
    pixels.begin()
    color = pixels.Color(255, 0, 0)
    white = pixels.Color(255, 255, 255)
    black = pixels.Color(0, 0, 0)

    bbox_top_left_x = 0.0
    bbox_top_left_y = 0.0
    bbox_bottom_right_x = 0.0
    bbox_bottom_right_y = 0.0
    bbox_incr = 0.1

    steps = 10
    fill_mode = False

    while True:
        iteration = 0
        for edge_frac in numpy.linspace(0, 1.0, num=steps):
            print("edge_frac =", edge_frac)
            if iteration % steps == 0:
                fill_mode = not fill_mode
            if fill_mode:
                print("fill")
                bbox_bottom_right_x = edge_frac
                bbox_bottom_right_y = edge_frac
            else:
                print("empty")
                bbox_top_left_x = edge_frac
                bbox_top_left_y = edge_frac

            active_pixels = map_bbox_to_leds((bbox_top_left_x, bbox_top_left_y), (bbox_bottom_right_x, bbox_bottom_right_y))

            pixels.fill(black, 0, NUM_PIXELS)
            for i in active_pixels:
                pixels.setPixelColor(i, white)
            pixels.show()
            time.sleep(DELAY)
            iteration += 1

if __name__ == "__main__":
    run_bbox()
