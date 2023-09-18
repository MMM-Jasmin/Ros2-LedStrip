# colcon build --packages-select led_strip_node
# ros2 run led_strip_node led_strip_listener
# ros2 topic pub --once /object_det/objects std_msgs/String 'data: HELLO'
# ros2 topic pub --once /object_det/objects std_msgs/String 'data: {"DETECTED_OBJECTS": [{"TrackID": 1, "name": "person", "center": [0.500,0.692], "w_h": [0.991,0.579]}, {"TrackID": 2, "name": "book", "center": [0.912,0.290], "w_h": [0.083,0.143]}, {"TrackID": 3, "name": "book", "center": [0.973,0.290], "w_h": [0.051,0.137]}], "DETECTED_OBJECTS_AMOUNT": 3 }'

# ros2 topic pub --once /object_det/objects std_msgs/String "data: {\"name\": \"person\"}"

import sys
import os
import numpy as np
import json
import time

import board
import neopixel_spi as neopixel

from multiprocessing import Process

import rclpy
from rclpy.node import Node

from std_msgs.msg import String

NUM_PIXELS = 38
NUM_LEDS_HORIZ = 7
NUM_LEDS_VERT = 12
PIXEL_ORDER = neopixel.GRB
COLORS = (0xFF0000, 0x00FF00, 0x0000FF)
DELAY = 0.05
TIMEOUT = 1 # seconds

white = 0xFFFFFF
black = 0x000000
foreground = white
# background = 0x444444
# background = black
background = 0x111111

last_person_timestamp = time.time()

spi = board.SPI()
pixels = neopixel.NeoPixel_SPI(spi, NUM_PIXELS, auto_write=False)


def map_horiz_led(width_frac):
    return int(np.round((NUM_LEDS_HORIZ - 1) * width_frac))

def map_vert_led(width_frac):
    return int(np.round((NUM_LEDS_VERT - 1) * width_frac))

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

def clear_leds():
    pixels.fill(0x000000)
    pixels.show()

def run_bbox_test():
    color = 0xFF0000
    white = 0xFFFFFF
    black = 0x000000
    # background = 0x444444
    foreground = white
    background = black

    bbox_incr = 0.1
    steps = 10
    fill_mode = False

    iteration = 0
    while True:
        bbox_top_left_x = 0.0
        bbox_top_left_y = 0.0
        bbox_bottom_right_x = 0.0
        bbox_bottom_right_y = 0.0

        for edge_frac in np.linspace(0.0, 1.0, num=steps):
            bbox_bottom_right_x = edge_frac
            bbox_bottom_right_y = edge_frac
            active_pixels = map_bbox_to_leds((bbox_top_left_x, bbox_top_left_y), (bbox_bottom_right_x, bbox_bottom_right_y))
            pixels.fill(background)
            for i in active_pixels:
                pixels[i] = white
            pixels.show()
            time.sleep(DELAY)

        for edge_frac in np.linspace(0.0, 1.0, num=steps):
            bbox_top_left_x = edge_frac
            bbox_top_left_y = edge_frac
            active_pixels = map_bbox_to_leds((bbox_top_left_x, bbox_top_left_y), (bbox_bottom_right_x, bbox_bottom_right_y))
            pixels.fill(background)
            for i in active_pixels:
                pixels[i] = white
            pixels.show()
            time.sleep(DELAY)
        iteration += 1


def attract_mode():
    print("Attract mode")
    while True:
        pixels.fill(0xFF0000)
        time.sleep(DELAY)
        pixels.fill(0x00FF00)
        time.sleep(DELAY)
        pixels.fill(0x0000FF)
        time.sleep(DELAY)

attract_process = Process(target=attract_mode)


class LedStrip(Node):

    def __init__(self):
        super().__init__('led_strip')
        self.subscription = self.create_subscription(
            String,
            '/object_det/objects',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        # self.get_logger().info('I heard: "%s"' % msg.data)
        if attract_process.is_alive():
            attract_process.join()
        
        last_person_timestamp = time.time()
        det_json = json.loads(msg.data)['DETECTED_OBJECTS']
        person_dict = [x for x in det_json if x['name'] == 'person']
        center_list = [x['center'] for x in person_dict if 'center' in x.keys()]
        w_h_list = [x['w_h'] for x in person_dict if 'w_h' in x.keys()]
        num_persons = len(min(center_list, w_h_list))
        
        pixels.fill(background)
        active_pixels = []
        for i in range(num_persons):
            (cx, cy) = center_list[i]
            (w, h) = w_h_list[i]
            w2 = w / 2.0
            h2 = h / 2.0
            bbox_top_left_x = cx - w2
            bbox_top_left_y = cy - h2
            bbox_bottom_right_x = cx + w2
            bbox_bottom_right_y = cy + h2
            bbox_top_left = (bbox_top_left_x, bbox_top_left_y)
            bbox_bottom_right = (bbox_bottom_right_x, bbox_bottom_right_y)
            leds_bbox = self.update_bbox(bbox_top_left, bbox_bottom_right)
            active_pixels += leds_bbox
        for i in active_pixels:
            if i >= NUM_PIXELS:
                print("Warning: Index exceeds number of pixels")
                continue
            pixels[i] = foreground
        pixels.show()
        
    def update_bbox(self, bbox_top_left, bbox_bottom_right):
        (bbox_top_left_x, bbox_top_left_y) = bbox_top_left
        (bbox_bottom_right_x, bbox_bottom_right_y) = bbox_bottom_right
        active_pixels = map_bbox_to_leds((bbox_top_left_x, bbox_top_left_y), (bbox_bottom_right_x, bbox_bottom_right_y))
        return active_pixels

    def attract_mode():
        while True:
            pass

def person_watchdog():
    if time.time() - last_person_timestamp > TIMEOUT:
        if not attract_process.is_alive():
            attract_process.start()
    time.sleep(TIMEOUT)

watchdog_process = Process(target=person_watchdog)

def main(args=None):
    rclpy.init(args=args)

    led_strip = LedStrip()
    # p = Process(target=run_bbox_test)

    try:
        # p.start()
        watchdog_process.start()
        rclpy.spin(led_strip)
    except KeyboardInterrupt:
        # p.join()
        watchdog_process.join()
        clear_leds()
        print('exit')
    except BaseException:
        print('exception:', file=sys.stderr)
        raise
    # finally:
    #     Destroy the node explicitly
    #     (optional - Done automatically when node is garbage collected)
    #     led_strip.destroy_node()
    #     rclpy.shutdown()

if __name__ == '__main__':
    main()
