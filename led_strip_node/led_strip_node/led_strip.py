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

from threading import Thread, Event
from queue import Queue

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

NUM_PIXELS = 38
NUM_LEDS_HORIZ = 7
NUM_LEDS_VERT = 12
PIXEL_ORDER = neopixel.GRB
#PIXEL_ORDER = neopixel.GRBW
DELAY = 0.05

white = 0xFFFFFF
black = 0x000000
red = 0xFF0000
green = 0x0000FF
blue = 0x00FF00

#white = 0xFFFFFF00
#black = 0x00000000
#red = 0xFF000000
#green = 0x0000FF00
#blue = 0x00FF0000

#foreground = white
foreground = blue
#background = 0x444444
#background = black
#background = 0x111111
background = white

attract_mode = False

last_person_timestamp = time.time()

spi = board.SPI()
pixels = neopixel.NeoPixel_SPI(spi, NUM_PIXELS, pixel_order=PIXEL_ORDER, auto_write=False)


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

def map_edge_bottom_range_to_led(width_frac_start, width_frac_end):
    led_bottom_idx_start = map_edge_bottom_to_led(width_frac_start)
    led_bottom_idx_end = map_edge_bottom_to_led(width_frac_end)
    led_bottom_idx_min = min(led_bottom_idx_start, led_bottom_idx_end)
    led_bottom_idx_max = max(led_bottom_idx_start, led_bottom_idx_end)
    return [*range(led_bottom_idx_min, led_bottom_idx_max + 1)]
    
def map_edge_right_range_to_led(width_frac_start, width_frac_end):
    led_right_idx_start = map_edge_right_to_led(width_frac_start)
    led_right_idx_end = map_edge_right_to_led(width_frac_end)
    led_right_idx_min = min(led_right_idx_start, led_right_idx_end)
    led_right_idx_max = max(led_right_idx_start, led_right_idx_end)
    return [*range(led_right_idx_min, led_right_idx_max + 1)]
    
def map_edge_left_range_to_led(width_frac_start, width_frac_end):
    led_left_idx_start = map_edge_left_to_led(width_frac_start)
    led_left_idx_end = map_edge_left_to_led(width_frac_end)
    led_left_idx_min = min(led_left_idx_start, led_left_idx_end)
    led_left_idx_max = max(led_left_idx_start, led_left_idx_end)
    return [*range(led_left_idx_min, led_left_idx_max + 1)]
    
def map_edge_top_range_to_led(width_frac_start, width_frac_end):
    led_top_idx_start = map_edge_top_to_led(width_frac_start)
    led_top_idx_end = map_edge_top_to_led(width_frac_end)
    led_top_idx_min = min(led_top_idx_start, led_top_idx_end)
    led_top_idx_max = max(led_top_idx_start, led_top_idx_end)
    return [*range(led_top_idx_min, led_top_idx_max + 1)]
    
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
    pixels.fill(black)
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

leds = []

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
        
        det_json = json.loads(msg.data)['DETECTED_OBJECTS']
        person_dict = [x for x in det_json if x['name'] == 'person']
        center_list = [x['center'] for x in person_dict if 'center' in x.keys()]
        w_h_list = [x['w_h'] for x in person_dict if 'w_h' in x.keys()]
        num_persons = min(len(center_list), len(w_h_list))
        #print("num_persons", num_persons)

        global last_person_timestamp
        if num_persons > 0:
            last_person_timestamp = time.time()
        else:
            return

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
            # leds_bbox = self.bbox_to_all_sides(bbox_top_left, bbox_bottom_right)
            leds_bbox = self.bbox_to_two_sides(bbox_top_left, bbox_bottom_right)
            active_pixels += leds_bbox
        global leds
        leds = active_pixels

    def bbox_to_all_sides(self, bbox_top_left, bbox_bottom_right):
        (bbox_top_left_x, bbox_top_left_y) = bbox_top_left
        (bbox_bottom_right_x, bbox_bottom_right_y) = bbox_bottom_right
        active_pixels = map_bbox_to_leds((bbox_top_left_x, bbox_top_left_y), (bbox_bottom_right_x, bbox_bottom_right_y))
        return active_pixels

    def bbox_to_two_sides(self, bbox_top_left, bbox_bottom_right):
        (bbox_top_left_x, bbox_top_left_y) = bbox_top_left
        (bbox_bottom_right_x, bbox_bottom_right_y) = bbox_bottom_right
        center_x = (bbox_bottom_right_x + bbox_top_left_x) / 2.0
        center_y = (bbox_bottom_right_y + bbox_top_left_y) / 2.0
        active_pixels = []

        if center_x < 0.5:
            active_pixels += map_edge_left_range_to_led(bbox_top_left_y, bbox_bottom_right_y)
        else:
            active_pixels += map_edge_right_range_to_led(bbox_top_left_y, bbox_bottom_right_y)
        
        if center_y < 0.5:
            active_pixels += map_edge_top_range_to_led(bbox_top_left_x, bbox_bottom_right_x)
        else:
            active_pixels += map_edge_bottom_range_to_led(bbox_top_left_x, bbox_bottom_right_x)
        return active_pixels


def attract_sequence_1(exit_event: Event):
    ANIMATION_DELAY = 0.5
    global attract_mode
    pixels.fill(red)
    pixels.show()
    time.sleep(ANIMATION_DELAY)
    #if not attract_mode or exit_event:
    #    return
    pixels.fill(green)
    pixels.show()
    time.sleep(ANIMATION_DELAY)
    #if not attract_mode or exit_event:
    #    return
    pixels.fill(blue)
    pixels.show()
    time.sleep(ANIMATION_DELAY)

def attract_sequence_2(exit_event: Event):
    ANIMATION_DELAY = 0.02
    ANIMATION_BACKGROUND = 0x111111
    global attract_mode
    #while attract_mode and not exit_event.is_set():
    pixels.fill(ANIMATION_BACKGROUND)
    for i in range(NUM_PIXELS):
        pixels[i] = red
        pixels.show()
        time.sleep(ANIMATION_DELAY)
        pixels[i] = ANIMATION_BACKGROUND
    #if not attract_mode or exit_event:
    #    return
    for i in range(NUM_PIXELS):
        pixels[i] = green
        pixels.show()
        time.sleep(ANIMATION_DELAY)
        pixels[i] = ANIMATION_BACKGROUND
    #if not attract_mode or exit_event:
    #    return
    for i in range(NUM_PIXELS):
        pixels[i] = blue
        pixels.show()
        time.sleep(ANIMATION_DELAY)
        pixels[i] = ANIMATION_BACKGROUND

def led_loop(exit_event: Event):
    global attract_mode
    global leds
    while not exit_event.is_set():
        while not attract_mode and not exit_event.is_set():
            #print("ATTRACT WAITING")
            #time.sleep(1)
            active_pixels = leds
            pixels.fill(background)
            for i in active_pixels:
                if i >= NUM_PIXELS:
                    print("Warning: Index exceeds number of pixels")
                    continue
                pixels[i] = foreground
            pixels.show()
            time.sleep(0.2)
        while attract_mode and not exit_event.is_set():
            attract_sequence_1(exit_event)
            attract_sequence_2(exit_event)

def person_watchdog(exit_event: Event):
    global last_person_timestamp
    global attract_mode
    while not exit_event.is_set():
        time_diff = time.time() - last_person_timestamp
        #print("last_person_timestamp:", last_person_timestamp)
        #print("time_diff:", time_diff)
        if time_diff > 1.0:
            if not attract_mode:
                print("Attract mode ON")
            attract_mode = True
        else:
            if attract_mode:
                print("Attract mode OFF")
            attract_mode = False
        time.sleep(0.5)

def main(args=None):
    rclpy.init(args=args)

    led_strip = LedStrip()
    exit_event = Event()
    led_loop_thread = Thread(target=led_loop, args=(exit_event,))
    person_watchdog_thread = Thread(target=person_watchdog, args=(exit_event,))

    try:
        person_watchdog_thread.start()
        led_loop_thread.start()
        rclpy.spin(led_strip)
    except KeyboardInterrupt:
        exit_event.set()
        led_loop_thread.join()
        person_watchdog_thread.join()
        clear_leds()
        print('exit')
    except BaseException:
        print('exception:', file=sys.stderr)
        raise
    finally:
        # Destroy the node explicitly
        # (optional - Done automatically when node is garbage collected)
        led_strip.destroy_node()
        # rclpy.shutdown()

if __name__ == '__main__':
    main()
