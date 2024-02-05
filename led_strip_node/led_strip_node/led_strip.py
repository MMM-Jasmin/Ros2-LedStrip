# colcon build --packages-select led_strip_node
# ros2 run led_strip_node led_strip_listener
# ros2 topic pub --once /object_det/objects std_msgs/String 'data: HELLO'
# ros2 topic pub --once /object_det/objects std_msgs/String 'data: {"DETECTED_OBJECTS": [{"TrackID": 1, "name": "person", "center": [0.500,0.692], "w_h": [0.991,0.579]}, {"TrackID": 2, "name": "book", "center": [0.912,0.290], "w_h": [0.083,0.143]}, {"TrackID": 3, "name": "book", "center": [0.973,0.290], "w_h": [0.051,0.137]}], "DETECTED_OBJECTS_AMOUNT": 3 }'

# ros2 topic pub --once /object_det/objects std_msgs/String "data: {\"name\": \"person\"}"

import sys, os, time, json
import numpy as np
np.set_printoptions(threshold=sys.maxsize)

import math, colorsys, cv2

import board
import neopixel_spi as neopixel

from threading import Thread, Event
from queue import Queue

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, ColorRGBA

def getHexCodeFromRGB( red, green, blue):
    return (int(red) << 16 | int(blue) << 8 | int(green))

def getHexCodeFromHSV( hue, saturation, value):
    (h, s, v) = (hue / 360, saturation / 255, value / 255)
    (r, g, b) = colorsys.hsv_to_rgb(h, s, v)
    (r, g, b) = (int(r * 255), int(g * 255), int(b * 255))

    return getHexCodeFromRGB(r, g, b)

def getRGBFromHSV( hue, saturation, value):
    (h, s, v) = (hue / 360, saturation / 255, value / 255)
    (r, g, b) = colorsys.hsv_to_rgb(h, s, v)
    (r, g, b) = (int(r * 255), int(g * 255), int(b * 255))

    return [r, g, b]

def lerp(a: float, b: float, t: float) -> float:
    """Linear interpolate on the scale given by a to b, using t as the point on that scale.
    Examples
    --------
        50 == lerp(0, 100, 0.5)
        4.2 == lerp(1, 5, 0.8)
    """
    return (1 - t) * a + t * b

NUM_LEDS_HORIZ = 7
NUM_LEDS_VERT = 12

NUM_PIXELS = 2 * NUM_LEDS_HORIZ + 2 * NUM_LEDS_VERT
print("--initializing for " + str(NUM_PIXELS) + " leds--")

raster_size = 20
raster_margin = 15

raster_width = NUM_LEDS_HORIZ * raster_size 
raster_height = NUM_LEDS_VERT * raster_size

PIXEL_ORDER = neopixel.GRB
#PIXEL_ORDER = neopixel.GRBW

attract_mode = False
hue_dist = 360 / NUM_PIXELS
rainbow_hue = np.linspace(0, 360, num=NUM_PIXELS)

spi = board.SPI()
pixels = neopixel.NeoPixel_SPI(spi, NUM_PIXELS, pixel_order=PIXEL_ORDER, auto_write=False)

idle_color = [240., 0., 64.]

lerp_rgb = np.array([idle_color]*NUM_PIXELS)
flash_color = [0.0, 0.0, 0.0]

flash_waiting_timer = 0

sat_image = np.zeros((raster_height + raster_margin * 2, raster_width + raster_margin * 2), dtype=np.uint8)

person_det_counter = 0

class LedStrip(Node):

    def __init__(self):
        super().__init__('led_strip')
        self.subscription = self.create_subscription( String, '/object_det/objects', self.listener_callback, 10)
        self.subscription_action = self.create_subscription( ColorRGBA, '/led/action/flash', self.action_callback, 10)
        self.subscription  # prevent unused variable warning

        #pub = self.create_publisher(ColorRGBA, '/led/action/flash', 10)
        #msg = ColorRGBA()
        #msg.r = 255.0
        #msg.g = 0.0
        #msg.b = 0.0
        #pub.publish(msg)

    def clear_leds(self):
        pixels.fill(getHexCodeFromRGB(0,0,0))
        pixels.show()

    def action_callback(self, msg):
        global flash_waiting_timer
        global flash_color
        
        print("flashing in the following color")
        print(msg)

        flash_color = [int(msg.r), int(msg.g), int(msg.b)]
        flash_waiting_timer = 0.5


    def listener_callback(self, msg):
        # self.get_logger().info('I heard: "%s"' % msg.data)
        
        det_json = json.loads(msg.data)['DETECTED_OBJECTS']
        person_dict = [x for x in det_json if x['name'] == 'person']
        center_list = [x['center'] for x in person_dict if 'center' in x.keys()]
        w_h_list = [x['w_h'] for x in person_dict if 'w_h' in x.keys()]
        num_persons = min(len(center_list), len(w_h_list))

        global person_det_counter
        global attract_mode
        if num_persons > 0:
            if person_det_counter > 10: 
                if attract_mode:
                    print("attract_mode false")
                    attract_mode = False
            else:
                person_det_counter += 1 
        else:
            if person_det_counter < 1:
                if not attract_mode:
                    print("attract_mode true")
                    attract_mode = True
            else:
                person_det_counter -= 2
            return

        global sat_image
        box_image = np.copy(sat_image)

        threshhold = 5
        box_image = np.where(box_image < threshhold, 0, box_image - threshhold)
              
        for i in range(num_persons):
            (cx, cy) = center_list[i]
            (cx, cy) = (int(cx * raster_width), int(cy * raster_height))
            (w, h) = w_h_list[i]
            (w, h) = (int(w * raster_width + 20), int(h * raster_height + 20))

            top     = int(math.floor(cy - h/2))
            bottom  = int(math.ceil(cy + h/2))
            left    = int(math.floor(cx - w/2))
            right   = int(math.ceil(cx + w/2))

            for y in range(top, bottom):
                if (y + raster_margin) < 0 or y + raster_margin > (raster_height + raster_margin * 2 -1):
                    continue
                for x in range(left, right):
                    if (x + raster_margin) < 0 or (x + raster_margin) > (raster_width + raster_margin * 2 - 1):
                        continue
                    box_image[y + raster_margin][x + raster_margin] = 255

        box_image = cv2.blur(box_image,(15,15),cv2.BORDER_REFLECT)

        np.copyto(sat_image, box_image)


def led_loop(exit_event: Event):
    global attract_mode
    global flash_color
    global lerp_rgb
    global flash_waiting_timer

    flashing_ongoing = False

    while not exit_event.is_set():
        if attract_mode:
            # taste the rainbow!
            for i in range(NUM_PIXELS):
                color = getHexCodeFromHSV(rainbow_hue[i],255,255)
                pixels[i] = color

                rainbow_hue[i] += 1.0
                rainbow_hue[i] = rainbow_hue[i] % 360

        else:
            # create satturation array to turn the color blue depending on the person bbox
            box_sat = []
            for i in range(NUM_LEDS_HORIZ): # get all the bottom pixel
                box_sat.append(sat_image[raster_height + raster_margin][raster_margin + i * raster_size])
            for i in range(NUM_LEDS_VERT): # get all the right pixel
                box_sat.append(sat_image[raster_height + raster_margin - i * raster_size][raster_margin + raster_width])
            for i in range(NUM_LEDS_HORIZ): # get all the top pixel
                box_sat.append(sat_image[raster_margin][raster_margin + raster_width - i * raster_size])
            for i in range(NUM_LEDS_VERT): # get all the left pixel
                box_sat.append(sat_image[raster_margin + i * raster_size ][raster_margin])

            # use default hue and value, change only the saturation
            for index in range(len(lerp_rgb)):
                if flash_waiting_timer > 0.0:
                    lerp_rgb[index] = flash_color
                    flashing_ongoing = True
                else:
                    colorToBe = getRGBFromHSV(idle_color[0],box_sat[index],idle_color[2])
                    lerp_rgb[index][0] = lerp(lerp_rgb[index][0], colorToBe[0], 0.5)
                    lerp_rgb[index][1] = lerp(lerp_rgb[index][1], colorToBe[1], 0.5)
                    lerp_rgb[index][2] = lerp(lerp_rgb[index][2], colorToBe[2], 0.5)
                
                color = getHexCodeFromRGB(lerp_rgb[index][0], lerp_rgb[index][1], lerp_rgb[index][2])
                pixels[index] = color

        pixels.show()
        
        if flashing_ongoing:
            print("sleeping for " + str(flash_waiting_timer) + " seconds")
            time.sleep(flash_waiting_timer)
            flash_waiting_timer = 0.0
            flashing_ongoing = False
        else:
            time.sleep(1./60.)

        

def main(args=None):
    rclpy.init(args=args)

    led_strip = LedStrip()
    exit_event = Event()
    led_loop_thread = Thread(target=led_loop, args=(exit_event,))

    try:
        led_loop_thread.start()
        rclpy.spin(led_strip)
    except KeyboardInterrupt:
        exit_event.set()
        led_loop_thread.join()
        led_strip.clear_leds()
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
