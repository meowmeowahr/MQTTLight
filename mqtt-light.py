"""
MQTTLight
Purpose: Raspberry Pi MQTT Light Controller for ws281x LEDs
Author: Kevin Ahr
Email: meowmeowahr@gmail.com
"""

import paho.mqtt.client as mqtt
import pyfiglet
import configparser
import logging
import json
import lighting
import threading
import time
import board
import sys


__author__ = "Kevin Ahr"
__version__ = "0.3.1"

# bold banner
print("\x1b[1m" + pyfiglet.figlet_format("MQTTLight") + "\x1b[0m")
print("v{}".format(__version__))

# variables
config = configparser.ConfigParser()
config.read('config.ini')

logging.basicConfig(level=logging.DEBUG, format='%(asctime)s - %(levelname)s - %(message)s', filename='mqttlight.log', filemode='w')

host = config['MQTT']['Host']
port = config.getint('MQTT', 'Port')
state_topic = config['MQTT']['LEDStateTopic']
command_topic = config['MQTT']['LEDCommandTopic']
online_topic = config['MQTT']['OnlineTopic']

# see if there is "LED" section in config
if config.has_section('LED'):
    led_pin = config.getint('LED', 'Pin')
    num_pix = config.getint('LED', 'NumPix')
    pix_order = config['LED']['PixOrder']
    led_type = 0
    lighting.start(led_pin, num_pix, order=pix_order)
elif config.has_section('DumbLED'):
    r_pin = config.getint('DumbLED', 'RedPin')
    g_pin = config.getint('DumbLED', 'GreenPin')
    b_pin = config.getint('DumbLED', 'BluePin')
    led_type = 1
    lighting.start_dumb(r_pin, g_pin, b_pin)


global RED, GREEN, BLUE, BRIGHTNESS, EFFECT, STATE
RED = 255
GREEN = 255
BLUE = 255
BRIGHTNESS = 127
EFFECT = "None"
STATE = "OFF"

global old_red, old_green, old_blue, old_brightness, old_effect, old_state
old_red = RED
old_green = GREEN
old_blue = BLUE
old_brightness = BRIGHTNESS
old_effect = EFFECT
old_state = STATE

# mqtt client
def on_connect(client, userdata, flags, rc):
    logging.info("Connected with result code %s", str(rc))

    # subscribe to topics
    client.subscribe(command_topic)
    client.publish(online_topic, "online")
    client.publish(state_topic, construct_message())
    logging.debug("Published Message: {}, to {}".format(construct_message(), state_topic))

def on_message(client, userdata, msg):
    global RED, GREEN, BLUE, BRIGHTNESS, EFFECT, STATE
    # decode message
    message = json.loads(msg.payload.decode('utf-8'))
    # log
    logging.debug("Recieved Message: {}".format(message))

    old_brigtness = BRIGHTNESS
    old_red, old_green, old_blue = RED, GREEN, BLUE

    STATE = message['state']
    if STATE == "OFF":
        lighting.pixels.fill((0,0,0))
        lighting.pixels.show()

    # try to get brightness
    try:
        BRIGHTNESS = message['brightness']
    except KeyError:
        pass

    # try to get color
    try:
        RED = message['color']['r']
        GREEN = message['color']['g']
        BLUE = message['color']['b']
    except KeyError:
        pass

    # try to get effect
    try:
        EFFECT = message['effect']
    except KeyError:
        pass

    # publish state
    client.publish(state_topic, construct_message())

client = mqtt.Client()
client.on_connect = on_connect
client.on_message = on_message
client.connect(host, port, 60)

stop = False

def loop():
    while True:
        client.loop()
        if stop:
            sys.exit()

def round_tuple(t):
    return tuple(map(lambda x: round(x), t))

def light_loop():
    while True:
        if EFFECT == "None":
            if STATE == "OFF":
                # make actual rgb with brightness
                if led_type == 0:
                    actual_old = lighting.pixels[0]
                elif led_type == 1:
                    actual_old = (lighting.pixels.red, lighting.pixels.green, lighting.pixels.blue)
                lighting.fade_all(actual_old, (0,0,0))
            else:
                # make actual rgb with brightness
                if led_type == 0:
                    actual_old = lighting.pixels[0]
                elif led_type == 1:
                    actual_old = (lighting.pixels.red, lighting.pixels.green, lighting.pixels.blue)
                actual_new = (RED * (BRIGHTNESS/255), GREEN * (BRIGHTNESS/255), BLUE * (BRIGHTNESS/255))
                lighting.fade_all(round_tuple(actual_old), round_tuple(actual_new))
        elif EFFECT == "Breathing":
            lighting.breath((RED, GREEN, BLUE), BRIGHTNESS)
            time.sleep(0.01)
        elif EFFECT == "Colorloop":
            lighting.colorloop(BRIGHTNESS)
            time.sleep(0.03)
        elif EFFECT == "ColoredLights":
            if led_type == 0:
                lighting.classic_colors(BRIGHTNESS)
            else:
                logging.critical("{} is not supported for led mode {}".format(EFFECT, led_type))
                break
        elif EFFECT == "WipeRedToGreen":
            if led_type == 0:
                lighting.wipe_red_to_green(BRIGHTNESS)
                time.sleep(0.05)
            else:
                logging.critical("{} is not supported for led mode {}".format(EFFECT, led_type))
                break
        elif EFFECT == "Rainbow":
            if led_type == 0:
                lighting.rainbow_cycle(BRIGHTNESS)
                time.sleep(0.01)
            else:
                logging.critical("{} is not supported for led mode {}".format(EFFECT, led_type))
                break
        elif EFFECT == "Magic":
            if led_type == 0:
                lighting.magic_cycle(BRIGHTNESS)
                time.sleep(0.05)
            else:
                logging.critical("{} is not supported for led mode {}".format(EFFECT, led_type))
                break
        elif EFFECT == "Fire":
            if led_type == 0:
                lighting.fire_cycle(BRIGHTNESS)
                time.sleep(0.05)
        elif EFFECT == "FadeColorToWhite":
            if led_type == 0:
                lighting.fade_classic_to_white(BRIGHTNESS)
                time.sleep(0.05)
            else:
                logging.critical("{} is not supported for led mode {}".format(EFFECT, led_type))
                break
        elif EFFECT == "FlashColorToWhite":
            if led_type == 0:
                lighting.flash_classic_to_white(BRIGHTNESS)
                time.sleep(1)
            else:
                logging.critical("{} is not supported for led mode {}".format(EFFECT, led_type))
                break
        elif EFFECT == "Random":
            if led_type == 0:
                lighting.randomc((RED, GREEN, BLUE), BRIGHTNESS)
                time.sleep(0.3)
            else:
                logging.critical("{} is not supported for led mode {}".format(EFFECT, led_type))
                break
        elif EFFECT == "RandomColor":
            if led_type == 0:
                lighting.random_color(BRIGHTNESS)
                time.sleep(0.3)
            else:
                logging.critical("{} is not supported for led mode {}".format(EFFECT, led_type))
                break
    global stop
    stop = True

def construct_message():
    msg =  '{{"brightness": {0}, "color_mode": "rgb", "color": {{ "r": {1}, "g": {2}, "b": {3}}}, "effect": "{4}", "state": "{5}", "transition": 2}}'.format(BRIGHTNESS, RED, GREEN, BLUE, EFFECT, STATE)
    client.publish(state_topic, msg)
    return msg
    
if __name__ == "__main__":
    # start mqtt loop
    try:
        # start light loop
        light_thread = threading.Thread(target=light_loop, daemon=True)
        light_thread.start()
        loop()
    except Exception as e:
        logging.error("Program experienced Exception: %s", e)
        try:
            client.publish(online_topic, "offline")
            client.disconnect()
        except Exception as e:
            logging.error("Program experienced Exception %s, while disconnecting", e)
