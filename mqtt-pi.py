"""
MQTTPi
Purpose: Raspberry Pi MQTT Light Controller for ws281x LEDs, with extra functionality for other devices
Author: Kevin Ahr
Email: meowmeowahr@gmail.com
"""

import json
import logging
import os
import threading
import time

import adafruit_ads1x15.ads1115 as ADS
import board
import busio
import gpiozero
import paho.mqtt.client as mqtt
import pyfiglet
from adafruit_ads1x15.analog_in import AnalogIn
from adafruit_servokit import ServoKit
import psutil

from light_funcs import *

__author__ = "Kevin Ahr"
__version__ = "0.6.9"

# bold banner
print("\033[96m" + pyfiglet.figlet_format("M Q T T Pi") + "\x1b[0m")
print("v{}".format(__version__))

# path
path = os.path.dirname(os.path.realpath(__file__))

# variables
config = json.load(open(os.path.join(path, "config.json")))

logging.basicConfig(level=logging.DEBUG, format='%(asctime)s - %(levelname)s - %(message)s', filename="mqttlight.log", filemode="w")

host = config["mqtt"]["host"]
port = config["mqtt"]["port"]

state_topics = []
command_topics = []
online_topics = []

neopixels = []
np_state_topics = []
np_command_topics = []
np_online_topics = []
np_colors = []
np_brightnesses = []
np_effects = []
np_states = []

dumbpixels = []
dp_state_topics = []
dp_command_topics = []
dp_online_topics = []
dp_colors = []
dp_brightnesses = []
dp_states = []
dp_effects = []
dp_threads = []

simpleouts = []
so_state_topics = []
so_command_topics = []
so_online_topics = []
so_current_states = []

simpleins = []
si_before = []
si_state_topics = []
si_online_topics = []
si_payloads = []

cputemps = []
cput_state_topics = []
cput_last_temps = []

cpuusages = []
cpuu_state_topics = []
cpuu_last_values = []

diskusages = []
dsku_state_topics = []
dsku_last_usages = []

ads1115_devices = []
ads1115_chans = []
ads1115_chans_before = []
ads1115_chan_state_topics = []
ads1115_chan_online_topics = []
ads1115_chans_max = []
ads1115_chans_min = []

servokits = []
svo_state_topics = []
svo_command_topics = []
svo_online_topics = []
svo_current_states = []
svo_devids = []
svo_channels = []

components = config["items"]
component_num = 0

ANIMATION_STEP = 0

def hex_str_to_hex(hex_str):
    return(int(hex_str, base=16))

# i2c
i2c = busio.I2C(board.SCL, board.SDA)

BIT16SIGNEDMAX = 32767

class DumbAnimations:
    def breath(rgb, dp, brightness=255):
        global ANIMATION_STEP
        ANIMATION_STEP += 1
        if ANIMATION_STEP > 255:
            ANIMATION_STEP = 0

        if ANIMATION_STEP < 128:
            dp.color = multiply_tuple(rgb, (brightness/255)*(map_range(ANIMATION_STEP, 0, 128, 0, 100))/100/255)
        else:
            dp.color = multiply_tuple(rgb, (brightness/255)*(map_range(ANIMATION_STEP, 128, 255, 100, 0))/100/255)

    def colorloop(dp, brightness=255):
        global ANIMATION_STEP
        ANIMATION_STEP += 1
        if ANIMATION_STEP > 255:
            ANIMATION_STEP = 0
            
        dp.color = (multiply_tuple(wheel(ANIMATION_STEP), brightness/255/255))

def filter_dict(input_dict, key_list):
    """
    Filter a dictionary based on a list of keys.

    Parameters:
    - input_dict (dict): The dictionary to be filtered.
    - key_list (list): The list of keys to include in the filtered dictionary.

    Returns:
    - dict: The filtered dictionary.
    """
    return {key: input_dict[key] for key in key_list if key in input_dict}

# mqtt client
def on_connect(client, userdata, flags, rc):
    logging.info("Connected with result code %s", str(rc))

    # subscribe to topics
    for i in range(len(command_topics)):
        client.subscribe(command_topics[i])

def on_message(client: mqtt.Client, userdata, msg):
    # see if the message came from a neopixel component
    if msg.topic in np_command_topics:
        index = np_command_topics.index(msg.topic)
        logging.info("Neopixel index {} set to {}".format(index, msg.payload))
        # decode the message
        payload: dict = json.loads(msg.payload)

        # set variables
        try:
            np_colors[index] = (payload["color"]["r"], payload["color"]["g"], payload["color"]["b"])
        except KeyError:
            pass
        try:
            np_brightnesses[index] = payload["brightness"]
        except KeyError:
            pass
        try:
            np_effects[index] = payload["effect"]
        except KeyError:
            pass

        np_states[index] = payload["state"]

        # change npd.json from driver
        npd = json.load(open(neopixels[index]))
        npd["state"] = np_states[index]
        npd["brightness"] = np_brightnesses[index]
        npd["color"]["r"] = np_colors[index][0]
        npd["color"]["g"] = np_colors[index][1]
        npd["color"]["b"] = np_colors[index][2]
        npd["effect"] = np_effects[index]
        print(npd)

        client.publish(np_state_topics[index], construct_message(npd["brightness"], npd["color"]["r"], npd["color"]["g"], npd["color"]["b"], npd["effect"], npd["state"]), retain=False)
        json.dump(npd, open(neopixels[index], "w")) # save the file
    elif msg.topic in dp_command_topics:
        index = dp_command_topics.index(msg.topic)
        logging.info("DumbPixel index {} set to {}".format(index, msg.payload))
        # decode the message
        payload = json.loads(msg.payload)
        # publish payload to state topic
        client.publish(dp_state_topics[index], msg.payload)
        # set variables
        try:
            dp_colors[index] = (payload["color"]["r"], payload["color"]["g"], payload["color"]["b"])
        except KeyError:
            pass
        try:
            dp_brightnesses[index] = payload["brightness"]
        except KeyError:
            pass
        try:
            dp_effects[index] = payload["effect"]
        except KeyError:
            pass
        dp_states[index] = payload["state"]
    elif msg.topic in so_command_topics:
        index = so_command_topics.index(msg.topic)
        logging.info("SimpleOut index {} set to {}".format(index, msg.payload))
        # decode the message
        payload = msg.payload.decode("utf-8")
        # publish payload to state topic
        client.publish(so_state_topics[index], msg.payload)
        if payload == "ON":
            so_current_states[index] = 1
            simpleouts[index].on()
        elif payload == "OFF":
            so_current_states[index] = 0
            simpleouts[index].off()
    elif msg.topic in svo_command_topics:
        index = svo_command_topics.index(msg.topic)
        logging.info("ServoKit index {} set to {}".format(index, msg.payload))
        # decode the message
        payload = msg.payload.decode("utf-8")

        # publish the message to state topic
        client.publish(svo_state_topics[index], msg.payload)

        # get the dev_id
        dev_id = svo_devids[index]
        # get the channel
        channel = svo_channels[index]
        # set the servo
        for i in range(len(servokits)):
            if servokits[i][1] == dev_id:
                servokits[i][0].servo[channel].angle = int(payload)

def construct_message(brightness=127, red=255, green=255, blue=255, effect="None", state="OFF"):
    msg = '{{"brightness": {0}, "color_mode": "rgb", "color": {{ "r": {1}, "g": {2}, "b": {3}}}, "effect": "{4}", "state": "{5}", "transition": 2}}'.format(brightness, red, green, blue, effect, state)
    return msg

# loop
def loop():
    while True:
        time.sleep(0.01)
        client.loop()
        # refresh simpleins
        for i in range(len(simpleins)):
            if si_before[i] != simpleins[i].value:
                if simpleins[i].value == 1:
                    client.publish(si_state_topics[i], si_payloads[i][0])
                else:
                    client.publish(si_state_topics[i], si_payloads[i][1])
                si_before[i] = simpleins[i].value
        # refresh ads1115
        for i in range(len(ads1115_devices)):
            time.sleep(0.01)
            for j in range(len(ads1115_chans)):
                if ads1115_devices[i][1] == ads1115_chans[j][0]:
                    chan = AnalogIn(ads1115_devices[i][0], ads1115_chans[j][1])
                    if chan.value != ads1115_chans_before[j]:
                        client.publish(ads1115_chan_state_topics[j], round(map_range(chan.value+1, 0, BIT16SIGNEDMAX, ads1115_chans_min[j], ads1115_chans_max[j])))
                        ads1115_chans_before[j] = chan.value

def update_loop():
    while True:
        for i in range(len(online_topics)):
                client.publish(online_topics[i], "online")

        for index in range(len(cputemps)):
            if cput_last_temps[index] != cputemps[index].temperature:
                cput_last_temps[index] = cputemps[index].temperature
                client.publish(cput_state_topics[index], cputemps[index].temperature)

        for index in range(len(cpuusages)):
            if cpuu_last_values[index] != psutil.cpu_percent(0.5):
                cpuu_last_values[index] = psutil.cpu_percent(0.5)
                client.publish(cpuu_state_topics[index], psutil.cpu_percent(0.5))

        for index in range(len(diskusages)):
            if dsku_last_usages[index] != diskusages[index].usage:
                dsku_last_usages[index] = diskusages[index].usage
                client.publish(dsku_state_topics[index], round(diskusages[index].usage, 1))

        time.sleep(10)


def dp_loop(index):
    while True:
        if dp_effects[index] == "None":
            if dp_states[index] == "OFF":
                dumbpixels[index].color = (0, 0, 0)
            else:
                for i in range(100):
                    dumbpixels[index].color = round_tuple(color_fade(dumbpixels[index].color, multiply_tuple((dp_colors[index][0]/255, dp_colors[index][1]/255, dp_colors[index][2]/255), dp_brightnesses[index]/255), i/100))
                    time.sleep(0.01)
        elif dp_effects[index] == "Breathing":
            if dp_states[index] == "OFF":
                dumbpixels[index].color = (0, 0, 0)
            else:
                DumbAnimations.breath(dp_colors[index], dumbpixels[index], dp_brightnesses[index])
                time.sleep(0.01)
        elif dp_effects[index] == "Colorloop":
            if dp_states[index] == "OFF":
                dumbpixels[index].color = (0, 0, 0)
            else:
                DumbAnimations.colorloop(dumbpixels[index], dp_brightnesses[index])
                time.sleep(0.01)


def init():
    global component_num
    for i in range(len(components)):
        if components[i]["type"] != "ADS1115_dev" and components[i]["type"] != "PCA9685_svo_dev":
            state_topics.append(components[i]["data"]["state_topic"])
            try:
                command_topics.append(components[i]["data"]["command_topic"])
            except KeyError:
                # some components don't have a command topic
                pass
            if components[i]["type"] not in ["CPUTemp", "DiskUsage", "CPUUsage"]:
                online_topics.append(components[i]["data"]["online_topic"])

        component_num += 1

        if components[i]["type"] == "NeoPixel":
            np_state_topics.append(components[i]["data"]["state_topic"])
            np_command_topics.append(components[i]["data"]["command_topic"])
            np_online_topics.append(components[i]["data"]["online_topic"])
            neopixels.append(os.path.join(components[i]["data"]["driver_directory"], "npd.json"))
            np_colors.append((255, 255, 255))
            np_brightnesses.append(127)
            np_effects.append("None")
            np_states.append("OFF")
        elif components[i]["type"] == "DumbPixel":
            dp_state_topics.append(components[i]["data"]["state_topic"])
            dp_command_topics.append(components[i]["data"]["command_topic"])
            dp_online_topics.append(components[i]["data"]["online_topic"])
            dumbpixels.append(gpiozero.RGBLED(components[i]["data"]["r_pin"], components[i]["data"]["g_pin"], components[i]["data"]["b_pin"], active_high=components[i]["data"]["active_high"]))
            dp_colors.append((255, 255, 255))
            dp_brightnesses.append(127)
            dp_states.append("OFF")
            dp_effects.append("None")
        elif components[i]["type"] == "SimpleOutput":
            so_state_topics.append(components[i]["data"]["state_topic"])
            so_command_topics.append(components[i]["data"]["command_topic"])
            so_online_topics.append(components[i]["data"]["online_topic"])
            so_current_states.append(components[i]["data"]["startup_state"])
            simpleouts.append(gpiozero.OutputDevice(components[i]["data"]["pin"], active_high=components[i]["data"]["active_high"], initial_value=components[i]["data"]["startup_state"]))
        elif components[i]["type"] == "CPUTemp":
            cput_state_topics.append(components[i]["data"]["state_topic"])
            cput_last_temps.append(0)
            cputemps.append(gpiozero.CPUTemperature())
        elif components[i]["type"] == "CPUUsage":
            cpuu_state_topics.append(components[i]["data"]["state_topic"])
            cpuu_last_values.append(0)
            cpuusages.append(None)
        elif components[i]["type"] == "DiskUsage":
            dsku_state_topics.append(components[i]["data"]["state_topic"])
            dsku_last_usages.append(0)
            diskusages.append(gpiozero.DiskUsage())
        elif components[i]["type"] == "SimpleInput":
            si_state_topics.append(components[i]["data"]["state_topic"])
            si_online_topics.append(components[i]["data"]["online_topic"])
            si_payloads.append((components[i]["data"]["high_payload"], components[i]["data"]["low_payload"]))
            si_before.append(None)
            if components[i]["data"]["bounce_time"] == 0:
                simpleins.append(gpiozero.DigitalInputDevice(components[i]["data"]["pin"], pull_up=components[i]["data"]["pull_up"]))
            else:
                simpleins.append(gpiozero.DigitalInputDevice(components[i]["data"]["pin"], pull_up=components[i]["data"]["pull_up"], bounce_time=components[i]["data"]["bounce_time"]))
        elif components[i]["type"] == "ADS1115_dev":
            ads = ADS.ADS1115(i2c, address=hex_str_to_hex(components[i]["data"]["addr"]))
            chan_0 = AnalogIn(ads, ADS.P0)
            chan_1 = AnalogIn(ads, ADS.P1)
            chan_2 = AnalogIn(ads, ADS.P2)
            chan_3 = AnalogIn(ads, ADS.P3)
            ads1115_devices.append((ads, components[i]["data"]["dev_id"], chan_0, chan_1, chan_2, chan_3))
        elif components[i]["type"] == "ADS1115_chan":
            ads1115_chans.append((components[i]["data"]["dev_id"], components[i]["data"]["chan"]))
            ads1115_chans_before.append(None)
            ads1115_chan_state_topics.append(components[i]["data"]["state_topic"])
            ads1115_chan_online_topics.append(components[i]["data"]["online_topic"])
            ads1115_chans_max.append(components[i]["data"]["max_value"])
            ads1115_chans_min.append(components[i]["data"]["min_value"])
        elif components[i]["type"] == "PCA9685_svo_dev":
            kit = ServoKit(channels=components[i]["data"]["channels"], address=hex_str_to_hex(components[i]["data"]["addr"]), frequency=components[i]["data"]["freq"])
            servokits.append((kit, components[i]["data"]["dev_id"]))
        elif components[i]["type"] == "PCA9685_svo_chan":
            svo_state_topics.append(components[i]["data"]["state_topic"])
            svo_online_topics.append(components[i]["data"]["online_topic"])
            svo_command_topics.append(components[i]["data"]["command_topic"])
            svo_current_states.append(components[i]["data"]["start_angle"])
            svo_devids.append(components[i]["data"]["dev_id"])
            svo_channels.append(components[i]["data"]["chan"])
            # set the servo to the startup state
            for j in range(len(servokits)):
                if servokits[j][1] == components[i]["data"]["dev_id"]:
                    servokits[j][0].servo[components[i]["data"]["chan"]].angle = components[i]["data"]["start_angle"]
                    client.publish(svo_state_topics[j], str(components[i]["data"]["start_angle"]))
        else:
            logging.warning("Unknown Device Type: {}".format(components[i]["type"]))

    for i in range(len(neopixels)):
        # change npd.json from driver
        npd = json.load(open(neopixels[i]))
        npd["state"] = "OFF"
        npd["brightness"] = 127
        npd["color"]["r"] = 255
        npd["color"]["g"] = 255
        npd["color"]["b"] = 255
        npd["effect"] = "None"
        json.dump(npd, open(neopixels[i], 'w')) # save the file

    for i in range(len(dumbpixels)):
        dp_threads.append(threading.Thread(target=dp_loop, args=(i,), daemon=True))
        dp_threads[i].start()

    logging.info("Found Components{}".format(components))
    logging.info("Loaded {} components".format(component_num))


if __name__ == "__main__":
    try:
        # connect to mqtt broker
        client = mqtt.Client()
        client.on_connect = on_connect
        client.on_message = on_message
        if config["mqtt"]["username"] != "":
            client.username_pw_set(config["mqtt"]["username"], config["mqtt"]["password"])

        client.connect(host, port, 60)
        init()
        # write online message
        for i in range(len(online_topics)):
            client.publish(online_topics[i], "online")
        # write state message to all neopixels
        for i in range(len(neopixels)):
            client.publish(np_state_topics[i], construct_message(), retain=False)
        # write state message to all dumbpixels
        for i in range(len(dumbpixels)):
            client.publish(dp_state_topics[i], construct_message())

        update_thread = threading.Thread(target=update_loop, daemon=True)
        update_thread.start()

        loop()
    except Exception as e:
        logging.error("Program experienced Exception: %s", e)
        try:
            # publish to online topics
            for i in range(len(online_topics)):
                client.publish(online_topics[i], "offline")

            client.disconnect()
        except Exception as e:
            logging.error("Program experienced Exception %s, while disconnecting", e)
    finally:
        logging.info("MQTTPi stopped")
        try:
            # publish to online topics
            for i in range(len(online_topics)):
                client.publish(online_topics[i], "offline")

            client.disconnect()
        except Exception as e:
            logging.error("Program experienced Exception %s, while disconnecting", e)
