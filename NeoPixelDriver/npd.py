import json
import time
import random
import math
from watchdog.observers import Observer
from watchdog.events import FileSystemEventHandler
import neopixel
import board

import light_funcs

# Define the number of NeoPixels and pin
num_pixels = 50
pixel_pin = board.D18  # Change this to the pin your NeoPixels are connected to

# Create NeoPixel object
pixels = neopixel.NeoPixel(pixel_pin, num_pixels, brightness=1.0, auto_write=False, pixel_order="RGB")

class AnimationState:
    def __init__(self):
        self.state = "OFF"
        self.color = {"r": 255, "g": 255, "b": 255}
        self.effect = "None"
        self.brightness = 0.0

# Replace this function with your animation logic
def update_animation(json_data, pixels, animation_state: AnimationState):
    animation_state.state = json_data["state"]
    animation_state.color = json_data["color"]
    animation_state.effect = json_data["effect"]
    animation_state.brightness = json_data["brightness"]

    print(f"Updating animation - State: {animation_state.state}, Color: {animation_state.color}, Effect: {animation_state.effect}, Brightness: {animation_state.brightness}")

# Define the file path to monitor
file_path = "npd.json"

# Set the desired FPS for your animation
slow_fps = 5
basic_fps = 30
regular_fps = 45
fast_fps = 60
ufast_fps = 120

animation_state = AnimationState()

# Load initial animation state
with open("npd.json", "r") as file:
    try:
        json_data = json.load(file)
        update_animation(json_data, pixels, animation_state)
    except json.JSONDecodeError as e:
        print(f"Error decoding JSON: {e}")

# Define the event handler for file changes
class MyHandler(FileSystemEventHandler):
    def on_modified(self, event):
        if not event.is_directory:
            with open(file_path, "r") as file:
                try:
                    json_data = json.load(file)
                    update_animation(json_data, pixels, animation_state)
                except json.JSONDecodeError as e:
                    print(f"Error decoding JSON: {e}")

# Animation-specific functions
def generate_color_pattern(length):
    colors = [
        (255, 0, 0),   # Red
        (0, 255, 0),   # Green
        (255, 255, 0), # Yellow
        (0, 0, 255),   # Blue
        (255, 165, 0)  # Orange
    ]

    pattern = []

    while len(pattern) < length:
        pattern.extend(colors)

    return pattern[:length]

def mix_colors(color1, color2, position):
    """
    Mix two RGB colors based on a position value.

    Parameters:
    - color1: Tuple representing the first RGB color (e.g., (255, 0, 0) for red).
    - color2: Tuple representing the second RGB color.
    - position: A value between 0 and 1 indicating the position between the two colors.

    Returns:
    - A tuple representing the resulting mixed color.
    """
    mixed_color = tuple(int((1 - position) * c1 + position * c2) for c1, c2 in zip(color1, color2))
    return mixed_color

def rindex(lst, value):
    lst.reverse()
    try:
        i = lst.index(value)
    except ValueError:
        return None
    lst.reverse()
    return len(lst) - i - 1


# Create the observer and start monitoring
event_handler = MyHandler()
observer = Observer()
observer.schedule(event_handler, path="./", recursive=False)
observer.start()

try:
    COLORS = [
        (255, 0, 0),   # Red
        (0, 255, 0),   # Green
        (255, 255, 0), # Yellow
        (0, 0, 255),   # Blue
        (255, 127, 0), # Orange
        (0, 0, 0)      # Off
    ]

    animation_step = 1
    previous_animation = ""

    fade_stage = 0
    swipe_stage = 0

    while True:
        if previous_animation != animation_state.effect: # reset animaton data
            pixels.fill((0, 0, 0))

            previous_animation = animation_state.effect
            animation_step = 1
            fade_stage = 0
            swipe_stage = 0

        # Set NeoPixels based on the "None" effect
        if animation_state.effect == "None" and animation_state.state == "ON":
            pixels.fill((animation_state.color["r"], animation_state.color["g"], animation_state.color["b"]))
            pixels.brightness = animation_state.brightness / 255.0
            time.sleep(1 / basic_fps)
        elif animation_state.effect == "Rainbow" and animation_state.state == "ON":
            for i in range(num_pixels):
                pixel_index = (i * 256 // num_pixels) + animation_step
                pixels[i] = light_funcs.wheel(pixel_index & 255)
            pixels.brightness = animation_state.brightness / 255.0
            time.sleep(1 / fast_fps)
        elif animation_state.effect == "GlitterRainbow" and animation_state.state == "ON":
            for i in range(num_pixels):
                pixel_index = (i * 256 // num_pixels) + animation_step
                pixels[i] = light_funcs.wheel(pixel_index & 255)
            pixels.brightness = animation_state.brightness / 255.0
            led = random.randint(0, num_pixels-1)
            pixels[led] = (255, 255, 255)
            time.sleep(1 / fast_fps)
        elif animation_state.effect == "Colorloop" and animation_state.state == "ON":
            pixels.fill(light_funcs.wheel(animation_step))
            pixels.brightness = animation_state.brightness / 255.0
            time.sleep(1 / fast_fps)
        elif animation_state.effect == "Magic" and animation_state.state == "ON":
            for i in range(num_pixels):
                pixel_index = (i * 256 // num_pixels) + animation_step
                color = float(math.sin(pixel_index / 4 - num_pixels))
                # convert the -1 to 1 to 110 to 180
                color = light_funcs.map_range(color, -1, 1, 120, 200)
                pixels[i] = light_funcs.wheel(int(color) & 255)
            pixels.brightness = animation_state.brightness / 255.0
            time.sleep(1 / basic_fps)
        elif animation_state.effect == "Fire" and animation_state.state == "ON":
            for i in range(num_pixels):
                pixel_index = (i * 256 // num_pixels) + animation_step
                color = float(math.sin(pixel_index / 4 - num_pixels))
                # convert the -1 to 1 to 110 to 180
                color = light_funcs.map_range(color, -1, 1, 70, 85)
                pixels[i] = light_funcs.wheel(int(color) & 255)
            pixels.brightness = animation_state.brightness / 255.0
            time.sleep(1 / regular_fps)
        elif animation_state.effect == "ColoredLights" and animation_state.state == "ON":
            for index, color in enumerate(generate_color_pattern(num_pixels)):
                pixels[index - 1] = color
            pixels.brightness = animation_state.brightness / 255.0
            time.sleep(1 / basic_fps)
        elif animation_state.effect == "Fade" and animation_state.state == "ON":
            if fade_stage == 0:
                pixels.fill((animation_state.color["r"] * animation_step // 255, animation_state.color["g"] * animation_step // 255, animation_state.color["b"] * animation_step // 255))
                pixels.show()
                if animation_step == 255:
                    fade_stage = 1
            else:
                pixels.fill((animation_state.color["r"] * (255 - animation_step) // 255, animation_state.color["g"] * (255 - animation_step) // 255, animation_state.color["b"] * (255 - animation_step) // 255))
                pixels.show()
                if animation_step == 255:
                    fade_stage = 0
            
            pixels.brightness = animation_state.brightness / 255.0
            time.sleep(1 / fast_fps)
        elif animation_state.effect == "FadeColorToWhite" and animation_state.state == "ON":
            if fade_stage == 0:
                for index, color in enumerate(generate_color_pattern(num_pixels)):
                    pixels[index - 1] = mix_colors((0, 0, 0), color, animation_step / 255)
                if animation_step == 255:
                    fade_stage = 1
            elif fade_stage == 1:
                for index, color in enumerate(generate_color_pattern(num_pixels)):
                    pixels[index - 1] = mix_colors(color, (0, 0, 0), animation_step / 255)
                if animation_step == 255:
                    fade_stage = 2
            elif fade_stage == 2:
                for index, color in enumerate(generate_color_pattern(num_pixels)):
                    pixels[index - 1] = mix_colors((0, 0, 0), (255, 255, 255), animation_step / 255)
                if animation_step == 255:
                    fade_stage = 3
            elif fade_stage == 3:
                for index, color in enumerate(generate_color_pattern(num_pixels)):
                    pixels[index - 1] = mix_colors((255, 255, 255), (0, 0, 0),  animation_step / 255)
                if animation_step == 255:
                    fade_stage = 0
            
            pixels.brightness = animation_state.brightness / 255.0
            time.sleep(1 / fast_fps)
        elif animation_state.effect == "FlashColorToWhite" and animation_state.state == "ON":
            if animation_step // 25 % 2:
                for index, color in enumerate(generate_color_pattern(num_pixels)):
                    pixels[index - 1] = color
            else:
                pixels.fill((255, 255, 255))
            
            pixels.brightness = animation_state.brightness / 255.0
            time.sleep(1 / basic_fps)
        elif animation_state.effect == "WipeRedToGreen" and animation_state.state == "ON":
            if swipe_stage == 0:
                last_pixel = rindex(list(pixels), [255, 0, 0])
                if last_pixel == None:
                    last_pixel = -1

                if last_pixel + 2 > num_pixels:
                    swipe_stage = 1
                else:
                    pixels[last_pixel + 1] = (255, 0, 0)
            else:
                last_pixel = rindex(list(pixels), [0, 255, 0])
                if last_pixel == None:
                    last_pixel = -1

                if last_pixel + 2 > num_pixels:
                    swipe_stage = 0
                else:
                    pixels[last_pixel + 1] = (0, 255, 0)

            pixels.brightness = animation_state.brightness / 255.0
            time.sleep(1 / fast_fps)
        elif animation_state.effect == "Random" and animation_state.state == "ON":
            for i in range(num_pixels):
                pixels[i] = (255, 255, 255) if random.randint(0, 1) == 1 else (0, 0, 0)

            pixels.brightness = animation_state.brightness / 255.0
            time.sleep(1 / slow_fps)
        elif animation_state.effect == "RandomColor" and animation_state.state == "ON":
            for i in range(num_pixels):
                pixels[i] = COLORS[random.randint(0, 5)]

            pixels.brightness = animation_state.brightness / 255.0
            time.sleep(1 / slow_fps)
        else: # off state / animation unknown
            pixels.fill((0, 0, 0))
            pixels.brightness = 0.0
            time.sleep(1 / basic_fps)
        
        pixels.show()
        animation_step += 1
        if animation_step > 255:
            animation_step = 1

except KeyboardInterrupt:
    observer.stop()

observer.join()
