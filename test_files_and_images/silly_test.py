import io
import numpy as np
import requests
from PIL import Image
from src.platform_control.stewart import StewartPlatform

port = ""
platform =  StewartPlatform(port)
platform.send_command("L") # turns on the laser
platform.send_command("MAXA 5") # sets the rotation scale to 1 deg
platform.send_command("MAXP 1") # sets the position scale to 1 mm

for i in range(20):
    # rotate platform
    x = np.random.rand() * 2 - 1
    platform.send_motion_command("RX " + str(x))

    # take picture
    r = requests.get("", timeout=10)
    r.raise_for_status()

    image = Image.open(io.BytesIO(r.content)).convert("RGB")
    image.save("test2/" + str(i) + ".png")

platform.close()