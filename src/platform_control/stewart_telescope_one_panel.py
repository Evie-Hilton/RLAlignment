from src.telescope.telescope import Telescope
from src.platform_control.PICamStewart import PICamStewart
import yaml
import numpy as np

"""
    this stewart telescope is a raspberry pi connected to one stewart platform and one camera.
    this class allows you to talk to the raspberry pi.
"""
class StewartTelescope1(Telescope):
    """
        config must contain:
            - the url which connects to the pi containing the camera and platform
    """
    def __init__(self, config):
        telescope_config = self.unload_config(config)
        super().__init__(telescope_config)

        self.controller = PICamStewart(self.config["PIURL"])
        self.controller.send_command(f"MAXA {self.config["rotation_scale"]}") # set the scale of the rotation
        self.controller.send_command("L") # turn the laser on

        self.init_scatter = self.config["init_scatter"] # the amount of scatter after reset
        self.center_pix = self.config["center_pix"]

    """
        given a path to a telescope config file, it loads the file
        and sets all necessary items
    """
    def unload_config(self, path):
        with open(
            path,
            "r"
        ) as f:
            config = yaml.safe_load(f)
        return config

    """
        rotation looks like (rx, ry), where rx and ry are floats normalized between -1 and 1
    """
    def rotate_panel(self, rotation):
        self.controller.send_command(f"RX {rotation[0]}")
        self.controller.send_command(f"RY {rotation[1]}")

    def reset(self):
        self.controller.send_command("h")
        scat = np.random.rand(-self.init_scatter, self.init_scatter, 2)
        self.controller.send_command(f"RX {scat[0]}")
        self.controller.send_command(f"RY {scat[1]}")

    def update(self):
        self.image = self.process_image(np.asarray(self.controller.take_picture().convert("L"))) # turns the image into black and white and processes it before returning

    """
        image is a black and white PIL image with shape (height, width)
    """
    def process_image(self, image):
        u = self.center_pix[0]
        v = self.center_pix[1]
        left = int(round(u)) - self.img_size[0] // 2
        top = int(round(v)) - self.img_size[1] // 2
        cropped = image.crop((left, top, left + self.img_size[0], top + self.img_size[1]))
        return np.asarray(cropped)