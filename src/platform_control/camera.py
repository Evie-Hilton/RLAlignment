import io
import numpy as np
import requests
from PIL import Image

"""
    creates a connection to the picamera and sets everything up
    so that creating an object of this type very easily allows
    taking a picture with it.
"""
class Camera:
    """
        Pass the url where the picamera server is hosted. this code
        has only been tested where the server is set up on a 
        raspberry pi and connection happens over a tailscale network.
    """
    def __init__(self, url):
        self.url = url
        self.session = requests.Session()

    """
        Returns a Pillow image object. This is so saving the image
        to disc instead of using it in ML is easy. simply call
        cam.take_picture().save("image_path.png") to take an image
        and save it to a location. 

        To turn the image into a numpy array, simply do:
        image = camera.take_picture()
        numpy_image = np.asarray(image)
    """
    def take_picture(self):
        r = self.session.get(self.url, timeout=10)
        r.raise_for_status()
        image = Image.open(io.BytesIO(r.content)).convert("RGB")
        return image

    """
        The connection between this code and the server remains
        open until this function is called, so make sure to call
        it at the end of your scripts. Keeping the channel open
        is for efficiency reasons.
    """
    def close(self):
        self.session.close()