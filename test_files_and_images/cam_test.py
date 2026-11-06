import io
import numpy as np
import requests
from PIL import Image
from src.platform_control.camera import Camera

cam = Camera("")

cam.take_picture().save('test.bmp')