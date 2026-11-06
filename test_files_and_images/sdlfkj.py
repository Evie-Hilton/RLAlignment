from src.platform_control.camera import Camera
from src.platform_control.stewart import StewartPlatform

port = ""
platform =  StewartPlatform(port)

cam = Camera("")

platform.send_command("L")

cam.take_picture().save("test5.bmp")

platform.close()
cam.close()