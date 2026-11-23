from src.platform_control.PICamStewart import PICamStewart

cam = PICamStewart("http://100.116.141.81:8000")

cam.take_picture().save("picture.png")