from src.platform_control.PICamStewart import PICamStewart

cam = PICamStewart("http://100.116.141.81:8000")

command = ""
cam.take_picture().save('picture.png')
while command != "stop":
    command = input("enter command: ")
    if command == "stop":
        continue
    else:
        try:
            print(cam.send_command(command))
        except:
            command = ""
    cam.take_picture().save('picture.png')

# take a picture right before close to make sure the laser is off
cam.take_picture().save('picture.png')
cam.close()