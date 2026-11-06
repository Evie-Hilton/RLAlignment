from src.platform_control.stewart import StewartPlatform

port = ""
platform =  StewartPlatform(port)

while True:
    platform.send_command("L")
    _ = input()
    platform.send_command("l")
    _ = input()