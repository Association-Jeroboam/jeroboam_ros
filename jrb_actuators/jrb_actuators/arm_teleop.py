#!/usr/bin/env python
# coding: utf-8

from .lib import custom_dxl_API as API
import os
import time

if os.name == "nt":
    import msvcrt

    def getch():
        return msvcrt.getch().decode()

else:
    import sys, tty, termios

    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)

    def getch():
        try:
            tty.setraw(sys.stdin.fileno())
            ch = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return ch


from dynamixel_sdk import *  # Uses Dynamixel SDK library
import RPi.GPIO as GPIO


def speed2rapportCyclique(speed):
    # speed entre -100 et 100
    # Si speed < 0 alors voltage A > voltage B
    return 0.025 * speed + 7


def main(args=None):
    channel_pompe = 12
    channel_vanne = 35
    frequence = 50
    GPIO.setmode(GPIO.BOARD)
    GPIO.setup(channel_pompe, GPIO.OUT)
    GPIO.setup(channel_vanne, GPIO.OUT)
    pPompe = GPIO.PWM(channel_pompe, frequence)
    pVanne = GPIO.PWM(channel_vanne, frequence)
    pPompe.start(speed2rapportCyclique(0))
    pVanne.start(speed2rapportCyclique(0))

    def startPump():
        pPompe.ChangeDutyCycle(speed2rapportCyclique(100))
        pVanne.ChangeDutyCycle(speed2rapportCyclique(0))

    def stopPump():
        pPompe.ChangeDutyCycle(speed2rapportCyclique(0))
        pVanne.ChangeDutyCycle(speed2rapportCyclique(100))
        time.sleep(0.5)
        pVanne.ChangeDutyCycle(speed2rapportCyclique(0))

    # Protocol version
    PROTOCOL_VERSION = 2.0  # See which protocol version is used in the Dynamixel

    # Default setting
    BAUDRATE = 57600  # Dynamixel default baudrate : 57600
    DEVICENAME = "/dev/ttyACM0"  # Check which port is being used on your controller
    # ex) Windows: "COM1"   Linux: "/dev/ttyUSB0" Mac: "/dev/tty.usbserial-*"

    API.initHandlers(DEVICENAME, BAUDRATE, PROTOCOL_VERSION)
    API.reboot(254)  # 254 for broadcast
    time.sleep(2)

    bras = API.bras(16, 14, 22, 1, 8, 2)

    # rateaux = API.rakes()
    bras.setTorque(1)

    # centre reservoir : 112.75 ; -22.5
    x = 112.75
    y = -22.5

    # startPump()

    # rateaux.setTorque(1)
    # rateaux.close()
    # bras.initSlider()

    while 1:
        print("Press any key to continue! (or press ESC to quit!)")
        value = getch()
        if value == chr(0x1B):
            break
        elif value == "q":
            x += 10
        elif value == "d":
            x -= 10
        elif value == "z":
            y -= 10
        elif value == "s":
            y += 10

        bras.setArmPosition(x, y)

        # bras.joinD.setLED(2)
        # bras.setTorque(1)
        # bras.slider.setGoalPosition(5000)
        # bras.joinD.setLED(4)

        # rateaux.open()
        # getch()
        # rateaux.close()

    # rateaux.setTorque(0)
    bras.setTorque(0)

    API.closePort()
    # stopPump()
    # pPompe.stop()
    # pVanne.stop()
