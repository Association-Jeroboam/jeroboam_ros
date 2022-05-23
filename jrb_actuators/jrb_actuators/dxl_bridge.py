#!/usr/bin/env python
# -*- coding: utf-8 -*-

import time
import math

from jrb_msgs.msg import (ServoStatus, ServoConfig, ServoAngle, SliderConfig, SliderPosition, SliderStatus)

def formatAngle(angle):
    angle = angle % 360
    if angle > 180:
        return angle - 360
    else:
        return angle

def bitfield(n):
    return [1 if digit=='1' else 0 for digit in bin(n)[2:].zfill(8)]

###################################################

sub_xl320_status = self.create_subscription(ServoStatus, "servo_status", self.xl320_status_cb, 10)
pub_xl320_target = self.create_publisher(ServoAngle, "servo_angle_target", 10)
pub_xl320_config = self.create_publisher(ServoConfig, "servo_config", 10)
xl320_status={}
sub_xl430_status = self.create_subscription(SliderStatus, "slider_status", self.430_status_cb, 10)
pub_xl430_target = self.create_publisher(SliderPosition, "slider_position_target", 10)
pub_xl430_config = self.create_publisher(SliderConfig, "slider_config", 10)
xl430_status={}

def xl320_status_cb(self, msg: ServoStatus):
    if not msg.id in xl320_status :
        xl320_status[msg.id] = ServoStatus()
    xl320_status[msg.id] = msg

def xl430_status_cb(self, msg: SliderStatus):
    if not msg.id in xl430_status :
        xl430_status[msg.id] = SliderStatus()
    xl430_status[msg.id] = msg

class XL320:
    def __init__(
        self,
        DXL_ID,
        CW_Angle_Limit=0,
        CCW_Angle_Limit=1023,
        Moving_Speed=2047,
        Max_Torque=1023,
        PID=[32,0,0]
    ):
        self.ID = DXL_ID
        self.offset=0
        self.torque=0
        self.reverseRotation=0

        self.CW_Angle_Limit = CW_Angle_Limit
        self.CCW_Angle_Limit = CCW_Angle_Limit

        self.servo_config_msg = ServoConfig()
        self.servo_order_msg = ServoAngle()

        self.setConfig(Max_Torque,Moving_Speed,PID)

        self.setTorque(0)
        # self.setDriveMode(2)
        # time.sleep(0.01)
        #self.setMaxTorque(Max_Torque)
        # time.sleep(0.01)
        # self.setMaxSpeed(Moving_Speed)
        # self.setPunch(50)

        # self.setAngleLimits(CW_Angle_Limit, CCW_Angle_Limit)

        # self.goalPosition = self.getPresentPosition()
        self.setLED(4)

        print("XL320 ", DXL_ID, " was well initialized (temp:",self.getPresentTemperature(),"°C  Voltage:",self.getPresentVoltage(),"V)")

    def setConfig(self,torque_limit,moving_speed,pid):
        self.servo_config_msg.id = self.ID
        self.servo_config_msg.torque_limit = torque_limit
        self.servo_config_msg.moving_speed = moving_speed
        self.servo_config_msg.pid.pid = pid
        self.servo_config_msg.pid.bias = 0 # ?
        self.servo_config_msg.cw_angle_limit = self.CW_Angle_Limit
        self.servo_config_msg.ccw_angle_limit = self.CCW_Angle_Limit
        self.sendConfig()

    def sendConfig(self):
        pub_xl320_config.publish(self.servo_config_msg)

    def setReverseRotation(self,value):
        self.reverseRotation=value
        self.setAngleLimits(self.CW_Angle_Limit, self.CCW_Angle_Limit)
        print("XL320 with ID",self.ID,"has its rotation reversed.")

    def setPunch(self, punch):
        #todo
        # writeValue(portHandler, 2, self.ID, 51, punch)
        pass

    def setLED(self, value):
        #todo
        pass

    def setHomingOffset(self, offset):
        oldOffset = self.offset
        self.offset=offset
        torque=self.torque
        self.setTorque(0)
        self.setAngleLimits(self.CW_Angle_Limit-oldOffset,self.CCW_Angle_Limit-oldOffset)
        self.setTorque(torque)
        print("Homing offset set to", offset,"for ID",self.ID,"(CW=",self.CW_Angle_Limit,", CCW=",self.CCW_Angle_Limit,")")

    def setAngleLimits(self, CW_Angle_Limit, CCW_Angle_Limit):
        torque=self.torque
        self.setTorque(0)
        CW_Angle_Limit+=self.offset
        CCW_Angle_Limit+=self.offset

        if CW_Angle_Limit > CCW_Angle_Limit:
            print("Warning for ID", self.ID, ": CW_Angle_Limit>CCW_Angle_Limit (=> auto reverse)")
            tmp=CW_Angle_Limit
            CW_Angle_Limit=CCW_Angle_Limit
            CCW_Angle_Limit=tmp

        if CW_Angle_Limit < 0:
            print("Error for ID", self.ID, ": CW_Angle_Limit<0")
            CW_Angle_Limit = 0
        if CCW_Angle_Limit > 1023:
            print("Error for ID", self.ID, ": CCW_Angle_Limit>1023")
            CCW_Angle_Limit = 1023

        self.CW_Angle_Limit = CW_Angle_Limit
        self.CCW_Angle_Limit = CCW_Angle_Limit

        if self.reverseRotation :
            tmp=CW_Angle_Limit
            CW_Angle_Limit=1023-CCW_Angle_Limit
            CCW_Angle_Limit=1023-tmp

        self.servo_config_msg.CW_Angle_Limit = self.CW_Angle_Limit
        self.servo_config_msg.CCW_Angle_Limit = self.CCW_Angle_Limit
        self.sendConfig()
        self.setTorque(torque)


    def setMaxTorque(self, Max_Torque):
        if Max_Torque < 0:
            print("Error for ID", self.ID, ": Max_Torque<0")
            Max_Torque = 0
        if Max_Torque > 1023:
            print("Error for ID", self.ID, ": Max_Torque>1023")
            Max_Torque = 1023

        self.servo_config_msg.torque_limit = Max_Torque
        self.sendConfig()

    def setMaxSpeed(self, Max_Speed):
        if Max_Speed < 0:
            print("Error for ID", self.ID, ": Max_Speed<0")
            Max_Speed = 0
        if Max_Speed > 2047:
            print("Error for ID", self.ID, ": Max_Speed>2047")
            Max_Speed = 2047

        self.servo_config_msg.moving_speed = Max_Speed
        self.sendConfig()

    def setPositionPID(self, P, I, D):
        self.servo_config_msg.pid.pid = [P,I,D]
        self.sendConfig()
        # self.pid_P = P
        # self.pid_I = I
        # self.pid_D = D

    def setGoalPosition(self, POSITION):
        POSITION+=self.offset

        if self.CW_Angle_Limit > POSITION:
            print(
                "Error for ID",
                self.ID,
                ": Position goal (",
                POSITION,
                ") < CW_Angle_Limit (",
                self.CW_Angle_Limit,
                ")",
            )
            POSITION = self.CW_Angle_Limit
        elif self.CCW_Angle_Limit < POSITION:
            print(
                "Error for ID",
                self.ID,
                ": Position goal (",
                POSITION,
                ") > CCW_Angle_Limit (",
                self.CCW_Angle_Limit,
                ")",
            )
            POSITION = self.CCW_Angle_Limit
        self.goalPosition = POSITION
        if self.reverseRotation :
            POSITION=1023-POSITION
        self.servo_order_msg.radian=(math.radians(300)/1024)*POSITION
        pub_xl320_target.publish(self.servo_order_msg)
        self.setTorque(
            self.torque
        )

    def getPresentPosition(self):
        dxl_present_position = xl320_status[self.ID].angle
        if self.reverseRotation :
            dxl_present_position=1023-dxl_present_position
        dxl_present_position-=self.offset
        return dxl_present_position

    def getPresentTemperature(self):
        return xl320_status[self.ID].temperature

    def setTorque(self, VALUE):
        # self.torque = VALUE
        # writeValue(portHandler, 1, self.ID, 24, VALUE)
        # if VALUE : self.setLED(2)
        # else : self.setLED(7)

        #todo
        pass

    def getPresentVelocity(self):
        return xl320_status[self.ID].speed
    
    def waitMoveEnd(self,timeout):
        t_speed = time.time() + timeout  # timeout en secondes
        speed = 0
        while t_speed > time.time() :
            lastspeed = speed
            speed = self.getPresentVelocity()
            if speed == 0 and lastspeed != 0:
                return 1  # arret après un front descendant sur speed
            time.sleep(0.1)
        return 0

class XL430:
    def __init__(
        self,
        DXL_ID,
    ):
        self.ID = DXL_ID
        if not DXL_ID in xl430_status
            print(
                "Error for ID",
                self.ID,
                ": DXL with ID ",
                DXL_ID,
                " does not appear to be XL430",
            )
        self.setTorque(0)

        self.slider_config_msg = SliderConfig()
        self.setConfig(10,[640,0,4000])

        print("XL430 ", DXL_ID, " was well initialized (temp:",self.getPresentTemperature(),"°C  Voltage:",self.getPresentVoltage(),"V)")

    def setConfig(self,speed_limit,pid):
        self.slider_config_msg.id = self.ID
        self.slider_config_msg.speed_limit = speed_limit
        self.slider_config_msg.speed_pid.pid = [100,1000,0]
        self.slider_config_msg.position_pid.pid = pid
        self.slider_config_msg.feedforward_gains = [0,0]
        self.sendConfig()

    def sendConfig(self):
        pub_xl430_config.publish(self.slider_config_msg)

    def getPresentTemperature(self):
        return xl430_status[self.ID].temperature

    def setMaxSpeed(self, Max_Speed):
        self.maxSpeed = Max_Speed
        if Max_Speed < 0:
            print("Error for ID", self.ID, ": Max_Speed<0")
            Max_Speed = 0
        if Max_Speed > 4095:
            print("Error for ID", self.ID, ": Max_Speed>4095")
            Max_Speed = 4095
        self.slider_config_msg.speed_limit=Max_Speed
        self.sendConfig()

    def setPositionPID(self, P, I, D):
        self.slider_config_msg.position_pid.pid=[P,I,D]
        self.sendConfig()
        
    # def setGoalPosition(self, POSITION):
    #     if self.driveMode == 3:
    #         if self.CW_Angle_Limit > POSITION:
    #             print(
    #                 "Error for ID",
    #                 self.ID,
    #                 ": Position goal (",
    #                 POSITION,
    #                 ") < CW_Angle_Limit (",
    #                 self.CW_Angle_Limit,
    #                 ")",
    #             )
    #             POSITION = self.CW_Angle_Limit
    #         elif self.CCW_Angle_Limit < POSITION:
    #             print(
    #                 "Error for ID",
    #                 self.ID,
    #                 ": Position goal (",
    #                 POSITION,
    #                 ") > CCW_Angle_Limit (",
    #                 self.CCW_Angle_Limit,
    #                 ")",
    #             )
    #             POSITION = self.CCW_Angle_Limit

    #     writeValue(portHandler, 4, self.ID, 116, POSITION)
    #     self.setTorque(
    #         self.torque
    #     )  # Updating the goal_position register seems to enable torque, so this line releases the motor if it haven't to be enabled

    # def setGoalSpeed(self, SPEED):
    #     if self.driveMode == 1:
    #         writeValue(portHandler, 4, self.ID, 104, SPEED)
    #         self.setTorque(
    #             self.torque
    #         )  # Updating the goal_position register seems to enable torque, so this line releases the motor if it haven't to be enabled
    #     else:
    #         print("You can't set speed goal if you are not in Velocity Mode")

    def getPresentPosition_mm(self):
        dxl_present_position = xl430_status[self.ID].position
        return dxl_present_position

    def getPresentVelocity(self):
        dxl_present_velocity = xl430_status[self.ID].speed
        return dxl_present_velocity

    # def reboot(self):
    #     packetHandler.reboot(portHandler, self.ID)

    # def setReverseRotation(self,value):
    #     if value :
    #         driveModeValue=5
    #     else :
    #         driveModeValue=4
    #     writeValue(portHandler, 1, self.ID, 10, driveModeValue)        
    #     self.reverseRotation = value
    #     print("XL430 with ID",self.ID,"has its rotation reversed.")


    # def setDriveMode(self, MODE): #dans la doc c'est pas appelé driveMode mais operatingMode car driveMode est utilisé pour autre chose
    #     writeValue(
    #         portHandler, 1, self.ID, 11, MODE
    #     )  # XL320 : 1: wheel mode   2: joint mode   // XL430 : 1:Velocity  3:Position  4:Extended position  16:PWM
    #     self.driveMode = MODE

    def setTorque(self, VALUE):
        # self.torque = VALUE
        # writeValue(portHandler, 1, self.ID, 64, VALUE)

        #todo
        pass


    def waitMoveEnd(self,timeout):
        t_speed = time.time() + timeout  # timeout en secondes
        speed = 0
        while t_speed > time.time() :
            lastspeed = speed
            speed = self.getPresentVelocity()
            if speed == 0 and lastspeed != 0:
                return 1  # arret après un front descendant sur speed
            time.sleep(0.1)
        return 0

class bras:
    def __init__(self, side, ID_A, ID_B, ID_C, ID_D, ID_E, ID_Slider):
        if side != "left" and side != "right" :
            print("Bras : side doit être ""left"" ou ""right""")
            return

        self.side=side
        self.joinA = XL320(ID_A, 360, 700, 160)
        self.joinB = XL320(ID_B, 512 - 333, 512 + 333, 250)
        self.joinC = XL320(ID_C, 160, 580, 450)
        self.joinD = XL320(ID_D, 205, 819, 500)
        self.joinE = XL320(ID_E,0,1023,150)

        self.slider = XL430(ID_Slider)

        if side=="right" :
            self.joinA.setAngleLimits(1023-self.joinA.CCW_Angle_Limit,1023-self.joinA.CW_Angle_Limit)
            self.joinB.setAngleLimits(1023-self.joinB.CCW_Angle_Limit,1023-self.joinB.CW_Angle_Limit)
            self.joinC.setReverseRotation(1)
            self.joinD.setReverseRotation(1)
            # self.joinE.setReverseRotation(1)
            self.slider.setReverseRotation(1)

        self.slider.setDriveMode(4)

        # self.joinD.setPunch(32)
        # self.joinD.setPunch(70)

        self.joinA.setPositionPID(60, 10, 0)
        self.joinB.setPositionPID(60, 25, 0)
        self.joinC.setPositionPID(70, 30, 0)
        self.joinD.setPositionPID(120, 30, 0)
        self.joinE.setPositionPID(70, 30, 0)

    def xy2angles(self,x_sucker,y_sucker):
        h1 = 54  # entraxe A et B
        h2 = 91  # entraxe B et ventouse
        if math.sqrt(x_sucker**2 + y_sucker**2) > (h1 + h2):
            print(
                "Point trop distant pour être atteint par le bras (",
                x_sucker,
                ";",
                y_sucker,
                ")",
            )
            return []

        a = 2 * x_sucker
        b = 2 * y_sucker
        c = h1**2 - h2**2 + x_sucker**2 + y_sucker**2
        d = (2 * a * c) ** 2 - 4 * (a**2 + b**2) * (c**2 - b**2 * h1**2)

        if d < 0 :
            print("Erreur : d<0 donc sqrt impossible. a=",a,"  b=",b,"  c=",c,"  d=",d,"  x_sucker=",x_sucker,"  y_sucker=",y_sucker)
            return []
        xB1 = (2 * a * c - math.sqrt(d)) / (2 * (a**2 + b**2))
        xB2 = (2 * a * c + math.sqrt(d)) / (2 * (a**2 + b**2))

        if y_sucker == 0:
            yB1 = b / 2 + math.sqrt(h2**2 - ((2 * c - a**2) / (2 * a)) ** 2)
            yB2 = b / 2 - math.sqrt(h2**2 - ((2 * c - a**2) / (2 * a)) ** 2)
        else:
            yB1 = (c - a * xB1) / b
            yB2 = (c - a * xB2) / b

        angle_A1 = math.atan2(yB1, xB1)
        angle_A2 = math.atan2(yB2, xB2)
        angle_B1 = math.atan2(y_sucker - yB1, x_sucker - xB1) - angle_A1
        angle_B2 = math.atan2(y_sucker - yB2, x_sucker - xB2) - angle_A2

        angle_A1 = formatAngle(math.degrees(angle_A1))
        angle_A2 = formatAngle(math.degrees(angle_A2))
        angle_B1 = formatAngle(math.degrees(angle_B1))
        angle_B2 = formatAngle(math.degrees(angle_B2))

        if self.isReachableAngle("A", angle_A1) and self.isReachableAngle("B", angle_B1):
            if self.isReachableAngle("A", angle_A2) and self.isReachableAngle("B", angle_B2):
                # les 2 positions sont atteignable
                if self.side=="left" :
                    if angle_A1 > angle_A2:
                        angle_A = angle_A1
                        angle_B = angle_B1
                    else:
                        angle_A = angle_A2
                        angle_B = angle_B2
                
                else :
                    if angle_A1 > angle_A2:
                        angle_A = angle_A2
                        angle_B = angle_B2
                    else:
                        angle_A = angle_A1
                        angle_B = angle_B1


            else:  # seulement position 1 atteignable
                angle_A = angle_A1
                angle_B = angle_B1

        elif self.isReachableAngle("A", angle_A2) and self.isReachableAngle("B", angle_B2):  # seulement position 2 atteignable
            angle_A = angle_A2
            angle_B = angle_B2

        else:
            print(
                "Aucune solution trouvée pour positionner la ventouse en (",
                x_sucker,
                ";",
                y_sucker,
                ")",
            )
            return []
        return [angle_A,angle_B]

    def setArmPosition(self, x_sucker, y_sucker, angle_C=0, angle_D=90, angle_E=0):
        angles=self.xy2angles(x_sucker, y_sucker)
        if angles :        
            self.goToAngle("A", angles[0])
            self.goToAngle("B", angles[1])
            self.goToAngle("C", angle_C)
            self.goToAngle("D", angle_D)
            self.goToAngle("E", angle_E)

    def setTorque(self, value):
        self.joinA.setTorque(value)
        self.joinB.setTorque(value)
        self.joinC.setTorque(value)
        self.joinD.setTorque(value)
        self.joinE.setTorque(value)
        self.slider.setTorque(value)

    def getState(self):
        pos_to_rad = math.radians(300) / 1024

        joints = []

        joints.append(self.joinA.getPresentPosition())
        time.sleep(0.001)
        joints.append(self.joinB.getPresentPosition())
        time.sleep(0.001)
        joints.append(self.joinC.getPresentPosition())
        time.sleep(0.001)
        joints.append(self.joinD.getPresentPosition())
        time.sleep(0.001)
        joints.append(self.joinE.getPresentPosition())
        time.sleep(0.001)

        return [self.getSlidePosition_mm() / 1000] + [pos * pos_to_rad for pos in joints]

    def goToAngle(self, join, angle):
        ratio = 614 / 180
        if join == "A":
            self.joinA.setGoalPosition(1023 - (angle * ratio + (512 - 307)))
        elif join == "B":
            self.joinB.setGoalPosition(angle * ratio + 512)
        elif join == "C":
            self.joinC.setGoalPosition(angle * ratio + (512 - 307))
        elif join == "D":
            self.joinD.setGoalPosition(angle * ratio + 512)
        elif join == "E":
        #     alpha = 180 - ((self.joinA.goalPosition - (512 - 307)) / ratio)
        #     beta = (self.joinB.goalPosition - 512) / ratio
        #     alpha = formatAngle(alpha)
        #     beta = formatAngle(beta)
        #     angle = (angle - alpha - beta) % 360
        #     angle = formatAngle(angle)
        #     value=1023 - (angle * ratio + 512+307)
        #     self.joinE.setGoalPosition(value)
            self.joinE.setGoalPosition(angle * ratio + 512)


        else:
            print("Join inconnu :", join)

    def isReachableAngle(self, join, angle):
        ratio = 614 / 180
        if join == "A":
            value = 1023 - (angle * ratio + (512 - 307))
            if self.joinA.CW_Angle_Limit > value or self.joinA.CCW_Angle_Limit < value:
                #print("Join A : angle=",angle,"  value=",value,"  CW=",self.joinA.CW_Angle_Limit,"  CCW=",self.joinA.CCW_Angle_Limit)
                return 0
            else:
                return 1
        elif join == "B":
            value = angle * ratio + 512
            if self.joinB.CW_Angle_Limit > value or self.joinB.CCW_Angle_Limit < value:
                # print("Join B : angle=",angle,"  value=",value,"  CW=",self.joinB.CW_Angle_Limit,"  CCW=",self.joinB.CCW_Angle_Limit)
                return 0
            else:
                return 1
        else:
            print("Join inconnu :", join)

    # def initSlider(self, positionEnButeeHaute=0):

    #     self.setTorque(0)

    #     speed_setup = self.slider.maxSpeed
    #     self.slider.setMaxSpeed(500)

    #     self.slider.setHomingOffset(0)  # reset de l'offset pour avoir presentPosition=ActualPosition
    #     print("reset offset done")
    #     offset = self.slider.getPresentPosition()  # actual position
    #     print("actual position =",offset)
    #     self.slider.setHomingOffset(offset)  # offset = actualPosition => presentPosition=0

    #     self.slider.setTorque(1)

    #     self.slider.setGoalPosition(-10000)

    #     self.slider.waitMoveEnd(5)

    #     pos = self.slider.getPresentPosition()
    #     print("final pos =",pos)
    #     self.setTorque(0)
    #     print("calculated offset=",(positionEnButeeHaute - (pos - offset)))
    #     self.slider.setHomingOffset(positionEnButeeHaute - (pos - offset))
    #     # print("Final Position=",self.slider.getPresentPosition())

    #     self.slider.setMaxSpeed(speed_setup)


    def setSliderPosition_mm(self, mm):
        value = int(-36.93191489 * mm + 8679)
        self.slider.setGoalPosition(value)

    # def getSlidePosition_mm(self):
    #     return (self.slider.getPresentPosition() - 8679) * (1 / -36.93191489)

def mirrorAngle(angle):
    return 1024-angle

class rakes:
    def __init__(self, ID_gauche_bas=7, ID_droit_bas=15, ID_gauche_haut=5, ID_droit_haut=18):
        self.gaucheB = XL320(ID_gauche_bas, 520 - 220, 500, 300)
        self.droitB = XL320(ID_droit_bas, 590, 570 + 220, 300)

        self.gaucheH = XL320(ID_gauche_haut, mirrorAngle(500), mirrorAngle(520 - 220), 300)
        self.droitH = XL320(ID_droit_haut, mirrorAngle(570 + 220), mirrorAngle(590), 300)

        self.droitH.setHomingOffset(55)

    def setTorque(self, value):
        self.gaucheB.setTorque(value)
        self.droitB.setTorque(value)
        self.droitH.setTorque(value)
        self.gaucheH.setTorque(value)
        self.torque = value

    def open(self):
        self.gaucheB.setGoalPosition(471)
        self.gaucheH.setGoalPosition(mirrorAngle(471))
        self.droitB.setGoalPosition(607)
        self.droitH.setGoalPosition(mirrorAngle(607))


    def close(self):
        torqueSetup = self.torque

        self.setTorque(0)
        self.gaucheB.setPositionPID(100, 30, 10)
        self.droitB.setPositionPID(100, 30, 10)
        self.gaucheH.setPositionPID(100, 30, 10)
        self.droitH.setPositionPID(100, 30, 10)
        self.setTorque(1)

        self.gaucheB.setGoalPosition(300)
        self.gaucheH.setGoalPosition(mirrorAngle(300))
        self.droitB.setGoalPosition(790)
        self.droitH.setGoalPosition(mirrorAngle(790))

        time.sleep(0.3)

        # self.setTorque(0)
        self.gaucheB.setPositionPID(20, 0, 0)
        self.droitB.setPositionPID(20, 0, 0)
        self.gaucheH.setPositionPID(20, 0, 0)
        self.droitH.setPositionPID(20, 0, 0)
        # self.setTorque(1)
        self.open()
        time.sleep(0.3)

        self.gaucheB.setGoalPosition(300)
        self.gaucheH.setGoalPosition(mirrorAngle(300))
        self.droitB.setGoalPosition(790)
        self.droitH.setGoalPosition(mirrorAngle(790))
        time.sleep(0.3)

        self.gaucheB.setGoalPosition(self.gaucheB.getPresentPosition())
        self.droitB.setGoalPosition(self.droitB.getPresentPosition())
        self.gaucheH.setGoalPosition(self.gaucheH.getPresentPosition())
        self.droitH.setGoalPosition(self.droitH.getPresentPosition())

        time.sleep(0.1)
        self.setTorque(torqueSetup)