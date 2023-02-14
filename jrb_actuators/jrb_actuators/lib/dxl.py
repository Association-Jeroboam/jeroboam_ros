import time
import math

connected_XL430=[]
connected_XL320=[]

def formatAngle(angle):
    angle = angle % 360
    if angle > 180:
        return angle - 360
    else:
        return angle

def setTorque4All(node, VALUE):
        #ID 254 = broadcast
        node.sendGenericCommand(1, 254, 24, VALUE) #pour les XL320
        node.sendGenericCommand(1, 254, 64, VALUE) #pour les XL430

class XL320():
    def __init__(
        self,
        node,
        DXL_ID,
        CW_Angle_Limit=0,
        CCW_Angle_Limit=1023,
        Moving_Speed=2047,
        Max_Torque=1023,
    ):
        self.ID = DXL_ID
        self.CW_Angle_Limit = 512         #to avoid crach for undefine if getModel is not working (due to ping error)
        self.CCW_Angle_Limit = 512        #to avoid crach for undefine if getModel is not working (due to ping error)
        self.offset=0
        self.torque=0
        self.reverseRotation=0
        self.node=node

#        if 350 == getModel(DXL_ID):
#            self.model_number = 350
        connected_XL320.append(DXL_ID)

#        else:
#            #print("Error : DXL with ID ", DXL_ID, " does not appear to be XL320")
#            return
        self.setTorque(0)
        self.setDriveMode(2)
        time.sleep(0.01)
        self.setMaxTorque(Max_Torque)
        time.sleep(0.01)
        self.setMaxSpeed(Moving_Speed)
        self.setPunch(50)

        self.setAngleLimits(CW_Angle_Limit, CCW_Angle_Limit)

#        self.goalPosition = self.getPresentPosition()
        self.setLED(3)

#        print("XL320 ", DXL_ID, " was well initialized (temp:",self.getPresentTemperature(),"°C  Voltage:",self.getPresentVoltage(),"V)")

    def setPunch(self, punch):
        self.node.sendGenericCommand(2, self.ID, 51, punch)

    def setLED(self, value):
        self.node.sendGenericCommand(2, self.ID, 25, value)

    def setReverseRotation(self,value):
        self.reverseRotation=value
        self.setAngleLimits(self.CW_Angle_Limit, self.CCW_Angle_Limit)
        self.node.get_logger().info(f"XL320 with ID {self.ID} has its rotation reversed.")


    def setHomingOffset(self, offset):
        oldOffset = self.offset
        self.offset=offset
        torque=self.torque
        self.setTorque(0)
        self.setAngleLimits(self.CW_Angle_Limit-oldOffset,self.CCW_Angle_Limit-oldOffset)
        self.setTorque(torque)
        self.node.get_logger().info(f"Homing offset set to {offset} for ID {self.ID} (CW={self.CW_Angle_Limit}, CCW={self.CCW_Angle_Limit})")

    def setAngleLimits(self, CW_Angle_Limit, CCW_Angle_Limit):
        torque=self.torque
        self.setTorque(0)
        CW_Angle_Limit+=self.offset
        CCW_Angle_Limit+=self.offset

        if CW_Angle_Limit > CCW_Angle_Limit:
            self.node.get_logger().warning(f"Warning for ID {self.ID}: CW_Angle_Limit>CCW_Angle_Limit (=> auto reverse)")
            tmp=CW_Angle_Limit
            CW_Angle_Limit=CCW_Angle_Limit
            CCW_Angle_Limit=tmp

        if CW_Angle_Limit < 0:
            self.node.get_logger().error(f"Error for ID {self.ID}: CW_Angle_Limit<0")
            CW_Angle_Limit = 0
        if CCW_Angle_Limit > 1023:
            self.node.get_logger().error(f"Error for ID {self.ID}: CCW_Angle_Limit>1023")
            CCW_Angle_Limit = 1023

        self.CW_Angle_Limit = CW_Angle_Limit
        self.CCW_Angle_Limit = CCW_Angle_Limit

        if self.reverseRotation :
            tmp=CW_Angle_Limit
            CW_Angle_Limit=1023-CCW_Angle_Limit
            CCW_Angle_Limit=1023-tmp

        self.node.sendGenericCommand(2, self.ID, 6, CW_Angle_Limit)
        time.sleep(0.005)
        self.node.sendGenericCommand(2, self.ID, 8, CCW_Angle_Limit)

        self.setTorque(torque)





    def setMaxTorque(self, Max_Torque):
        if Max_Torque < 0:
            self.node.get_logger().error(f"Error for ID {self.ID}: Max_Torque<0")
            Max_Torque = 0
        if Max_Torque > 1023:
            self.node.get_logger().error(f"Error for ID {self.ID}: Max_Torque>1023")
            Max_Torque = 1023

        self.maxTorque = Max_Torque
        self.node.sendGenericCommand(2, self.ID, 15, Max_Torque)

    def setMaxSpeed(self, Max_Speed):
        if Max_Speed < 0:
            self.node.get_logger().error(f"Error for ID {self.ID}: Max_Speed<0")
            Max_Speed = 0
        if Max_Speed > 2047:
            self.node.get_logger().error(f"Error for ID {self.ID}: Max_Speed>2047")
            Max_Speed = 2047

        self.maxSpeed = Max_Speed

        self.node.sendGenericCommand(2, self.ID, 32, Max_Speed)

    def setPositionPID(self, P, I, D):
        self.node.sendGenericCommand(1, self.ID, 29, P)
        self.node.sendGenericCommand(1, self.ID, 28, I)
        self.node.sendGenericCommand(1, self.ID, 27, D)

        self.pid_P = P
        self.pid_I = I
        self.pid_D = D

#    def printPositionPID(self):
#        P = readValue(portHandler, 2, self.ID, 29)
#        I = readValue(portHandler, 2, self.ID, 28)
#        D = readValue(portHandler, 2, self.ID, 27)
#        print("P:", P, "  I:", I, "  D:", D)

    def setGoalPosition(self, POSITION):
        POSITION+=self.offset

        if self.CW_Angle_Limit > POSITION:
            self.node.get_logger().warning(F"ID {self.ID}: Position goal ({POSITION}) < CW_Angle_Limit ({self.CW_Angle_Limit})")
            POSITION = self.CW_Angle_Limit
        elif self.CCW_Angle_Limit < POSITION:
            self.node.get_logger().warning(f"ID {self.ID}: Position goal ({POSITION}) > CCW_Angle_Limit ({self.CCW_Angle_Limit})")
            POSITION = self.CCW_Angle_Limit
        self.goalPosition = POSITION
        if self.reverseRotation :
            POSITION=1023-POSITION
        self.node.sendGenericCommand(2, self.ID, 30, int(POSITION))
        self.setTorque(
            self.torque
        )  # Updating the goal_position register seems to enable torque, so this line releases the motor if it haven't to be enabled

#    def getPresentPosition(self):
#        dxl_present_position = readValue(portHandler, 2, self.ID, 37)
#        if self.reverseRotation :
#            dxl_present_position=1023-dxl_present_position
#        dxl_present_position-=self.offset
#        return dxl_present_position
#
#    def getPresentTemperature(self):
#        present_temp = readValue(portHandler, 1, self.ID, 46)
#        return present_temp
#
#    def getPresentVoltage(self):
#        present_voltage=readValue(portHandler, 1, self.ID, 45)/10.0
#        return present_voltage

#    def reboot(self):
#        packetHandler.reboot(portHandler, self.ID)

    def setDriveMode(self, MODE):
        self.node.sendGenericCommand(1, self.ID, 11, MODE)  # XL430 : 1:Velocity  3:Position  4:Extended position  16:PWM
        self.driveMode = MODE

    def setTorque(self, VALUE):
        self.torque = VALUE
        self.node.sendGenericCommand(1, self.ID, 24, VALUE)
        if VALUE : self.setLED(6)
        else : self.setLED(7)


#    def getPresentVelocity(self):
#        #todo
#        pass
    
#    def waitMoveEnd(self,timeout):
#        t_speed = time.time() + timeout  # timeout en secondes
#        speed = 0
#        while t_speed > time.time() :
#            lastspeed = speed
#            speed = self.getPresentVelocity()
#            if speed == 0 and lastspeed != 0:
#                return 1  # arret après un front descendant sur speed
#            time.sleep(0.1)
#        return 0



class bras:
    def __init__(self, node, side, ID_A, ID_B, ID_C, ID_D, ID_E, ID_Slider):
        if side != "left" and side != "right" :
            self.node.get_logger().error("Bras : side doit être ""left"" ou ""right""")
            return
        self.node=node
        self.side=side
        self.joinA = XL320(node, ID_A, 360, 700, 160)
        self.joinB = XL320(node, ID_B, 512 - 333, 512 + 333, 250)
        self.joinC = XL320(node, ID_C, 160, 580, 450)
        self.joinD = XL320(node, ID_D, 205, 819, 500)
        self.joinE = XL320(node, ID_E,0,1023,150)

        #self.slider = XL430(node, ID_Slider)

        if side=="right" :
            self.joinA.setAngleLimits(1023-self.joinA.CCW_Angle_Limit,1023-self.joinA.CW_Angle_Limit)
            self.joinB.setAngleLimits(1023-self.joinB.CCW_Angle_Limit,1023-self.joinB.CW_Angle_Limit)
            self.joinC.setReverseRotation(1)
            self.joinD.setReverseRotation(1)
            # self.joinE.setReverseRotation(1)
            
#            self.slider.setReverseRotation(1)

#        self.slider.setDriveMode(4)

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
            self.node.get_logger().warning(f"Point trop distant pour être atteint par le bras ({x_sucker};{y_sucker})")
            return []

        a = 2 * x_sucker
        b = 2 * y_sucker
        c = h1**2 - h2**2 + x_sucker**2 + y_sucker**2
        d = (2 * a * c) ** 2 - 4 * (a**2 + b**2) * (c**2 - b**2 * h1**2)

        if d < 0 :
            self.node.get_logger().error(f"d<0 donc sqrt impossible. a={a}  b={b}  c={c}  d={d}  x_sucker={x_sucker}  y_sucker={y_sucker}")
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
            self.node.get_logger().warning(f"Aucune solution trouvée pour positionner la ventouse en ({x_sucker};{y_sucker})")
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
#        self.slider.setTorque(value)

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
            self.node.get_logger().error(f"Join inconnu : {join}")

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
                #print("Join B : angle=",angle,"  value=",value,"  CW=",self.joinB.CW_Angle_Limit,"  CCW=",self.joinB.CCW_Angle_Limit)
                return 0
            else:
                return 1
        else:
            self.node.get_logger().error(f"Join inconnu : {join}")

    def initSlider(self, positionEnButeeHaute=0):

        self.setTorque(0)

        speed_setup = self.slider.maxSpeed
        self.slider.setMaxSpeed(500)

        self.slider.setHomingOffset(0)  # reset de l'offset pour avoir presentPosition=ActualPosition
        self.node.get_logger().debug("reset offset done")
        offset = self.slider.getPresentPosition()  # actual position
        self.node.get_logger().debug(f"actual position ={offset}")
        self.slider.setHomingOffset(offset)  # offset = actualPosition => presentPosition=0

        #self.slider.setTorque(1)
        #self.slider.setGoalPosition(-10000)
        #self.slider.waitMoveEnd(5)

        pos = self.slider.getPresentPosition()
        self.node.get_logger().debug(f"final pos ={pos}")
        self.setTorque(0)
        self.node.get_logger().debug(f"calculated offset={(positionEnButeeHaute - (pos - offset))}")
        self.slider.setHomingOffset(positionEnButeeHaute - (pos - offset))
        # self.node.get_logger().debug(f"Final Position={self.slider.getPresentPosition()}")

        self.slider.setMaxSpeed(speed_setup)



    def setSliderPosition_mm(self, mm):
        value = int(-36.93191489 * mm + 8679)
        self.slider.setGoalPosition(value)

    def getSlidePosition_mm(self):
        return (self.slider.getPresentPosition() - 8679) * (1 / -36.93191489)

def mirrorAngle(angle):
    return 1024-angle

class rakes:
    def __init__(self, node, ID_gauche_bas=7, ID_droit_bas=15, ID_gauche_haut=5, ID_droit_haut=18):
        self.node = node
        self.gaucheB = XL320(node,ID_gauche_bas, 520 - 220, 500, 300)
        self.droitB = XL320(node,ID_droit_bas, 590, 570 + 220, 300)

        self.gaucheH = XL320(node,ID_gauche_haut, mirrorAngle(500), mirrorAngle(520 - 220), 300)
        self.droitH = XL320(node,ID_droit_haut, mirrorAngle(570 + 220), mirrorAngle(590), 300)

        self.droitH.setHomingOffset(55)

    def setTorque(self, value):
        self.gaucheB.setTorque(value)
        self.droitB.setTorque(value)
        self.droitH.setTorque(value)
        self.gaucheH.setTorque(value)
        self.torque = value

    def open(self):
        self.node.get_logger().info("Opening rakes...")
        self.gaucheB.setGoalPosition(471)
        self.gaucheH.setGoalPosition(mirrorAngle(471))
        self.droitB.setGoalPosition(607)
        self.droitH.setGoalPosition(mirrorAngle(607))


    def close(self):
        self.node.get_logger().info("Closing rakes...")
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

#        self.gaucheB.setGoalPosition(self.gaucheB.getPresentPosition())
#        self.droitB.setGoalPosition(self.droitB.getPresentPosition())
#        self.gaucheH.setGoalPosition(self.gaucheH.getPresentPosition())
#        self.droitH.setGoalPosition(self.droitH.getPresentPosition())

        time.sleep(0.1)
        self.setTorque(torqueSetup)