import time
import math
import numpy as np
from rcl_interfaces.msg import ParameterDescriptor, ParameterType, IntegerRange

CONTROL_MODE_WHEEL = 1
CONTROL_MODE_JOINT = 2

rawToRad_XL320=math.radians(300)/1023
rawToRad_XL430=256*(2*math.pi)/1048575
mmToRaw = 36.93191489 #pour les sliders


connected_XL320={}
connected_XL430={}

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

def mirrorAngle(angle):
    return 1024-angle

def toUnsigned(n,base):
    if(n<0) : n2 = (n+2**base)
    else : n2=n
    return n2

def toSigned(n,base):
    if base == 32 :
        n1 = n & 0xffffffff
        n2 = n1 | (-(n1 & 0x80000000))
    elif base == 16 :
        n1 = n & 0xffff
        n2 = n1 | (-(n1 & 0x8000))
    elif base == 8 :
        n1 = n & 0xffff
        n2 = n1 | (-(n1 & 0x8000))
    else :
        print(f"toSigned : error base ({base})")
    return n2

class XL320:
    def __init__(
        self,
        node,
        DXL_ID,
        CW_Angle_Limit=0,
        CCW_Angle_Limit=1023,
        Max_Speed=1023,
        Max_Torque=1023,
        Drive_Mode=CONTROL_MODE_JOINT
    ):
        self.ID = DXL_ID
        self.offset=0
        self.torque=0
        self.reverseRotation=False
        self.node=node
        self.target_position=-1
        self.current_position=-1
        self.target_reached= False
        self.driveMode = Drive_Mode
        self.maxSpeed = Max_Speed
        self.maxTorque = Max_Torque
        self.punch = 50
        self.CW_Angle_Limit = CW_Angle_Limit
        self.CCW_Angle_Limit = CCW_Angle_Limit
        self.pid_P = 32
        self.pid_I = 0
        self.pid_D = 0
        self.isReady=False

        self.sendConfig()

    def ping(self):
        if self.node.pingDXL(self.ID) != 350 :
            self.node.get_logger().error(f"DXL with ID {self.ID} not detected")
            return 0
        else :
            connected_XL320[self.ID] = self
            self.node.get_logger().info(f"XL320 {self.ID} was well pinged (temp:{self.getPresentTemperature()}°C  Voltage:{self.getPresentVoltage()}V)")
            return 1


    def sendConfig(self):
        self.isReady=False
        if not self.ping() : return
        self.setTorque(0)
        self.setDriveMode(self.driveMode)
        self.setMaxSpeed(self.maxSpeed)
        self.setPunch(self.punch)
        self.setAngleLimits(self.CW_Angle_Limit, self.CCW_Angle_Limit)
        self.setPositionPID(self.pid_P,self.pid_I,self.pid_D)
        self.setHomingOffset(self.offset)
        self.setReverseRotation(self.reverseRotation)
        self.setMaxTorque(self.maxTorque)
        self.setLED(2)
        self.isReady=True
   
    def sendVolatileConfig(self):
        self.isReady=False
        if not self.ping() : return
        self.setTorque(0)
        self.setMaxTorque(self.maxTorque)
        self.setMaxSpeed(self.maxSpeed)
        self.setPunch(self.punch)
        self.setPositionPID(self.pid_P,self.pid_I,self.pid_D)
        self.setLED(2)
        self.isReady=True


    def saveCurrentPosition(self, position, isRadian=False):
        if isRadian :
            position=int(position/rawToRad_XL320)
            
        if position < 0 :
            if position != -1 :
                self.node.get_logger().error(f"DXL {self.ID} : current_position cannot be negative ({position})")
            return
        
        if self.reverseRotation :
            position=1023-position

        self.current_position=position
        if self.target_position == -1 :
            self.target_position = position #initialiser la target à la position actuelle si aucune target n'a déjà été demandée

        # check if target reached
        if abs( self.current_position - self.target_position ) < 10 :
            self.target_reached=True
        else :
            #self.node.get_logger().info(f"For ID {self.ID} : current_position ({self.current_position}) - target_position ({self.target_position}) ={abs( self.current_position - self.target_position )}")
            pass

    #def setRadianState(self,radian):
    #    if radian < 0 :
    #        self.node.get_logger().error(f"DXL {self.ID} : current_position can't be negative ({radian})")
    #        return
    #    self.current_position= int(radian/rawToRad_XL320)
    #    #self.node.get_logger().info(f"XL320 with ID {self.ID} has its current_position set to {self.current_position}.")
    #    if abs( self.current_position - self.target_position ) < 0.08*rawToRad_XL320 :
    #        self.target_reached=True
    #        #self.node.get_logger().info(f"XL320 with ID {self.ID} has its target reached ({self.current_position}).")
    #    else :
    #        self.node.get_logger().info(f"For ID {self.ID} : current_position - target_position ={abs( self.current_position - self.target_position )}")
    #        pass

    def setPunch(self, punch):
        self.node.sendGenericCommand(2, self.ID, 51, punch)
        self.punch = punch

    def setLED(self, value):
        self.node.sendGenericCommand(2, self.ID, 25, value)

    def setReverseRotation(self,value):
        self.reverseRotation=value
        if value :
            self.setAngleLimits(self.CW_Angle_Limit, self.CCW_Angle_Limit)
            #self.node.get_logger().info(f"XL320 with ID {self.ID} has its rotation reversed.")

    def setHomingOffset(self, offset):
        oldOffset = self.offset
        self.offset=offset
        torque=self.torque
        self.setTorque(0)
        self.setAngleLimits(self.CW_Angle_Limit-oldOffset,self.CCW_Angle_Limit-oldOffset)
        self.setTorque(torque)
        #self.node.get_logger().info(f"Homing offset set to {offset} for ID {self.ID} (CW={self.CW_Angle_Limit}, CCW={self.CCW_Angle_Limit})")

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
        self.node.sendGenericCommand(2, self.ID, 8, CCW_Angle_Limit)

        self.setTorque(torque)

    def setMaxTorque(self, Max_Torque):
        if self.torque :
            self.node.get_logger().error(F"Impossible to set MaxTorque when torque is set")
        else : 
            if Max_Torque < 0:
                self.node.get_logger().error(f"Error for ID {self.ID}: Max_Torque<0")
                Max_Torque = 0
            if Max_Torque > 1023:
                self.node.get_logger().error(f"Error for ID {self.ID}: Max_Torque>1023")
                Max_Torque = 1023

            self.maxTorque = Max_Torque
            self.node.sendGenericCommand(2, self.ID, 15, Max_Torque)

    def setMaxSpeed(self, Max_Speed):
        if self.driveMode == CONTROL_MODE_JOINT :
            if Max_Speed < 0:
                self.node.get_logger().error(f"Error for ID {self.ID}: Max_Speed<0")
                Max_Speed = 0
            elif Max_Speed > 1023:
                self.node.get_logger().error(f"Error for ID {self.ID}: Max_Speed>1023")
                Max_Speed = 1023
            self.node.sendGenericCommand(2, self.ID, 32, Max_Speed)
            #self.node.get_logger().info(f"XL320 {self.ID}: Max_Speed set to {Max_Speed} = {self.node.readValue(2, self.ID, 32)}")
        self.maxSpeed = Max_Speed

    def setPositionPID(self, P, I, D):
        self.node.sendGenericCommand(1, self.ID, 29, P)
        self.node.sendGenericCommand(1, self.ID, 28, I)
        self.node.sendGenericCommand(1, self.ID, 27, D)

        self.pid_P = P
        self.pid_I = I
        self.pid_D = D

    def setGoalPosition(self, POSITION):
        POSITION=int(POSITION)
        POSITION+=self.offset
        if self.CW_Angle_Limit > POSITION:
            self.node.get_logger().warning(F"ID {self.ID}: Position goal ({POSITION}) < CW_Angle_Limit ({self.CW_Angle_Limit})")
            POSITION = self.CW_Angle_Limit
        elif self.CCW_Angle_Limit < POSITION:
            self.node.get_logger().warning(f"ID {self.ID}: Position goal ({POSITION}) > CCW_Angle_Limit ({self.CCW_Angle_Limit})")
            POSITION = self.CCW_Angle_Limit
        if self.reverseRotation :
            POSITION=1023-POSITION

        self.target_reached=False

        self.target_position=POSITION

        #if self.reverseRotation :
        #    self.target_position=1023-self.target_position

        self.node.sendGenericCommand(2, self.ID, 30, POSITION)
        #self.setTorque(self.torque)  # Updating the goal_position register seems to enable torque, so this line releases the motor if it haven't to be enabled

    def setGoalSpeed(self, SPEED, reverse=False):
        if self.driveMode != CONTROL_MODE_WHEEL :
            self.node.get_logger().warn(F"ID {self.ID}: Goal speed received but DriveMode is not set to Wheel Mode")
            return
        if SPEED > 1023 : SPEED = 1023
        elif SPEED < 0 : SPEED = 0
        if SPEED > self.maxSpeed :
            self.node.get_logger().warn(F"ID {self.ID}: Goal speed received > maxSpeed configured ({SPEED} > {self.maxSpeed})") 
            SPEED=self.maxSpeed
        if reverse ^ self.reverseRotation : SPEED += 1024 # ^ : ou exclusif
        
        #self.node.get_logger().info(F"setGoalSpeed : id={self.ID} / SPEED={SPEED} / reverse={reverse}") 
        self.node.sendGenericCommand(2, self.ID, 32, SPEED)
        if self.torque : self.target_reached = True
  
    def getPresentPosition(self):
        dxl_present_position = self.node.readValue(2, self.ID, 37)
        if dxl_present_position < 0 :   
            if dxl_present_position == -1 :
                return -1
            self.node.get_logger().error(f"DXL {self.ID} : current_position cannot be negative ({dxl_present_position})")

        if self.reverseRotation :
            dxl_present_position=1023-dxl_present_position
            
        dxl_present_position-=self.offset
        self.saveCurrentPosition(dxl_present_position)

        #LED
        if dxl_present_position < self.CW_Angle_Limit or dxl_present_position > self.CCW_Angle_Limit :
            self.setLED(5) #violet
        else :
            self.setLED(3) #jaune

        return dxl_present_position
    
    def getPresentTemperature(self):
        present_temp = self.node.readValue(1, self.ID, 46)
        return present_temp

    def getPresentVoltage(self):
        present_voltage=self.node.readValue(1, self.ID, 45)/10.0
        return present_voltage
    
    def reboot(self):
        self.node.sendRebootCommand(self.ID)

    def setDriveMode(self, MODE):
        #self.node.get_logger().warn(F"ID {self.ID}: setDriveMode to {MODE}")
        if self.torque :
            self.node.get_logger().error(F"Impossible to set DriveMode when torque is set")
        else : 
            self.driveMode = MODE
            self.node.sendGenericCommand(1, self.ID, 11, MODE)  # XL320 : 1:Wheel Mode 2:Joint Mode
            if MODE == CONTROL_MODE_WHEEL :
                self.node.sendGenericCommand(2, self.ID, 32, 0) #Setting Moving_speed to 0

    #def getDriveMode(self, MODE):
    #    #self.node.get_logger().warn(F"ID {self.ID}: setDriveMode to {MODE}")
    #    self.node.readValue(2, self.ID, 11)
    #
    #    if self.torque :
    #        self.node.get_logger().error(F"Impossible to set DriveMode when torque is set")
    #    else : 
    #        self.driveMode = MODE
    #        self.node.sendGenericCommand(1, self.ID, 11, MODE)  # XL320 : 1:Wheel Mode 2:Joint Mode
    #        if MODE == CONTROL_MODE_WHEEL :
    #            self.node.sendGenericCommand(2, self.ID, 32, 0) #Setting Moving_speed to 0

    def setTorque(self, VALUE):
        self.torque = VALUE
        self.node.sendGenericCommand(1, self.ID, 24, VALUE)
        #if VALUE : self.setLED(6)
        #else : self.setLED(7)

class XL430:
    def __init__(
        self,
        node,
        DXL_ID,
        CW_Angle_Limit=0,
        driveMode=4,
        CCW_Angle_Limit=4095,
        maxSpeed=885,
    ):
        self.node=node
        self.ID = DXL_ID
        self.driveMode = driveMode
        self.CW_Angle_Limit = CW_Angle_Limit
        self.CCW_Angle_Limit = CCW_Angle_Limit
        self.maxSpeed = maxSpeed
        self.P = 640
        self.I = 0
        self.D = 4000
        self.reverseRotation = False
        self.offset = 0
        self.isReady = False
        self.target_reached = False
        self.target_position=-1
        self.current_position=-1

        self.sendConfig()

    def sendConfig(self):
        self.isReady=False
        if not self.ping() :
            return
        self.isReady=False
        self.setTorque(0)
        self.setDriveMode(self.driveMode)
        if self.driveMode == 3:
            self.setAngleLimits(self.CW_Angle_Limit, self.CCW_Angle_Limit)
        self.setMaxSpeed(self.maxSpeed)
        self.setPositionPID(self.P, self.I, self.D)
        self.setReverseRotation(self.reverseRotation)
        self.setHomingOffset(self.offset)
        self.setLED(1)
        self.isReady=True

    def ping(self):
        if self.node.pingDXL(self.ID) != 1060 :
            self.node.get_logger().error(f"DXL with ID {self.ID} not detected")
            return 0
        else :
            connected_XL430[self.ID] = self
            self.node.get_logger().info(f"XL430 {self.ID} was well pinged (temp:{self.getPresentTemperature()}°C  Voltage:{self.getPresentVoltage()}V)")
            return 1

    def setAngleLimits(self, CW_Angle_Limit, CCW_Angle_Limit):
        if self.driveMode != 3:
            self.node.get_logger().warn(f"Warning for ID {self.ID} : Angle limits are not used for selected drive mode ({self.driveMode})")
            return

        if CW_Angle_Limit > CCW_Angle_Limit:
            self.node.get_logger().error(f"Error for ID {self.ID} : CW_Angle_Limit>CCW_Angle_Limit")
            return
        if CW_Angle_Limit < 0:
            self.node.get_logger().warn(f"Error for ID {self.ID} : CW_Angle_Limit<0")
            CW_Angle_Limit = 0
        if CCW_Angle_Limit > 4095:
            self.node.get_logger().warn(f"Error for ID {self.ID} : CCW_Angle_Limit>4095")
            CCW_Angle_Limit = 4095

        self.node.sendGenericCommand(4, self.ID, 52, CW_Angle_Limit)
        self.node.sendGenericCommand(4, self.ID, 48, CCW_Angle_Limit)

        self.CW_Angle_Limit = CW_Angle_Limit
        self.CCW_Angle_Limit = CCW_Angle_Limit

    def setMaxSpeed(self, Max_Speed): #885=100% de la PWM
        if Max_Speed < 0:
            self.node.get_logger().warn(f"Error for ID {self.ID} : Max_Speed<0")
            Max_Speed = 0
        if Max_Speed > 885:
            self.node.get_logger().warn(f"Error for ID {self.ID} : Max_Speed>885")
            Max_Speed = 885
        
        self.maxSpeed = Max_Speed

        self.node.sendGenericCommand(4, self.ID, 36, Max_Speed)

    def setPositionPID(self, P, I, D):
        self.P = P
        self.I = I
        self.D = D
        #self.node.get_logger().info(f"ID {self.ID} : Setting PID ({P},{I},{D})")
        self.node.sendGenericCommand(2, self.ID, 78, P)
        self.node.sendGenericCommand(2, self.ID, 76, I)
        self.node.sendGenericCommand(2, self.ID, 80, D)

    def printPositionPID(self):
        P = self.node.readValue(2, self.ID, 78)
        I = self.node.readValue(2, self.ID, 76)
        D = self.node.readValue(2, self.ID, 80)
        self.node.get_logger().info(f"P: {P} I: {I} D: {D}")

    def getPresentTemperature(self):
        return self.node.readValue(1, self.ID, 146)

    def getPresentVoltage(self):
        return self.node.readValue(2, self.ID, 144)/10
        
    def getPresentVelocity(self):
        dxl_present_velocity = self.node.readValue(4, self.ID, 128)
        return dxl_present_velocity
       
    def getPresentLoad(self):
        dxl_present_load = self.node.readValue(2, self.ID, 126)
        self.node.get_logger().info(f"load = {dxl_present_load}")
        return dxl_present_load

    def printTorqueEnable(self, debug=""):
        torqueEnable = self.node.readValue(1, self.ID, 64)
        self.node.get_logger().info(f"{debug} torqueEnable={torqueEnable}")

    def getPresentPosition(self):
        dxl_present_position = self.node.readValue(4, self.ID, 132)

        #dxl_present_position-=self.offset
        self.saveCurrentPosition(dxl_present_position)
        return dxl_present_position

    def saveCurrentPosition(self, position, isRadian=False):
        if isRadian :
            position=int(position/rawToRad_XL430)
            
        self.current_position=position
        if self.target_position == -1 :
            self.target_position = position #initialiser la target à la position actuelle si aucune target n'a déjà été demandée

        # check if target reached
        if abs( self.current_position - self.target_position ) < 20 :
            #print(self.current_position - self.target_position)
            self.target_reached=True
        else :
            #self.node.get_logger().info(f"For ID {self.ID} : current_position ({self.current_position}) - target_position ({self.target_position}) ={abs( self.current_position - self.target_position )}")
            pass

    def setGoalPosition(self, POSITION):
        if self.driveMode == 3:
            if self.CW_Angle_Limit > POSITION:
                self.node.get_logger().warn(f"Error for ID {self.ID} : Position goal ({POSITION}) < CW_Angle_Limit ({self.CW_Angle_Limit})")
                POSITION = self.CW_Angle_Limit
            elif self.CCW_Angle_Limit < POSITION:
                self.node.get_logger().warn(f"Error for ID {self.ID} : Position goal ({POSITION}) > CCW_Angle_Limit ({self.CCW_Angle_Limit})")
                POSITION = self.CCW_Angle_Limit

        self.target_reached=False
        self.node.sendGenericCommand(4, self.ID, 116, POSITION)
        self.target_position=POSITION

        self.setTorque(
            self.torque
        )  # Updating the goal_position register seems to enable torque, so this line releases the motor if it haven't to be enabled

    def setGoalSpeed(self, SPEED):
        if self.driveMode == 1:
            self.node.sendGenericCommand(4, self.ID, 104, SPEED)
            self.target_reached=True
            self.setTorque(
                self.torque
            )  # Updating the goal_position register seems to enable torque, so this line releases the motor if it haven't to be enabled
        else:
            self.node.get_logger().error("You can't set speed goal if you are not in Velocity Mode")

    def reboot(self):
        self.node.sendRebootCommand(self.ID)

    def setReverseRotation(self,value):
        if value :
            driveModeValue=5
        else :
            driveModeValue=4
        self.node.sendGenericCommand(1, self.ID, 10, driveModeValue)        
        self.reverseRotation = value
        #self.node.get_logger().info(f"XL430 with ID {self.ID} has its rotation reversed.")

    def setDriveMode(self, MODE): #dans la doc c'est pas appelé driveMode mais operatingMode car driveMode est utilisé pour autre chose
        self.node.sendGenericCommand(1, self.ID, 11, MODE)  # XL320 : 1: wheel mode   2: joint mode   // XL430 : 1:Velocity  3:Position  4:Extended position  16:PWM
        self.driveMode = MODE

    def setTorque(self, VALUE):
        self.torque = VALUE
        self.node.sendGenericCommand(1, self.ID, 64, VALUE)

    def setLED(self, VALUE):
        self.node.sendGenericCommand(1, self.ID, 65, VALUE)

    def setHomingOffset(self, offset):
        # if self.reverseRotation :
        #     offset=1044479-offset
        if offset > 1044479 :
            self.node.get_logger().warn(f"Erreur : offset ({offset}) trop grand ( > 1044479 ) pour XL430 avec ID {self.ID}")
            return
        elif offset < -1044479 :
            self.node.get_logger().warn(f"Erreur : offset ({offset}) trop petit ( < -1044479 ) pour XL430 avec ID {self.ID}")
            return

        self.offset = offset

        self.node.sendGenericCommand(4, self.ID, 20, offset)

    def waitMoveEnd(self,timeout):
        t_speed = time.time() + timeout  # timeout en secondes
        speed = 1
        lastspeed = 0
        while t_speed > time.time() :
            self.node.get_logger().debug(f"waitMoveEnd : speed : {speed} last_speed : {lastspeed}")
            if speed == 0 and lastspeed != 0:  #arret après un front descendant sur speed
                return 1
            else :
                time.sleep(0.1)
                lastspeed = speed
                speed = self.getPresentVelocity()
        return 0

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
        self.slider = XL430(node, ID_Slider)

        if side=="right" :
            self.joinA.setAngleLimits(1023-self.joinA.CCW_Angle_Limit,1023-self.joinA.CW_Angle_Limit)
            self.joinB.setAngleLimits(1023-self.joinB.CCW_Angle_Limit,1023-self.joinB.CW_Angle_Limit)
            self.joinC.setReverseRotation(True)
            self.joinD.setReverseRotation(True)
            # self.joinE.setReverseRotation(True)            
            self.slider.setReverseRotation(True)

        self.slider.setDriveMode(4)

        # self.joinD.setPunch(32)
        # self.joinD.setPunch(70)

        self.joinA.setPositionPID(60, 20, 0)
        self.joinB.setPositionPID(60, 35, 0)
        self.joinC.setPositionPID(70, 30, 0)
        self.joinD.setPositionPID(120, 30, 0)
        self.joinE.setPositionPID(70, 30, 0)
        self.slider.setPositionPID(1000, 1000, 1000)

    def getState(self):
        joints = []
        joints.append(self.joinA.current_position)
        joints.append(self.joinB.current_position)
        if self.side == "right":
            joints.append(1023-self.joinC.current_position)
            joints.append(1023-self.joinD.current_position)
        else :
            joints.append(self.joinC.current_position)
            joints.append(self.joinD.current_position)
        joints.append(self.joinE.current_position)

        return [(self.getSliderPosition_mm()+15) / 1000] + [pos * rawToRad_XL320 for pos in joints]

    def putArmOnDisk(self,x,y):
        #calcul du meilleur point sur le cercle où placer la ventouse
        #self.node.get_logger().warn(f"putArmDisk at x={x} y={y}")

        for point in self.points_sur_cercle(x,y,33,36):
            angles = self.xy2angles(point[0],point[1],False)
            if angles:
                dist=sum(angles)
                if "closest_point" in locals() :
                    if closest_point[3] > dist :
                        closest_point=[point[0],point[1],point[2],dist]
                else:
                    closest_point=[point[0],point[1],point[2],dist]
        
        if not "closest_point" in locals() :
            #self.node.get_logger().warn(f"No point reachable")
            return 0

        #closest_point=[x,y,0]

        #positionnement de la ventouse
        #self.node.get_logger().info(f"Point retenu : x={closest_point[0]}, y={closest_point[1]}")
        self.setArmPosition(closest_point[0],closest_point[1])
        self.setAbsoluteVentouseAngle(closest_point[2])
        end_time = time.time() + 2 #timeout
        while not self.isTargetReached :
            if time.time() > end_time :
                self.node.get_logger().warn(f"Timeout to put arm {self.side} on disk (x,y)")
                return 0
            time.sleep(0.01)
            
        #positionnement du slider
        end_time = time.time() + 2 #timeout
        self.setSliderPosition_mm(10)
        while not self.isTargetReached :
            if time.time() > end_time :
                self.node.get_logger().warn(f"Timeout to put arm {self.side} on disk (slider)")
                return 0
            time.sleep(0.01)

        return 1

        return 1

    def points_sur_cercle(self,X, Y, R, nb_points):
        points = []
        for angle in np.arange(0,360,360/nb_points):
            # Calcul des coordonnées du point
            x = X + R * math.cos(math.radians(angle))
            y = Y + R * math.sin(math.radians(angle))

            # Ajout des coordonnées du point au tableau
            points.append([x, y, angle])

        return points    

    def xy2angles(self,x_sucker,y_sucker,verbose=True):
        h1 = 54  # entraxe A et B
        h2 = 91  # entraxe B et ventouse

        x_sucker=round(x_sucker,2)
        y_sucker=round(y_sucker,2)

        if math.sqrt(x_sucker**2 + y_sucker**2) > (h1 + h2):
            if verbose : self.node.get_logger().warning(f"Point trop distant pour être atteint par le bras ({x_sucker};{y_sucker})")
            return []

        a = 2 * x_sucker
        b = 2 * y_sucker
        c = h1**2 - h2**2 + x_sucker**2 + y_sucker**2
        d = (2 * a * c) ** 2 - 4 * (a**2 + b**2) * (c**2 - b**2 * h1**2)

        if d < 0 :
            self.node.get_logger().error(f"d<0 donc sqrt impossible. a={a}  b={b}  c={c}  d={d}  x_sucker={x_sucker}  y_sucker={y_sucker}")
            return []
        
        if a+b == 0 :
            self.node.get_logger().error(f"a+b=0 donc imposible d'effectuer la division")
            return []

        xB1 = (2 * a * c - math.sqrt(d)) / (2 * (a**2 + b**2))
        xB2 = (2 * a * c + math.sqrt(d)) / (2 * (a**2 + b**2))

        if y_sucker == 0:
            temp=(h2**2 - ((2 * c - a**2) / (2 * a)) ** 2)
            if temp < 0 :
                self.node.get_logger().error(f"sqrt impossible sur nombre negatif ({temp}). a={a}  b={b}  c={c}  d={d}  x_sucker={x_sucker}  y_sucker={y_sucker}")
                return []
            yB1 = b / 2 + math.sqrt(temp)
            yB2 = b / 2 - math.sqrt(temp)
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
            if verbose : self.node.get_logger().warning(f"Aucune solution trouvée pour positionner la ventouse en ({x_sucker};{y_sucker})")
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

    def getAngles(self):
        joints = []
        joints.append(self.joinA.current_position)
        joints.append(self.joinB.current_position)
        joints.append(self.joinC.current_position)
        joints.append(self.joinD.current_position)
        joints.append(self.joinE.current_position)
        return [self.getSliderPosition_mm() / 1000] + [pos * rawToRad_XL320 for pos in joints]

    def getTargetAngles(self):
        joints = []
        joints.append(self.joinA.target_position)
        joints.append(self.joinB.target_position)
        joints.append(self.joinC.target_position)
        joints.append(self.joinD.target_position)
        joints.append(self.joinE.target_position)
        return [self.getSliderPosition_mm() / 1000] + [pos * rawToRad_XL320 for pos in joints]
    
    def getGapAngles(self):
        joints = []
        joints.append(abs(self.joinA.target_position-self.joinA.current_position))
        joints.append(abs(self.joinB.target_position-self.joinB.current_position))
        joints.append(abs(self.joinC.target_position-self.joinC.current_position))
        joints.append(abs(self.joinD.target_position-self.joinD.current_position))
        joints.append(abs(self.joinE.target_position-self.joinE.current_position))
        return [self.getSliderPosition_mm() / 1000] + [pos * rawToRad_XL320 for pos in joints]

    def getJoinStatus(self):
        joints = []
        joints.append(bool(self.slider.target_reached))
        joints.append(self.joinA.target_reached)
        joints.append(self.joinB.target_reached)
        joints.append(self.joinC.target_reached)
        joints.append(self.joinD.target_reached)
        joints.append(self.joinE.target_reached)
        return joints

    def isTargetReached(self):
        if(self.slider.target_reached and self.joinA.target_reached and self.joinB.target_reached and self.joinC.target_reached and self.joinD.target_reached and self.joinE.target_reached):
            return True
        else:
            return False
        
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

    def setAbsoluteVentouseAngle(self, angle):
        #A tester
        ratio = 614 / 180
        alpha = 180 - ((self.joinA.target_position - (512 - 307)) / ratio)
        beta = (self.joinB.target_position - 512) / ratio
        alpha = formatAngle(alpha)
        beta = formatAngle(beta)
        angle = (angle - alpha - beta) % 360
        angle = formatAngle(angle)
        value=1023 - (angle * ratio + 512+307)
        self.goToAngle('E',int((value-512)/ratio))

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

    def initSlider(self, positionEnButeeBasse=0):
        if not self.slider.isReady :
            return 0
        self.slider.setTorque(0)

        speed_setup = self.slider.maxSpeed
        self.slider.setMaxSpeed(500)

        self.slider.setHomingOffset(0)  
        
        self.slider.setTorque(1)
        self.slider.setGoalPosition(1000000)
        self.slider.waitMoveEnd(6)

        pos = self.slider.getPresentPosition()
        self.slider.setTorque(0)

        offset=positionEnButeeBasse - pos
        if self.slider.reverseRotation :
            offset=-offset
        self.slider.setHomingOffset(offset)
        self.node.get_logger().info(f"Offset for slider {self.side} set to {offset}")

        self.slider.setMaxSpeed(speed_setup)
        self.slider.setTorque(1)

        return 1


    def setSliderPosition_mm(self, mm):
        value = -int(mmToRaw * mm)
        self.slider.setGoalPosition(value)

    def getSliderPosition_mm(self):
        return -int(self.slider.current_position / mmToRaw)

class rakes:
    def __init__(self, node, ID_gauche_bas=7, ID_droit_bas=15, ID_gauche_haut=5, ID_droit_haut=18):
        self.node = node
        self.gaucheB = XL320(node,ID_gauche_bas, 520 - 220, 500, 300)
        self.droitB = XL320(node,ID_droit_bas, 590, 570 + 220, 300)

        self.gaucheH = XL320(node,ID_gauche_haut, mirrorAngle(500), mirrorAngle(520 - 220), 300)
        self.droitH = XL320(node,ID_droit_haut, mirrorAngle(570 + 220), mirrorAngle(590), 300)

        self.torque=False

        #self.droitH.setHomingOffset(55)

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
        #torqueSetup = self.torque

        #PID venere
        #self.setTorque(0)
        #self.gaucheB.setPositionPID(100, 30, 10)
        #self.droitB.setPositionPID(100, 30, 10)
        #self.gaucheH.setPositionPID(100, 30, 10)
        #self.droitH.setPositionPID(100, 30, 10)
        #self.setTorque(1)

        #self.gaucheB.setGoalPosition(300)
        #self.gaucheH.setGoalPosition(mirrorAngle(300))
        #self.droitB.setGoalPosition(790)
        #self.droitH.setGoalPosition(mirrorAngle(790))

        #time.sleep(0.3)

        ##pid soft
        ## self.setTorque(0)
        #self.gaucheB.setPositionPID(20, 0, 0)
        #self.droitB.setPositionPID(20, 0, 0)
        #self.gaucheH.setPositionPID(20, 0, 0)
        #self.droitH.setPositionPID(20, 0, 0)
        ## self.setTorque(1)
        #self.open()
        #time.sleep(0.3)

        self.gaucheB.setGoalPosition(300)
        self.gaucheH.setGoalPosition(mirrorAngle(300))
        self.droitB.setGoalPosition(790)
        self.droitH.setGoalPosition(mirrorAngle(790))
        time.sleep(0.3)

        #self.gaucheB.setGoalPosition(self.gaucheB.getPresentPosition())
        #self.droitB.setGoalPosition(self.droitB.getPresentPosition())
        #self.gaucheH.setGoalPosition(self.gaucheH.getPresentPosition())
        #self.droitH.setGoalPosition(self.droitH.getPresentPosition())

        #time.sleep(0.1)
        #self.setTorque(torqueSetup)

class ball_system:
    def __init__(self, node, lift_left_id=12, lift_right_id=6, roller_left_id=23, roller_right_id=24, figer_left_id=20, figer_right_id=13):
        
        self.node = node

        self.lift_right = XL320(node,lift_right_id, Max_Speed=600)
        self.lift_left = XL320(node,lift_left_id, Max_Speed=600)
        self.lift_left.setReverseRotation(True)

        self.roller_right = XL320(node,roller_right_id,Drive_Mode=CONTROL_MODE_WHEEL)
        self.roller_left = XL320(node,roller_left_id,Drive_Mode=CONTROL_MODE_WHEEL)
        self.roller_left.setReverseRotation(True)
        self.roller_right.setPositionPID(120, 30, 0)
        self.roller_left.setPositionPID(120, 30, 0)

        self.figer_right = XL320(node,figer_right_id, 390, 600, 1023, 300)
        self.figer_left = XL320(node,figer_left_id, 390, 600, 1023, 300)
        self.figer_left.setReverseRotation(True)

        self.setTorque(False)

        self.start_fingers_loop = False
        self.fingers_step = 0
        self.fingers_loop_timer = self.node.create_timer(0.3, self.on_fingers_loop_timer)

    def setTorque(self, value):
        self.lift_right.setTorque(value)
        self.lift_left.setTorque(value)
        self.roller_right.setTorque(value)
        self.roller_left.setTorque(value)
        self.figer_right.setTorque(value)
        self.figer_left.setTorque(value)
        self.torque = value

    def startRoller_in(self):
        self.roller_left.setGoalSpeed(1023,True)
        self.roller_right.setGoalSpeed(1023,True)
        return [self.roller_right,self.roller_left]

    def startRoller_out(self):
        self.roller_left.setGoalSpeed(1023)
        self.roller_right.setGoalSpeed(1023)
        return [self.roller_right,self.roller_left]

    def stopRoller(self):
        self.roller_left.setGoalSpeed(0)
        self.roller_right.setGoalSpeed(0)
        return [self.roller_right,self.roller_left]

    def on_fingers_loop_timer(self):
        if self.start_fingers_loop :
            if self.fingers_step >= 3 :
                self.setFingersOnCenter()
                self.fingers_step=0
            else :
                self.setFingersOnSide()
                self.fingers_step+=1

    def startFingersLoop(self) :
        self.start_fingers_loop = True
        return []
    
    def stopFingersLoop(self) :
        self.start_fingers_loop = False
        return []

    def setFingersOnSide(self):
        self.figer_right.setGoalPosition(390)
        self.figer_left.setGoalPosition(390)
        return [self.figer_left,self.figer_right]

    def setFingersOnCenter(self):
        self.figer_right.setGoalPosition(600)
        self.figer_left.setGoalPosition(600)
        return [self.figer_left,self.figer_right]
  
    def setRollerUp(self):
        self.lift_right.setGoalPosition(1023)
        self.lift_left.setGoalPosition(1023)
        return [self.lift_right,self.lift_left]
        
    def setRollerDown(self):
        self.lift_right.setGoalPosition(0)
        self.lift_left.setGoalPosition(0)
        return [self.lift_right,self.lift_left]

    def setRollerMiddle(self):
        self.lift_right.setGoalPosition(250)
        self.lift_left.setGoalPosition(250)
        return [self.lift_right,self.lift_left]
