#!/usr/bin/env python
# -*- coding: utf-8 -*-

from dynamixel_sdk import *
import time
import math

connected_XL430=[]
connected_XL320=[]

def initHandlers(DEVICENAME, BAUDRATE, PROTOCOL_VERSION):
    # Initialize PortHandler instance
    # Set the port pathsudo ap
    # Get methods and members of PortHandlerLinux or PortHandlerWindows
    global portHandler
    portHandler = PortHandler(DEVICENAME)

    # Initialize PacketHandler instance
    # Set the protocol version
    # Get methods and members of Protocol1PacketHandler or Protocol2PacketHandler
    global packetHandler
    packetHandler = PacketHandler(PROTOCOL_VERSION)

    # Open port
    if portHandler.openPort():
        print("Succeeded to open the port")
    else:
        print("Failed to open the port")
        print("Press any key to terminate...")
        getch()
        quit()

    # Set port baudrate
    if portHandler.setBaudRate(BAUDRATE):
        print("Succeeded to change the baudrate")
    else:
        print("Failed to change the baudrate")
        print("Press any key to terminate...")
        getch()
        quit()


def closePort():
    portHandler.closePort()


def getModel(DXL_ID):
    for i in range(0, 5):
        dxl_model_number, dxl_comm_result, dxl_error = packetHandler.ping(
            portHandler, DXL_ID
        )
        if dxl_comm_result != COMM_SUCCESS:
            # print(
            #     "Ping of ",
            #     DXL_ID,
            #     ": %s" % packetHandler.getTxRxResult(dxl_comm_result),
            # )
            pass
        elif dxl_error != 0:
            print(
                "Ping of ", DXL_ID, ": %s" % packetHandler.getRxPacketError(dxl_error)
            )
        else:
            return dxl_model_number
    print("Ping of ", DXL_ID, ": Unable to get model number. Please check electrical connection")
    return -1

def resetTorque4All():
    time.sleep(0.01)
    for xl320 in connected_XL320 :
        writeValue(portHandler, 1, xl320, 24, 0)
        writeValue(portHandler, 1, xl320, 25, 7) #blanc
        time.sleep(0.005)
    for xl430 in connected_XL430 :
        writeValue(portHandler, 1, xl430, 64, 0)
        time.sleep(0.005)

def setGoalPosition(DXL_ID, POSITION):
    dxl_model_number = getModel(DXL_ID)
    if dxl_model_number == 350:
        writeValue(portHandler, 2, DXL_ID, 30, POSITION)
    elif dxl_model_number == 1060:
        writeValue(portHandler, 4, DXL_ID, 116, POSITION)
    else:
        print("Unknown DXL model number (%d)" % dxl_model_number)


def getPresentPosition(DXL_ID):
    dxl_model_number = getModel(DXL_ID)
    if dxl_model_number == 350:
        dxl_present_position = readValue(portHandler, 2, DXL_ID, 37)
        return dxl_present_position
    elif dxl_model_number == 1060:
        dxl_present_position = readValue(portHandler, 4, DXL_ID, 132)
        return dxl_present_position
    else:
        print("Unknown DXL model number (%d)" % dxl_model_number)
        return -1


def setDriveMode(DXL_ID, MODE):
    writeValue(
        portHandler, 1, DXL_ID, 11, MODE
    )  # XL320 : 1: wheel mode   2: joint mode   // XL430 : 1:Velocity  3:Position  4:Extended position  16:PWM
    self.driveMode = MODE


def setTorque(DXL_ID, VALUE):
    dxl_model_number = getModel(DXL_ID)
    if dxl_model_number == 350:
        writeValue(portHandler, 1, DXL_ID, 24, VALUE)
    elif dxl_model_number == 1060:
        writeValue(portHandler, 1, DXL_ID, 64, VALUE)
    else:
        print("Unknown DXL model number (%d)" % dxl_model_number)


def setPositionPID(DXL_ID, P, I, D):
    dxl_model_number = getModel(DXL_ID)
    if dxl_model_number == 350:
        writeValue(portHandler, 1, DXL_ID, 29, P)

        writeValue(portHandler, 1, DXL_ID, 28, I)

        writeValue(portHandler, 1, DXL_ID, 27, D)

    elif dxl_model_number == 1060:
        writeValue(portHandler, 2, DXL_ID, 84, P)

        writeValue(portHandler, 2, DXL_ID, 82, I)

        writeValue(portHandler, 2, DXL_ID, 80, D)
    else:
        print("Unknown DXL model number (%d)" % dxl_model_number)


def printPositionPID(DXL_ID):
    dxl_model_number = getModel(DXL_ID)
    if dxl_model_number == 350:
        P = readValue(portHandler, 2, DXL_ID, 29)
        I = readValue(portHandler, 2, DXL_ID, 28)
        D = readValue(portHandler, 2, DXL_ID, 27)
        print("P:", P, "  I:", I, "  D:", D)

    elif dxl_model_number == 1060:
        P = readValue(portHandler, 2, DXL_ID, 84)
        I = readValue(portHandler, 2, DXL_ID, 82)
        D = readValue(portHandler, 2, DXL_ID, 80)
        print("P:", P, "  I:", I, "  D:", D)

    else:
        print("Unknown DXL model number (%d)" % dxl_model_number)


def reboot(ID):
    packetHandler.reboot(portHandler, ID)


def formatAngle(angle):
    angle = angle % 360
    if angle > 180:
        return angle - 360
    else:
        return angle

def bitfield(n):
    return [1 if digit=='1' else 0 for digit in bin(n)[2:].zfill(8)]

def getHardwareError(ID):  # a debug
    if XL320 :
        if ID in XL320 :
            return "Hardware error unknown (XL320)"

    elif XL430 :
        if ID in XL430 :
            hard_error=bitfield(readValue(portHandler,1,ID,70))

    #print(hard_error)
    errror_msg=""
    if hard_error[0] :
         errror_msg+=" Input Voltage Error"
    if hard_error[2] :
         errror_msg+=" Overheating Error"
    if hard_error[3] :
         errror_msg+=" Motor Encoder Error"
    if hard_error[4] :
         errror_msg+=" Electrical Shock Error"
    if hard_error[5] :
         errror_msg+=" Overload Error"
    return errror_msg

def writeValue(portHandler, size, ID, address, value):
    if ( ID in connected_XL320 ) or ( ID in connected_XL430 ) or ID==254:

        if size == 1:
            dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(
                portHandler, ID, address, value
            )
        elif size == 2:
            dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(
                portHandler, ID, address, value
            )
        elif size == 3:
            dxl_comm_result, dxl_error = packetHandler.write3ByteTxRx(
                portHandler, ID, address, value
            )
        elif size == 4:
            dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(
                portHandler, ID, address, value
            )
        else:
            print("Incorrect size")
            return

        if dxl_comm_result != COMM_SUCCESS:
            # print(
            #     "Comm_result for ID",
            #     ID,
            #     " address",
            #     address,
            #     ": %s" % packetHandler.getTxRxResult(dxl_comm_result),
            # )
            pass
        elif dxl_error != 0:
            error_msg=packetHandler.getRxPacketError(dxl_error)
            if(error_msg == "[RxPacketError] The data value exceeds the limit value!"):
                print("value :",value,"  size :",size)

            if(error_msg == "[RxPacketError] Hardware error occurred. Check the error at Control Table (Hardware Error Status)!"):
                #error_msg=getHardwareError(ID)
                pass

            print(
                "DXL error for ID",
                ID,
                " address",
                address,
                ": %s" % error_msg,
            )


def readValue(portHandler, size, ID, address):
    if ( ID in connected_XL320 ) or ( ID in connected_XL430 ):
        if size == 1:
            value, dxl_comm_result, dxl_error = packetHandler.read1ByteTxRx(
                portHandler, ID, address
            )
            if dxl_comm_result != COMM_SUCCESS:
                time.sleep(0.05)
                value, dxl_comm_result, dxl_error = packetHandler.read1ByteTxRx(
                    portHandler, ID, address
                )
        elif size == 2:
            value, dxl_comm_result, dxl_error = packetHandler.read2ByteTxRx(
                portHandler, ID, address
            )
            if dxl_comm_result != COMM_SUCCESS:
                time.sleep(0.05)
                value, dxl_comm_result, dxl_error = packetHandler.read2ByteTxRx(
                    portHandler, ID, address
                )
        elif size == 3:
            value, dxl_comm_result, dxl_error = packetHandler.read3ByteTxRx(
                portHandler, ID, address
            )
            if dxl_comm_result != COMM_SUCCESS:
                time.sleep(0.05)
                value, dxl_comm_result, dxl_error = packetHandler.read3ByteTxRx(
                    portHandler, ID, address
                )
        elif size == 4:
            value, dxl_comm_result, dxl_error = packetHandler.read4ByteTxRx(
                portHandler, ID, address
            )
            if dxl_comm_result != COMM_SUCCESS:
                time.sleep(0.05)
                value, dxl_comm_result, dxl_error = packetHandler.read4ByteTxRx(
                    portHandler, ID, address
                )
        else:
            print("Incorrect size")
            return -1

        if dxl_comm_result != COMM_SUCCESS:
            # print(
            #     "Comm_result for ID",
            #     ID,
            #     " address",
            #     address,
            #     ": %s" % packetHandler.getTxRxResult(dxl_comm_result),
            # )
            pass
        elif dxl_error != 0:
            print(
                "DXL error for ID",
                ID,
                " address",
                address,
                ": %s" % packetHandler.getRxPacketError(dxl_error),
            )
        else:
            return value
    return -1


###################################################


class XL320:
    def __init__(
        self,
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

        if 350 == getModel(DXL_ID):
            self.model_number = 350
            connected_XL320.append(DXL_ID)
        else:
            #print("Error : DXL with ID ", DXL_ID, " does not appear to be XL320")
            return
        self.setTorque(0)
        self.setDriveMode(2)
        time.sleep(0.01)
        self.setMaxTorque(Max_Torque)
        time.sleep(0.01)
        self.setMaxSpeed(Moving_Speed)
        self.setPunch(50)

        self.setAngleLimits(CW_Angle_Limit, CCW_Angle_Limit)

        self.goalPosition = self.getPresentPosition()
        self.setLED(4)

        print("XL320 ", DXL_ID, " was well initialized (temp:",self.getPresentTemperature(),"°C  Voltage:",self.getPresentVoltage(),"V)")

    def setReverseRotation(self,value):
        self.reverseRotation=value
        self.setAngleLimits(self.CW_Angle_Limit, self.CCW_Angle_Limit)
        print("XL320 with ID",self.ID,"has its rotation reversed.")

    def setPunch(self, punch):
        writeValue(portHandler, 2, self.ID, 51, punch)

    def setLED(self, value):
        writeValue(portHandler, 1, self.ID, 25, value)

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

        writeValue(portHandler, 2, self.ID, 6, CW_Angle_Limit)
        time.sleep(0.005)
        writeValue(portHandler, 2, self.ID, 8, CCW_Angle_Limit)

        self.setTorque(torque)





    def setMaxTorque(self, Max_Torque):
        if Max_Torque < 0:
            print("Error for ID", self.ID, ": Max_Torque<0")
            Max_Torque = 0
        if Max_Torque > 1023:
            print("Error for ID", self.ID, ": Max_Torque>1023")
            Max_Torque = 1023

        self.maxTorque = Max_Torque
        writeValue(portHandler, 2, self.ID, 15, Max_Torque)

    def setMaxSpeed(self, Max_Speed):
        if Max_Speed < 0:
            print("Error for ID", self.ID, ": Max_Speed<0")
            Max_Speed = 0
        if Max_Speed > 2047:
            print("Error for ID", self.ID, ": Max_Speed>2047")
            Max_Speed = 2047

        self.maxSpeed = Max_Speed

        writeValue(portHandler, 2, self.ID, 32, Max_Speed)

    def setPositionPID(self, P, I, D):
        writeValue(portHandler, 1, self.ID, 29, P)
        writeValue(portHandler, 1, self.ID, 28, I)
        writeValue(portHandler, 1, self.ID, 27, D)

        self.pid_P = P
        self.pid_I = I
        self.pid_D = D

    def printPositionPID(self):
        P = readValue(portHandler, 2, self.ID, 29)
        I = readValue(portHandler, 2, self.ID, 28)
        D = readValue(portHandler, 2, self.ID, 27)
        print("P:", P, "  I:", I, "  D:", D)

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
        writeValue(portHandler, 2, self.ID, 30, int(POSITION))
        self.setTorque(
            self.torque
        )  # Updating the goal_position register seems to enable torque, so this line releases the motor if it haven't to be enabled

    def getPresentPosition(self):
        dxl_present_position = readValue(portHandler, 2, self.ID, 37)
        if self.reverseRotation :
            dxl_present_position=1023-dxl_present_position
        dxl_present_position-=self.offset
        return dxl_present_position

    def getPresentTemperature(self):
        present_temp = readValue(portHandler, 1, self.ID, 46)
        return present_temp

    def getPresentVoltage(self):
        present_voltage=readValue(portHandler, 1, self.ID, 45)/10.0
        return present_voltage

    def reboot(self):
        packetHandler.reboot(portHandler, self.ID)

    def setDriveMode(self, MODE):
        writeValue(
            portHandler, 1, self.ID, 11, MODE
        )  # XL430 : 1:Velocity  3:Position  4:Extended position  16:PWM
        self.driveMode = MODE

    def setTorque(self, VALUE):
        self.torque = VALUE
        writeValue(portHandler, 1, self.ID, 24, VALUE)
        if VALUE : self.setLED(2)
        else : self.setLED(7)


    def printTorqueEnable(self, debug=""):
        torqueEnable = readValue(portHandler, 1, self.ID, 24)
        print(debug, "torqueEnable=", torqueEnable)

    def reboot(self):
        packetHandler.reboot(portHandler, self.ID)

    def getPresentVelocity(self):
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


class XL430:
    def __init__(
        self,
        DXL_ID,
        CW_Angle_Limit=0,
        driveMode=4,
        CCW_Angle_Limit=4095,
        Moving_Speed=1023,
    ):
        self.ID = DXL_ID
        self.driveMode = driveMode
        if 1060 == getModel(DXL_ID):
            self.model_number = 1060
            connected_XL430.append(DXL_ID)
        else:
            print(
                "Error for ID",
                self.ID,
                ": DXL with ID ",
                DXL_ID,
                " does not appear to be XL430",
            )
        self.setTorque(0)
        if driveMode == 3:
            self.setAngleLimits(CW_Angle_Limit, CCW_Angle_Limit)
            time.sleep(0.005)
        self.setMaxSpeed(Moving_Speed)
        self.setDriveMode(driveMode)
        self.setPositionPID(640, 0, 4000)
        self.setReverseRotation(0)

        print("XL430 ", DXL_ID, " was well initialized (temp:",self.getPresentTemperature(),"°C  Voltage:",self.getPresentVoltage(),"V)")

    def setAngleLimits(self, CW_Angle_Limit, CCW_Angle_Limit):
        if self.driveMode != 3:
            print(
                "Warning for ID",
                self.ID,
                ": Angle limits are not used for selected drive mode (",
                self.driveMode,
                ")",
            )
            return

        if CW_Angle_Limit > CCW_Angle_Limit:
            print("Error for ID", self.ID, ": CW_Angle_Limit>CCW_Angle_Limit")
        if CW_Angle_Limit < 0:
            print("Error for ID", self.ID, ": CW_Angle_Limit<0")
            CW_Angle_Limit = 0
        if CCW_Angle_Limit > 4095:
            print("Error for ID", self.ID, ": CCW_Angle_Limit>1023")
            CCW_Angle_Limit = 4095

        writeValue(portHandler, 4, self.ID, 52, CW_Angle_Limit)

        time.sleep(0.005)
        writeValue(portHandler, 4, self.ID, 48, CCW_Angle_Limit)

        self.CW_Angle_Limit = CW_Angle_Limit
        self.CCW_Angle_Limit = CCW_Angle_Limit

    def getPresentTemperature(self):
        return readValue(portHandler, 1, self.ID, 146)

    def getPresentVoltage(self):
        return readValue(portHandler, 2, self.ID, 144)/10

    def setMaxSpeed(self, Max_Speed):
        self.maxSpeed = Max_Speed
        if Max_Speed < 0:
            print("Error for ID", self.ID, ": Max_Speed<0")
            Max_Speed = 0
        if Max_Speed > 4095:
            print("Error for ID", self.ID, ": Max_Speed>4095")
            Max_Speed = 4095

        writeValue(portHandler, 4, self.ID, 44, Max_Speed)

    def setPositionPID(self, P, I, D):
        self.P = P
        self.I = I
        self.D = D
        writeValue(portHandler, 2, self.ID, 78, P)
        writeValue(portHandler, 2, self.ID, 76, I)
        writeValue(portHandler, 2, self.ID, 80, D)

    def printPositionPID(self):
        P = readValue(portHandler, 2, self.ID, 78)
        I = readValue(portHandler, 2, self.ID, 76)
        D = readValue(portHandler, 2, self.ID, 80)
        print("P:", P, "  I:", I, "  D:", D)

    def setGoalPosition(self, POSITION):
        if self.driveMode == 3:
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

        writeValue(portHandler, 4, self.ID, 116, POSITION)
        self.setTorque(
            self.torque
        )  # Updating the goal_position register seems to enable torque, so this line releases the motor if it haven't to be enabled

    def setGoalSpeed(self, SPEED):
        if self.driveMode == 1:
            writeValue(portHandler, 4, self.ID, 104, SPEED)
            self.setTorque(
                self.torque
            )  # Updating the goal_position register seems to enable torque, so this line releases the motor if it haven't to be enabled
        else:
            print("You can't set speed goal if you are not in Velocity Mode")

    def getPresentPosition(self):
        dxl_present_position = readValue(portHandler, 4, self.ID, 132)
        return dxl_present_position

    def getPresentVelocity(self):
        dxl_present_velocity = readValue(portHandler, 4, self.ID, 128)
        return dxl_present_velocity

    def reboot(self):
        packetHandler.reboot(portHandler, self.ID)

    def setReverseRotation(self,value):
        if value :
            driveModeValue=5
        else :
            driveModeValue=4
        writeValue(portHandler, 1, self.ID, 10, driveModeValue)        
        self.reverseRotation = value
        print("XL430 with ID",self.ID,"has its rotation reversed.")


    def setDriveMode(self, MODE): #dans la doc c'est pas appelé driveMode mais operatingMode car driveMode est utilisé pour autre chose
        writeValue(
            portHandler, 1, self.ID, 11, MODE
        )  # XL320 : 1: wheel mode   2: joint mode   // XL430 : 1:Velocity  3:Position  4:Extended position  16:PWM
        self.driveMode = MODE

    def setTorque(self, VALUE):
        self.torque = VALUE
        writeValue(portHandler, 1, self.ID, 64, VALUE)

    def printTorqueEnable(self, debug=""):
        torqueEnable = readValue(portHandler, 1, self.ID, 64)
        print(debug, "torqueEnable=", torqueEnable)

    def reboot(self):
        packetHandler.reboot(portHandler, self.ID)

    def setHomingOffset(self, offset):
        # if self.reverseRotation :
        #     offset=1044479-offset
        if offset > 1044479 :
            print("Erreur : offset (",offset,") trop grand ( > 1044479 ) pour XL430 avec ID",self.ID)
            return
        elif offset < -1044479 :
            print("Erreur : offset (",offset,") trop petit ( < -1044479 ) pour XL430 avec ID",self.ID)
            return
        self.offset = offset
        writeValue(portHandler, 4, self.ID, 20, offset)

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
            self.joinE.setGoalPosition(angle)


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

    def initSlider(self, positionEnButeeHaute=0):

        self.setTorque(0)

        speed_setup = self.slider.maxSpeed
        self.slider.setMaxSpeed(500)

        self.slider.setHomingOffset(0)  # reset de l'offset pour avoir presentPosition=ActualPosition
        offset = self.slider.getPresentPosition()  # actual position
        self.slider.setHomingOffset(offset)  # offset = actualPosition => presentPosition=0

        self.slider.setTorque(1)

        self.slider.setGoalPosition(-10000)

        self.slider.waitMoveEnd(5)

        pos = self.slider.getPresentPosition()
        self.setTorque(0)
        self.slider.setHomingOffset(positionEnButeeHaute - (pos - offset))
        # print("Final Position=",self.slider.getPresentPosition())

        self.slider.setMaxSpeed(speed_setup)



    def setSliderPosition_mm(self, mm):
        value = int(-36.93191489 * mm + 8679)
        self.slider.setGoalPosition(value)

    def getSlidePosition_mm(self):
        return (self.slider.getPresentPosition() - 8679) * (1 / -36.93191489)

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