#!/usr/bin/env python
# -*- coding: utf-8 -*-

from dynamixel_sdk import *
import time
import math


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
            print(
                "Ping of ",
                DXL_ID,
                ": %s" % packetHandler.getTxRxResult(dxl_comm_result),
            )
        elif dxl_error != 0:
            print(
                "Ping of ", DXL_ID, ": %s" % packetHandler.getRxPacketError(dxl_error)
            )
        else:
            return dxl_model_number
    print("Ping of ", DXL_ID, ": Unable to get model number")
    return -1


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


def writeValue(portHandler, size, ID, address, value):
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
        print(
            "Comm_result for ID",
            ID,
            " address",
            address,
            ": %s" % packetHandler.getTxRxResult(dxl_comm_result),
        )
    elif dxl_error != 0:
        print(
            "DXL error (",
            dxl_error,
            ")for ID",
            ID,
            " address",
            address,
            ": %s" % packetHandler.getRxPacketError(dxl_error),
        )


def readValue(portHandler, size, ID, address):
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
        print(
            "Comm_result for ID",
            ID,
            " address",
            address,
            ": %s" % packetHandler.getTxRxResult(dxl_comm_result),
        )
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
        if 350 == getModel(DXL_ID):
            self.model_number = 350
        else:
            print("Error : DXL with ID ", DXL_ID, " does not appear to be XL320")
            return
        self.setTorque(0)
        self.setDriveMode(2)
        self.setAngleLimits(CW_Angle_Limit, CCW_Angle_Limit)
        time.sleep(0.01)
        self.setMaxTorque(Max_Torque)
        time.sleep(0.01)
        self.setMaxSpeed(Moving_Speed)
        self.setPunch(50)
        self.goalPosition = self.getPresentPosition()

    def setPunch(self, punch):
        writeValue(portHandler, 2, self.ID, 51, punch)

    def setLED(self, value):
        writeValue(portHandler, 1, self.ID, 25, value)

    def setAngleLimits(self, CW_Angle_Limit, CCW_Angle_Limit):
        if CW_Angle_Limit > CCW_Angle_Limit:
            print("Error for ID", self.ID, ": CW_Angle_Limit>CCW_Angle_Limit")
        if CW_Angle_Limit < 0:
            print("Error for ID", self.ID, ": CW_Angle_Limit<0")
            CW_Angle_Limit = 0
        if CCW_Angle_Limit > 1023:
            print("Error for ID", self.ID, ": CCW_Angle_Limit>1023")
            CCW_Angle_Limit = 1023

        writeValue(portHandler, 2, self.ID, 6, CW_Angle_Limit)
        time.sleep(0.005)
        writeValue(portHandler, 2, self.ID, 8, CCW_Angle_Limit)

        self.CW_Angle_Limit = CW_Angle_Limit
        self.CCW_Angle_Limit = CCW_Angle_Limit

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
        writeValue(portHandler, 2, self.ID, 30, int(POSITION))
        self.setTorque(
            self.torque
        )  # Updating the goal_position register seems to enable torque, so this line releases the motor if it haven't to be enabled

    def getPresentPosition(self):
        dxl_present_position = readValue(portHandler, 2, self.ID, 37)
        return dxl_present_position

    def reboot(self):
        packetHandler.reboot(portHandler, self.ID)

    def setDriveMode(self, MODE):
        writeValue(
            portHandler, 1, self.ID, 11, MODE
        )  # XL320 : 1: wheel mode   2: joint mode   // XL430 : 1:Velocity  3:Position  4:Extended position  16:PWM
        self.driveMode = MODE

    def setTorque(self, VALUE):
        self.torque = VALUE
        writeValue(portHandler, 1, self.ID, 24, VALUE)

    def printTorqueEnable(self, debug=""):
        torqueEnable = readValue(portHandler, 1, self.ID, 24)
        print(debug, "torqueEnable=", torqueEnable)

    def reboot(self):
        packetHandler.reboot(portHandler, self.ID)


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
            time.sleep(0.01)
        self.setMaxSpeed(Moving_Speed)
        self.setDriveMode(driveMode)
        self.setPositionPID(640, 0, 4000)

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

    def setDriveMode(self, MODE):
        writeValue(
            portHandler, 1, self.ID, 10, MODE
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
        self.offset = offset
        writeValue(portHandler, 4, self.ID, 20, offset)


class bras:
    def __init__(self, ID_A, ID_B, ID_C, ID_D, ID_E, ID_Slider):
        self.joinA = XL320(ID_A, 360, 700, 160)
        self.joinB = XL320(ID_B, 512 - 333, 512 + 333, 250)
        self.joinC = XL320(ID_C, 160, 580, 450)
        self.joinD = XL320(ID_D, 0, 1023, 500)
        self.joinE = XL320(ID_E)

        self.slider = XL430(ID_Slider)

        self.slider.setDriveMode(4)

        # self.joinD.setPunch(32)
        # self.joinD.setPunch(70)

        self.joinA.setPositionPID(60, 10, 0)
        self.joinB.setPositionPID(60, 25, 0)
        self.joinC.setPositionPID(70, 30, 0)
        self.joinD.setPositionPID(120, 30, 0)
        self.joinE.setPositionPID(70, 30, 0)

    def setArmPosition(self, x_sucker, y_sucker, angle_C=0, angle_D=90):
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
            return

        a = 2 * x_sucker
        b = 2 * y_sucker
        c = h1**2 - h2**2 + x_sucker**2 + y_sucker**2
        d = (2 * a * c) ** 2 - 4 * (a**2 + b**2) * (c**2 - b**2 * h1**2)

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

        # print("xB1:",xB1)
        # print("yB1:",yB1)
        # print("Angle_A1:",angle_A1)
        # print("Angle_B1:",angle_B1)
        # print("xB2:",xB2)
        # print("yB2:",yB2)
        # print("Angle_A2:",angle_A2)
        # print("Angle_B2:",angle_B2)

        if self.isReachable("A", angle_A1) and self.isReachable("B", angle_B1):
            if self.isReachable("A", angle_A2) and self.isReachable("B", angle_B2):
                # les 2 positions sont atteignable
                if angle_A1 > angle_A2:
                    angle_A = angle_A1
                    angle_B = angle_B1
                else:
                    angle_A = angle_A2
                    angle_B = angle_B2

            else:  # seulement position 1 atteignable
                angle_A = angle_A1
                angle_B = angle_B1

        elif self.isReachable("A", angle_A2) and self.isReachable(
            "B", angle_B2
        ):  # seulement position 2 atteignable
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
            return

        # print("Angle_A:",angle_A)
        # print("Angle_B:",angle_B)
        # print("Angle_C:",angle_C)
        # print("Angle_D:",angle_D)

        self.goToAngle("A", angle_A)
        self.goToAngle("B", angle_B)
        self.goToAngle("C", angle_C)
        self.goToAngle("D", angle_D)

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
        # elif join =='E' :
        #     alpha=180-((self.joinA.getPresentPosition()-(512-307))/ratio)
        #     beta=((self.joinB.getPresentPosition()-512)/ratio)
        #     if alpha > 180 : alpha=alpha-360
        #     if beta > 180 : beta=beta-360
        #     angle=(angle-alpha-beta)%360
        #     if angle > 180 : angle=angle-360
        #     self.joinE.setGoalPosition(1023-(angle*ratio+512))
        elif join == "E":
            alpha = 180 - ((self.joinA.goalPosition - (512 - 307)) / ratio)
            beta = (self.joinB.goalPosition() - 512) / ratio
            alpha = formatAngle(alpha)
            beta = formatAngle(beta)
            angle = (angle - alpha - beta) % 360
            angle = formatAngle(angle)
            self.joinE.setGoalPosition(1023 - (angle * ratio + 512))
        else:
            print("Join inconnu :", join)

    def isReachable(self, join, angle):
        ratio = 614 / 180
        if join == "A":
            value = 1023 - (angle * ratio + (512 - 307))
            if self.joinA.CW_Angle_Limit > value or self.joinA.CCW_Angle_Limit < value:
                # print("Join A : angle=",angle,"  value=",value,"  CW=",self.joinA.CW_Angle_Limit,"  CCW=",self.joinA.CCW_Angle_Limit)
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

        self.slider.setHomingOffset(
            0
        )  # reset de l'offset pour avoir presentPosition=ActualPosition
        offset = self.slider.getPresentPosition()  # actual position
        self.slider.setHomingOffset(
            offset
        )  # offset = actualPosition => presentPosition=0

        self.slider.setTorque(1)

        self.slider.setGoalPosition(-10000)

        t_speed = time.time() + 10  # timeout de 10s
        speed = 0
        while t_speed - time.time() > 0:
            lastspeed = speed
            speed = self.slider.getPresentVelocity()
            if speed == 0 and lastspeed != 0:
                t_speed = time.time()  # arret après un front descendant sur speed
            # print("Speed=",speed)
            time.sleep(0.1)

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


class rakes:
    def __init__(self, ID_gauche=7, ID_droit=15):
        self.gauche = XL320(ID_gauche, 520 - 220, 500, 300)
        self.droit = XL320(ID_droit, 590, 570 + 220, 300)

    def setTorque(self, value):
        self.droit.setTorque(value)
        self.gauche.setTorque(value)
        self.torque = value

    def open(self):
        self.gauche.setGoalPosition(471)
        time.sleep(0.01)
        self.droit.setGoalPosition(607)

    def close(self):
        torqueSetup = self.torque

        self.gauche.setPositionPID(100, 30, 10)
        self.droit.setPositionPID(100, 30, 10)

        self.gauche.setGoalPosition(300)
        self.droit.setGoalPosition(790)
        time.sleep(0.3)

        # self.setTorque(0)
        self.gauche.setPositionPID(20, 0, 0)
        self.droit.setPositionPID(20, 0, 0)
        # self.setTorque(1)
        self.open()
        time.sleep(0.3)

        self.gauche.setGoalPosition(300)
        self.droit.setGoalPosition(790)
        time.sleep(0.3)

        self.gauche.setGoalPosition(self.gauche.getPresentPosition())
        self.droit.setGoalPosition(self.droit.getPresentPosition())

        time.sleep(0.1)
        self.setTorque(torqueSetup)
