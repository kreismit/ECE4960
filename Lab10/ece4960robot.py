import asyncio
from constants import Descriptors, ACK_VALUE, Commands, getCommandName
from threading import Lock
import time
import struct

''' Update (kreismit, 11-22-2020)
    Only uses bleak interface now
    Supports commands:
    - sendCommand: generic, works for any command code
    - ping: should receive a message back "ping pong" if the robot is there
    - sendMessage: send a message (text) to the robot over bluetooth
    - testByteStream: get back generic data stream to test connection
    - setMotors: run motors at certain values; slow and should not be used for automated control
    - setGains: change proportional, integral, and derivative gains for turning, as well as open-loop gains & offsets
    - getOdom: returns current odometry data from robot
    - setVel: start driving at a certain linear and rotational velocity; positive values mean forward and counterclockwise
    - getVel: return current linear velocity (-127 to 127) and angular velocity (units of deg/s)
    - pushMessage: put message in transmission queue
    - availMessage: returns true if there is a message of nonzero length waiting
    - getMessage: returns any available messages, or 0 if none are available
'''

class Robot:
    def __init__(self, btif=None):
        self.bt_interface = btif
        self.messageQueue = list()
        self.theLock = Lock()
        self.mValues = {"left": 0, "right": 1}
        self.lastTime = time.time()
        self.updateFlag = False
        self.isRunning = True # drop connection when isRunning is false

    async def sendCommand(self, cmd, length=0, data=bytearray([])):
        await self.bt_interface.write_gatt_char(
            Descriptors["RX_CHAR_UUID"].value,
            bytearray([cmd.value, length]) + data)

    async def sendMessage(self, msg):
        await self.bt_interface.write_gatt_char(
            Descriptors["RX_CHAR_UUID"].value,
            bytearray([Commands.SER_RX.value, len(msg) + 1]) +
            msg.encode() + b'\x00')

    async def ping(self):
        self.now = time.time()
        await self.bt_interface.write_gatt_char(
            Descriptors["RX_CHAR_UUID"].value,
            bytearray([Commands.PING.value] + 98*[0]))

    async def testByteStream(self, length):
        print(f"Length is {length}")
        await self.bt_interface.write_gatt_char(
            Descriptors["RX_CHAR_UUID"].value,
            bytearray([Commands.START_BYTESTREAM_TX.value] + [1] + [length] + 96*[0]))

    async def setMotors(self, l, r):
        await self.bt_interface.write_gatt_char(
            Descriptors["RX_CHAR_UUID"].value,
            bytearray([Commands.SET_MOTORS.value, 2, l, r]))
    
    async def setGains(self, kp, ki, kd, kX, calib, offset):
        # write gains (as floats)
        #print(struct.pack("<BBffffff",Commands.SET_GAINS.value, 6, kp, ki, kd, kX, calib, offset))
        await self.bt_interface.write_gatt_char(
            Descriptors["RX_CHAR_UUID"].value,
            bytearray(struct.pack("<BBffffff",Commands.SET_GAINS.value, 6,
            kp, ki, kd, kX, calib, offset)))
    
    async def getOdom(self):
        await self.bt_interface.write_gatt_char(
            Descriptors["RX_CHAR_UUID"].value,
            bytearray([Commands.GET_ODOM.value, 0]))
    
    async def setVel(self, lin, ang):
        # write velocity settings, which don't need to be integers
        #print(struct.pack("<BBff",Commands.SET_VEL.value, 2, ang, lin))
        await self.bt_interface.write_gatt_char(
            Descriptors["RX_CHAR_UUID"].value,
            bytearray(struct.pack("<BBff",Commands.SET_VEL.value, 2,
            ang, lin)))
    
    async def getVel(self):
        await self.bt_interface.write_gatt_char(
            Descriptors["RX_CHAR_UUID"].value,
            bytearray([Commands.GET_VEL.value, 0]))
    
    def pushMessage(self, msg):
        self.messageQueue.append(msg)

    def availMessage(self):
        return len(self.messageQueue) > 0

    def getMessage(self):
        if(len(self.messageQueue) > 0):
            return self.messageQueue.pop(0)
        else:
            return 0
    def quitBT(self): # trivial function to drop the Bluetooth connection
        self.isRunning = False
