#!/usr/bin/env python3
import asyncio
from bleak import discover, BleakClient
import time
from constants import Descriptors, Commands, getCommandName
from ece4960robot import Robot
from settings import Settings
from struct import pack, unpack, calcsize
from math import sin, cos, radians

async def getRobot():
    devices = await discover(device=Settings["adapter"], timeout=5)
    # for d in devices:
    #    print(d.name)
    p_robot = [d for d in devices if d.name == "MyRobot"]
    if (len(p_robot) > 0):
        #    print(p_robot[0].address)
        return p_robot[0]
    else:
        return None

class robotHolder():
    def __init__(self):
        self.robot = None
    def getRobot(self):
        return self.robot
    def setRobot(self, robot):
        self.robot = robot

theRobotHolder = robotHolder()

async def robotRun(loop):

    # Handle is the TX characteristic UUID; does not change in our simple case.
    # Robot sends "enq" every 2 seconds to keep the connection "fresh"
    # Otherwise, it's a struct of the form:
    # bytes(type + length + data)
    # This struct shouldn't be more than 99 bytes long.
    def simpleHandler(handle, value):
        global time  # This is apparently needed.
        if (value == "enq".encode()):
            pass
        else:
            fmtstring = "BB"+str(len(value)-2)+"s"
            code, length, data = unpack(fmtstring, value)
            '''
            Python doesn't have a switch statement, nor easily compatible
            enum support. This might be the easiest way to handle commands.
            '''
            if (Settings["OutputRawData"]):
                print(f"Code: {getCommandName(code)} Length: {length} Data: {data}")

            # Somewhat detach console output from Bluetooth handling.
            if (code == Commands.SER_TX.value):
                theRobot.pushMessage(str(data, encoding="UTF-8"))

            # Unpack a tuple (x,theta, range), which are little-endian floats.
            if (code == Commands.GET_ODOM.value):
                theRobot.odomd = unpack("<ffff", data)
                #print(odomd)

            # Unpack a tuple (xDot,thetaDot, range): little-endian floats.
            if (code == Commands.GET_VEL.value):
                theRobot.veld = unpack("<ffff", data)    # float == f
                #print(veld)

            # Unpack the motor power levels
            if (code == Commands.GET_MOTORS.value):
                theRobot.levels = unpack("<BB", data) # uint8_t == B
                # print(theRobot.levels) # for debugging

            # Example of command-response.
            if (code == Commands.PONG.value):
                print(f"Got pong: round trip {time.time() - theRobot.now}")
                if(Settings["pingLoop"]):
                    loop.create_task(theRobot.ping())
                    # theRobot.newPing = True

            # Unpack from an example stream that transmits a 2-byte and a
            # 4-byte integer as quickly as possible, both little-endian.
            if (code == Commands.BYTESTREAM_TX.value):
                print(f"Received {length} bytes of data")
                print(unpack("<fff",data)) # float == f
                #print(unpack("<QQQ",data)) # unsigned long (long) == Q
                #print(data)	# show the raw, for debugging
    
    
    async def stayConnected():
        while theRobot.isRunning:
            if(theRobot.availMessage()):    # keep checking for BT messages
                print(f"BTDebug: {theRobot.getMessage()}")
            await asyncio.sleep(0.1)
    
    # You can put a UUID (MacOS) or MAC address (Windows and Linux)
    # in Settings["Cached"].
    if (not Settings["cached"]):
        theRobot_bt = await getRobot()

    else:
        theRobot_bt = type("", (), {})()
        theRobot_bt.address = Settings["cached"]
    # print(theRobot_bt.address)
    while (not theRobot_bt):
        print("Robot not found")
        theRobot_bt = await getRobot()

    if (not Settings["cached"]):
        print(f"New robot found. Must cache \
            {theRobot_bt.address} manually in settings.py")

    async with BleakClient(theRobot_bt.address, loop=loop, device=Settings["adapter"]) as client:
        # if (await client.is_connected()):
        #    print("Robot connected!")
        # srv = await client.get_services()
        # print(srv)
        await client.is_connected()
        theRobot = Robot(client) # instantiate Robot class (always uses Bleak)
        theRobotHolder.setRobot(theRobot) # fit Robot object into robotHolder
        # Initialize class variables: sensor reading and motor value tuples
        theRobot.odomd = (0,0,0,0) # x position, y position, yaw angle, ToF (floats)
        theRobot.veld = (0,0,0,0)  # x vel., y vel., angular vel., ToF (floats)

        theRobot.isRunning = True # keeps connection alive while it's true

        await client.start_notify(Descriptors["TX_CHAR_UUID"].value,
                                  simpleHandler)

        # await client.write_gatt_char(Descriptors["RX_CHAR_UUID"].value, msg)
        
        await asyncio.gather(stayConnected())
        
        # async for msg in checkMessages():
        #    print(f"BTDebug: {msg}")


def readCache():
    odom = [0,0,0]    # the robot's onboard odometry resets each time
    # the Bluetooth connection dies. Save the last value and add each displacement to the last.
    with open("odometryCache.txt", "r") as file:
        line = file.readline() # should only be one line
        j = 0
        for i in range(3):
            num = ""
            while not (line[j]==","):
                num = num + line[j]
                j = j + 1
            j = j + 1 # again, to get past the comma
            odom[i] = float(num)
        
    return odom

def writeCache(odom):
    with open("odometryCache.txt", "w") as file:
        for i in range(3):
            file.write(str(odom[i])+",")

def resetCache():
    writeCache((0,0,0))
