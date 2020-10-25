#!/usr/bin/env python3
import asyncio
from bleak import discover, BleakClient
import time
from constants import Descriptors, Commands, getCommandName
from ece4960robot import Robot
from settings import Settings
from struct import unpack, calcsize

rpy = (0,0,0,0)
xyzd = (0,0,0,0)     # globals for R/P/Y angles and X/Y/Z rot. velocities
levels = (0,0)      # global tuple for motor values

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


async def robotTest(loop):

    # Handle is the TX characteristic UUID; does not change in our simple case.
    # Robot sends "enq" every 2 seconds to keep the connection "fresh"
    # Otherwise, it's a struct of the form:
    # bytes(type + length + data)
    # This struct shouldn't be more than 99 bytes long.
    def simpleHandler(handle, value):
        global time,xyzd,rpy,levels  # This is apparently needed.
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
                print(
                    f"Code: {getCommandName(code)} Length: {length} Data: {data}")

            # Somewhat detach console output from Bluetooth handling.
            if (code == Commands.SER_TX.value):
                theRobot.pushMessage(str(data, encoding="UTF-8"))

            # Unpack an array of 3 angles, which are little-endian floats.
            if (code == Commands.GIVE_ANGLES.value):
                rpy = unpack("<ffff", data)
                #print(rpy)

            # Unpack an array of 3 raw readings: little-endian ints.
            if (code == Commands.GIVE_RAW.value):
                xyzd = unpack("<ffff", data)    # float == f
                #print(xyzd)

            # Unpack the motor power levels
            if (code == Commands.GET_MOTORS.value):
                levels = unpack("<BB", data) # uint8_t == B
                # print(levels) # for debugging

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

    async def checkMessages():
        while True:
            if(theRobot.availMessage()):
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
        theRobot = Robot(client, bleak=True)
        await client.start_notify(Descriptors["TX_CHAR_UUID"].value,
                                  simpleHandler)

        # await client.write_gatt_char(Descriptors["RX_CHAR_UUID"].value, msg)

        async def myRobotTasks():
            global xyzd, levels  # Python's dark side: globals aren't global unless you declare them again!

            # await theRobot.ping()
            '''
            Sending motor values: 0 is full speed reverse and 255 is full speed ahead.
            128, halfway in the middle, is stop. The reason for this is that Python is
            sending a single-byte value, but the motor take a byte for the power level
            and a bit for the direction.
            '''
            await theRobot.sendCommand(Commands.REQ_RAW)
            # All the backbone of this system is already set up. 
            # theRobot.loopTask() sets the motor values "left" and "right"
            # continuously and automatically.
            for i in range(128,256):           # starts at 128, ends at 255
                power = i                      # ramp up
                motorPower = int(2*(power-127.5))   # parse the motor power as the robot would
                theRobot.updateMotor("left", power)  # left is reversed
                theRobot.updateMotor("right", power) # right is forward
                # Every loop, display actual speed and motor power
                print("{},{}".format(xyzd[2],levels[1])) # note that the two motor powers are the same
                await asyncio.sleep(0.05)
            # await asyncio.sleep(1) # easy but doesn't give continuous motor power readings
            for i in range(20):
                # Every loop, display actual speed and motor power
                print("{},{}".format(xyzd[2],levels[1]))    # these should be nearly constant
                await asyncio.sleep(0.05)
            for i in range(255,127,-1):       # starts at 255, ends at 128
                power = i                     # ramp down
                motorPower = int(2*(power-127.5))   # parse the motor power as the robot would
                theRobot.updateMotor("left", power)  # left is reversed
                theRobot.updateMotor("right", power) # right is forward
                # Every loop, display actual speed and motor power
                print("{},{}".format(xyzd[2],levels[1]))
                await asyncio.sleep(0.05)
            theRobot.updateMotor("left",128)           # stop
            theRobot.updateMotor("right",128)
            await asyncio.sleep(1)
            

        async def motorLoop():
            while True:
                await theRobot.loopTask()
                await asyncio.sleep(0.1)

        await asyncio.gather(checkMessages(), myRobotTasks(), motorLoop())
        # async for msg in checkMessages():
        #    print(f"BTDebug: {msg}")


if __name__ == "__main__":
    loop = asyncio.get_event_loop()
    loop.run_until_complete(robotTest(loop))
    # theRobot = loop.run_until_complete(getRobot())
    # print(theRobot.address)
