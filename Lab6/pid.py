#!/usr/bin/env python3
import asyncio
from bleak import discover, BleakClient
import time
from constants import Descriptors, Commands, getCommandName
from ece4960robot import Robot
from settings import Settings
from struct import unpack, calcsize

# Initialize stuff for PID
xyz = (0,0,0)   # sensor reading tuple
setpoint = 10   # also known as reference, "r"
kp = 1          # proportional gain
ki = 0          # integral gain
kd = 0          # derivative gain
z = 0           # current angular velocity
e = setpoint    # current error for PID loop
inte = 0        # integral of error
tNow = time.clock_gettime(time.CLOCK_MONOTONIC)  # current time coord (sec)
# CLOCK_MONOTONIC is perfect for our purposes; only on Linux!

# and other variables
right = 0   # address of right drive motor
left = 1    # address of left drive motor
rFwd = 1    # forward direction for right motor
rRev = 0    # backward direction for right motor
lFwd = 0    # forward direction for left motor
lRev = 1    # backward direction for left motor
offset = 4  # left side gets (offset) more power than right side
calib = 1.08 # left side gets (calib) times more power than right

async def getRobot():
    devices = await discover(device=Settings["adapter"], timeout=2)
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
                print(
                    f"Code: {getCommandName(code)} Length: {length} Data: {data}")

            # Somewhat detach console output from Bluetooth handling.
            if (code == Commands.SER_TX.value):
                theRobot.pushMessage(str(data, encoding="UTF-8"))

            # Unpack an array of 3 angles, which are little-endian floats.
            if (code == Commands.GIVE_ANGLES.value):
                global rpy
                rpy = unpack("<fff", data)
                #print(rpy)

            # Unpack an array of 3 raw readings: little-endian ints.
            if (code == Commands.GIVE_RAW.value):
                global xyz
                xyz = unpack("<III", data)
                print(xyz)

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
                print(unpack("<fff",data)) # for 3 32-bit floats
                #print(unpack("<QQQ",data)) # for 3 64-bit integers
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
            # pass
            
            await theRobot.sendCommand(Commands.REQ_RAW) # update sensors
            # PID loop (more likely PI)
            global z, e, tNow, setpoint, inte, xyz
            while True:
                await asyncio.sleep(0.05)
                zLast = z
                eLast = e
                tLast = tNow
                z = xyz[2]          # current sensor value
                e = setpoint - z    # error
                tNow = time.clock_gettime(time.CLOCK_MONOTONIC)
                dt = tNow - tLast
                de = e - eLast      # derivative of error
                inte = inte + e
                output = kp*e + ki*inte + kd*(de/dt)
                # write motor power
                #await theRobot.setMotors(right,rFwd,output)
                #await theRobot.setMotors(left,lRev,calib*output+offset)
                #await theRobot.sendCommand(Commands.SET_MOTORS, length=3, data=bytearray([right, rFwd, int(output)]))
                #await theRobot.sendCommand(Commands.SET_MOTORS, length=3, data=bytearray([left, lRev, int(calib*output+offset)]))
                if output < 0:          # can't send negative numbers in a bytearray
                    output = 0
                elif output > 255:
                    output = 255
                output = int(output)    # must be an integer
                theRobot.updateMotor("left", output)  # left is reversed
                theRobot.updateMotor("right", output) # right is forward
                #print(output) # for debugging

            # for i in range(0, 50):
            #     print("Sending message")
            #     await theRobot.sendMessage("Testing message")
            #     await asyncio.sleep(1)

            # await theRobot.testByteStream(4)

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
