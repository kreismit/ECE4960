#!/usr/bin/env python3
import asyncio
from bleak import discover, BleakClient
import time
from constants import Descriptors, Commands, getCommandName
from ece4960robot import Robot
from settings import Settings
from struct import pack, unpack, calcsize

# Initialize stuff for PID
xyzd = (0,0,0,0)   # sensor reading tuple
rpy = (0,0,0,0)  # calculated angle tuple
levels = (0,0)  # motor output tuple
setpoint = 50  # also known as reference, "r"
kp = 0.75       # proportional gain
ki = 1.5        # integral gain
# For ki=1, the motor power increases by 1 after e=1 for 1 sec.
kd = 0.0        # derivative gain
# For kd=1, the motor power increases by 1 when e drops by 1/sec.
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
            # PID loop happens offline on the robot. Print useful info.
            global tNow, setpoint, kp, ki, kd, xyzd, levels
            tStart = tNow
            data = pack("<ifff", setpoint, kp, ki, kd)
            length = 18 # Four four-byte numbers and 2 one-byte numbers
            # Start PID-controlled spin
            await theRobot.sendCommand(Commands.START_SPIN, length, data)
            await theRobot.sendCommand(Commands.REQ_RAW) # update sensors
            while (tNow-tStart < 15):   # run for 15 s
                await asyncio.sleep(0.05)
                await theRobot.sendCommand(Commands.GET_MOTORS) # get levels
                z = xyzd[2]          # current sensor value
                tNow = time.clock_gettime(time.CLOCK_MONOTONIC)  # get time
                # Every loop, display:
                # * time (s) since start of loop
                # * desired speed (0-255)
                # * actual speed
                # * motor power levels
                print("{},{},{},{}".format(tNow-tStart, setpoint,z,levels[1]))
            theRobot.updateMotor("left",128) # stop after 15 sec. of spinning
            theRobot.updateMotor("right",128)

        async def motorLoop():
            while True:
                await theRobot.loopTask()
                await asyncio.sleep(0.01)

        await asyncio.gather(checkMessages(), myRobotTasks(), motorLoop())
        # async for msg in checkMessages():
        #    print(f"BTDebug: {msg}")


if __name__ == "__main__":
    loop = asyncio.get_event_loop()
    loop.run_until_complete(robotTest(loop))
    # theRobot = loop.run_until_complete(getRobot())
    # print(theRobot.address)
