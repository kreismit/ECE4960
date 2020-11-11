#!/usr/bin/env python3
import asyncio
from bleak import discover, BleakClient
import time
from constants import Descriptors, Commands, getCommandName
from ece4960robot import Robot
from settings import Settings
from struct import pack, unpack, calcsize

# Initialize sensor reading and motor value tuples
xthd = (0,0,0,0)   # gyro readings and ToF (floats)
odomd = (0,0,0,0)  # calculated angles and ToF (floats)
levels = (0,0)  # motor output (8-bit unsigned ints)
# Initialize PID control constants
setpointTh =  0 # also known as reference, "r"
kpTh = 0.75     # proportional gain, angular speed
kiTh = 1.5      # integral gain, angular speed
# For ki=1, the motor power increases by 1 after e=1 for 1 sec.
kdTh = 0.0      # derivative gain, angular speed
# For kd=1, the motor power increases by 1 when e drops by 1/sec.

setpointX = 0.75 # control target for horizontal velocity
kpX = 100       # proportional gain, linear speed
kiX = 0         # integral gain, linear speed
kdX = 0         # derivative gain, linear speed
tStart = time.monotonic()  # current time coord (sec)

odomList = [] # array of odometry readings and angles

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
        global time,xthd,odomd,levels  # This is apparently needed.
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
            if (code == Commands.GET_ODOM.value):
                odomd = unpack("<fff", data)
                #print(odomd)

            # Unpack an array of 3 raw readings: little-endian ints.
            if (code == Commands.GET_VEL.value):
                xthd = unpack("<fff", data)    # float == f
                #print(xthd)

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
        global odomd, odomList
        while odomd[1] == 0:   # wait for values to come
            if(theRobot.availMessage()):    # but still check for BT messages
                print(f"BTDebug: {theRobot.getMessage()}")
            await asyncio.sleep(0.1)
            #print("Waiting for angle...") # for debugging
        tStart = time.monotonic()
        t = tStart
        while t < 15:       # Run for a bit
            t = time.monotonic() - tStart
            if(theRobot.availMessage()):
                print(f"BTDebug: {theRobot.getMessage()}")
            await asyncio.sleep(0.02)

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
            global setpoint, kp, ki, kd, odomd # "import" globals
            # PID loop happens offline on the robot. Start it and print useful info.
            data = pack("<ffffff", kpX, kiX, kdX, kpTh, kiTh, kdTh)
            length = 24 # Six four-byte numbers
            await theRobot.sendCommand(Commands.SET_GAINS,length,data)
            data = pack("<ff", setpointX,setpointTh)
            length = 8 # Two four-byte numbers
            await theRobot.sendCommand(Commands.SET_VEL,length,data)
            await theRobot.sendCommand(Commands.GET_ODOM)
            while odomd[1] == 0:   # wait for values to come
                await asyncio.sleep(0.1)
                print("Waiting for X...") # for debugging
            xStart = odomd[0]
            print("Initial odometry position: {}".format(xStart))
            x = 0
            while x < 1:       # Run until the robot thinks it's moved 1 m
                x = odomd[0] - xStart   # update relative displacement
                print(x) # for debugging
                await asyncio.sleep(0.1)
            data = pack("<ff", 0,0)
            length = 8 # Two four-byte numbers
            await theRobot.sendCommand(Commands.SET_VEL,length,data)
            for i in range(50):
                x = abs(odomd[0] - xStart)   # update relative displacement (strictly positive)
                print(x) # for debugging

        await asyncio.gather(myRobotTasks())
        # async for msg in checkMessages():
        #    print(f"BTDebug: {msg}")


loop = asyncio.get_event_loop() # instantiate the loop object
asyncio.run(getRobot())     # find the robot on Bluetooth
asyncio.run(robotTest(loop)) # and run the tasks.
