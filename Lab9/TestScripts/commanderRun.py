#!/usr/bin/env python3
# This code only works in Python 3.7 or 3.8.
import asyncio
from bleak import discover, BleakClient
import time
from constants import Descriptors, Commands, getCommandName
from ece4960robot import Robot
from settings import Settings
from struct import pack, unpack, calcsize
from math import sin, cos, radians

# Initialize sensor reading and motor value tuples
odomd = (0,0,0,0) # x position, y position, yaw angle, ToF (floats)
veld = (0,0,0,0)  # x vel., y vel., angular vel., ToF (floats)

# Initialize PID control constants
rTh =  0        # reference, "r", a.k.a. setpoint for angular speed
kpTh = 0.75     # proportional gain, angular speed
kiTh = 1.5      # integral gain, angular speed
# For ki=1, the motor power increases by 1 after e=1 for 1 sec.
kdTh = 0.0      # derivative gain, angular speed
# For kd=1, the motor power increases by 1 when e drops by 1/sec.

rX = 0.0        # reference input for linear speed
kpX = 127       # proportional gain, linear speed
kiX = 0         # integral gain, linear speed
kdX = 0         # derivative gain, linear speed
''' update: this now runs open-loop since odometry is bad. 
kpX is the scaling factor (H).'''

tStart = time.monotonic()  # current time coord (sec)
howLong = 0     # how long to drive at a set speed (set in function)

odomList = []   # array of odometry readings and angles

isRunning = True # "kill switch" to stop checking messages once we're done


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


async def robotRun(loop, order):

    # Handle is the TX characteristic UUID; does not change in our simple case.
    # Robot sends "enq" every 2 seconds to keep the connection "fresh"
    # Otherwise, it's a struct of the form:
    # bytes(type + length + data)
    # This struct shouldn't be more than 99 bytes long.
    def simpleHandler(handle, value):
        global time,veld,odomd,levels  # This is apparently needed.
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
                odomd = unpack("<ffff", data)
                #print(odomd)

            # Unpack a tuple (xDot,thetaDot, range): little-endian floats.
            if (code == Commands.GET_VEL.value):
                veld = unpack("<ffff", data)    # float == f
                #print(veld)

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
    
    async def setVel():
        # Tell the robot how fast to drive (or stop)
        global odomChanged
        data = pack("<ff", rX, rTh)
        length = 8 # Two four-byte numbers
        await theRobot.sendCommand(Commands.SET_VEL,length,data)
    
    async def spin():
        global odomd, odomList, rX, rTh, kpX, kiX, kdX, kpTh, kiTh, kdTh
        # Send control gains
        data = pack("<ffffff", kpX, kiX, kdX, kpTh, kiTh, kdTh)
        length = 24 # Six four-byte numbers
        await theRobot.sendCommand(Commands.SET_GAINS,length,data)
        await theRobot.sendCommand(Commands.GET_ODOM)
        while odomd[2] == 0:   # wait for values to come
            if(theRobot.availMessage()):    # but still check for BT messages
                print(f"BTDebug: {theRobot.getMessage()}")
            await asyncio.sleep(0.1)
            #print("Waiting for angle...") # for debugging
        angStart = odomd[2]
        #print(angStart) # for debugging
        
        rTh = 30 # spin at 30°/second
        # Send the velocity command
        data = pack("<ff", rX, rTh)
        length = 8 # Two four-byte numbers
        await theRobot.sendCommand(Commands.SET_VEL,length,data)
        ang = 0
        while ang < 360:       # Run until the robot has made a full turn
            ang = abs(odomd[2] - angStart)   # update relative turn amount (strictly positive)
            odomList.append((ang,odomd[3]))  # save the RELATIVE angle and range to the list
            #print(ang) # for debugging
            if(theRobot.availMessage()):
                print(f"BTDebug: {theRobot.getMessage()}")
            await asyncio.sleep(0.02)
        
        rTh = 0 # Stop the robot
        # Send the velocity command
        data = pack("<ff", rX, rTh)
        length = 8 # Two four-byte numbers
        await theRobot.sendCommand(Commands.SET_VEL,length,data)
        
    async def drive():
        global rX, rTh, howLong, kpX, kiX, kdX, kpTh, kiTh, kdTh
        # Send control gains
        data = pack("<ffffff", kpX, kiX, kdX, kpTh, kiTh, kdTh)
        length = 24 # Six four-byte numbers
        await theRobot.sendCommand(Commands.SET_GAINS,length,data)
        # Request odometry data
        await theRobot.sendCommand(Commands.GET_ODOM)
        while odomd[2] == 0:   # wait for values to come
            if(theRobot.availMessage()): # while waiting, check for BT messages
                print(f"BTDebug: {theRobot.getMessage()}")
            await asyncio.sleep(0.1)
        
        # Send the velocity command (don't care what it is - set elsewhere)
        data = pack("<ff", rX, rTh)
        length = 8 # Two four-byte numbers
        await theRobot.sendCommand(Commands.SET_VEL,length,data)
        tStart = time.monotonic()
        t = 0
        while t < howLong:   # wait for values to come
            if(theRobot.availMessage()):  # while waiting, check for BT messages
                print(f"BTDebug: {theRobot.getMessage()}")
            await asyncio.sleep(0.1)
            t = time.monotonic()-tStart     # watch the time too
        rX = 0
        rTh = 0 # Stop the robot
        # Send the velocity command
        data = pack("<ff", rX, rTh)
        length = 8 # Two four-byte numbers
        await theRobot.sendCommand(Commands.SET_VEL,length,data)
        
    async def odom():
        global odomd
        await theRobot.sendCommand(Commands.GET_ODOM)
        while odomd[2] == 0:   # wait for values to come
            if(theRobot.availMessage()):    # but still check for BT messages
                print(f"BTDebug: {theRobot.getMessage()}")
            await asyncio.sleep(0.1)
            #print("Waiting for angle...") # for debugging
        # and the output is printed to a global variable.
    
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
            global kpX, kiX, kdX, kpTh, kiTh, kdTh, odomd, rX, rTh
            data = pack("<ffffff", kpX, kiX, kdX, kpTh, kiTh, kdTh)
            length = 24 # Six four-byte numbers
            await theRobot.sendCommand(Commands.SET_GAINS,length,data)
            await theRobot.sendCommand(Commands.GET_ODOM)
            rXLast = rX
            rThLast = rTh
            while isRunning:    # keep updating stuff until the code stops
                if not(rX==rXLast and rTh==rThLast): # if either one changes
                    data = pack("<ff", rX, rTh)
                    length = 8 # Two four-byte numbers
                    await theRobot.sendCommand(Commands.SET_VEL,length,data)
                    rXLast = rX # update "last" values
                    rTHLast = rTh
                    # and we should keep asking for odometry values
                    await theRobot.sendCommand(Commands.GET_ODOM)
                    await asyncio.sleep(0.05) # a whole 1/20 second
        
        if order=="obsLoop":
            await asyncio.gather(spin())
        elif order=="move":
            await asyncio.gather(drive())
        else:
            await asyncio.gather(odom())
        # async for msg in checkMessages():
        #    print(f"BTDebug: {msg}")

def readCache():
    odomAbsolute = [0,0,0]    # the robot's onboard odometry resets each time
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
            odomAbsolute[i] = float(num)
        
    return odomAbsolute

def writeCache(odomAbsolute):
    with open("odometryCache.txt", "w") as file:
        for i in range(3):
            file.write(str(odomAbsolute[i])+",")

def resetCache():
    writeCache((0,0,0))

def observationLoop():
    global odomList             # list of (theta, range) tuples
    loop = asyncio.get_event_loop()
    asyncio.run(getRobot())     # find the robot on Bluetooth
    asyncio.run(robotRun(loop, "obsLoop")) # and run the tasks.
    
    '''Post-process the angle array.'''
    length = len(odomList)      # number of entries in list
    out = []                    # output list
    
    for i in range(length):     # loop through the entire list after spinning
        currAngle = round(odomList[i][0])   # current angle (integer)
        if (currAngle%20 == 0) and (currAngle/20 == len(out)-1):
            # if the angle is a multiple of 20° AND we haven't already checked it
            #out.append(odomList[i][1])   # save the range to the list
            out.append(odomList[i])   # save the angle and the range to the list
        elif currAngle/20 > (len(out)+1): # if we skipped one
            j = i
            angleJ = round(odomList[j][0])
            while angleJ < (len(out)+1):
                j = j - 1                # average two closest angles
                angleJ = abs(round(odomList[j][0]))
            #out.append(0.5*odomList[i][1]+0.5*odomList[j][1])
            r = 0.5*odomList[i][1]+0.5*odomList[j][1]
            th = 0.5*currAngle + 0.5*angleJ
            out.append((th,r))
            
    return out
    
def move(linSpeed, angSpeed, thisLong):
    # Drive a certain distance with a certain curvature, and then stop.
    global rX, rTh, howLong
    rX = linSpeed
    rTh = angSpeed
    howLong = thisLong
    odomAbsolute = readCache() # the robot's onboard odometry resets each time
    # the Bluetooth connection dies. Save the last value and add each displacement to the last.
    x = odomAbsolute[0]
    y = odomAbsolute[1]
    th = odomAbsolute[2]
    loop = asyncio.get_event_loop()
    asyncio.run(getRobot())     # find the robot on Bluetooth
    asyncio.run(robotRun(loop, "move")) # and run the tasks.
    x = x + odomd[0]*cos(radians(th)) - odomd[1]*sin(radians(th))
    y = y + odomd[0]*sin(radians(th)) + odomd[1]*cos(radians(th))
    th = th + odomd[2]
    writeCache([x,y,th])
    return (x,y,th)

def returnPose():
    odomAbsolute = readCache() # the robot's onboard odometry resets each time
    # the Bluetooth connection dies. Save the last value and add each displacement to the last.
    x = odomAbsolute[0]
    y = odomAbsolute[1]
    th = odomAbsolute[2]
    loop = asyncio.get_event_loop()
    asyncio.run(getRobot())     # find the robot on Bluetooth
    asyncio.run(robotRun(loop, "pullOdom")) # and run the tasks.
    x = x + odomd[0]*cos(radians(th)) - odomd[1]*sin(radians(th))
    y = y + odomd[0]*sin(radians(th)) + odomd[1]*cos(radians(th))
    th = th + odomd[2]
    writeCache([x,y,th])
    return (x,y,th)

    

