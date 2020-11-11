#!/usr/bin/python
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
print(odomAbsolute)

odomAbsolute = [0,0,0]
with open("odometryCache.txt", "w") as file:
    for i in range(3):
        file.write(str(odomAbsolute[i])+",")
