#!/usr/bin/python3
#import testOdom
import commanderOld

#readings = testOdom.testMyCode()

#readings = commander.observationLoop()

# The first reading needs to be in the direction
# of the robot's original heading.
# But, out has readings at angles 20:20:360.
#readings.insert(0, readings.pop(-1))  # move the last element to the front
#print(readings)

#print(commander.returnPose())

commanderOld.move(0, 100, 10)
