#!/usr/bin/python3
import numpy as np
import matplotlib.pyplot as pylot

sq2m = 6/39.37      # tile squares to meters conversion
deg2rad = np.pi/180 # degrees to radians conversion

for i in range(3):
    filename = "Rotation" + str(i+1) + ".csv"
    polar = np.genfromtxt(filename,delimiter=',')
    lenCart = len(polar[:,1])   # length of array to be generated
    thetas = polar[0:-1,0]      # first column
    rs = polar[0:-1,1]          # second column
    cart = np.empty([lenCart,3]) # starting array for Cartesian coordinates
    if i==0:
        xr = 3.5*sq2m           # x offset, m
        yr = 3*sq2m           # y offset, m (-0.5 for turn radius)
    elif i==1:
        xr = 3.5*sq2m           # x offset, m
        yr = 14*sq2m           # y offset, m (-0.5 for turn radius)
    elif i==2:
        xr = 3.5*sq2m           # x offset, m
        yr = 24*sq2m           # y offset, m (-0.5 for turn radius)
    xy = np.array([(xr,yr)]).reshape(2,1)   # 2x1 column vector
    for j in range(lenCart-1):
        th = (-thetas[j]-25)*deg2rad  # angles are in degrees; backwards
        d = np.zeros([3,1])       # 3x1 vector for ToF dist. measurement
        d[0] = rs[j]/1000       # distances are in mm (first element of d)
        d[1] = 0.5              # y offset (distance from center point of arc)
        d[2] = 1                # and the last entry should be 1
        R = np.array([(np.cos(th), -np.sin(th)),(np.sin(th), np.cos(th))])
        T = np.block([[R, xy],[0,0,1]])
        Td= np.matmul(T,d)       # and the matrix math pays off.
        cart[j,:] = Td.reshape(3)
    if i==0:
        cart1 = cart
    elif i==1:
        cart2 = cart
    elif i==2:
        cart3 = cart
cartesianCoords = np.concatenate((cart1,cart2,cart3),axis=0) # looong x,y array
#cartesianCoords = cart1
xs = cartesianCoords[:,0]
ys = cartesianCoords[:,1]
fig = pylot.figure()
ax = fig.add_axes([0,0,1,1])
pylot.plot(xs,ys,'g.')
ax.set_xlim(-1,5)
ax.set_ylim(-1,5)
pylot.show()
output = np.vstack([xs,ys]).T
np.savetxt("Points.csv",output,delimiter=",")
