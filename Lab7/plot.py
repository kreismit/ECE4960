#!/usr/bin/python3
import numpy as np
import matplotlib.pyplot as plt
from scipy import pi

plotData = np.genfromtxt('test.csv',delimiter=',')
thetas = plotData[0:-1,0]    # first column
rs = plotData[0:-1,1]        # second column
plt.polar(thetas*pi/180, rs, 'b.')
#plt.polar(rs,thetas)
plt.show()
