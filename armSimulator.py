from pylab import *
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import time

fig = figure()
ax = Axes3D(fig)
X = np.arange(-4, 4, 0.25)
#Y = np.arange(-4, 4, 0.25)
#X, Y = np.meshgrid(X, Y)
#R = np.sqrt(X**2 + Y**2)
#Z = np.sin(R)

#ax.plot_surface(X, Y, Z, rstride=1, cstride=1, cmap='hot')
ax.plot_surface(X, Y, Z)

show()
time.sleep(2)

print('a')
fig.close()
print('b')
