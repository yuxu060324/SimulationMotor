import numpy as np
import matplotlib.pyplot as plt
from control.matlab import *

# from scipy import arange

pi = np.pi
# モータの特性
R = 0.65
L = 0.0021
J = 0.0005
Ke = 0.03
Kt = 0.03

G1 = tf(1, [L, R]) * Kt * tf(1, [J, 0])
G2 = tf(Ke, 1)

Gp = feedback(G1, G2)
print(Gp)

(y1, t1) = step(Gp, T=np.arange(0, 3, 0.0001))

y_rpm = y1 * 60 / (2 * pi)

plt.plot(t1, y_rpm)
plt.grid(True)
plt.xlabel('Time[s]')
plt.ylabel('Rev[rpm]')
plt.xlim(0, 3)
plt.ylim(0, 350)
plt.show()
