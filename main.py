import numpy as np
import matplotlib.pyplot as plt
from control.matlab import *
from scipy.integrate import odeint
import sympy as sym

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

# odeint

def system(y, t):
    if t < 10 :
        u = 0
    else:
        u = 1
    dydt = (-y + u) / 5
    return dydt

y0 = 0.5
t = np.arange(0, 40, 0.04)
y = odeint(system, y0, t)

plt.plot(t, y)
plt.plot(t, 1 * (t>=10))
plt.show()

# ラプラス変換

Np = [1, 3]
Dp = [1, 5, 8, 4]
P = tf(Np, Dp)
print(P)

Np = [1, 3]
Dp1 = [1, 1]
Dp2 = [1, 2]
P = tf(Np, Dp1) * tf([1], Dp2)**2
print(P)

P = tf([1], [1, 2, 3])
print(P)

# 状態空間モデル

A = '1, 1, 2; 2, 1, 1; 3, 4, 5'
B = '2; 0; 1'
C = '1, 1, 0'
D = 0

P = ss(A, B, C, D)
print(P)

# システムの結合

S1 = tf([0, 1], [1, 1])
S2 = tf([1, 1], [1, 1, 1])
S = series(S1, S2)
print(S)

S = parallel(S1, S2)
print(S)