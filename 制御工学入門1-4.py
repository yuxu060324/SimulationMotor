import control
import numpy as np
import sympy as sp
import scipy
import matplotlib.pyplot as plt
from control.matlab import *
from scipy.integrate import odeint

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
    if t < 10:
        u = 0
    else:
        u = 1
    dydt = (-y + u) / 5
    return dydt


y0 = 0.5
t = np.arange(0, 40, 0.04)
y = odeint(system, y0, t)

plt.plot(t, y)
plt.plot(t, 1 * (t >= 10))
plt.show()

# ラプラス変換

Np = [1, 3]
Dp = [1, 5, 8, 4]
P = tf(Np, Dp)
print(P)

Np = [1, 3]
Dp1 = [1, 1]
Dp2 = [1, 2]
P = tf(Np, Dp1) * tf([1], Dp2) ** 2
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

S = feedback(S1, S2).minreal()
print(S)

S1 = tf(1, [1, 1])
S2 = tf(1, [1, 2])
S3 = tf([3, 1], [1, 0])
S4 = tf([1, 0], 1)

P12 = feedback(S1, S2)
P1234 = feedback(S3 * P12, S4)
print(P1234)

# 状態空間モデル→伝達関数モデル

P = tf([0, 1], [1, 1, 1])

Pss = tf2ss(P)
print(Pss)

# 伝達関数モデル→状態空間モデル

Ptf = ss2tf(Pss)
print(Ptf)

# 伝達関数モデルから状態空間モデル

A = '1 2 3; 3 2 1; 4 5 0'
B = '1; 0; 1'
C = '0 2 1'
D = '0'
Pss = ss(A, B, C, D)

# reachable:可制御正準形, observable:可観測正準形
Pr, T = control.canonical_form(Pss, form='observable')
print(Pr)


#
def linestyle_generator():
    linestyle = ['-', '--', '-.', ':']
    lineID = 0
    while True:
        yield linestyle[lineID]
        lineID = (lineID + 1) % len(linestyle)


def plot_set(fig_ax, *args):
    fig_ax.set_xlabel(args[0])
    fig_ax.set_ylabel(args[1])
    fig_ax.grid(ls=':')
    if len(args) == 3:
        fig_ax.legend(loc=args[2])


def bodeplot_set(fig_ax, *args):
    # ゲイン線図のグリッドとy軸ラベルの設定
    fig_ax.grid(which='both', ls=':')
    fig_ax.set_ylabel('Gain [dB]')
    # 位相線図のグリッドとｘ軸、ｙ軸ラベルの設定
    fig_ax[1].grid(which='both', ls=':')
    fig_ax[1].set_xlabel('$\omega$ [rad/s]')
    fig_ax[1].set_ylabel('Phase [deg]')
    # 凡例の表示
    if len(args) > 0:
        fig_ax[1].legend(loc=args[0])
    if len(args) > 1:
        fig_ax[0].legend(loc=args[1])


T, K = 0.5, 1
P = tf([0, K], [T, 1])
y, t = step(P, np.arange(0, 5, 0.01))

fig, ax = plt.subplots()
ax.plot(t, y)
plot_set(ax, 't', 'y')

# 1次遅れ系のステップ応答(時定数Tの変化)
LS = linestyle_generator()

K = 1
T = (1, 0.5, 0.1)
for i in range(len(T)):
    y, t = step(tf([0, K], [T[i], 1]), np.arange(0, 5, 0.01))
    ax.plot(t, y, ls=next(LS), label='T='+str(T[i]))

plot_set(ax, 't', 'y', 'best')

# 1次遅れ系のステップ応答(ゲインKの変化)
T = 0.5
K = [1, 2, 3]
for i in range(len(K)):
    y, t = step(tf([0, K[i]], [T, 1]), np.arange(0, 5, 0.01))
    ax.plot(t, y, ls=next(LS), label='K='+str(K[i]))

plot_set(ax, 't', 'y', 'upper left')

# 2次遅れ系のステップ応答
zeta, omega_n = 0.4, 5  # 減衰係数と固有角周波数の設定

P = tf([0, omega_n**2], [1, 2*zeta*omega_n, omega_n**2])
y, t = step(P, np.arange(0, 5, 0.01))

fig, ax = plt.subplots()
ax.plot(t, y)
plot_set(ax, 't', 'y')
plt.show()

# 2次遅れ系のステップ応答(減衰関数)
fig, ax = plt.subplots()
LS = linestyle_generator()

zeta = [1, 0.7, 0.4]
omega_n = [1, 5, 10]
for i in range(len(zeta)):
    for j in range(len(omega_n)):
        P = tf([0, omega_n[j]**2], [1, 2*zeta[i]*omega_n[j], omega_n[j]**2])
        y, t = step(P, np.arange(0,5, 0.01))

        pltargs = {'ls': next(LS)}
        pltargs['label'] = '$\zeta$='+str(zeta[i])+', $\omega_n$='+str(omega_n[j])
        ax.plot(t, y, **pltargs)

plot_set(ax, 't', 'y', 'best')
plt.show()

# 伝達関数のステップ応答性
Ps, t = step(tf([1, 3], [1, 3, 2]), np.arange(0, 5, 0.01))
print(Ps)
plt.plot(t, Ps, label="")
plt.legend()
plt.show()

# 状態空間モデルの時間応答

A = [[0, 1],
     [-4, -5]]
B = [[0],
     [1]]
C = np.eye(2)
D = np.zeros([2, 1])
P = ss(A, B, C, D)

Td = np.arange(0, 5, 0.01)
X0 = [-0.3, 0.4]

x, t = initial(P, Td, X0)

plt.plot(t, x[:, 0])
plt.plot(t, x[:, 1])
plt.show()

# 逆ラプラス変換
sp.init_printing()
s = sp.Symbol('s')
t = sp.Symbol('t', positive=True)

A = np.array([[0, 1], [-4, -5]])
G = s*sp.eye(2)-A
exp_At = sp.inverse_laplace_transform(sp.simplify(G.inv()), s, t)

A = np.array([[0, 1], [-4, -5]])
t = 5
scipy.linalg.expm(A*t)

# 状態空間モデルの零状態応答
Td = np.arange(0, 5, 0.01)
x, t = step(P, Td)

fig, ax = plt.subplots()
ax.plot(t, x[:, 0])
ax.plot(t, x[:, 1])
plt.show()

# 状態空間モデルの時間応答
Td = np.arange(0, 5, 0.01)
Ud = 1 * (Td > 0)   #入力関数 u(t)
X0 = [-0.3, 0.4]

xst, t = step(P, Td)    #零状態応答
xin, _ = initial(P, Td, X0) #零入力応答
x, _, _ = lsim(P, Ud, Td, X0)

fig, ax = plt.subplots(1, 2, figsize=(6, 2.3))
for i in [0, 1]:
    ax[i].plot(t, x[:, i], label='response')
    ax[i].plot(t, xst[:, i], ls='--', label='zero state')
    ax[i].plot(t, xin[:, i], ls='-.', label='zero input')

plot_set(ax[0], 't', '$x_1$')
plot_set(ax[1], 't', '$x_2$', 'best')
fig.tight_layout()
plt.show()

# 練習問題
Td = np.arange(0, 5, 0.01)
Ud = 3 * np.sin(5*Td)
X0 = [0.5, 1]

# 位相面図
w = 1.5
Y, X = np.mgrid[-w:w:100j, -w:w:100j]

A = np.array([[0, 1], [-4, 5]])
s, v = np.linalg.eig(A)
U = A[0, 0]*X + A[0, 1]*Y
V = A[1, 0]*X + A[1, 1]*Y

t = np.arange(-1.5, 1.5, 0.01)
fig, ax = plt.subplots()

if s.imag[0]==0 and s.imag[1]==0:
    ax.plot(t, (v[1,0]/v[0,0])*t, ls='-')
    ax.plot(t, (v[1,1]/v[0,1])*t, ls='-')

ax.streamplot(X, Y, U, V, density=0.7, color='k')
plot_set(ax, '$x_1$', '$x_2$')
plt.show()

# 1次遅れ系の周波数特性
K = 1
T = [1, 0.5, 0.1]

LS = linestyle_generator()
fig, ax = plt.subplots(2, 1)
for i in range(len(T)):
    P = tf([0, K], [T[i], 1])
    gain, phase, w = bode(P, logspace(-2, 2))
    pltargets = {'ls':next(LS), 'label': 'T='+str(T[i])}
    ax[0].semilogx(w, 20*np.log(gain), **pltargets)
    ax[1].semilogx(w, phase*180/np.pi, **pltargets)

# bodeplot_set(ax, 3, 3)
plt.show()

# 2次遅れ系の周波数特性
zeta = [1, 0.7, 0.4]
omega_n = 1

LS = linestyle_generator()
fig, ax = plt.subplots(2, 1)
for i in range(len(zeta)):
    P = tf([0, omega_n**2], [1, 2*zeta[i]*omega_n, omega_n**2])
    gain, phase, w = bode(P, logspace(-2, 2))

    pltargs = {'ls':next(LS)}
    pltargs['label'] = '$\zeta$='+str(zeta[i])
    ax[0].semilogx(w, 20*np.log10(gain), **pltargs)
    ax[1].semilogx(w, phase*180/np.pi, **pltargs)

plt.show()


