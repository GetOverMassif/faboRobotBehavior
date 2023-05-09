import matplotlib.pyplot as plt
import numpy as np
from mpl_toolkits.mplot3d import Axes3D
import math

# 外轮廓函数
rho, theta = np.mgrid[0:1:40j, 0:2 * np.pi:40j]
z = rho ** 2
x = rho * np.cos(theta)
y = rho * np.sin(theta)

fig = plt.figure()
ax = Axes3D(fig)

# 添加等高线图:rstride,cstride指的是行跨度/列跨度,越大越稀疏
surf = ax.plot_surface(x, y, z, rstride=1, cstride=1, cmap=plt.get_cmap('rainbow'), alpha=0.6)
# zdir:等高线从哪里压下去,offset:等高线位于轴的哪个位置
ax.contourf(x, y, z, zdir='z', offset=0, cmap='rainbow', alpha=0.6)
ax.set_zlim(0, 1)

ax.set_xlabel(r'$x$')
ax.set_ylabel(r'$y$')
ax.set_zlabel(r'$z$')

ax = fig.gca(projection='3d')

# 画出各个维度标度线
joyx = np.linspace(0, 0, 40)
joyy = np.linspace(0, 1, 40)
joyz = joyy ** 2
ax.plot(joyx, joyy, joyz, label='Joy', color='#fcb001', linewidth=3)
ax.plot(joyx, joyy, color='#fcb001', linewidth=3)

sadx = np.linspace(0, 0, 40)
sady = np.linspace(-1, 0, 40)
sadz = sady ** 2
ax.plot(sadx, sady, sadz, label='Sadness', color='#0e87cc', linewidth=3)
ax.plot(sadx, sady, color='#0e87cc', linewidth=3)

angx = np.linspace(0, -1, 40)
angy = np.linspace(0, 0, 40)
angz = angx ** 2
ax.plot(angx, angy, angz, label='Anger', color='#ff0789', linewidth=3)
ax.plot(angx, angy, color='#ff0789', linewidth=3)

ferx = np.linspace(0, 1, 40)
fery = np.linspace(0, 0, 40)
ferz = angx ** 2
ax.plot(ferx, fery, ferz, label='Fear', color='#2a7e19', linewidth=3)
ax.plot(ferx, fery, color='#2a7e19', linewidth=3)

trux = np.linspace(0, 1 / math.sqrt(2), 40)
truy = np.linspace(0, 1 / math.sqrt(2), 40)
truz = (truy ** 2) * 2
ax.plot(trux, truy, truz, label='Trust', color='#9ce143', linewidth=3)
ax.plot(trux, truy, color='#9ce143', linewidth=3)

disx = np.linspace(0, -1 / math.sqrt(2), 40)
disy = np.linspace(0, -1 / math.sqrt(2), 40)
disz = (disy ** 2) * 2
ax.plot(disx, disy, disz, label='Disgust', color='#a442a0', linewidth=3)
ax.plot(disx, disy, color='#a442a0', linewidth=3)

antx = np.linspace(0, -1 / math.sqrt(2), 40)
anty = np.linspace(0, 1 / math.sqrt(2), 40)
antz = (anty ** 2) * 2
ax.plot(antx, anty, antz, label='Antipation', color='#ff5b00', linewidth=3)
ax.plot(antx, anty, color='#ff5b00', linewidth=3)

supx = np.linspace(0, 1 / math.sqrt(2), 40)
supy = np.linspace(0, -1 / math.sqrt(2), 40)
supz = (supy ** 2) * 2
ax.plot(supx, supy, supz, label='Surprise', color='#02ccfe', linewidth=3)
ax.plot(supx, supy, color='#02ccfe', linewidth=3)

# annotation
xs = (0, 0, -1, 1, 1 / math.sqrt(2), -1 / math.sqrt(2), -1 / math.sqrt(2), 1 / math.sqrt(2))
ys = (1, -1, 0, 0, 1 / math.sqrt(2), -1 / math.sqrt(2), 1 / math.sqrt(2), -1 / math.sqrt(2))
zs = (1, 1, 1, 1, 1, 1, 1, 1)
zdir = ('Joy', 'Sadness', 'Anger', 'Fear', 'Trust', 'Disgust', 'Antipation', 'Surprise')
for x, y, z, zdir in zip(xs, ys, zs, zdir):
    label = '$%s$' % zdir
    ax.text(x, y, z, label)

# 绘制阈值分割线,下列三个阈值参数可直接修改
mild = 0.35
moderate = 0.6
intense = 0.8

rho_xy = np.mgrid[0:0:40j]  # 这个是xy平面上的小圈圈参数,修改时不用动
thz_xy = rho_xy ** 2

theta1 = np.mgrid[0:2 * np.pi:40j]
rho1 = np.mgrid[mild:mild:40j]
thx1 = rho1 * np.cos(theta1)
thy1 = rho1 * np.sin(theta1)
thz1 = rho1 ** 2
ax.plot3D(thx1, thy1, thz1, '--', color='#000000')
ax.plot3D(thx1, thy1, thz_xy, '--', color='#000000')

theta2 = np.mgrid[0:2 * np.pi:40j]
rho2 = np.mgrid[moderate:moderate:40j]
thx2 = rho2 * np.cos(theta2)
thy2 = rho2 * np.sin(theta2)
thz2 = rho2 ** 2
ax.plot3D(thx2, thy2, thz2, '--', color='#000000')
ax.plot3D(thx2, thy2, thz_xy, '--', color='#000000')

theta3 = np.mgrid[0:2 * np.pi:40j]
rho3 = np.mgrid[intense:intense:40j]
thx3 = rho3 * np.cos(theta2)
thy3 = rho3 * np.sin(theta2)
thz3 = rho3 ** 2
ax.plot3D(thx3, thy3, thz3, '--', color='#000000')
ax.plot3D(thx3, thy3, thz_xy, '--', color='#000000')

# 添加颜色棒
cb = fig.colorbar(surf, shrink=0.7, aspect=20, label='$Emotion$')
cb.set_ticks([mild, moderate, intense])
cb.ax.set_yticklabels(['$%s-mild$' % mild, '$%s-moderate$' % moderate, '$%s-intense$' % intense])

# 显示图例
ax.legend()

plt.show()

