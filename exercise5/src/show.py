import matplotlib.pyplot as plt
import numpy as np

fig, axes = plt.subplots(4, 3, sharex='col', sharey='row')

files = ["exercise5_ideal_ff.csv", "exercise5_real_ff.csv", "exercise5_real_ff_fb.csv"]
titles = ["Ideal Robot FF", "Real Robot FF", "Real Robot FF+FB (PI)"]

for i, file in enumerate(files):
    t, x, y, z, fx, fy, fz, mx, my, mz = np.loadtxt(file, unpack=True, delimiter=',')

    ax = axes[0, i]
    ax.plot(t, z)
    ax.grid(True)
    ax.set_ylabel("z [m]")

    ax = axes[1, i]
    ax.plot(t, fx)
    ax.plot(t, t * 0.0, 'g:')
    ax.set_ylabel("fx [N]")

    ax = axes[2, i]
    ax.plot(t, fy)
    ax.plot(t, t * 0.0, 'g:')
    ax.set_ylabel("fy [N]")

    ax = axes[3, i]
    ax.plot(t, fz)
    ax.plot(t, t * 0.0 - 1.0, 'g:')
    ax.set_ylabel("fz [N]")

for ax in axes.flatten():
    ax.grid(True)

for i, ax in enumerate(axes[0]):
    ax.set_title(titles[i])

for ax in axes[3]:
    ax.set_xlabel("time [s]")

plt.show()
