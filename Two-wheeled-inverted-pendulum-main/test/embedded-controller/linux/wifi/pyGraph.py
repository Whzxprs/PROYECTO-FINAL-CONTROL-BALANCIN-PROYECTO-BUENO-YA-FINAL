# Graphing control of position
import numpy as np
import matplotlib.pyplot as plt

# Reading the data from the file
with open('datos.txt', 'r') as file:
    data = np.loadtxt(file, delimiter='\t', usecols = (0,1,2,3,4,5,6,7,8))  # Assuming data is space-separated
    # Transpose to match MATLAB's row-major structure
    data = data.T  

# Extracting data
t = data[0, :]
alpha = data[1, :]
theta = data[2, :]
v = data[3, :]
vd = data[4, :]
omegal = data[5, :]
omegar = data[6, :]
ul = data[7, :]
ur = data[8, :]

# Plotting the data
plt.figure(figsize=(10, 12))

# Subplot 1: v_d and v
plt.subplot(5, 1, 1)
plt.plot(t, vd, label=r'$v_d\left[\frac{m}{s}\right]$')
plt.plot(t, v, label=r'$v\left[\frac{m}{s}\right]$')
plt.legend(loc='best', fontsize=10)
plt.grid(True)

# Subplot 2: theta
plt.subplot(5, 1, 2)
plt.plot(t, theta, label=r'$\theta[rad]$')
plt.legend(loc='best', fontsize=10)
plt.grid(True)

# Subplot 3: alpha
plt.subplot(5, 1, 3)
plt.plot(t, alpha, label=r'$\alpha[rad]$')
plt.legend(loc='best', fontsize=10)
plt.grid(True)

# Subplot 4: omegal and ul
plt.subplot(5, 1, 4)
plt.plot(t, omegal, label=r'$\omega_l\left[\frac{rad}{s}\right]$')
plt.plot(t, ul, label=r'$u_l[V]$')
plt.ylim([-12, 12])
plt.legend(loc='best', fontsize=10)
plt.grid(True)

# Subplot 5: omegar and ur
plt.subplot(5, 1, 5)
plt.plot(t, omegar, label=r'$\omega_r\left[\frac{rad}{s}\right]$')
plt.plot(t, ur, label=r'$u_r[V]$')
plt.ylim([-12, 12])
plt.legend(loc='best', fontsize=10)
plt.xlabel(r'$t[s]$', fontsize=12)
plt.grid(True)

# Adjust layout
plt.tight_layout()
plt.show()


