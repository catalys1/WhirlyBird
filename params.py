import numpy as np
import control as ctrl

# -------------------------------------------------------- #
# WHIRLYBIRD PARAMS
# -------------------------------------------------------- #

# Physical parameters
g  = 9.81
l1 = 0.85
l2 = 0.3048
m1 = 0.891
m2 = 1
d  = 0.178
h  = 0.65
r  = 0.12
Jx = 0.0047
Jy = 0.0014
Jz = 0.0041
km = 5.803  # As measured
sig_gyro = 8.7266E-5
sig_pixel = 0.05

#Initial conditions
phi0 = 0.0
theta0 = 0.0
psi0 = 0.0
phidot0 = 0.0
thetadot0 = 0.0
psidot0 = 0.0
Feq = (m1*l1 - m2*l2)*g / l1

Ts = 0.01
tau = 0.05

# -------------------------------------------------------- #
# CONTROL PARAMETERS
# -------------------------------------------------------- #
#kp_th = 2.604
#kd_th = 3.472
#kp_th = 86.8056
#kd_th = 12.2743
zeta_lon = 0.707
zeta_lat = 0.75

# Longitudinal (theta)
b_th = l1/(m1*l1**2+m2*l2**2+Jy)
tr_th = 1.5
wn_th = 2.2 / tr_th
kp_th = wn_th**2/b_th
kd_th = 2*zeta_lon*wn_th/b_th
ki_th = 0.01

# Lateral (phi, psi)
tr_phi = 0.35
wn_phi = 2.2 / tr_phi
kp_phi = wn_phi**2 * Jx
kd_phi = 2*zeta_lat*wn_phi*Jx

tr_psi = tr_phi
wn_psi = 2.2 / tr_psi
b_psi = l1*Feq/(m1*l1**2+m2*l2**2+Jz)

kp_psi = wn_psi**2/b_psi
kd_psi = 2*zeta_lat*wn_psi/b_psi
ki_psi = 0.015


# -------------------------------------------------------- #
# FULL STATE 
# -------------------------------------------------------- #
# Longitudinal
num = (m1*l1 - m2*l2)*g*np.sin(theta0)
denom = (m1*l1**2 + m2*l2**2 + Jy)
A_lon = np.matrix([
	[0.0, 1.0],
	[num/denom, 0.0] 
])
B_lon = np.matrix([
	[0.0],
	[l1/denom]
])
C_lon = np.matrix([
	[1.0, 0.0]
])
cl_poles_lon = list(np.roots([1,2*zeta_lon*wn_th,wn_th**2]))
K_lon = ctrl.acker(A_lon, B_lon, cl_poles_lon)
kr_lon = -1.0 / (C_lon * (np.linalg.inv(A_lon - B_lon*K_lon) * B_lon)).item(0)

# Lateral
zeta_phi = 0.75
zeta_psi = 0.707
c = l1*Feq / (m1*l1**2 + m2*l2**2 + Jz)
A_lat = np.matrix([
	[0.0, 0.0, 1.0, 0.0],
	[0.0, 0.0, 0.0, 1.0],
	[0.0, 0.0, 0.0, 0.0],
	[c  , 0.0, 0.0, 0.0]
])
B_lat = np.matrix([
	[0.0],
	[0.0],
	[1.0/Jx],
	[0.0]
])
C_lat = np.matrix([
	[1,0,0,0],
	[0,1,0,0]
])
Cr_lat = C_lat[1,:]
cl_poles_lat = list(np.roots([1,2*zeta_phi*wn_phi,wn_phi**2])) + \
		list(np.roots([1,2*zeta_psi*wn_psi,wn_psi**2]))
K_lat = ctrl.acker(A_lat, B_lat, cl_poles_lat)
print A_lat
print B_lat
print K_lat
print B_lat*K_lat
print A_lat - B_lat*K_lat
print np.linalg.inv(A_lat - B_lat*K_lat)
print np.linalg.inv(A_lat - B_lat*K_lat)*B_lat
print C_lat * (np.linalg.inv(A_lat - B_lat*K_lat)*B_lat)
kr_lat = -1.0 / (Cr_lat * (np.linalg.inv(A_lat - B_lat*K_lat) * B_lat)).item(0)


if __name__ == '__main__':
	print '-- PID --'
	print '  Theta: kp = %s, kd = %s' % (kp_th, kd_th)
	print '  Phi: kp = %s, kd = %s' % (kp_phi, kd_phi)
	print '  Psi: kp = %s, kd = %s' % (kp_psi, kd_psi)
	print '-- FULL STATE --'
	print '  theta poles = {:.3f}, {:.3f}'.format(*cl_poles_lon)
	print '  K_lon = %s' % K_lon
	print '  kr_lon = %s' % kr_lon
	print '  phi poles = {:.3f}, {:.3f}'.format(*cl_poles_lat[:2])
	print '  psi poles = {:.3f}, {:.3f}'.format(*cl_poles_lat[2:])
	print '  K_lat = %s' % K_lat
	print '  kr_lat = %s' % kr_lat
