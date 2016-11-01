
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
km = 0
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
# Control parameters
#kp_th = 2.604
#kd_th = 3.472
#kp_th = 86.8056
#kd_th = 12.2743
zeta = 0.7

b_th = l1/(m1*l1**2+m2*l2**2+Jy)
tr_th = 1.4
wn_th = 2.2 / tr_th
kp_th = wn_th**2/b_th
kd_th = 2*zeta*wn_th/b_th

tr_phi = 0.3
wn_phi = 2.2 / tr_phi
kp_phi = wn_phi**2 * Jx
kd_phi = 2*zeta*wn_phi*Jx

tr_psi = 10.0 * tr_phi
wn_psi = 2.2 / tr_psi
b_psi = l1*Feq/(m1*l1**2+m2*l2**2+Jz)

kp_psi = wn_psi**2/b_psi
kd_psi = 2*zeta*wn_psi/b_psi

if __name__ == '__main__':
	print 'Theta: kp = %s, kd = %s' % (kp_th, kd_th)
	print 'Phi: kp = %s, kd = %s' % (kp_phi, kd_phi)
	print 'Psi: kp = %s, kd = %s' % (kp_psi, kd_psi)
