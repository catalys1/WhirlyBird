
s_phi = np.sin(phi)
s_theta = np.sin(theta)
c_phi = np.cos(phi)
c_theta = np.cos(theta)
c_psi = np.cos(psi)

M = np.matrix([
  [Jx, 0, -Jx*s_theta],
  [0, m1*l1**2 + m2*l2**2 + Jy*c_phi**2 + Jz*s_phi**2, (Jy-Jz)*s_phi*c_phi*c_theta],
  [-Jx*s_phi, (Jy-Jz)*s_phi*c_phi*c_theta, 
    (m1*l1**2+m2*l2**2+Jy*s_phi**2)*c_theta**2+Jx*s_theta**2]
])


qddot = np.linalg.inv(M) * (Q-c-dP_dq)
