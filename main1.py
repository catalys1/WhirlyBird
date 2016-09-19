import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as mpatch
import matplotlib.animation as ani
import params as P
from WhirlybirdAnimation import WhirlybirdAnimation

animator = WhirlybirdAnimation()

t = np.arange(0.0,4*np.pi,0.1)
theta = np.pi/2 * np.sin(2*t)
phi   = np.pi/2 * np.sin(t)
psi   = np.pi/2 * np.sin(1.5*t)

def init():
	hands = animator.drawWhirlybird((P.theta0,P.phi0,P.psi0))
	return hands

def draw(i):
	hands = animator.drawWhirlybird((theta[i],phi[i],psi[i]))
	return hands

def convertForces(self, u):
	F, tau = u

	fl = F/2 + tau/(2*d)
	fr = F/2 - tau/(2*d)

	return [fl, fr]

anim = ani.FuncAnimation(animator.fig,draw,len(t),interval=100,blit=False,init_func=init)
#anim.save('planarVTOL.mp4', ani.FFMpegWriter())
plt.show()
