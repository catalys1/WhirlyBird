#!/usr/bin/python

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as mpatch
import matplotlib.animation as ani
import params as P
import argparse
from WhirlybirdAnimation import WhirlybirdAnimation
from WhirlybirdDynamics import WhirlybirdDynamics


def convertForces(u):
	F, tau = u

	fl = F/2 + tau/(2*P.d)
	fr = F/2 - tau/(2*P.d)

	return [fl, fr]

parser = argparse.ArgumentParser()
parser.add_argument('inputs', nargs='*')
args = parser.parse_args()

u = [float(x) for x in args.inputs]
F, tau = 0,0
if not u:
  F = 5.22
elif len(u) == 1:
  F = u[0]
else:
  F, tau = u[0], u[1]
u = convertForces([F, tau])

animator = WhirlybirdAnimation()
dynamics = WhirlybirdDynamics()

t_start = 0.0
t_end = 10.0
t_Ts  = 0.01
t_elapse = 0.1
t_pause = 0.01

t = t_start

while t < t_end:
  plt.ion()
  plt.figure(animator.fig.number)
  plt.pause(0.001)
  t_temp = t+t_elapse
  while t < t_temp:
    dynamics.propogateDynamics(u)
    t += t_Ts
  plt.figure(animator.fig.number)
  animator.drawWhirlybird(dynamics.Outputs())
