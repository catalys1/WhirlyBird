#!/usr/bin/python

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as mpatch
import matplotlib.animation as ani
import params as P
import argparse
from WhirlybirdAnimation import WhirlybirdAnimation
from WhirlybirdDynamics import WhirlybirdDynamics
from WhirlybirdController import WhirlybirdController


#def convertForces(u):
#	F, tau = u
#
#	fl = F/2 + tau/(2*P.d)
#	fr = F/2 - tau/(2*P.d)
#
#	return [fl, fr]

parser = argparse.ArgumentParser()
parser.add_argument('inputs', type=float, help='Reference height')
args = parser.parse_args()

h_ref = args.inputs

controller = WhirlybirdController()
animator = WhirlybirdAnimation()
dynamics = WhirlybirdDynamics(controller)

t_start = 0.0
t_end = 20.0
t_Ts  = 0.01
t_elapse = 0.1
t_pause = 0.01

t = t_start

hz = 0.05
period = 1.0 / hz
t_count = t
go_up = True
go_dwn = False

inpt = []
outpt = []

while t < t_end:

  if t_count < period/2.0:
    if go_up:
      h_ref = 15 * np.pi/180.0
      go_up = False
      go_dwn = True
  else:
    if go_dwn:
      h_ref = -h_ref #15 * np.pi/180.0
      go_up = True
      go_dwn = False
    
  if t_count >= period:
    t_count = 0.0
  t_count += .1

  inpt.append(h_ref)

  plt.figure(animator.fig.number)
  plt.pause(0.001)
  t_temp = t+t_elapse
  while t < t_temp:
    dynamics.propogateDynamics(h_ref)
    t += t_Ts
  plt.figure(animator.fig.number)
  outputs = dynamics.Outputs()
  outpt.append(outputs[1])
  animator.drawWhirlybird(outputs)

dynamics.Outputs()

t = np.linspace(t_start, t_end, len(inpt))
print len(t), len(inpt), len(outpt)
plt.figure()
plt.plot(t, inpt, 'b:')
plt.plot(t, outpt, 'r-')
plt.show()
#raw_input()



