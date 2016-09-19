from mpl_toolkits.mplot3d import Axes3D
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
import matplotlib.pyplot as plt
import numpy as np
import params as P


class WhirlybirdAnimation(object):

  def __init__(self):
    self.flagInit=True # Used to indicate initialization
    self.fig = plt.figure()
    self.ax = Axes3D(self.fig) # Create a 3D axes in the figure
    # A list object that will contain the lists of vertices of the
    # Whirlybird
    self.verts = self.getWhirlybirdVertices()
    # A list that will contain handles to the Poly3DCollection
    # Objects so that they can be modified.
    self.PolyCollections = []
    # Set axis limits
    _axis_limit = 1
    self.ax.set_zlim3d([-_axis_limit,_axis_limit])
    self.ax.set_ylim3d([-_axis_limit,_axis_limit])
    self.ax.set_xlim3d([-_axis_limit,_axis_limit])
    # Set title and labels
    self.ax.set_title('Whirlybird')
    self.ax.set_xlabel('East')
    self.ax.set_ylabel('North')
    self.ax.set_zlabel('-Down')
    # Change viewing angle
    self.ax.view_init(self.ax.elev, self.ax.azim+90)


  def getWhirlybirdVertices(self):
    # Base
    baseVerts = np.matrix(
      [[0,0,-P.h],
       [0,0,0]])

    bodyVerts = np.matrix(
    [[-P.l2, 0, 0],
    [P.l1, 0, 0],
    [P.l1, P.d-P.r, 0],
    [P.l1+P.r, P.d-P.r, 0],
    [P.l1+P.r, P.d+P.r, 0],
    [P.l1-P.r, P.d+P.r, 0],
    [P.l1-P.r, P.d-P.r, 0],
    [P.l1, P.d-P.r, 0],
    [P.l1, 0, 0],
    [P.l1, -P.d+P.r, 0],
    [P.l1+P.r, -P.d+P.r, 0],
    [P.l1+P.r, -P.d-P.r, 0],
    [P.l1-P.r, -P.d-P.r, 0],
    [P.l1-P.r, -P.d+P.r, 0],
    [P.l1, -P.d+P.r, 0],
    [P.l1, 0, 0]])
    return (baseVerts, bodyVerts)

  
  def drawWhirlybird(self, u):
    phi, theta, psi = u
    
    verts = []
    verts.append(np.asarray(self.verts[0]))
    vt = self.rotate(self.verts[1].T,phi,theta,psi).T
    #vt = self.translate(vt,pn,pe,pd)
    vt = self.transformXYZtoNED(vt)
    verts.append(np.asarray(vt))

    if self.flagInit:
      base = Poly3DCollection([verts[0]], facecolor='blue',edgecolor='black',lw=2)
      self.PolyCollections.append(base)
      body = Poly3DCollection([verts[1]], facecolor='red',edgecolor='black',lw=2)
      self.PolyCollections.append(body)
      for p in self.PolyCollections:
        self.ax.add_collection3d(p)
      self.flagInit = False
    else:
      for i in xrange(len(self.PolyCollections)):
        self.PolyCollections[i].set_verts([verts[i]])


  def transformXYZtoNED(self, XYZ):
    R = np.matrix([[0,1,0],
                   [1,0,0],
                   [0,0,-1]])
    NED = XYZ*R
    return NED


  def rotate(self, XYZ, phi, theta, psi):
    # Define rotation matrix
    R_roll = np.matrix([[1, 0, 0],
                        [0, np.cos(phi), -np.sin(phi)],
                        [0, np.sin(phi), np.cos(phi)]])
    R_pitch = np.matrix([[ np.cos(theta), 0, np.sin(theta)],
                        [0, 1, 0],
                        [-np.sin(theta), 0, np.cos(theta)]])
    R_yaw = np.matrix([[np.cos(psi),-np.sin(psi), 0],
                        [np.sin(psi), np.cos(psi), 0],
                        [0, 0, 1]])
    R = R_yaw*R_pitch*R_roll;
    # rotate vertices
    XYZ = R*XYZ
    return XYZ


#  def translate(self, XYZ, pn, pe, pd):
 #   return XYZ + [pn,pe,pd]
   

if __name__ == "__main__":
    # Desired configuration of Whirlybird
    phi = 15.0*np.pi/180
    theta = 45.0*np.pi/180
    psi = 90.0*np.pi/180
    # Instantiate class
    draw_Whirlybird = WhirlybirdAnimation()
    # Draw the Whirlybird with desired configuration
    draw_Whirlybird.drawWhirlybird([phi,theta,psi])
    plt.show()
  
