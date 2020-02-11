import numpy as np

class CNC_5dof:

  def __init__(self):
    print("Initializing CNC_5dof")
    self.offset_sensor = 
    self.Lx = 100
    self.Ly = 200
    self.Lz = 280
    self.h = 80
    self.l = 160

  def fwd_kin(self, d1, d2, d3, theta4, theta5):
    """
    Calculates forward kinematics based on 3 translational and 2 rotational
    joints.
    Args:
      d1: joint variable representing displacement along x axis
      d2: joint variable representing displacement along y axis
      d3: joint variable representing displacement along z axis
      theta4: joint variable representing rotation about x axis
      theta5: joint variable representing rotation about z axis
    Returns:
    """
    print("Calculating forward kinematics")
    tf_T_L = np.array( [ [-1,0,0,-d3], [0,1,0,-d2], [0,0,-1,-d1], [0,0,0,1]] )
    tf_L_O = np.array( [ [1,0,0,self.Lx],
                         [0,1,0,self.Ly],
                         [0,0,1,self.Lz + self.h],
                         [0,0,0,1]] )

    tf_1 = np.array( [ [1,0,0,0],
                       [0,np.cos(theta4),-np.sin(theta4),0],
                       [0,np.sin(theta4),np.cos(theta4),0],
                       [0,0,0,1]] )
    tf_2 = np.array( [ [1,0,0,0], [0,1,0,0], [0,0,1,self.l], [0,0,0,1]] )
    tf_3 = np.array( [ [np.cos(theta5),-np.sin(theta5),0,0],
                       [np.sin(theta5),np.cos(theta5),0,0],
                       [0,0,1,0],
                       [0,0,0,1]] )
    tf_O_W = tf_1 * tf_2 * tf_3

    tf_T_W = tf_T_L * tf_L_O * tf_O_W
    tf_W_T = -tf_T_W[0:3,0:3].T * tf_T_W[0:3,3]

    return tf_W_T

  def inv_kin(self, p, v):
    """
    Calculates inverse kinematics based on desired location p and desired
    orientation, vector v.
    Args:
      p: desired 3D location of tool tip
      v: desired z-axis orientation of tool tip relative to workpiece
          coordinate frame
    Returns:
      d1: joint variable representing displacement along x axis
      d2: joint variable representing displacement along y axis
      d3: joint variable representing displacement along z axis
      theta4: joint variable representing rotation about x axis
              [0, pi/2] radians
      theta5: joint variable representing rotation about z axis
              [-pi, pi] radians
    """
    print("Calculating inverse kinematics.")
    theta4 = np.arccos(v[2])
    theta5 = np.arctan(v[0]/v[1])

    A = np.array([ [-np.sin(theta4)*np.sin(theta5), np.cos(theta4)*np.sin(theta5), -np.cos(theta5)],
                   [-np.sin(theta4)*np.cos(theta5), np.cos(theta4)*np.cos(theta5), np.sin(theta5)],
                   [np.cos(theta4), -np.sin(theta4), 0] ])
    B = np.array([ p[0] + self.Lx*np.cos(theta5) + self.Ly*np.cos(theta4)*np.sin(theta5) + (self.Lz + self.h)*np.sin(theta4)*np.sin(theta5),
                   p[1] - self.Lx*np.sin(theta5) + self.Ly*np.cos(theta4)*np.cos(theta5) + (self.Lz + self.h)*np.sin(theta4)*np.cos(theta5),
                   p[2] - self.Ly*np.sin(theta4) + (self.Lz + self.h)*np.cos(theta4) - self.l] )