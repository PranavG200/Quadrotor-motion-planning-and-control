import numpy as np
from scipy.interpolate import CubicSpline
import matplotlib.pyplot as plt
from random import uniform

def unicycle_spline(t0, tf, obs):
  # UNICYCLE_SPLINE returns a spline object representing a path from
  # (y(t0),z(t0)) = (0,0) to (y(t0),z(t0)) = (10,0) that avoids a circular
  # obstacle, such that d\dt y(t) > 0
  #   @param t0 - initial time
  #   @param tf - final time
  #
  #   @return y_spline - spline object for desired y trajectory
  #   @return z_spline - spline object for desired z trajectory
  y0 = 0;
  z0 = 0;

  yf = 10;
  zf = 0;
  time = np.uint8(tf-t0)

  # TODO: design the spline here
  t = np.linspace(t0,tf,4)
  y = np.linspace(y0,yf,4)
  z = np.linspace(z0,zf,4)

  for i in range(1,3):
      #if (np.linalg.norm(np.array((y[i],z[i]))-np.array((obs.y,obs.z)))<obs.radius):
      if (z[i]>=obs.z):
          z[i] = obs.z + obs.radius +1#2*np.sqrt(obs.radius**2 - np.abs(obs.y-y[i])**2)
      else:
          z[i] = obs.z - obs.radius - 1# 2*np.sqrt(obs.radius**2 - np.abs(obs.y-y[i])**2)

  y_spline = CubicSpline(t, y);
  z_spline = CubicSpline(t, z);


  return y_spline, z_spline
