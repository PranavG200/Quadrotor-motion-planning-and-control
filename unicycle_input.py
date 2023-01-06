import numpy as np
from scipy.interpolate import CubicSpline
import matplotlib.pyplot as plt

def unicycle_input(t : float, y_spline : CubicSpline, z_spline: CubicSpline) -> np.ndarray:
  #UNICYCLE_INPUT returns input to the unicycle
  #   @param t - current time
  #   @param y_spline - spline object for desired y trajectory
  #   @param z_spline - spline object for desired z trajectory
  #   
  #   @return u - input u(t) to the unicycle system

  # TODO: modify u to return the correct input for time t.
  u = np.zeros(2);
  y1=y_spline.derivative(1)
  z1=z_spline.derivative(1)

  y2=y_spline.derivative(2)
  z2=z_spline.derivative(2)

  u[1] = np.sqrt(y1(t)**2 + z1(t)**2)
  u[0] = (z2(t)*y1(t) - y2(t)*z1(t))/(u[1]**2)

  return u
