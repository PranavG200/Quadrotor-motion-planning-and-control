import numpy as np

def Ab_i1(i, n, d, dt_i, w_i, w_ip1):
  '''
  Ab_i1(i, n, d, dt_i, w_i, w_ip1) computes the linear equality constraint
  constants that require the ith polynomial to meet waypoints w_i and w_{i+1}
  at it's endpoints.
  Parameters:
     i - index of the polynomial.
     n - total number of polynomials.
     d - the number of terms in each polynomial.
     dt_i - Delta t_i, duration of the ith polynomial.
     w_i - waypoint at the start of the ith polynomial.
     w_ip1 - w_{i+1}, waypoint at the end of the ith polynomial.
  Outputs
     A_i1 - A matrix from linear equality constraint A_i1 v = b_i1
     b_i1 - b vector from linear equality constraint A_i1 v = b_i1
  '''

  A_i1 = np.zeros((4, 2*d*n))
  b_i1 = np.zeros((4, 1))
  b_i1[0] = w_i[0]
  b_i1[1] = w_i[1]
  b_i1[2] = w_ip1[0]
  b_i1[3] = w_ip1[1]
  A_i1[0,2*d*i] = 1
  A_i1[1,2*d*i+1] = 1

  # TODO: fill in values for A_i1 and b_i1
  for c in range(d):
      A_i1[2,2*d*i +2*c] = dt_i**c
      A_i1[3,2*d*i +2*c+1] = dt_i**c

  return A_i1, b_i1

def Ab_i2(i, n, d, dt_i):

  A_i2 = np.zeros((2, 2*d*n))
  b_i2 = np.zeros((2, 1))

  # TODO: fill in values for A_i1 and b_i1
  A_i2[0,2*d*(i+1)+2] = -1
  A_i2[1,2*d*(i+1)+3] = -1

  for c in range(1,d):
      A_i2[0,2*d*i +2*c] = c*(dt_i**(c-1))
      A_i2[1,2*d*i +2*c+1] = c*(dt_i**(c-1))

  return A_i2, b_i2

def Ab_i3(i, n, d, dt_i):

  A_i3 = np.zeros((2, 2*d*n))
  b_i3 = np.zeros((2, 1))

  # TODO: fill in values for A_i1 and b_i1
  A_i3[0,2*d*(i+1)+4] = -2
  A_i3[1,2*d*(i+1)+5] = -2

  for c in range(2,d):
      A_i3[0,2*d*i +2*c] = c*(c-1)*(dt_i**(c-2))
      A_i3[1,2*d*i +2*c+1] = c*(c-1)*(dt_i**(c-2))

  return A_i3, b_i3

def Ab_i4(i, n, d, dt_i):

  A_i4 = np.zeros((2, 2*d*n))
  b_i4 = np.zeros((2, 1))

  # TODO: fill in values for A_i1 and b_i1
  A_i4[0,2*d*(i+1)+6] = -6
  A_i4[1,2*d*(i+1)+7] = -6

  for c in range(3,d):
      A_i4[0,2*d*i +2*c] = c*(c-1)*(c-2)*(dt_i**(c-3))
      A_i4[1,2*d*i +2*c+1] = c*(c-1)*(c-2)*(dt_i**(c-3))

  return A_i4, b_i4

def Ab_i5(i, n, d, dt_i):

  A_i5 = np.zeros((2, 2*d*n))
  b_i5 = np.zeros((2, 1))

  # TODO: fill in values for A_i1 and b_i1
  A_i5[0,2*d*(i+1)+8] = -24
  A_i5[1,2*d*(i+1)+9] = -24

  for c in range(4,d):
      A_i5[0,2*d*i +2*c] = c*(c-1)*(c-2)*(c-3)*(dt_i**(c-4))
      A_i5[1,2*d*i +2*c+1] = c*(c-1)*(c-2)*(c-3)*(dt_i**(c-4))

  return A_i5, b_i5