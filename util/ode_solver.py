from scipy import integrate
import numpy as np  
from matplotlib import pyplot as plt
class Kinematics:
	def __init__(self, K):
		self.k = K

	def uni_ctrl(self, t, x):
		dy = -x[1]-1.5*x[0]**2-0.5*x[0]**3;
		v = - self.k[0]*x[0] -self.k[1]*dy
		K = (-3*x[0] - 1.5*x[0]**2)*(-x[1]-1.5*x[0]**2-0.5*x[0]**3);
		return K-v


	def derivative(self, z, t):
		x = z
		ua = self.uni_ctrl(t, x)
		return [ -x[1]-1.5*x[0]**2-0.5*x[0]**3,ua]

	def trajectory(self,x, tout):
		z_num = integrate.odeint(self.derivative,x,tout)
		return z_num

# tout = np.linspace(0,10,100)

# x0 = [-4,1]
# k = [1,1]
# model = Kinematics(k)


# # z_num = integrate.odeint(derivative,x0,tout)
# # x_num = z_num[:,0]
# # y_num = z_num[:,1]
# z = model.trajectory(x0, tout)
# x_num = z[:,0]
# y_num = z[:,1]
# plt.figure()

# plt.plot(x_num,y_num,color = 'red')
# plt.show()


