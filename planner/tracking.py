from scipy import integrate
import numpy as np  
from matplotlib import pyplot as plt
from flat_traject import *


class Kinematics:
    def __init__(self, K, desired_traj):
        self.k = K
        self.desired_traj = desired_traj
        self.X = []

    def car_ctrl(self, x,t):
        # Desired ouput
        yd = np.dot(self.desired_traj.poly3_coeff(),self.desired_traj.poly3(t))
        dyd = np.dot(self.desired_traj.poly3_coeff(),self.desired_traj.dpoly3(t))
        d2yd = np.dot(self.desired_traj.poly3_coeff(),self.desired_traj.d2poly3(t))
        
        # Current outpt 
        y = np.array([x[0],x[1]]).reshape(2,1)
        dy = x[3]*np.array([np.cos(x[2]),np.sin(x[2])]).reshape(2,1)
        
        
        v = d2yd - self.k[0]*(y - yd) - self.k[1]*(dy - dyd) 
        #print(v)
        G11 = -x[3]**2*np.sin(x[2])
        G12 = np.cos(x[2])
        G21 = x[3]**2*np.cos(x[2])
        G22 = np.sin(x[2])
        G = np.array([[G11,G12],[G21,G22]])
        #print(G)
        u = np.matmul(np.linalg.inv(G),v)
                
        return u.reshape(-1,)

    def noise(self,x,t):
        N = np.random.normal(0.,0.1,1)
        return N
        
    def derivative(self, x, t):
       
        u = self.car_ctrl(x,t)
        dx1 = x[3]*np.cos(x[2])
        dx2 = x[3]*np.sin(x[2])
        dx3 = u[0]*x[3] # Let l = 1
        dx4 = u[1]
        #print(t)
        return [dx1,dx2,dx3,dx4]

    def trajectory(self,x0, tout):
        # (1) Use Scipy ODE 
        #z_num = integrate.odeint(self.derivative,x0,tout)
        
        # (2) Integrate by ourselves --- with noise injected
        self.X = x0
        Xs = np.array(self.X)
        
        num = tout.shape[0]
        dt = tout[1]-tout[0]
        cntr = 0
        
        for t in tout:
            # Inject noise at 100Hz
            cntr += 1
            if cntr % 100 == 0:
                X = [self.X[0]+np.random.rand(),
                self.X[1]+np.random.rand(),
                self.X[2]+np.random.rand(),
                self.X[3]]
                #print("noisy")
            else:
                X = self.X
                
            dX = self.derivative(X,t)
            self.X[0] += dX[0]*dt
            self.X[1] += dX[1]*dt
            self.X[2] += dX[2]*dt
            self.X[3] += dX[3]*dt
            Xs = np.concatenate((Xs,self.X),axis=0)
            #print(self.X[2])
        Xs = Xs.reshape(num+1,4)
        #print(Xs)
        #print(z_num[:,2])
        return Xs

def main():
    start = [-5,-3,0.1]
    goal = [0,0,1.]
    T = 5.
    traj = Trajectory(start, goal, .1, T)
    traj.plot_desired_traj()

    tout = np.linspace(0,T,10000)
    x0 = [-5.2,-3,0.1,0.1]
    k = [1,1]
    model = Kinematics(k,traj)


    # # z_num = integrate.odeint(derivative,x0,tout)
    # # x_num = z_num[:,0]
    # # y_num = z_num[:,1]
    z = model.trajectory(x0, tout)
    x_num = z[:,0]
    y_num = z[:,1]
    #plt.figure()

    plt.plot(x_num,y_num,color = 'red')
    plt.show()

if __name__ == '__main__':
    main()

