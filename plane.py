import numpy as np

from pyquaternion import Quaternion

class Plane:

    # State Vectors
    attitude 	        = np.array([0,0,0])
    attitude_dot	    = np.array([0,0,0])

    position	        = np.array([0,0,0])
    position_dot        = np.array([0,0,0])

    rot_mat = np.array([[0,0,0],[0,0,0],[0,0,0]])

    # Plane rendering points
    base_points		= np.array([[0,2,-6,0,0,-6,-2,-4,0,0],[0,0,0,8,-8,0,0,0,4,-4],[0,0,0,0,0,2,0,0,0,0]])
    

    # Flight Metrics
    elevation   = 0
    airspeed    = 0
    
    # Plane Qualities
    mass        = 0
    inertia     = 10

    # Controls
    throttle    = 0
    r_flap      = 0
    l_flap      = 0
    elevator    = 0


    def __init__(self, mass):

        self.mass = mass

    def setX(self, attitude, attitude_dot, position, position_dot):

        self.attitude 		= np.array(attitude)
        self.attitude_dot	= np.array(attitude_dot)
        self.position		= np.array(position)
        self.position_dot	= np.array(position_dot)

        self.gen_rot()

    def getX(self):

        return self.attitude, self.attitude_dot, self.position, self.position_dot

    def getRot(self):

        return self.rot_mat

    def getPoints(self):

        #self.gen_rot()

        offset = np.transpose(np.tile(self.position, (len(self.base_points[0]), 1)))

        points = np.add(np.dot(self.rot_mat, self.base_points),offset)

        return points


    def physics_step(self, t_step=0.1):

        # Position step
        F = self.getForces()

        dx = self.position_dot * t_step

        dv = (F/self.mass)*t_step

        self.position = self.position + dx

        self.position_dot = self.position_dot + dv

        self.elevation = self.position[2]

        # Rotation step
        T = self.getTorques()

        dt = np.cross(self.attitude_dot,self.rot_mat) * t_step

        dw = (T/self.inertia)*t_step

        self.rot_mat = self.rot_mat + dt

        self.attitude_dot = self.attitude_dot + dw

        print(self.attitude)

        if (self.collision()):

            self.position[2] = 0
            self.position_dot = np.array([0,0,0])
            self.attitude_dot = np.array([0,0,0])


    def getForces(self):

        F = np.array([0,0,0])

        F = F + self.gravity_Force()

        F = F + self.thrust_Force()

        return F


    def getTorques(self):

        T = np.array([0,0,0])

        return T


    def gravity_Force(self):

        if (self.elevation == 0):

            return np.array([0,0,0])

        else:

            return np.array([0,0,-9.8*self.mass])
		
    def thrust_Force(self):

        thrust = np.array([10*self.throttle,0,0])

        return np.transpose(np.dot(self.rot_mat, np.transpose(thrust)))
        

    def collision(self):

        if (self.position[2] <= 0):

            return True

        return False

    def set_controls(self, throttle, r_flap, l_flap, elevator):

        self.throttle = throttle
        self.elevator = elevator
        self.r_flap = r_flap
        self.l_flap = l_flap

    def gen_rot(self):

        qroll 	= Quaternion(axis=[1,0,0],angle=self.attitude[0])
        qpitch	= Quaternion(axis=[0,1,0],angle=self.attitude[1])
        qyaw	= Quaternion(axis=[0,0,1],angle=self.attitude[2])

        qtotal = qroll * qpitch * qyaw

        self.rot_mat = qtotal.rotation_matrix


