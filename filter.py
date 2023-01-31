# ---------------------------------------------------------------------
# Project "Track 3D-Objects Over Time"
# Copyright (C) 2020, Dr. Antje Muntzinger / Dr. Andreas Haja.
#
# Purpose of this file : Kalman filter class
#
# You should have received a copy of the Udacity license together with this program.
#
# https://www.udacity.com/course/self-driving-car-engineer-nanodegree--nd013
# ----------------------------------------------------------------------
#

# imports
import numpy as np

# add project directory to python path to enable relative imports
import os
import sys
PACKAGE_PARENT = '..'
SCRIPT_DIR = os.path.dirname(os.path.realpath(os.path.join(os.getcwd(), os.path.expanduser(__file__))))
sys.path.append(os.path.normpath(os.path.join(SCRIPT_DIR, PACKAGE_PARENT)))
import misc.params as params 
dt=params.dt
q=params.q

dim_state=params.dim_state
from student.trackmanagement import Track
from student.measurements import Sensor
class Filter:
    '''Kalman filter class'''
    def __init__(self):
        pass
    def F(self):
        ############
        # TODO Step 1: implement and return system matrix F
        ############
        
        return np.matrix([[1,0,dt,0,0,0],[0,1,0,0,0,0],[0,0,1,dt,0,0],[0,0,0,1,0,0],[0,0,0,0,1,dt],[0,0,0,0,0,1]])
        
        ############
        # END student code
        ############ 

    def Q(self):
        ############
        # TODO Step 1: implement and return process noise covariance Q
        ############
        q1=((dt**3)/3)*q
        q2=((dt**2)/2)*q
        q3=dt*q
        return np.matrix([[q1,0,q2,0],[0,q1,0,q2],[q2,0,q3,0],[0,q2,0,q3]])
        
        ############
        # END student code
        ############ 

    def predict(self, track):
        ############
        # TODO Step 1: predict state x and estimation error covariance P to next timestep, save x and P in track
        ############
        F=self.F()
        print(track,"$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$")
        # predict_x=F*track.x
        # P=F*track.P*np.transpose(F)+self.Q()
        # Track.set_x(predict_x)
        # Track.set_P(P)
        # track.append(predict_x,P)
        # pass    
        
        ############
        # END student code
        ############ 

    def update(self, track, meas):
        ############
        # TODO Step 1: update state x and covariance P with associated measurement, save x and P in track
        ############
        # H=Sensor.get_H(self,track.x)
        # gamma=gamma(self,track,meas)
        # S=H*track.P*np.transpose(H)+meas.R
        # K=track.P*np.transpose(H)*np.linalg.inv(S)
        # x=x+K*gamma
        # I=np.identity(dim_state)
        # P=(I-K*H)*track.P
        # track.append(x,P)
        ############
        # END student code
        ############ 
        track.update_attributes(meas)
    
    def gamma(self, track, meas):
        ############
        # TODO Step 1: calculate and return residual gamma
        ############
        # gamma=meas.z-Sensor.get_hx(self,track.x)
        return 
        
        ############
        # END student code
        ############ 

    def S(self, track, meas, H):
        ############
        # TODO Step 1: calculate and return covariance of residual S
        ############

        return 0
        
        ############
        # END student code
        ############ 