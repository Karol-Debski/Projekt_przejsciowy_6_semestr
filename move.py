from ctypes import sizeof
import rospy
import numpy as np
from std_msgs.msg import Float64
from numpy import arctan2

def inverse_kinematic(z, y):
    hip_length = 165
    knee_length = 240
    leg_length = hip_length + knee_length
    pk_z = z
    pk_y = y
    #First set
    
    q1=np.pi/2-arctan2(leg_length-pk_z,pk_y)+np.arccos((pow(knee_length,2)-pow(hip_length,2)-(pow(leg_length-pk_z,2)+
    pow(pk_y,2)))/(-2*hip_length*np.sqrt(pow(leg_length-pk_z,2)+pow(pk_y,2))))
    q2=-(np.pi-np.arccos(((pow(leg_length-pk_z,2)+pow(pk_y,2))-pow(hip_length,2)-pow(knee_length,2))/(-2*knee_length*hip_length)))
    
    #Second set
    #q1=np.pi/2-arctan2(leg_length-pk_z,pk_y)-np.arccos((pow(knee_length,2)-pow(hip_length,2)-(pow(leg_length-pk_z,2)+
    #pow(pk_y,2)))/(-2*hip_length*np.sqrt(pow(leg_length-pk_z,2)+pow(pk_y,2))))
    #q2=np.pi-np.arccos((((pow(leg_length-pk_z,2)+pow(pk_y,2))-pow(hip_length,2)-pow(knee_length,2))/(-2*knee_length*hip_length)))
    return q1,q2

def sine_trajectory():
    phase = np.linspace(0.0, 2.0*np.pi, 200)
    y = np.concatenate((np.linspace(-100.0, 100, 200),np.linspace(100.0, -100, 200)),axis=None)
    z = np.concatenate((75.0+50.0*np.sin(phase),75.0+50.0*np.sin(phase)),axis=None)
    return z, y

def move():
    pub1 = rospy.Publisher(
        '/biped/joint1_position_controller/command', Float64, queue_size=10)
    pub2 = rospy.Publisher(
        '/biped/joint2_position_controller/command', Float64, queue_size=10)
    rospy.init_node('move', anonymous=True)
    rate = rospy.Rate(10)  # 10hz
    while not rospy.is_shutdown():
        z, y = sine_trajectory()
        for i in range(1,400):
            q1,q2=inverse_kinematic(z[i],y[i])
            pub1.publish(q1)
            pub2.publish(q2)
            rospy.sleep(0.01)
        rate.sleep()

if __name__ == '__main__':
    try:
        move()
    except rospy.ROSInterruptException:
        pass
