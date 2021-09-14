#! /usr/bin/env python
import rospy
import PID
import sys
import numpy as np
from std_msgs import msg
from std_msgs.msg import Float32MultiArray, Float32
from geometry_msgs.msg import Twist
from tf.transformations import quaternion_from_euler

#Codigo que funciona con controladores PID tuneados manualmente
#En este momento no sirve nada no se queda estable
#funciona con la escena ros_test_manualVelLUA.ttt

target_pose = [0,0]

target_trust = 1
target_roll = 0
target_pitch = 0
target_yaw = 0

#PID Constants
P_trust = 5.3
I_trust = 0.2
D_trust = 0.22

P_roll = 0
I_roll = 0
D_roll = 0

P_pitch = 0
I_pitch = 0
D_pitch = 0

P_yaw = 0
I_yaw = 0
D_yaw = 0

#PID Objects
trust_pid = PID.PID(P_trust, I_trust, D_trust)
trust_pid.SetPoint = target_trust
trust_pid.setSampleTime(1)

roll_pid = PID.PID(P_roll, I_roll, D_roll)
roll_pid.SetPoint = target_roll
roll_pid.setSampleTime(1)

pitch_pid = PID.PID(P_pitch, I_pitch, D_pitch)
pitch_pid.SetPoint = target_pitch
pitch_pid.setSampleTime(1)

yaw_pid = PID.PID(P_yaw, I_yaw, D_yaw)
yaw_pid.SetPoint = target_yaw
yaw_pid.setSampleTime(1)

#PID Outputs
out_trust = 0
out_roll = 0
out_pitch = 0
out_yaw = 0

#Subscriber positions and orientations
pos_x, pos_y, pos_z, theta = 0,0,0,0
roll, pitch, yaw, trust = 0,0,0,0

#Motor Mixing Algoridim
mm1 = out_trust + out_roll + out_pitch + out_yaw
mm2 = out_trust - out_roll + out_pitch - out_yaw
mm3 = out_trust + out_roll - out_pitch - out_yaw
mm4 = out_trust - out_roll - out_pitch + out_yaw


def orientation_callback(msg): #Me regresa el angulo en el que se encuentra orientado en el marco robot
    global theta
    theta = msg.data
    


def position_callback(msg): #Me regresa la posicion en el marco inercial del robot
    global pos_x, pos_y, pos_z, roll, pitch, yaw, trust
    pos_z = msg.linear.z
    pos_x = msg.linear.x 
    pos_y = msg.linear.y

    roll = msg.angular.x
    pitch = msg.angular.y
    yaw = msg.angular.z
    

def simTime_callback(msg):
	global t, t_total, rho_total, rho
	t = msg.data



def readConfig (): #Configura los parametros del PID
    global target_trust, target_roll, target_pitch, target_yaw

    trust_pid.SetPoint = target_trust
    trust_pid.setKp (float(P_trust))
    trust_pid.setKi (float(I_trust))
    trust_pid.setKd (float(D_trust))

    roll_pid.SetPoint = target_roll
    roll_pid.setKp (float(P_roll))
    roll_pid.setKi (float(I_roll))
    roll_pid.setKd (float(D_roll))

    pitch_pid.SetPoint = target_pitch
    pitch_pid.setKp (float(P_pitch))
    pitch_pid.setKi (float(I_pitch))
    pitch_pid.setKd (float(D_pitch))

    yaw_pid.SetPoint = target_yaw
    yaw_pid.setKp (float(P_yaw))
    yaw_pid.setKi (float(I_yaw))
    yaw_pid.setKd (float(D_yaw))

#Funcion principal de movimiento
def main_write():
    global pos_x, pos_y,  pos_z, roll, pitch, yaw, trust, theta, out_trust, out_pitch, out_roll, out_yaw, mm1, mm2, mm3, mm4
    
    rospy.init_node('PID_Controller', anonymous=True) #Inicio nodo
    endPos = [-2,-2, -np.pi*3/4] #Posicion final por defecto

    if len(sys.argv) > 2: #Utilizando la posicion final entrada por parametro
        endPos[0] = float(sys.argv[1])
        endPos[1] = float(sys.argv[2])
        endPos[2] = float(sys.argv[3])

    pub = rospy.Publisher('drone_wheelsVel', Float32MultiArray, queue_size=10)
    rate = rospy.Rate(10) #10hz
    rospy.Subscriber("drone_position", Twist, position_callback)
    rospy.Subscriber("simulationTime", Float32, simTime_callback)
    rospy.Subscriber("drone_orientation", Float32, orientation_callback)

    propeller = Float32MultiArray()
    propeller.data = [0.0, 0.0, 0.0, 0.0]

    

    while not rospy.is_shutdown():

        readConfig()

        #Updating feedback loop
        trust_pid.update(pos_z)
        out_trust = trust_pid.output

        roll_pid.update(roll)
        out_roll = roll_pid.output

        pitch_pid.update(pitch)
        out_pitch = pitch_pid.output

        yaw_pid.update(yaw)
        out_yaw = yaw_pid.output

        #Motor Mixing Algoridim
        mm1 = out_trust + out_roll + out_pitch + out_yaw
        mm2 = out_trust - out_roll + out_pitch - out_yaw
        mm3 = out_trust + out_roll - out_pitch - out_yaw
        mm4 = out_trust - out_roll - out_pitch + out_yaw

        # mm1 = (5.335 + out_trust)*(1-out_roll+out_pitch+out_yaw)
        # mm2 = (5.335 + out_trust)*(1-out_roll-out_pitch-out_yaw)
        # mm3 = (5.335 + out_trust)*(1+out_roll-out_pitch+out_yaw)
        # mm4 = (5.335 + out_trust)*(1+out_roll+out_pitch-out_yaw)

        propeller.data = [mm1, mm2, mm3, mm4]
        pub.publish(propeller)
        #print(pos_z, pos_x, pos_y)
        print(round(pos_z,3),'Vel: ',round(mm1,3), round(mm2,3), round(mm3,3), round(mm4,3))
        #print(round(out_trust,3),round(out_roll,3),round(out_yaw,3),' = OUT_PID')
        #print(round(roll,3),round(pitch,3),round(yaw,3),'theta: ',round(theta,3))
        #sys.stdout.write("\033[K") # Clear to the end of line
        #sys.stdout.write("\033[F") # Cursor up one line

        rate.sleep()

if __name__ == '__main__':
	main_write()