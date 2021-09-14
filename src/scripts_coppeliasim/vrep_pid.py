#! /usr/bin/env python
import rospy
import sys
import numpy as np
from std_msgs import msg
from std_msgs.msg import Float32MultiArray, Float32
from geometry_msgs.msg import Twist
from tf.transformations import quaternion_from_euler

#intentt de copiar el control que ya tenia el dron en vrep para adaptarlo a necesidades
#En este momento ni se puede correr
#funciona con la escena ros_test_manualVelLUA.ttt

target_pose = [0,0]
#ruta = np.array([[-4,-4,1], [-4, 4,1], [-2, 4,1], [-2, -4,1], [0,-4,1], [0,4,1], [2,4,1], [2, 0,1], [3, 0,1], [3, 4,1], [4,4,1], [-4,-4,1]])
ruta = [[-4,-4,1], [-4, 4,1], [-2, 4,1], [-2, -4,1], [0,-4,1], [0,4,1], [2,4,1], [2, 0,1], [3, 0,1], [3, 4,1], [4,4,1], [-4,-4,1]]


pParam=2
iParam=0
dParam=0
vParam=-2

cumul=0
lastE=0
pAlphaE=0
pBetaE=0
psp2=0
psp1=0
m =0
prevEuler=0

#Subscriber positions and orientations
pos_x, pos_y, pos_z, theta = 0,0,0,0
roll, pitch, yaw, thrust = 0,0,0,0

#PID Outputs
out_trust = 0
out_roll = 0
out_pitch = 0
out_yaw = 0

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
def matrix_callback(msg):
    global m
    m = msg.data

# ----------------------------PID CONTROLLER
def PID(endPos):
    global mm1, mm2, mm3, mm4, thrust, pos_x, pos_y, pos_z, cumul, prevEuler, pAlphaE, pBetaE, psp1, psp2,lastE,m
#-- Vertical control:
    targetPos=endPos
    pos=[pos_x, pos_y, pos_z]
    #l=sim.getVelocity(heli)
    e=(targetPos[2]-pos[2])
    cumul=cumul+e
    pv=pParam*e
    thrust=5.335+pv+iParam*cumul+dParam*(e-lastE)#+l[3]*vParam
    lastE=e
    
    #-- Horizontal control: 
    sp = np.array(endPos) - np.array(pos)
    #m=sim.getObjectMatrix(d,-1)
    vx={1,0,0}
    print(m)
    vx=np.dot(m,vx)
    vy={0,1,0}
    vy=np.dot(m,vy)
    alphaE=(vy[3]-m[12])
    out_roll=0.25*alphaE+2.1*(alphaE-pAlphaE)
    betaE=(vx[3]-m[12])
    out_pitch=-0.25*betaE-2.1*(betaE-pBetaE)
    pAlphaE=alphaE
    pBetaE=betaE
    out_roll=out_roll+sp[2]*0.005+1*(sp[2]-psp2)
    out_pitch=out_pitch-sp[1]*0.005-1*(sp[1]-psp1)
    psp2=sp[2]
    psp1=sp[1]
    
    #-- Rotational control:
    euler=theta
    out_yaw=euler[3]*0.1+2*(euler[3]-prevEuler)
    prevEuler=euler[3]

    #-- Decide of the motor velocities:
    mm1 = thrust*(1-out_roll+out_pitch+out_yaw)
    mm2 = thrust*(1-out_roll-out_pitch-out_yaw)
    mm3 = thrust*(1+out_roll-out_pitch+out_yaw)
    mm4 = thrust*(1+out_roll+out_pitch-out_yaw)

    return [mm1, mm2, mm3, mm4]


#Funcion principal de movimiento
def main_write():
    global pos_x, pos_y,  pos_z, roll, pitch, yaw, thrust, theta, out_trust, out_pitch, out_roll, out_yaw, mm1, mm2, mm3, mm4, ruta
    
    rospy.init_node('PID_Controller', anonymous=True) #Inicio nodo
    endPos = [-2,-2, -np.pi*3/4] #Posicion final por defecto

    # if len(sys.argv) > 2: #Utilizando la posicion final entrada por parametro
    #     endPos[0] = float(sys.argv[1])
    #     endPos[1] = float(sys.argv[2])
    #     endPos[2] = float(sys.argv[3])

    pub = rospy.Publisher('drone_wheelsVel', Float32MultiArray, queue_size=10)
    rate = rospy.Rate(10) #10hz
    rospy.Subscriber("drone_position", Twist, position_callback)
    rospy.Subscriber("simulationTime", Float32, simTime_callback)
    rospy.Subscriber("drone_orientation", Float32, orientation_callback)
    rospy.Subscriber("objectMatrix", Float32MultiArray, matrix_callback)

    propeller = Float32MultiArray()
    propeller.data = [0.0, 0.0, 0.0, 0.0]

    while not rospy.is_shutdown():
        for point in ruta:
            mm1, mm2, mm3, mm4 = PID(point)
        
            propeller.data = [mm1, mm2, mm3, mm4]
            pub.publish(propeller)

            print(round(pos_z,3),'Vel: ',round(mm1,3), round(mm2,3), round(mm3,3), round(mm4,3))
        #print(round(out_trust,3),round(out_roll,3),round(out_yaw,3),' = OUT_PID')
        #sys.stdout.write("\033[K") # Clear to the end of line
        #sys.stdout.write("\033[F") # Cursor up one line

        rate.sleep()

if __name__ == '__main__':
	main_write()