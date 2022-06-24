from vpython import *
from numpy import *
#Calculating forward kinematics for Articular Robot.
print("Enter your theta degree for motor 1: ")
theta_motor1 = int(input())#degree of motor 1 rotation
print("Enter your theta degree for motor 2: ")
theta_motor2 = int(input())#degree of motor 2 rotation
print("Enter your theta degree for motor 3: ")
theta_motor3 = int(input())#degree of motor 3 rotation
#Creating cylinders for showing motors.
cyl1 = cylinder( pos=vector(0,0,0),axis=vector(0,1,0),color=color.red,radius=0.12,length=0.3)
cyl2 = cylinder( pos=vector(-0.15,1,0),axis=vector(1,0,0),color=color.red,radius=0.12,length=0.3)
cyl3 = cylinder( pos=vector(-0.15,1,2),axis=vector(1,0,0),color=color.red,radius=0.12,length=0.3)
#Curves show links between joints.
c1 = curve(vector(0,0,0), vector(0,1,0))
c2 = curve(vector(-0.15,1,0), vector(-0.15,1,1))
c3 = curve(vector(0.15,1,0), vector(0.15,1,1))
c4 = curve(vector(+0.15,1,1), vector(-0.15,1,1))
c5 = curve(vector(0,1,1), vector(0,1,2))
c6 = curve(vector(0.15,1,2), vector(0.15,1,3))
c7 = curve(vector(-0.15,1,2), vector(-0.15,1,3))
c8 = curve(vector(+0.15,1,3), vector(-0.15,1,3))
c9 = curve(vector(0,1,3), vector(0,1,4))
#Creating variables for rotation.

# Link lengths
a1 = 1  # length of link a1 by calculating vector length
a2 = 1  # length of link a2 by calculating vector length
a3 = 1 # length of link a3 by calculating vector length
a4 = 1  # length of link a4 by calculating vector length
a5 = 1 # length of link a5 by calculating vector length
a6 = 1  # length of link a6 by calculating vector length
#Creating radians variables for calculating sin and cos values.
thetaRa1 = (theta_motor1/180)*pi  # radians value of degree for motor1
thetaRa2 = (theta_motor2/180)*pi  # radians value of degree for motor2
thetaRa3 = (theta_motor3/180)*pi  # radians value of degree for motor3
# Creating matrices 
PT = [[thetaRa1, 0, a2, a1],
      [thetaRa2, 0, a4, a3],
      [thetaRa3, 0, a6, a5]]
# homogeneous rotation forward kinematics matrices and multiplication of matrices H01*H12*H23=H03
i = 0
H0_1 = [[cos(PT[i][0]), -sin(PT[i][0])*cos(PT[i][1]), sin(PT[i][0])*sin(PT[i][1]), PT[i][2]*cos(PT[i][0])],
        [sin(PT[i][0]), cos(PT[i][0])*cos(PT[i][1]), -cos(PT[i][0])*sin(PT[i][1]), PT[i][2]*sin(PT[i][0])],
        [0, sin(PT[i][1]), cos(PT[i][1]), PT[i][3]],
        [0, 0, 0, 1]]
i = 1
H1_2 = [[cos(PT[i][0]), -sin(PT[i][0])*cos(PT[i][1]), sin(PT[i][0])*sin(PT[i][1]), PT[i][2]*cos(PT[i][0])],
        [sin(PT[i][0]), cos(PT[i][0])*cos(PT[i][1]), -cos(PT[i][0])*sin(PT[i][1]), PT[i][2]*sin(PT[i][0])],
        [0, sin(PT[i][1]), cos(PT[i][1]), PT[i][3]],
        [0, 0, 0, 1]]
i = 2
H2_3 = [[cos(PT[i][0]), -sin(PT[i][0])*cos(PT[i][1]), sin(PT[i][0])*sin(PT[i][1]), PT[i][2]*cos(PT[i][0])],
        [sin(PT[i][0]), cos(PT[i][0])*cos(PT[i][1]), -cos(PT[i][0])*sin(PT[i][1]), PT[i][2]*sin(PT[i][0])],
        [0, sin(PT[i][1]), cos(PT[i][1]), PT[i][3]],
        [0, 0, 0, 1]]
H0_2 = dot(H0_1,H1_2)
H0_3 = dot(H0_2,H2_3)
print("H0_1 =")
print(matrix(H0_1))
print("H1_2 =")
print(matrix(H1_2))
print("H2_3 =")
print(matrix(H2_3))
print("Forward Kinematic :dot multiplication of H0_1 * H1_2 * H2_3 = H0_3 = ")
print(matrix(H0_3))
