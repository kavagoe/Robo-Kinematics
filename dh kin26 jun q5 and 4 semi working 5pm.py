import numpy as np
from numpy import array
from sympy import symbols, cos, sin, pi, simplify, sqrt, atan2, acos, atan
from sympy.matrices import Matrix, Transpose
import math
import tf

# convert from quadr to Euler
def Q_To_E(x, y, z, w):
    ysqr = y*y
    
    t0 = +2.0 * (w * x + y*z)
    t1 = +1.0 - 2.0 * (x*x + ysqr)
    X = math.degrees(math.atan2(t0, t1))
    
    t2 = +2.0 * (w*y - z*x)
    t2 =  1 if t2 > 1 else t2
    t2 = -1 if t2 < -1 else t2
    Y = math.degrees(math.asin(t2))
    
    t3 = +2.0 * (w * z + x*y)
    t4 = +1.0 - 2.0 * (ysqr + z*z)
    Z = math.degrees(math.atan2(t3, t4))
    return X, Y, Z

#Symbols
q1, q2, q3, q4, q5, q6, q7 = symbols('q1:8') # this is theta
d1,d2,d3,d4,d5,d6,d7 = symbols('d1:8')
a0,a1,a2,a3,a4,a5,a6 = symbols('a0:7')
al0, al1, al2, al3, al4, al5, al6 = symbols('al0:7')
wx, wy, wz = symbols('wx, wy, wz')
px, py, pz = symbols('px, py, pz')
pxf, pyf, pzf = symbols('pxf, pyf, pzf')
Yaw, Pitch, Roll = symbols('Yaw, Pitch, Roll')
pxtheta, pytheta, theta1, thata2 = symbols('pxtheta, pytheta, theta1, thata2')
leff = 0.11
# DH Param
s = {al0: 0,        a0: 0,          d1: 0.75,     
     al1: -pi/2,    a1: 0.35,       d2: 0,      q2: q2-pi/2, 
     al2: 0,        a2: 1.25,       d3: 0,           
     al3: -pi/2,    a3: -0.054,     d4: 1.50,        
     al4: pi/2,     a4: 0,          d5: 0,          
     al5: -pi/2,    a5: 0,          d6: 0,          
     al6: 0,        a6: 0,          d7: 0.303,   q7: 0}

#Transforms
T0_1 = Matrix([[             cos(q1),            -sin(q1),            0,              a0],
               [ sin(q1)*cos(al0), cos(q1)*cos(al0), -sin(al0), -sin(al0)*d1],
               [ sin(q1)*sin(al0), cos(q1)*sin(al0),  cos(al0),  cos(al0)*d1],
               [                   0,                   0,            0,               1]])
T0_1 = T0_1.subs(s)

T1_2 = Matrix([[             cos(q2),            -sin(q2),            0,              a1],
               [ sin(q2)*cos(al1), cos(q2)*cos(al1), -sin(al1), -sin(al1)*d2],
               [ sin(q2)*sin(al1), cos(q2)*sin(al1),  cos(al1),  cos(al1)*d2],
               [                   0,                   0,            0,               1]])
T1_2 = T1_2.subs(s)

T2_3 = Matrix([[             cos(q3),            -sin(q3),            0,              a2],
               [ sin(q3)*cos(al2), cos(q3)*cos(al2), -sin(al2), -sin(al2)*d3],
               [ sin(q3)*sin(al2), cos(q3)*sin(al2),  cos(al2),  cos(al2)*d3],
               [                   0,                   0,            0,               1]])
T2_3 = T2_3.subs(s)

T3_4 = Matrix([[             cos(q4),            -sin(q4),            0,              a3],
               [ sin(q4)*cos(al3), cos(q4)*cos(al3), -sin(al3), -sin(al3)*d4],
               [ sin(q4)*sin(al3), cos(q4)*sin(al3),  cos(al3),  cos(al3)*d4],
               [                   0,                   0,            0,               1]])
T3_4 = T3_4.subs(s)

T4_5 = Matrix([[             cos(q5),            -sin(q5),            0,              a4],
               [ sin(q5)*cos(al4), cos(q5)*cos(al4), -sin(al4), -sin(al4)*d5],
               [ sin(q5)*sin(al4), cos(q5)*sin(al4),  cos(al4),  cos(al4)*d5],
               [                   0,                   0,            0,               1]])
T4_5 = T4_5.subs(s)

T5_6 = Matrix([[             cos(q6),            -sin(q6),            0,              a5],
               [ sin(q6)*cos(al5), cos(q6)*cos(al5), -sin(al5), -sin(al5)*d6],
               [ sin(q6)*sin(al5), cos(q6)*sin(al5),  cos(al5),  cos(al5)*d6],
               [                   0,                   0,            0,               1]])
T5_6 = T5_6.subs(s)

T6_G = Matrix([[             cos(q7),            -sin(q7),            0,              a6],
               [ sin(q7)*cos(al6), cos(q7)*cos(al6), -sin(al6), -sin(al6)*d7],
               [ sin(q7)*sin(al6), cos(q7)*sin(al6),  cos(al6),  cos(al6)*d7],
               [                   0,                   0,            0,               1]])
T6_G = T6_G.subs(s)

T0_2 = T0_1 * T1_2 # base to 2
T0_3 = T0_2 * T2_3 # base to 3
T0_4 = T0_3 * T3_4 # base to 4
T0_5 = T0_4 * T4_5 # base to 5
T0_6 = T0_5 * T5_6 # base to 6
T0_G = T0_6 * T6_G # base to gripper

R_z = Matrix([[     cos(np.pi),            -sin(np.pi),             0,              0],
               [    sin(np.pi),             cos(np.pi),             0,              0],
               [            0,                      0,              1,              0],
               [            0,                      0,              0,              1]])


R_y = Matrix([[     cos(-np.pi/2),                  0,  sin(-np.pi/2),              0],
               [            0,                      1,              0,              0],
               [    -sin(-np.pi/2),                 0,  cos(-np.pi/2),              0],
               [            0,                      0,              0,              1]])
R_corr = simplify(R_z * R_y)
T_total = T0_G * R_corr


# Rotation Matrix for wrist centre solution
d7 = 0.303
dtr = pi/180
rtd = 180/pi

##pxf = 0.13
##pyf = 2.109
##pzf = 1.797
##ox =-0.17
##oy =0.18
##oz =0.66
##ow =0.7044
############################
#input for the X Y Z position and xyzw orientations of end effector
# below is Zero configuration
pxf = 2.15286
pyf = 0
pzf = 1.94653
ox =0
oy =0
oz =0
ow =1
# below is all zero except for joint 5 & 4 @ -0.35
##pxf = 2.13378
##pyf = 0.03607
##pzf = 2.0459
##ox =-0.1705
##oy =-0.174
##oz =0.0306
##ow =0.969
# below is j2 @ .28 except for joint 5 & 4 @ -0.35


# below is all zero except for joint 4 @ -0.35 is gamma
##pxf = 2.153
##pyf = 0
##pzf = 1.9465
##ox =-0.17
##oy =-0.0001
##oz =0
##ow =0.99
# below is all zero except for joint 4 @ -4.58 is gamma j1:45, j2:20
##pxf = 2.13958
##pyf = 0.9798
##pzf = 1.5679
##ox =0.7191
##oy =0.2233
##oz =0.066512
##ow =0.65469
# below is all zero except for joint 4 @ -4.58 is gamma j1:45, j2:20, j5:-0.61
##pxf = 2.1584
##pyf = 0.80
##pzf = 1.555
##ox =0.706
##oy =0.0179
##oz =-0.151
##ow =0.6914
# below is all zero except for joint 5 @ -0.75 is alpha
##pxf = 2.077
##pyf = 0
##pzf = 2.1466
##ox =0
##oy =-0.3532
##oz =0
##ow =0.95355

# below is joint 2 @ 0.41, joint 5@ -0.75 
##pxf = 2.48799
##pyf = 0
##pzf = 1.3526
##ox =0
##oy =-0.157719
##oz =0
##ow =0.98748

# below is joint 3 @ 0.24, joint 5@ 0.72 
##pxf = 1.96727
##pyf = 0
##pzf = 1.34
##ox =0
##oy =0.4626
##oz =0
##ow =0.886
# below is joint 2&3 @ 0.24, joint 5@ 0.72 
##pxf = 2.062
##pyf = 0
##pzf = 0.9384
##ox =0
##oy =0.56599
##oz =0
##ow =0.82441

#############################

#xyzw to Euler
euler = Q_To_E(ox,oy,oz,ow)

# Rotation matrix
R_x = Matrix([[ 1,              0,        0],
              [ 0,        cos(math.radians(euler[0])), -sin(math.radians(euler[0]))],
              [ 0,        sin(math.radians(euler[0])),  cos(math.radians(euler[0]))]])

R_y = Matrix([[ cos(math.radians(euler[1])),        0,  sin(math.radians(euler[1]))],
              [       0,        1,        0],
              [-sin(math.radians(euler[1])),        0,  cos(math.radians(euler[1]))]])

R_z = Matrix([[ cos(math.radians(euler[2])), -sin(math.radians(euler[2])),        0],
              [ sin(math.radians(euler[2])),  cos(math.radians(euler[2])),        0],
              [ 0,              0,        1]])
# Rotation Matrix multiplication
R_Roll = R_z*R_y*R_x

# Rotation Matrix for q4-5-6 since above did not work
##RotM = Matrix([[            cos(math.radians(euler[0]))*cos(math.radians(euler[1])), cos(math.radians(euler[0]))*sin(math.radians(euler[1]))*sin(math.radians(euler[2])) - sin(math.radians(euler[0]))*cos(math.radians(euler[2])),       cos(math.radians(euler[0]))*sin(math.radians(euler[1]))*cos(math.radians(euler[2])) + sin(math.radians(euler[0]))*sin(math.radians(euler[2]))],
##               [            sin(math.radians(euler[0]))*cos(math.radians(euler[1])), sin(math.radians(euler[0]))*sin(math.radians(euler[1]))*sin(math.radians(euler[2])) + cos(math.radians(euler[0]))*cos(math.radians(euler[2])),       sin(math.radians(euler[0]))*sin(math.radians(euler[1]))*cos(math.radians(euler[2])) - cos(math.radians(euler[0]))*sin(math.radians(euler[2]))],
##                [                   -sin(math.radians(euler[1])),                       cos(math.radians(euler[1]))*sin(math.radians(euler[2])),                              cos(math.radians(euler[1]))*cos(math.radians(euler[2]))]])
RotM = R_z*R_y*R_x

# px, py, pz location of end effector
pmatrix = Matrix([[pxf],[pyf],[pzf]])

# find the euler angles for nx, ny, nz for wrist centre calculation
wristn = Matrix([[cos(euler[2]*dtr)*cos(euler[1]*dtr)],[sin(euler[2]*dtr)*cos(euler[1]*dtr)],[-sin(euler[2]*dtr)]])



# Calculations for wrist centre
commong = d7*R_Roll*Matrix([[1],[0],[0]])
wristxyz = pmatrix-commong

print("wristcentre: ", wristxyz)


px = wristxyz[0]
py = wristxyz[1]
pz = wristxyz[2]

# find theta0
theta0 = math.radians(math.degrees(atan2(py, px)))
t0=theta0


# account for 0.054 drop from Joint 3
armangle = atan2(pz-(1.94645), px)
px = px-0.054*(sin(armangle))
pz = pz+0.054*(cos(armangle))


# determine the length from origin to wrist centre looking top down to determine the X value for theta 2
pxtheta = sqrt(py*py+px*px)

# link 1 length is 1.25 link 2 length is 1.500
l1 = 1.25 # a2
l2 = 1.5 # d4

# set joint 2 centre for theta 2 and 3 calculations
pxtheta = pxtheta-0.35
pztheta = pz - 0.75

## Calculateangle theta2 using law of cosines
D=(pxtheta*pxtheta + pztheta*pztheta - l1*l1-l2*l2)/(2*l1*l2)
# calculate theta 2
theta2 = atan2(-sqrt(1-D*D),D)
t2= theta2

# Using theta 2, calculate theta 1
S1=((l1+l2*cos(theta2))*pztheta-l2*sin(theta2)*pxtheta)/(pxtheta*pxtheta + pztheta*pztheta)
C1=((l1+l2*cos(theta2))*pxtheta+l2*sin(theta2)*pztheta)/(pxtheta*pxtheta + pztheta*pztheta)
theta1=atan2(S1,C1)
t1 = theta1 




# adjust theta 2 angle for zero configuration
theta2=-(theta2+pi/2)


# adjust theta 1 angle for zero configuration
theta1 =pi/2-theta1

print(" ")
print(" ")
print(" ")

print("theta0 end result is:", theta0 )

print("theta1  radiansend result is:", math.radians(math.degrees(theta1)))


print("theta2 end result is:" ,math.radians(math.degrees(theta2)))    

#print(T0_3)

# Theta 4,5,6 calc

R0_3=T0_3.extract([0,1,2],[0,1,2])
R0_3 = R0_3.evalf(subs={q1: theta0, q2: theta1, q3: theta2})

R3_6 = simplify(Transpose(R0_3))
R3_6 = R3_6 * RotM
r31 = R3_6[2,0]
r11 = R3_6[0,0]
r21 = R3_6[1,0]
r32 = R3_6[2,1]
r33 = R3_6[2,2]

beta  = atan2(-r31, sqrt(r11 * r11 + r21 * r21)) 
gamma = atan2(r32, r33) - pi/2
alpha = atan2(r21, r11) - pi/2
print(" ")
print(" ")
print(" ")
print("alpha in degrees is = ",(math.degrees(alpha)))
print("alpha is = ",math.radians(math.degrees(alpha)))
print("beta in degrees is = ",(math.degrees(beta)))
print("beta is = ",math.radians(math.degrees(beta)))
print("gamma in degrees is = ",(math.degrees(gamma)))
print("gamma is = ",math.radians(math.degrees(gamma)))



