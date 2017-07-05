import numpy as np
from numpy import array
from sympy import symbols, cos, sin, pi, simplify, sqrt, atan2, acos, atan
from sympy.matrices import Matrix
import math
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
print("T5_6")
#Transforms
T0_1 = Matrix([[             cos(q1),            -sin(q1),            0,              a0],
               [ sin(q1)*cos(al0), cos(q1)*cos(al0), -sin(al0), -sin(al0)*d1],
               [ sin(q1)*sin(al0), cos(q1)*sin(al0),  cos(al0),  cos(al0)*d1],
               [                   0,                   0,            0,               1]])
T0_1 = T0_1.subs(s)
print(T0_1)
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
# Rotation Matrix for wrist centre solution
d7 = 0.303
pxf = 2.1452
pyf = 0.220663
pzf = 1.94754
#euler = Q_To_E(0.0002368,0.000126,0.000389,0.99999)
euler = Q_To_E(0.0,0.000126,0.0,0.99999)
R_x = Matrix([[ 1,              0,        0],
              [ 0,        cos(math.radians(euler[0])), -sin(math.radians(euler[0]))],
              [ 0,        sin(math.radians(euler[0])),  cos(math.radians(euler[0]))]])

R_y = Matrix([[ cos(math.radians(euler[1])),        0,  sin(math.radians(euler[1]))],
              [       0,        1,        0],
              [-sin(math.radians(euler[1])),        0,  cos(math.radians(euler[1]))]])

R_z = Matrix([[ cos(math.radians(euler[2])), -sin(math.radians(euler[2])),        0],
              [ sin(math.radians(euler[2])),  cos(math.radians(euler[2])),        0],
              [ 0,              0,        1]])


R_Roll = R_x*R_y*R_z

# px, py, pz location of end effector
pmatrix = Matrix([[pxf],[pyf],[pzf]])
# find the euler angles for nx, ny, nz for wrist centre calculation
wristn = Matrix([[cos(math.radians(euler[2]))*cos(math.radians(euler[1]))],[sin(math.radians(euler[2]))*cos(math.radians(euler[1]))],[-sin(math.radians(euler[2]))]])
#print("wristmatrix:  ", wristn)

print("X,Y,Z", math.radians(euler[0]), math.radians(euler[1]),math.radians(euler[2]))
##print("R_ROLL:",R_Roll.evalf(subs={d7: 0.303, Roll: math.radians(euler[0]), Pitch: math.radians(euler[1]), Yaw: math.radians(euler[2]), px: 2.09, py: 0, pz: 2.3449}))
##print("T0_1 = ",T0_1.evalf(subs={q1: 0, q2: 0, q3: 0, q4: 0, q5: 0, q6: 0}))
##print("T0_2 = ",T0_2.evalf(subs={q1: 0, q2: 0, q3: 0, q4: 0, q5: 0, q6: 0}))
##print("T0_3 = ",T0_3.evalf(subs={q1: 0, q2: 0, q3: 0, q4: 0, q5: 0, q6: 0}))
##print("T0_4 = ",T0_4.evalf(subs={q1: 0, q2: 0, q3: 0, q4: 0, q5: 0, q6: 0}))
##print("T0_5 = ",T0_5.evalf(subs={q1: 0, q2: 0, q3: 0, q4: 0, q5: 0, q6: 0}))
##print("T0_6 = ",T0_6.evalf(subs={q1: 0, q2: 0, q3: 0, q4: 0, q5: 0, q6: 0}))
##print("T0_G = ",T0_G.evalf(subs={q1: 0, q2: 0, q3: 0, q4: 0, q5: 0, q6: 0}))
#print(" rotmat", RMatrix)
#print("R0_6 = ",R0_6.evalf(subs={q1: 0, q2: 0, q3: 0, q4: 0, q5: 0, q6: 0}))

#print("Z0_6 = ",z0_6.evalf(subs={q1: 0, q2: 0, q3: 0, q4: 0, q5: 0, q6: 0}))

T_total = T0_G * R_corr
#calcualte wrist centre location in respect to base
#wristxyz = pmatrix-(d7)*(R_Roll*Matrix([[1],[0],[0]]))
wristxyz = pmatrix-(d7)*wristn

print("wristcentre: ", wristxyz)
#print("wristcentre ",wristxyz.evalf(subs={d7: 0.303, Roll: math.radians(euler[0]), Pitch: math.radians(euler[1]), Yaw: math.radians(euler[2]), px: 0.4151, py: 1.8389, pz: 1.9281}))
#solve wrist joints
px = wristxyz[0]
py = wristxyz[1]
pz = wristxyz[2]
theta0 = math.radians(math.degrees(atan2(py, px)))
print("theta0 ",theta0)
armangle = atan2(pz-(1.94645), px)
px = px-0.054*(sin(armangle))
pz = pz+0.054*(cos(armangle))
#print("px, pz", px, pz)
# determine the length from origin to wrist centre looking top down to determine the X value for theta 2
pxtheta = sqrt(py*py+px*px)
#print("armangle:", sin(armangle))
#print("pxtheta=",pxtheta)
# wc is 0.696 up and 0.35 over in reference to Joint 2
# link 1 length is 1.25 link 2 length is 1.500
l1 = 1.25
l2 = 1.5
#pxtheta = px - 0.75
#pytheta = py - 0.35
pxtheta = pxtheta-0.35
pztheta = pz - 0.75
## Create symbols for joint variables
print (l2)
D=(pxtheta*pxtheta + pztheta*pztheta - l1*l1-l2*l2)/(2*l1*l2)
#print(D.evalf(subs={d7: 0.303, Yaw: 0, Pitch: 0,Roll:0, px: 2.153, py: 0, pz: 1.946}))
print(D)
theta2 = atan2(-sqrt(1-D*D),D)
#print("outcome 1 theta2:", theta2)
S1=((l1+l2*cos(theta2))*pztheta-l2*sin(theta2)*pxtheta)/(pxtheta*pxtheta + pztheta*pztheta)
C1=((l1+l2*cos(theta2))*pxtheta+l2*sin(theta2)*pztheta)/(pxtheta*pxtheta + pztheta*pztheta)
theta1=atan2(S1,C1)
theta2=-90-math.degrees(theta2)
theta1 =90-math.degrees(theta1)
#print("outcome 1 theta1",theta1.evalf(subs={d7: 0.303, Yaw: 0, Pitch: 0,Roll:0, px: 2.153, py: 0, pz: 1.946}))
##print("outcome 1 theta1",theta1)
##print("outcome 1 theta2",theta2)
print("theta0 end result is:", theta0 )
print("theta1 end result is:", math.radians(theta1))
print("theta2 end result is:" ,math.radians(theta2))    

alphaa= atan2(pztheta,pxtheta)

cln=sqrt(pxtheta*pxtheta+pztheta*pztheta)

phit = math.pi/2-acos((-cln*cln+l2*l2+l1*l1)/(2*l1*l2))

##print("theta2",phit)
##
##print("T0_1 = ",T0_1.evalf(subs={q1: theta0, q2: theta1, q3: theta2, q4: 0, q5: 0, q6: 0}))
##print("T0_2 = ",T0_2.evalf(subs={q1: theta0, q2: theta1, q3: theta2, q4: 0, q5: 0, q6: 0}))
##print("T0_3 = ",T0_3.evalf(subs={q1: theta0, q2: theta1, q3: theta2, q4: 0, q5: 0, q6: 0}))


