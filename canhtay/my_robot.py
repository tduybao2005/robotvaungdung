from roboticstoolbox import DHRobot, RevoluteMDH
from spatialmath import SE3
import numpy as np

# Bang tham so DH theo QUY UOC MODIFIED DH (Craig convention), giong DHViz:
# i-1_T_i duoc dinh nghia boi alpha_{i-1}, a_{i-1}, d_i, theta_i
#
# Hang 1 (Fixed - be do base, khong xoay): alpha0=0, a0=0, d1=0.2, theta1=0
# Hang 2 (Revolute - khop 1):              alpha1=90, a1=0,   d2=0, theta2=90 (offset)
# Hang 3 (Revolute - khop 2):              alpha2=0,  a2=0.2, d3=0, theta3=0
# Hang 4 (Revolute - khop 3):              alpha3=0,  a3=0.2, d4=0, theta4=0
# Hang 5 (Revolute - khop 4):              alpha4=0,  a4=0,   d5=0, theta5=-90 (offset)
# Hang 6 (Revolute - khop 5):              alpha5=0,  a5=0.2, d6=0, theta6=0
# Hang 7 (Fixed - frame dung cu cuoi tay): alpha6=-90, a6=0,  d7=0, theta7=0

L1 = RevoluteMDH(alpha=np.pi / 2, a=0,   d=0, offset=np.pi / 2)
L2 = RevoluteMDH(alpha=0,         a=0.2, d=0, offset=0)
L3 = RevoluteMDH(alpha=0,         a=0.2, d=0, offset=0)
L4 = RevoluteMDH(alpha=0,         a=0,   d=0, offset=-np.pi / 2)
L5 = RevoluteMDH(alpha=0,         a=0.2, d=0, offset=0)

my_robot = DHRobot([L1, L2, L3, L4, L5], name='5-DOF_robot')

# Hang 1 (Fixed): be do base - chi tinh tien len d=0.2m, khong xoay
my_robot.base = SE3(0, 0, 0.2)

# Hang 7 (Fixed): frame dung cu cuoi tay - xoay -90 do quanh truc X
my_robot.tool = SE3.Rx(-np.pi / 2)

print(my_robot)

q_zero = [0, 0, 0, 0, 0]

my_robot.plot(q_zero, block=True)
