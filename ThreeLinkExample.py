import modern_robotics as mr
import numpy as np
import math
from math import cos,sin
from matplotlib import pyplot as plt
from numpy import matmul
from modern_robotics import *
L1 = 1
L2 = 1
L3 = 1
PI = math.pi
th1 = PI/2
th2 = PI/2
th3 = PI/2

M  = np.array(
[[1,0,0,L1+L2+L3],
[0,1,0,0],
[0,0,1,0],
[0,0,0,1]])
p0 =np.array([0,0,0,1])

screw = 1
if screw==0:
	for i in range(180):
		plt.xlim(-3,3)
		plt.ylim(-3,3)

		th1 = i*PI/180
		th2 = i*PI/180
		th3 = i*PI/180
		T01 = np.array(
		[[cos(th1),-sin(th1),0,0],
		[sin(th1),cos(th1),0,0],
		[0,0,1,0],
		[0,0,0,1]])

		T12 = np.array(
		[[cos(th2),-sin(th2),0,L1],
		[sin(th2),cos(th2),0,0],
		[0,0,1,0],
		[0,0,0,1]])

		T23 = np.array(
		[[cos(th3),-sin(th3),0,L2],
		[sin(th3),cos(th3),0,0],
		[0,0,1,0],
		[0,0,0,1]])

		T34 = np.array(
		[[1,0,0,L3],
		[0,1,0,0],
		[0,0,1,0],
		[0,0,0,1]])
		
		T02 = matmul(T01,T12)
		T03 = matmul(matmul(T01,T12),T23)
		T04 = matmul(matmul(matmul(T01,T12),T23),T34)

		p1 = matmul(T01,p0)
		p2 = matmul(T02,p0)
		p3 = matmul(T03,p0)
		p4 = matmul(T04,p0)
		fig = plt.gcf()
		fig.set_size_inches(5, 5)
		plt.rcParams["figure.figsize"] = (14,14)
		plt.rcParams['lines.linewidth'] = 4
		plt.rcParams['lines.color'] = 'r'
		plt.rcParams['axes.grid'] = True 
		plt.title('DH parameter')
		plt.plot([p1[0],p2[0],p3[0],p4[0]], [p1[1],p2[1],p3[1],p4[1]],'.-')
		plt.plot([p1[0],p2[0],p3[0],p4[0]], [p1[1],p2[1],p3[1],p4[1]],'r.')
		plt.pause(0.05)
		plt.clf()

else:
	for i in range(180):
		S3 = [0,0,1,0,-(L1+L2),0]
		S2 = [0,0,1,0,-(L1),0]
		S1 = [0,0,1,0,0,0]
		th1 = i*PI/180
		th2 = i*PI/180
		th3 = i*PI/180
		Slist = np.array([S1,S2,S3]).T
		thetalist = np.array([th1, th2, th3])
		M  = np.array(
		[[1,0,0,L1+L2+L3],
		[0,1,0,0],
		[0,0,1,0],
		[0,0,0,1]])
		T04 = FKinSpace(M,Slist,thetalist)

		Slist = np.array([S1,S2]).T
		thetalist = np.array([th1, th2])

		M  = np.array(
		[[1,0,0,L1+L2],
		[0,1,0,0],
		[0,0,1,0],
		[0,0,0,1]])
		T03 = FKinSpace(M,Slist,thetalist)
		Slist = np.array([S1]).T
		thetalist = np.array([th1])
		M  = np.array(
		[[1,0,0,L1],
		[0,1,0,0],
		[0,0,1,0],
		[0,0,0,1]])
		T02 = FKinSpace(M,Slist,thetalist)
		Slist = np.array([]).T
		thetalist = np.array([])
		M  = np.array(
		[[1,0,0,0],
		[0,1,0,0],
		[0,0,1,0],
		[0,0,0,1]])
		T01 = FKinSpace(M,Slist,thetalist)
		
		p1 = matmul(T01,p0)
		p2 = matmul(T02,p0)
		p3 = matmul(T03,p0)
		p4 = matmul(T04,p0)
		fig = plt.gcf()
		fig.set_size_inches(5, 5)
		plt.rcParams["figure.figsize"] = (14,14)
		plt.rcParams['lines.linewidth'] = 4
		plt.rcParams['lines.color'] = 'r'
		plt.rcParams['axes.grid'] = True 
		plt.xlim(-3,3)
		plt.ylim(-3,3)
		plt.title('Screw')
		plt.plot([p1[0],p2[0],p3[0],p4[0]], [p1[1],p2[1],p3[1],p4[1]],'.-')
		plt.plot([p1[0],p2[0],p3[0],p4[0]], [p1[1],p2[1],p3[1],p4[1]],'r.')
		plt.pause(0.05)
		plt.clf()

