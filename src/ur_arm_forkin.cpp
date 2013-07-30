/*
 *    Universal Robots UR5 forward kinematics
 *    Copyright (C) 2012 Berk Calli
 *
 *    This program is free software: you can redistribute it and/or modify
 *    it under the terms of the GNU General Public License as published by
 *    the Free Software Foundation, either version 3 of the License, or
 *    (at your option) any later version.
 *
 *    This program is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *    GNU General Public License for more details.
 *
 *    You should have received a copy of the GNU General Public License
 *    along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <math.h>

#include "ur_arm/ur_arm_forkin.h"

URForwardKinSolver::URForwardKinSolver(){

	a.resize(6);
	alpha.resize(6);
	d.resize(6);
	theta.resize(6);
	H = new Eigen::MatrixXd[6];
	for (int i = 0;i<6; i++)
		H[i].resize(4,4);

	pi = 3.14159265;

	a(0) = 0;
	a(1) = -0.425;//l3;
	a(2) = -0.39225;//l5;
	a(3) = 0;
	a(4) = 0;
	a(5) = 0;

	alpha(0) = pi/2.0;
	alpha(1) = 0;
	alpha(2) = 0;
	alpha(3) = pi/2.0;
	alpha(4) = -pi/2.0;
	alpha(5) = 0;

	d(0) = 0.089159;//l1;
	d(1) = 0;//l2;
	d(2) = 0;//-l4;
	d(3) = 0.10915;//l6;
	d(4) = 0.09465;//l7;
	d(5) = 0.0823;//l8;
}

void URForwardKinSolver::solveForwardKin(double q[]){
	using std::sin;
	using std::cos;

	theta(0) = q[0];//pi/2.0+q[0];//3*pi/2.0-q[0];
	theta(1) = q[1];//pi/2.0-(q[1]+pi/2.0);
	theta(2) = q[2];//q[2];
	theta(3) = q[3];//3*pi/2.0-(q[3]+pi/2);
	theta(4) = q[4];
	theta(5) = q[5];

	for (int i = 0; i<6; i++){
		H[i](0,0) = cos(theta(i));
		H[i](0,1) = -sin(theta(i))*cos(alpha(i));
		H[i](0,2) = sin(theta(i))*sin(alpha(i));
		H[i](0,3) = a(i)*cos(theta(i));
		H[i](1,0) = sin(theta(i));
		H[i](1,1) = cos(theta(i))*cos(alpha(i));
		H[i](1,2) = -cos(theta(i))*sin(alpha(i));
		H[i](1,3) = a(i)*sin(theta(i));
		H[i](2,0) = 0;
		H[i](2,1) = sin(alpha(i));
		H[i](2,2) = cos(alpha(i));
		H[i](2,3) = d(i);
		H[i](3,0) = 0;
		H[i](3,1) = 0;
		H[i](3,2) = 0;
		H[i](3,3) = 1;
	}

	Eigen::MatrixXd H_res(4,4); H_res << 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1;

		H_res = H[0]*H[1]*H[2]*H[3]*H[4]*H[5];


	T[0] = H_res(0,3);
	T[1] = H_res(1,3);
	T[2] = H_res(2,3);

	for (int i = 0; i<3; i++)
		for(int j = 0; j<3; j++)
			R[i][j] = H_res(i,j);

}



void URForwardKinSolver::matMul33(double a1[][3], double a2[][3], double a3[][3])
{
   for(int i = 0; i < 3; i++)
	 for(int j = 0; j < 3; j++)
	   for(int k = 0; k < 3; k++)
		 a3[i][j] +=  a1[i][k] * a2[k][j];
}

void URForwardKinSolver::matMul44(double a1[][4], double a2[][4], double a3[][4])
{
   for(int i = 0; i < 4; i++)
	 for(int j = 0; j < 4; j++)
	   for(int k = 0; k < 4; k++)
		 a3[i][j] +=  a1[i][k] * a2[k][j];
}


void URForwardKinSolver::euler2rot(double yaw, double pitch, double roll, double resMat[][3]){
	double Rz[3][3] = {{cos(yaw),-sin(yaw),0},{sin(yaw),cos(yaw),0},{0,0,1}};
	double Ry[3][3] = {{cos(pitch),0,sin(pitch)},{0,1,0},{-sin(pitch),0,cos(pitch)}};
	double Rx[3][3] = {{1,0,0},{0,cos(roll),-sin(roll)},{0,sin(roll),cos(roll)}};
	double res1[3][3] = {{0,0,0},{0,0,0},{0,0,0}};
	matMul33(Rz,Ry,res1);
	matMul33(res1,Rx,resMat);
}


void URForwardKinSolver::vecMul3(double a1[][3], double a2[], double a3[])
{
   for(int i = 0; i < 3; i++)
	 for(int k = 0; k < 3; k++)
	 {
	   a3[i] +=  (a1[i][k] * a2[k]);
	 }
}

void URForwardKinSolver::vecMul4(double a1[][4], double a2[], double a3[])
{
   for(int i = 0; i < 4; i++)
	 for(int k = 0; k < 4; k++)
	 {
	   a3[i] +=  (a1[i][k] * a2[k]);
	 }
}
