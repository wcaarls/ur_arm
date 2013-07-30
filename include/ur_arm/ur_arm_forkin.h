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

#ifndef UR_ARM_FORKIN_H_
#define UR_ARM_FORKIN_H_

#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Core>

class URForwardKinSolver{

	Eigen::MatrixXd* H;
	Eigen::VectorXd a;
	Eigen::VectorXd alpha;
	Eigen::VectorXd d;
	Eigen::VectorXd theta;
	double pi;

public:
	double T[3];
	double R[3][3];
	URForwardKinSolver();
	void solveForwardKin(double q[]);
	void matMul33(double a1[][3], double a2[][3], double a3[][3]);
	void matMul44(double a1[][4], double a2[][4], double a3[][4]);
	void euler2rot(double yaw, double pitch, double roll, double resMat[][3]);
	void vecMul3(double a1[][3], double a2[], double a3[]);
	void vecMul4(double a1[][4], double a2[], double a3[]);
};

#endif /* UR_ARM_FORKIN_H_ */
