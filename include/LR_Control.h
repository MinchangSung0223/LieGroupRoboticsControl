
#ifndef LR_CONTROL_H
#define LR_CONTROL_H

#include "liegroup_robotics.h"
#include "urdf_parser/urdf_parser.h"
#include <cstring>
#include <iostream>
#include <fstream>
#include <stack>
#include <utility> // for std::pair
#include "pinocchio_header.h"


using namespace std;
using namespace lr;
class LR_Control {
public:
    LR_Control();  // Constructor
    pinocchio::Model model;    
    ScrewList Slist;
    ScrewList Blist;
    SE3 M;

	vector<Matrix6d> Glist;	
	vector<SE3> Mlist;	
    Vector3d g;
    void LRSetup(const char* urdf_path);
    void LRSetup(const char* urdf_path,Vector3d g);
    JVec HinfControl(JVec q, JVec dq, JVec q_des, JVec q_dot_des, JVec q_ddot_des, JVec& eint,double dt, JVec Hinf_K, JVec gamma);
    //JVec TaskHinfControl( JVec q,JVec q_dot,JVec q_ddot, SE3 T_des,Vector6d V_des,Vector6d V_dot_des,Vector6d eint);
    JVec TaskRobustControl(JVec q, JVec q_dot, JVec q_ddot, SE3 T_des, Vector6d V_des, Vector6d V_dot_des, Vector6d &lambda_int, double dt,Vector6d Task_Kp, Vector6d Task_Kv , JVec Task_K);
    MassMat MassMatrix(JVec q_);
    MassMat MassMatrixInverse(JVec q_);
    MatrixNd CoriolisMatrix(JVec q,JVec q_dot);
    JVec GravityForces(JVec q_);
    JVec ForwardDynamics( JVec q,JVec dq ,JVec tau);
};


#endif // LR_CONTROL_H

