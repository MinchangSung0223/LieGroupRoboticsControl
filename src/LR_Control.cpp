#include "LR_Control.h"

LR_Control::LR_Control()
{
    // Constructor implementation
    this->Slist.resize(6, JOINTNUM);
    this->Blist.resize(6, JOINTNUM);
    this->Glist;
    this->Mlist;
    this->M.resize(4, 4);
    this->g.resize(3);
    this->g << 0, 0, 0;
}

JVec LR_Control::TaskRobustControl(JVec q, JVec q_dot, JVec q_ddot, SE3 T_des, Vector6d V_des, Vector6d V_dot_des, Vector6d &lambda_int, double dt, Vector6d Task_Kp, Vector6d Task_Kv, JVec Task_K)
{
    SE3 T = lr::FKinBody(this->M, this->Blist, q);
    Jacobian Jb = lr::JacobianBody(this->Blist, q);
    Jacobian Jb_dot = lr::dJacobianBody(this->M, this->Blist, q, q_dot);
    SE3 T_err = lr::TransInv(T) * T_des;
    SE3 invT_err = TransInv(T_err);
    Vector6d V = Jb * q_dot;
    Vector6d V_err = V_des - Ad(invT_err) * V;
    Vector6d lambda = lr::se3ToVec(lr::MatrixLog6(T_err));
    lambda_int += lambda * dt;
    Vector6d lambda_dot = dlog6(-lambda) * V_err;
    Vector6d V_dot = Jb_dot * q_dot + Jb * q_ddot;
    Vector6d V_dot_err = V_dot_des - Ad(invT_err) * V_dot + ad(V_err) * V_dot;
    Vector6d lambda_dot_ref = Task_Kv.asDiagonal() * lambda + Task_Kp.asDiagonal() * lambda_int;
    Vector6d lambda_ddot_ref = Task_Kv.asDiagonal() * lambda_dot + Task_Kp.asDiagonal() * lambda;
    Vector6d V_ref = Ad(T_err) * (V_des + dexp6(-lambda) * (lambda_dot_ref));
    Vector6d V_dot_ref = Ad(T_err) * (V_dot_des + (dexp6(-lambda) * lambda_ddot_ref) + ad(V_err) * V_dot - (ddexp6(-lambda, -lambda_dot) * lambda_dot));
    pinvJacobian invJb = Jb.transpose() * (Jb * Jb.transpose()).inverse();
    JVec qddot_ref = invJb * (V_dot_ref - Jb_dot * q_dot);
    JVec q_dot_ref = invJb * V_ref;
    JVec edot = q_dot_ref - q_dot;
    JVec tau_ref = Task_K.asDiagonal() * edot;
    MassMat M = this->MassMatrix(q);
    MatrixNd C = this->CoriolisMatrix(q, q_dot);
    JVec G = this->GravityForces(q);
    JVec torques = M * qddot_ref + C * q_dot_ref + G + tau_ref;
    return torques;
}
MassMat LR_Control::MassMatrix(JVec q_)
{
    MassMat M = MassMat::Zero();
    pinocchio::Data data(model);
    Eigen::VectorXd q = q_;
    Eigen::VectorXd v = pinocchio::randomConfiguration(model);
    Eigen::VectorXd a = pinocchio::randomConfiguration(model);
    pinocchio::crba(model, data, q);
    data.M.triangularView<Eigen::StrictlyLower>() =
        data.M.transpose().triangularView<Eigen::StrictlyLower>();
    M = data.M;
    return M;
}
MassMat LR_Control::MassMatrixInverse(JVec q_)
{
    MassMat Minv = MassMat::Zero();
    pinocchio::Data data(model);
    Eigen::VectorXd v = pinocchio::randomConfiguration(model);
    Eigen::VectorXd a = pinocchio::randomConfiguration(model);
    Eigen::VectorXd q = pinocchio::randomConfiguration(model);
    pinocchio::computeMinverse(model, data, q_);
    data.Minv.triangularView<Eigen::StrictlyLower>() = data.Minv.transpose().triangularView<Eigen::StrictlyLower>();
    Minv = data.Minv;
    return Minv;
}
JVec LR_Control::GravityForces(JVec q_)
{
    JVec G = JVec::Zero();
    pinocchio::Data data(model);
    Eigen::VectorXd v = pinocchio::randomConfiguration(model);
    Eigen::VectorXd a = Eigen::VectorXd::Zero(model.nv);
    Eigen::VectorXd q = pinocchio::randomConfiguration(model);
    q = q_;
    Eigen::VectorXd tau = pinocchio::rnea(model, data, q, v * 0, a * 0);
    pinocchio::computeGeneralizedGravity(model, data, q);
    G = data.g;
    return G;
}

MatrixNd LR_Control::CoriolisMatrix(JVec q_, JVec q_dot)
{
    MatrixNd C = MatrixNd::Zero();
    pinocchio::Data data(model);

    Eigen::VectorXd v = pinocchio::randomConfiguration(model);
    Eigen::VectorXd a = Eigen::VectorXd::Zero(model.nv);
    Eigen::VectorXd q = pinocchio::randomConfiguration(model);
    q = q_;
    v = q_dot;
    Eigen::VectorXd tau = pinocchio::rnea(model, data, q, v, a);
    pinocchio::computeCoriolisMatrix(model, data, q, v);
    C = data.C;
    return C;
}
JVec LR_Control::ForwardDynamics(JVec q, JVec q_dot, JVec tau)
{
    pinocchio::Data data(model);
    JVec q_ddot = JVec::Zero();
    pinocchio::integrate(model, q, q_dot, tau);
    pinocchio::aba(model, data, q, q_dot, tau);
    q_ddot = data.ddq;
    return q_ddot;
}

JVec LR_Control::HinfControl(JVec q, JVec q_dot, JVec q_des, JVec q_dot_des, JVec q_ddot_des, JVec& e_int, double dt, JVec Hinf_K, JVec gamma)
{
    // gamma = invL2sqr
    MatrixNd Hinf_Kp = MatrixNd::Identity() * 100.0;
    MatrixNd Hinf_Kv = MatrixNd::Identity() * 20.0;
    MatrixNd Hinf_K_gamma = MatrixNd::Identity();
    for (int i = 0; i < JOINTNUM; i++)
    {
        Hinf_K_gamma(i, i) = Hinf_K(i) + 1.0 / gamma(i);
    }
    JVec e = q_des - q;
    JVec e_dot = q_dot_des - q_dot;
    e_int += e*dt;
    // MassMat Mmat = lr::MassMatrix(q, this->Mlist, this->Glist, this->Slist);
    MassMat Mmat = this->MassMatrix(q);
    // JVec C = lr::VelQuadraticForces(q, dq, this->Mlist, this->Glist, this->Slist);
    MatrixNd C = this->CoriolisMatrix(q, q_dot);
    // JVec G = lr::GravityForces(q, this->g, this->Mlist, this->Glist, this->Slist);
    JVec G = this->GravityForces(q);
    JVec q_ddot_ref = q_ddot_des + Hinf_Kv * e_dot + Hinf_Kp * e;
    JVec q_dot_ref = q_dot_des + Hinf_Kv * e_dot + Hinf_Kp * e_int;
    JVec torq = Mmat * q_ddot_ref + C * q_dot_ref + G + (Hinf_K_gamma) * (e_dot + Hinf_Kv * e + Hinf_Kp * e_int);
    return torq;
}



struct LR_info
{
    SE3 M;
    Eigen::MatrixXd Slist;
    std::vector<SE3> Mlist;
    std::vector<Matrix6d> Glist;
};

void print_LR_info(LR_info lr_info)
{
    std::cout << "=======================================M=======================================" << std::endl;
    std::cout << lr_info.M << std::endl;
    std::cout << "=======================================Slist===================================" << std::endl;
    std::cout << lr_info.Slist << std::endl;
    std::cout << "=======================================Mlist===================================" << std::endl;

    for (int i = 0; i < lr_info.Mlist.size(); i++)
    {
        SE3 M = lr_info.Mlist.at(i);
        std::cout << "---------------------------------------M" << i << "--------------------------------------" << std::endl;
        std::cout << M << std::endl;
    }
    std::cout << "=======================================Glist===================================" << std::endl;
    for (int i = 0; i < lr_info.Glist.size(); i++)
    {
        Matrix6d G = lr_info.Glist.at(i);
        std::cout << "---------------------------------------G" << i << "--------------------------------------" << std::endl;
        std::cout << G << std::endl;
    }
}
Eigen::MatrixXd list_Slist_to_matrix_Slist(std::vector<Vector6d> Slist)
{
    int N = Slist.size();
    Eigen::MatrixXd ret_Slist(6, N);
    for (int i = 0; i < N; i++)
    {
        ret_Slist.col(i) = Slist.at(i);
    }
    return ret_Slist;
}

SE3 Pose_to_SE3(urdf::Pose pose){
    SE3 T= SE3::Identity();
    T(0,3) = pose.position.x;
    T(1,3) = pose.position.y;
    T(2,3) = pose.position.z;
    
    double quat_w,quat_x,quat_y,quat_z;
    pose.rotation.getQuaternion(quat_x,quat_y,quat_z,quat_w);
    Eigen::Quaterniond quat=Eigen::Quaterniond(quat_w,quat_x,quat_y,quat_z);
    Eigen::Matrix3d R = quat.normalized().toRotationMatrix();
    T.block<3, 3>(0, 0)=R;
    return T;
}

void LR_Control::LRSetup(const char *urdf_path,Vector3d g){
    this->g = g;
    this->LRSetup(urdf_path);
}
void LR_Control::LRSetup(const char *urdf_path)
{
    //pinocchio setup
    pinocchio::urdf::buildModel(urdf_path, model);
    model.gravity.linear(this->g);
    //LR setup
 urdf::ModelInterfaceSharedPtr robot = urdf::parseURDFFile(urdf_path);
    if (!robot)
    {
        std::cerr << "ERROR: Model Parsing the URDF failed" << std::endl;
    }
    std::cout << "robot name is: " << robot->getName() << std::endl;
    std::cout << "---------- Successfully Parsed URDF ---------------" << std::endl;
    urdf::LinkConstSharedPtr root_link = robot->getRoot();
    std::cout << "root Link: " << root_link->name << " has " << root_link->child_links.size() << " child(ren)" << std::endl;

    std::stack<std::pair<urdf::LinkConstSharedPtr, int>> stack;
    stack.push(std::make_pair(root_link, 0));
    std::vector<Matrix6d> Glist;
    std::vector<SE3> Mlist;
    std::vector<Vector6d> Slist;

    SE3 M= SE3::Identity();

    SE3 prev_T_com = SE3::Identity();
    SE3 T_jnt = SE3::Identity();
    SE3 T_com = SE3::Identity();
    int count = 0;
    SE3 M_prev = SE3::Identity();
    while (!stack.empty())
    {
        auto current = stack.top();
        urdf::LinkConstSharedPtr link = current.first;
        int level = current.second;
        stack.pop();
        if (link->parent_joint)
        {
            Matrix6d G = Matrix6d::Identity();
            const auto &inertial = link->inertial;
            const auto &joint = link->parent_joint;
            double mass = inertial->mass;
            std::cout<<joint->name<<std::endl;
            G(0, 0) = mass;
            G(1, 1) = mass;
            G(2, 2) = mass;
            G(3, 3) = inertial->ixx;
            G(4, 4) = inertial->iyy;
            G(5, 5) = inertial->izz;
            G(3, 4) = G(4, 3) = inertial->ixy;
            G(3, 5) = G(5, 3) = inertial->ixz;

            const urdf::Pose &T_com_to_jnt_ = joint->parent_to_joint_origin_transform;
            const urdf::Pose &T_jnt_to_com_ = inertial->origin;
            SE3 T_com_to_jnt = Pose_to_SE3(T_com_to_jnt_);
            SE3 T_jnt_to_com = Pose_to_SE3(T_jnt_to_com_);
            T_jnt = T_jnt*T_com_to_jnt;
            SO3 R_jnt = T_jnt.block<3, 3>(0, 0);
            Vector3d p = T_jnt.block<3, 1>(0, 3);
            Vector3d w = R_jnt*Vector3d(joint->axis.x, joint->axis.y, joint->axis.z);
            Vector3d v = -w.cross(p);
            SE3 M_now = T_jnt*T_jnt_to_com;
            SE3 M_prev_now = TransInv(M_prev)*M_now;
            M = M_now;
            T_com = T_jnt*T_jnt_to_com;
            if (link->parent_joint->type == urdf::Joint::REVOLUTE || link->parent_joint->type == urdf::Joint::PRISMATIC )
            {
                //std::cout << "count : " << count << std::endl;
                Glist.push_back(G);
                Vector6d S = Vector6d::Zero();
                S.block<3, 1>(0, 0) = v;
                S.block<3, 1>(3, 0) = w;
                Slist.push_back(S);
                if (count++ > 0)
                {
                    Mlist.push_back(M_prev_now);
                }
                M_prev=M_now;
            }
            else if (link->parent_joint->type == urdf::Joint::FIXED){
                if(link->child_links.size()==0){
                    Mlist.push_back(M_prev_now);
                }
            }
        }
        for (auto &child_link : link->child_links)
        {
            if (child_link)
            {
                stack.push(std::make_pair(child_link, level + 2));
            }
        }
    }
    Eigen::MatrixXd Slist_ = list_Slist_to_matrix_Slist(Slist);
    LR_info lr_info;
    lr_info.M = M;
    lr_info.Slist = Slist_;
    lr_info.Mlist = Mlist;
    lr_info.Glist = Glist;
    this->Slist = lr_info.Slist;
    this->M = lr_info.M;
    this->Mlist = lr_info.Mlist;
    this->Glist = lr_info.Glist;
    this->Blist = Ad(TransInv(M)) * lr_info.Slist;
    print_LR_info(lr_info);
}
