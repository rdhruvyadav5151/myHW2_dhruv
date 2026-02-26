#include "ros2_kdl_package/kdl_control.hpp"
#include <cmath>

KDLControl::KDLControl() {
    // Hardcoded IIWA DH Parameters
    chain_.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ), KDL::Frame(KDL::Vector(0,0,0.34))));
    chain_.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotY), KDL::Frame(KDL::Vector(0,0,0))));
    chain_.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ), KDL::Frame(KDL::Vector(0,0,0.4))));
    chain_.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotY), KDL::Frame(KDL::Vector(0,0,0))));
    chain_.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ), KDL::Frame(KDL::Vector(0,0,0.4))));
    chain_.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotY), KDL::Frame(KDL::Vector(0,0,0))));
    chain_.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ), KDL::Frame(KDL::Vector(0,0,0.126))));
    
    nj_ = 7;
    fk_solver_ = std::make_shared<KDL::ChainFkSolverPos_recursive>(chain_);
    jac_solver_ = std::make_shared<KDL::ChainJntToJacSolver>(chain_);
}

KDL::JntArray KDLControl::velocity_ctrl_null(const KDL::JntArray &q, double t) {
    KDL::JntArray q_dot(nj_);
    
    // 1. Forward Kinematics to get current end-effector position
    KDL::Frame x_curr;
    fk_solver_->JntToCart(q, x_curr);

    // 2. Compute the Geometric Jacobian
    KDL::Jacobian J(nj_);
    jac_solver_->JntToJac(q, J);
    Eigen::MatrixXd J_eigen = J.data;

    // 3. Define the desired position and calculate the error (e_p)
    Eigen::Vector3d x_d(0.5, 0.0, 0.5); // Fixed target from kdl_params.yaml
    Eigen::Vector3d x_e = x_d - Eigen::Vector3d(x_curr.p.x(), x_curr.p.y(), x_curr.p.z());
    
    // Create desired task velocity vector (linear only for this task)
    Eigen::VectorXd v_des = Eigen::VectorXd::Zero(6);
    double Kp = 10.0; // Proportional gain
    v_des.head(3) = Kp * x_e; 

    // 4. Calculate the Moore-Penrose Pseudo-Inverse of the Jacobian
    Eigen::MatrixXd J_pinv = J_eigen.completeOrthogonalDecomposition().pseudoInverse();

    // 5. Calculate Secondary Task (q_0) for Joint Limit Avoidance
    Eigen::VectorXd q0(nj_);
    double lambda = 1.0;
    // [Inference] Using standard generic joint limits +/- 2.96 radians since exact IIWA limits were not provided in the assignment sheet.
    double q_max = 2.96; 
    double q_min = -2.96;
    
    for(int i = 0; i < nj_; i++) {
        // Gradient of the joint limit cost function W(q)
        double num = std::pow(q_max - q_min, 2) * (2 * q(i) - q_max - q_min);
        double den = lambda * std::pow(q_max - q(i), 2) * std::pow(q(i) - q_min, 2);
        q0(i) = num / den;
    }

    // 6. Calculate the Null Space Projector (I - J^+ * J)
    Eigen::MatrixXd I = Eigen::MatrixXd::Identity(nj_, nj_);
    Eigen::MatrixXd NullSpace = I - J_pinv * J_eigen;

    // 7. Final Control Law Application
    Eigen::VectorXd q_dot_eigen = J_pinv * v_des + NullSpace * q0;

    // Map back to KDL array format
    for(int i = 0; i < nj_; i++) {
        q_dot(i) = q_dot_eigen(i);
    }
    
    return q_dot; 
}

KDL::JntArray KDLControl::look_at_point_control(const KDL::JntArray &q, const KDL::Frame &aruco_pose) {
    KDL::JntArray q_dot(nj_);
    
    // 1. Calculate Camera Frame relative to Base
    KDL::Frame cam_frame;
    fk_solver_->JntToCart(q, cam_frame);

    // 2. Compute 's' (Normalized Vector to object)
    KDL::Vector c_P_o = aruco_pose.p; 
    double depth = c_P_o.Norm();
    
    if (depth < 0.01) return q_dot; // Prevent division by zero if target is too close
    
    Eigen::Vector3d s(c_P_o.x()/depth, c_P_o.y()/depth, c_P_o.z()/depth);
    
    // 3. Desired Vector (Look straight ahead along Optical Axis Z)
    Eigen::Vector3d s_d(0, 0, 1);

    // 4. Construct Interaction Matrix L(s)
    Eigen::MatrixXd L = Eigen::MatrixXd::Zero(3, 6);
    Eigen::Matrix3d I3 = Eigen::Matrix3d::Identity();

    // Skew symmetric matrix S(s) for rotational component
    Eigen::Matrix3d S_skew;
    S_skew << 0, -s(2), s(1),
              s(2), 0, -s(0),
             -s(1), s(0), 0;

    // Assemble L = [ -1/d * (I - s*s^T) , S(s) ] * Rotation Mapping
    Eigen::MatrixXd L_base(3, 6);
    L_base.block<3,3>(0,0) = (-1.0/depth) * (I3 - s * s.transpose());
    L_base.block<3,3>(0,3) = S_skew;
    
    // Extract camera rotation matrix R_c and map to spatial twist
    Eigen::Matrix3d R_c = Eigen::Map<Eigen::Matrix3d>(cam_frame.M.data).transpose();
    Eigen::MatrixXd R_block = Eigen::MatrixXd::Zero(6,6);
    R_block.block<3,3>(0,0) = R_c;
    R_block.block<3,3>(3,3) = R_c;
    
    L = L_base * R_block;

    // 5. Compute Camera Jacobian (J_c)
    KDL::Jacobian J(nj_);
    jac_solver_->JntToJac(q, J);
    Eigen::MatrixXd J_c = J.data; 

    // 6. Compute Control Output
    Eigen::MatrixXd LJ = L * J_c;
    Eigen::MatrixXd LJ_pinv = LJ.completeOrthogonalDecomposition().pseudoInverse();
    
    // Null space matrix N for secondary tasks
    Eigen::MatrixXd N = Eigen::MatrixXd::Identity(nj_, nj_) - LJ_pinv * LJ;
    Eigen::VectorXd q0 = Eigen::VectorXd::Zero(nj_); // Keeping secondary task zero for visual servoing stability
    
    // Final Visual Servoing Control Law
    double K_gain = 5.0; 
    Eigen::VectorXd q_dot_eigen = K_gain * LJ_pinv * s_d + N * q0;

    // Map back to KDL array format
    for(int i = 0; i < nj_; i++) {
        q_dot(i) = q_dot_eigen(i);
    }
    
    return q_dot;
}