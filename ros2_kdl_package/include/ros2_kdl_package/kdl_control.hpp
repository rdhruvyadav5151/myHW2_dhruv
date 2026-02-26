#ifndef KDL_CONTROL_HPP
#define KDL_CONTROL_HPP

#include <kdl/chain.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/frames.hpp>
#include <eigen3/Eigen/Dense>
#include <vector>

class KDLControl {
public:
    KDLControl();
    void resize(int n_joints);
    
    // Q1(b): Velocity Control with Null Space
    KDL::JntArray velocity_ctrl_null(const KDL::JntArray &q, double t);
    
    // Q2(b): Vision-Based Control
    KDL::JntArray look_at_point_control(const KDL::JntArray &q, const KDL::Frame &aruco_pose);

private:
    KDL::Chain chain_;
    std::shared_ptr<KDL::ChainFkSolverPos_recursive> fk_solver_;
    std::shared_ptr<KDL::ChainJntToJacSolver> jac_solver_;
    int nj_;
};

#endif