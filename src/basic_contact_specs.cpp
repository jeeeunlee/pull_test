#include <pull_test/basic_contact_specs.hpp>
#include <pull_test/my_utils/math/math_utilities.hpp>


BodyFramePointContactSpec::BodyFramePointContactSpec(
                            RobotSystem* robot, 
                            int _link_idx): ContactSpec(robot, 3) {
    my_utils::pretty_constructor(3, "Body Frame Point Contact Spec");

    contact_orientation_ = Eigen::MatrixXd::Identity(3,3);
    f_adhesion_max_ = 0.0;
    link_idx_ = _link_idx;
    setFrictionCoeff(0.0);
    updateContactSpec();
}

BodyFramePointContactSpec::~BodyFramePointContactSpec() {}

bool BodyFramePointContactSpec::_UpdateUf() {  
    // Uf isn't changing under the fixed friction coeff  
    return true;
}

bool BodyFramePointContactSpec::_UpdateMatA() {
    MatA_ = Eigen::MatrixXd::Zero(6, dim_contact_);
    // Eigen::MatrixXd R_wb = robot_->getBodyNodeIsometry(link_idx_).linear();
    Eigen::MatrixXd R_wb = contact_orientation_;
    Eigen::VectorXd p_wb = robot_->getBodyNodeIsometry(link_idx_).translation();
    
    Eigen::MatrixXd p_skew = my_utils::skew3(p_wb);
            
    MatA_.block(0,0,3,3) = p_skew*R_wb;
    MatA_.block(3,0,3,3) = R_wb; 
    return true;
}

void BodyFramePointContactSpec::setFrictionCoeff(double _mu) { 
    mu_ = _mu/sqrt(2.0); 
    setUf(mu_); 
}

void BodyFramePointContactSpec::setUf(double mu) {
    Uf_ = Eigen::MatrixXd::Zero(5, dim_contact_);
    // Linear
    Uf_(0, 2) = 1.;  // Fz >= 0

    Uf_(1, 0) = 1.0;
    Uf_(1, 2) = mu;
    Uf_(2, 0) = -1.0;
    Uf_(2, 2) = mu;

    Uf_(3, 1) = 1.0;
    Uf_(3, 2) = mu;
    Uf_(4, 1) = -1.0;
    Uf_(4, 2) = mu;
}

