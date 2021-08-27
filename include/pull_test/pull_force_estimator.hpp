#pragma once

class RobotSystem;
class ContactSpec;
class MagnetoContactContainer;
#include <Eigen/Dense>
#include <pull_test/contact_spec.hpp>


// case1: friction coeff for new contact is known
class PullForceEstimator {
    public:
        PullForceEstimator(RobotSystem* robot,
                    MagnetoContactContainer* contact_container);
        ~PullForceEstimator() {}

        void getPullForceThreshold(void* cmd);


    private:
        void _updateGravityWrench();
        void _updateConvexHull();
        void _buildAstanceMatrix();
        void _buildUMatrix();

        RobotSystem* robot_;
        MagnetoContactContainer* contact_container_; // fixed
        std::vector<ContactSpec*> contact_list_; // computation purpose
        Eigen::Vector3d gravity_;
        double mass_;

        Eigen::Vector6d Fg_;
        Eigen::MatrixXd U_;
        Eigen::MatrixXd Uav_;
        Eigen::MatrixXd Anew_;
        Eigen::MatrixXd Astance_;

        int new_contact_idx_;
        std::vector<int> next_step_idices_; // possible next footstep except new contact
        std::vector<int> link_idx_; // U2 ~


};
