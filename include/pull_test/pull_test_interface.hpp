#pragma once

#include <iostream>
#include <stdio.h>
#include <map>
#include <Eigen/Dense>

#include <pull_test/magneto/magneto_definition.hpp>

class PullForceEstimator;
class RobotSystem;
class MagnetoContactContainer;

class PullTestSensorData {
    public:
        PullTestSensorData( const Eigen::MatrixXd& orientation,
                            double f_pull = 30., double mu = 0.5) {
            q = Eigen::VectorXd::Zero(Magneto::n_adof);
            qdot = Eigen::VectorXd::Zero(Magneto::n_adof);
            virtual_q = Eigen::VectorXd::Zero(Magneto::n_vdof);
            virtual_qdot = Eigen::VectorXd::Zero(Magneto::n_vdof);

            new_contact = -1; //MagnetoFoot::AL;
            next_moving_foot = -1; //MagnetoFoot::AL;

            // default values for initial settings
            f_pull_specs = { {MagnetoFoot::AL, f_pull},
                              {MagnetoFoot::AR, f_pull},
                              {MagnetoFoot::BL, f_pull},
                              {MagnetoFoot::BR, f_pull} };

            friction_specs = {{MagnetoFoot::AL, mu},
                              {MagnetoFoot::AR, mu},
                              {MagnetoFoot::BL, mu},
                              {MagnetoFoot::BR, mu} };

            orientation_specs = {{MagnetoFoot::AL, orientation},
                              {MagnetoFoot::AR, orientation},
                              {MagnetoFoot::BL, orientation},
                              {MagnetoFoot::BR, orientation}};

            base_orientation = orientation;       
                              

        }
        ~PullTestSensorData() {}

        Eigen::VectorXd q;
        Eigen::VectorXd qdot;
        Eigen::VectorXd virtual_q;
        Eigen::VectorXd virtual_qdot;

        int new_contact;
        int next_moving_foot;

        std::map<int, double> f_pull_specs;
        std::map<int, double> friction_specs;

        std::map<int, Eigen::MatrixXd> orientation_specs;
        Eigen::MatrixXd base_orientation;

};

class PullTestCommandData {
    public:
        PullTestCommandData() {
            pull_force_threshold = 0.;
        }
        ~PullTestCommandData() {}

        double pull_force_threshold;
};

class PullTestInterface {
    public:
        PullTestInterface();
        ~PullTestInterface();        

        void getCommand(void* _sensor_data, void* _command_data);
    
    private:
        void updateContactSpec(PullTestSensorData* data);
        void updateRobotSystem(PullTestSensorData* data);


        PullForceEstimator* pull_force_estimator_;
        RobotSystem* robot_;
        MagnetoContactContainer* contact_container_;

        Eigen::VectorXd curr_config_;
        Eigen::VectorXd curr_qdot_;

};