
#include <math.h>
#include <string>
#include <stdio.h>
#include <pull_test/pull_test_interface.hpp>
#include <pull_test/pull_force_estimator.hpp>

#include <pull_test/magneto/magneto_contact_container.hpp>

#include <pull_test/my_robot_system/robot_system.hpp>
#include <pull_test/my_utils/io/io_utilities.hpp>
#include <pull_test/my_utils/math/math_utilities.hpp>

PullTestInterface::PullTestInterface() {
    // set robotsystem
    robot_ = new RobotSystem(
        6+3*4, THIS_COM "robot_description/Magneto.urdf");
    robot_->setActuatedJoint(Magneto::idx_adof);

    // set contact spec container (contact/adhesion information)
    contact_container_ = new MagnetoContactContainer(robot_);

    // set pull force estimator
    pull_force_estimator_ = new PullForceEstimator(robot_, contact_container_);
}

void PullTestInterface::getCommand(void* _sensor_data, void* _command_data) {
    PullTestSensorData* data = ((PullTestSensorData*)_sensor_data);
    PullTestCommandData* cmd = ((PullTestCommandData*)_command_data);

    // update robot_
    updateRobotSystem(data);    

    // update contact_container_ (maxpullforce, mu, new/next contact)  
    updateContactSpec(data);

    // compute pull force
    pull_force_estimator_->getPullForceThreshold(cmd);
}

void PullTestInterface::updateContactSpec(PullTestSensorData* data){

    contact_container_->updateContactSpec(MagnetoFoot::AL, 
                        data->f_pull_specs[MagnetoFoot::AL], 
                        data->friction_specs[MagnetoFoot::AL]);
    contact_container_->updateContactSpec(MagnetoFoot::AR, 
                        data->f_pull_specs[MagnetoFoot::AR], 
                        data->friction_specs[MagnetoFoot::AR]);
    contact_container_->updateContactSpec(MagnetoFoot::BL, 
                        data->f_pull_specs[MagnetoFoot::BL], 
                        data->friction_specs[MagnetoFoot::BL]);
    contact_container_->updateContactSpec(MagnetoFoot::BR, 
                        data->f_pull_specs[MagnetoFoot::BR], 
                        data->friction_specs[MagnetoFoot::BR]);

    int new_contact = data->new_contact;
    int next_moving_foot = data->next_moving_foot;

    contact_container_->updateContactList(new_contact, next_moving_foot);
}

void PullTestInterface::updateRobotSystem(PullTestSensorData* data) {
    curr_config_.setZero();
    curr_qdot_.setZero();

    for (int i = 0; i < Magneto::n_vdof; ++i) {
        curr_config_[Magneto::idx_vdof[i]] = data->virtual_q[i];
        curr_qdot_[Magneto::idx_vdof[i]] = data->virtual_qdot[i];
    }
    for (int i = 0; i < Magneto::n_adof; ++i) {
        curr_config_[Magneto::idx_adof[i]] = data->q[i];
        curr_qdot_[Magneto::idx_adof[i]] = data->qdot[i];
    }

    robot_->updateSystem(curr_config_, curr_qdot_, false);
}

