

#include <pull_test/pull_test_interface.hpp>

int main(int argc, char** argv) {  
    /* interface constructors */
    // default contact spec
    double fpull = -50.; // adhesion force at each foot 
    double mu = 0.5; // static friction coefficient
    double r(0.0), p(1.0), y(0.0); // base&foot orientation [rad]
    Eigen::Quaternion<double> q =
                Eigen::AngleAxisd(r, Eigen::Vector3d::UnitX())
                * Eigen::AngleAxisd(p, Eigen::Vector3d::UnitY())
                * Eigen::AngleAxisd(y, Eigen::Vector3d::UnitZ());
    Eigen::MatrixXd foot_ori = q.matrix();
    PullTestSensorData* sensor_data_ = new PullTestSensorData(foot_ori, fpull, mu);
    PullTestCommandData* command_ = new PullTestCommandData();
    PullTestInterface* interface_ = new PullTestInterface();

    /* required information */
    // robot configuration
    sensor_data_-> q = Eigen::VectorXd::Zero(Magneto::n_adof);
    double femur_joint_init = 1./10.*M_PI_2; // -1./10.*M_PI_2;
    double tibia_joint_init = -11./10.*M_PI_2; // -9./10.*M_PI_2;
    sensor_data_-> q << 0.0, femur_joint_init, tibia_joint_init,
                        0.0, femur_joint_init, tibia_joint_init, 
                        0.0, femur_joint_init, tibia_joint_init, 
                        0.0, femur_joint_init, tibia_joint_init;
    sensor_data_-> virtual_q = Eigen::VectorXd::Zero(Magneto::n_vdof);
    sensor_data_-> virtual_q[3] = y;
    sensor_data_-> virtual_q[4] = p;
    sensor_data_-> virtual_q[5] = r;

    // known contact spec for existent contacts
    sensor_data_-> f_pull_specs[MagnetoFoot::AL] = -50.;
    sensor_data_-> friction_specs[MagnetoFoot::AL] = 0.3;
    // contact planning
    sensor_data_-> new_contact = MagnetoFoot::AL;
    sensor_data_-> next_moving_foot = MagnetoFoot::AR;

    /* compute pull force threshold  */
    interface_->getCommand(sensor_data_, command_);
    std::cout<< "When we want to move [AR] next, minimum pull force required at new contact [AL] is" <<
      command_->pull_force_threshold << std::endl;
    double f_AR = command_->pull_force_threshold;

    // etc.
    sensor_data_-> next_moving_foot = MagnetoFoot::BR;
    interface_->getCommand(sensor_data_, command_);
    std::cout<< "When we want to move [BR] next, minimum pull force required at new contact [AL] is" <<
      command_->pull_force_threshold << std::endl;
    double f_BR = command_->pull_force_threshold;

    sensor_data_-> next_moving_foot = MagnetoFoot::BL;
    interface_->getCommand(sensor_data_, command_);
    std::cout<< "When we want to move [BL] next, minimum pull force required at new contact [AL] is" <<
      command_->pull_force_threshold << std::endl;
    double f_BL = command_->pull_force_threshold;

    std::cout << "When we are not sure which foot to move next, minimum pull force at AL is" <<
    std::max({f_AR, f_BR, f_BL}) << std::endl;
}