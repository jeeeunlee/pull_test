

#include <pull_test/pull_test_interface.hpp>

int main(int argc, char** argv) {  
    /* interface constructors */
    // default contact spec
    double fpull = 30.; // adhesion force at each foot 
    double mu = 0.5; // static friction coefficient
    double r(0.0), p(0.0), y(0.0); // base orientation
    Eigen::Quaternion<double> q =
                Eigen::AngleAxisd(r*M_PI, Eigen::Vector3d::UnitX())
                * Eigen::AngleAxisd(p*M_PI, Eigen::Vector3d::UnitY())
                * Eigen::AngleAxisd(y*M_PI, Eigen::Vector3d::UnitZ());
    Eigen::MatrixXd base_ori = q.matrix();
    PullTestSensorData* sensor_data_ = new PullTestSensorData(base_ori, fpull, mu);
    PullTestCommandData* command_ = new PullTestCommandData();
    PullTestInterface* interface_ = new PullTestInterface();

    /* required information */
    // robot configuration
    sensor_data_-> q = Eigen::VectorXd::Zero(Magneto::n_adof);
    // sensor_data_-> virtual_q = Eigen::VectorXd::Zero(Magneto::n_vdof);
    // known contact spec for existent contacts
    sensor_data_-> f_pull_specs[MagnetoFoot::BR] = 50.;
    sensor_data_-> friction_specs[MagnetoFoot::BR] = 0.3;
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