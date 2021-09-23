

#include <pull_test/pull_test_interface.hpp>
#include <map>
#include <string>

int main(int argc, char** argv) {  
    /* interface constructors */
    // default contact spec
    double fpull = -50.; // adhesion force at each foot 
    double mu = 0.5; // static friction coefficient
    double r(0.0), p(M_PI_2), y(0.0); // base&foot orientation [rad]
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
    double coxa_joint_init = 0.088;
    double femur_joint_init = 0.0*M_PI_2; // -1./10.*M_PI_2;
    double tibia_joint_init = -1.49; //1.0*M_PI_2; // -9./10.*M_PI_2;
    sensor_data_-> q << -coxa_joint_init, femur_joint_init, tibia_joint_init,
                        coxa_joint_init, femur_joint_init, tibia_joint_init, 
                        coxa_joint_init, femur_joint_init, tibia_joint_init, 
                        -coxa_joint_init, femur_joint_init, tibia_joint_init;
    sensor_data_->q[MagnetoADoF::AL_coxa_joint] = 0.1;

    sensor_data_-> virtual_q = Eigen::VectorXd::Zero(Magneto::n_vdof);
    sensor_data_-> virtual_q[3] = y;
    sensor_data_-> virtual_q[4] = p;
    sensor_data_-> virtual_q[5] = r;

    std::map<int, std::string> foot_name = {{0, "AL"}, {1, "AR"}, {2,"BL"}, {3,"BR"}};

    // contact planning
    


    while(1){  
      int num;

      // double coxa, femur, tibia;
      // std::cout << "al joint position : ";
      // num = std::scanf("%lf %lf %lf", &coxa, &femur, &tibia);
      // std::cout <<  "entered input: " << num << " val = " << 
      //         coxa <<", "<< femur <<", "<< tibia << std::endl;
      // sensor_data_->q[MagnetoADoF::AL_coxa_joint] = coxa;
      // sensor_data_->q[MagnetoADoF::AL_femur_joint] = femur;
      // sensor_data_->q[MagnetoADoF::AL_tibia_joint] = tibia;

      double coxa;
      std::cout << "coxa joint position : ";
      num = std::scanf("%lf", &coxa);
      sensor_data_->q[MagnetoADoF::AL_coxa_joint] = -coxa;
      sensor_data_->q[MagnetoADoF::AR_coxa_joint] = coxa;
      sensor_data_->q[MagnetoADoF::BL_coxa_joint] = coxa;
      sensor_data_->q[MagnetoADoF::BR_coxa_joint] = -coxa;

      // interface_->printRobotConfiguration(sensor_data_);


      int next_contact_foot;
      std::cout << " next contact foot : (AL:0, AR:1, BL:2, BR:3)";
      num = std::scanf("%d",&next_contact_foot);
      sensor_data_-> new_contact = next_contact_foot;

      // get friction
      double al_val(0.0), bl_val(0.0), ar_val(0.0), br_val(0.0);
      std::cout << "friction at al, bl, ar, br : ";
      num = std::scanf("%lf %lf %lf %lf",&al_val, &bl_val, &ar_val, &br_val);
      std::cout << "entered input: " << num << " val = " << 
              al_val <<", "<< bl_val <<", "<< ar_val <<", "<< br_val << std::endl;
      sensor_data_-> friction_specs[MagnetoFoot::AL] = al_val;
      sensor_data_-> friction_specs[MagnetoFoot::BL] = bl_val;
      sensor_data_-> friction_specs[MagnetoFoot::AR] = ar_val;
      sensor_data_-> friction_specs[MagnetoFoot::BR] = br_val;

      // get pull force      
      std::cout << "force at al, bl, ar, br : ";
      num = std::scanf("%lf %lf %lf %lf",&al_val, &bl_val, &ar_val, &br_val);
      std::cout << "entered input: " << num << " val = " << 
              al_val <<", "<< bl_val <<", "<< ar_val <<", "<< br_val << std::endl;
      sensor_data_-> f_pull_specs[MagnetoFoot::AL] = al_val;
      sensor_data_-> f_pull_specs[MagnetoFoot::BL] = bl_val;
      sensor_data_-> f_pull_specs[MagnetoFoot::AR] = ar_val;
      sensor_data_-> f_pull_specs[MagnetoFoot::BR] = br_val;    

      /* compute pull force threshold  */
      if(next_contact_foot != MagnetoFoot::AL) {
        sensor_data_-> next_moving_foot = MagnetoFoot::AL;
        std::cout<< "When we want to move [AL] next, minimum pull force required at new contact [" << foot_name[next_contact_foot] << "] : ";
        interface_->getCommand(sensor_data_, command_); //,true
        std::cout<< command_->pull_force_threshold << std::endl;
        double f_AL = command_->pull_force_threshold;
      }
      if(next_contact_foot != MagnetoFoot::AR) {
        sensor_data_-> next_moving_foot = MagnetoFoot::AR;
        std::cout<< "When we want to move [AR] next, minimum pull force required at new contact [" << foot_name[next_contact_foot] << "] : ";
        interface_->getCommand(sensor_data_, command_); //,true
        std::cout<< command_->pull_force_threshold << std::endl;
        double f_AR = command_->pull_force_threshold;
      }
      // etc.
      if(next_contact_foot != MagnetoFoot::BR) {
        sensor_data_-> next_moving_foot = MagnetoFoot::BR;
        std::cout<< "When we want to move [BR] next, minimum pull force required at new contact [" << foot_name[next_contact_foot] << "] : ";
        interface_->getCommand(sensor_data_, command_);
        std::cout<<command_->pull_force_threshold << std::endl;
        double f_BR = command_->pull_force_threshold;
      }
      if(next_contact_foot != MagnetoFoot::BL) {
        sensor_data_-> next_moving_foot = MagnetoFoot::BL;
        std::cout<< "When we want to move [BL] next, minimum pull force required at new contact [" << foot_name[next_contact_foot] << "] : ";
        interface_->getCommand(sensor_data_, command_);
        std::cout<<command_->pull_force_threshold << std::endl;
        double f_BL = command_->pull_force_threshold;
      }

      // std::cout << "When we are not sure which foot to move next, minimum pull force at BL is" <<
      // std::min({f_AL, f_AR, f_BR}) << std::endl;
    }
}