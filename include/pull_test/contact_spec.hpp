#pragma once

#include <stdio.h>
#include <Eigen/Dense>
#include <iostream>

#include <pull_test/my_robot_system/robot_system.hpp>
#include <pull_test/my_utils/io/io_utilities.hpp>

class ContactSpec {
   public:
    ContactSpec(RobotSystem* _robot, const int& _dim) {
        robot_ = _robot;
        dim_contact_ = _dim;
        b_set_contact_ = false;        
        MatA_ = Eigen::MatrixXd::Zero(6, dim_contact_);
    }
    virtual ~ContactSpec() {}
    
    int getDimRFConstratint() { return Uf_.rows(); }
    Eigen::MatrixXd getRFConstraintMtx() { return Uf_; }
    Eigen::MatrixXd getConfigurationMtx() { return MatA_; }
    int getDim() { return dim_contact_; }
    void unsetContact() { b_set_contact_ = false; }
    virtual bool updateContactSpec() {
        _UpdateMatA();
        _UpdateUf();
        b_set_contact_ = true;
        return true;
    }
    

   protected:    
        virtual bool _UpdateMatA() = 0;
        virtual bool _UpdateUf() = 0;

        RobotSystem* robot_;
        
        int dim_contact_;
        bool b_set_contact_;

        Eigen::MatrixXd MatA_;
        Eigen::MatrixXd Uf_;
};
