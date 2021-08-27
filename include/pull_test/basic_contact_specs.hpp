#pragma once

#include "contact_spec.hpp"

class RobotSystem;

class BodyFramePointContactSpec : public ContactSpec {
   public:
    BodyFramePointContactSpec(RobotSystem* robot, int _link_idx);
    virtual ~BodyFramePointContactSpec();

    int getLinkIdx() { return link_idx_; }
    
    void setFrictionCoeff(double _mu);
    void setUf(double mu);

    void setContactOrientation(const Eigen::MatrixXd& ori) {contact_orientation_ = ori;}
    Eigen::MatrixXd getContactOrientation() { return contact_orientation_; }

    void setMaxAdhesionForce(double _force){ f_adhesion_max_=_force; }
    double getMaxAdhesionForce(){return f_adhesion_max_;}

   protected:
    double mu_;
    int link_idx_;
    double f_adhesion_max_;
    Eigen::MatrixXd contact_orientation_;


    virtual bool _UpdateMatA();
    virtual bool _UpdateUf();

    

};
