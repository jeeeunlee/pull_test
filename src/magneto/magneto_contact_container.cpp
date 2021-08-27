#include <pull_test/magneto/magneto_contact_container.hpp>
#include <pull_test/magneto/magneto_definition.hpp>
#include <pull_test/basic_contact_specs.hpp>

MagnetoContactContainer::MagnetoContactContainer(RobotSystem* robot)
{
    alfoot_contact_ = new BodyFramePointContactSpec(
        robot, MagnetoBodyNode::AL_foot_link); 
    blfoot_contact_ = new BodyFramePointContactSpec(
        robot, MagnetoBodyNode::BL_foot_link);     
    arfoot_contact_ = new BodyFramePointContactSpec(
        robot, MagnetoBodyNode::AR_foot_link); 
    brfoot_contact_ = new BodyFramePointContactSpec(
        robot, MagnetoBodyNode::BR_foot_link);

    foot_list_ = {MagnetoBodyNode::AL_foot_link, 
                MagnetoBodyNode::BL_foot_link,
                MagnetoBodyNode::AR_foot_link,
                MagnetoBodyNode::BR_foot_link};
}

void MagnetoContactContainer::updateContactList(int new_contact, int next_moving_contact){
    contact_list_.clear();
    // add new contact first    
    contact_list_.push_back(getContact(new_contact));

    // add the rest of contact
    for(auto &foot : foot_list_){
        if(foot != new_contact && foot != next_moving_contact)
            contact_list_.push_back(getContact(foot));
    }
}

void MagnetoContactContainer::updateContactSpec(int foot,
                                const double& _fmax,
                                const double& _mu,
                                const Eigen::MatrixXd& _ori) {
    auto contact = getContact(foot);
    ((BodyFramePointContactSpec*)contact)->setMaxAdhesionForce(_fmax);
    ((BodyFramePointContactSpec*)contact)->setFrictionCoeff(_mu);    
    ((BodyFramePointContactSpec*)contact)->setContactOrientation(_ori);

    contact->updateContactSpec();
}


ContactSpec* MagnetoContactContainer::getContact(int foot){
    switch(foot){
    case MagnetoFoot::AL:
        return alfoot_contact_;
    case MagnetoFoot::AR:
        return arfoot_contact_;
    case MagnetoFoot::BL:
        return blfoot_contact_;
    case MagnetoFoot::BR:
        return brfoot_contact_;
    default:
        return nullptr;
    }
}


