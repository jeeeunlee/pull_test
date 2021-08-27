#pragma once

#include <pull_test/contact_spec.hpp>

class RobotSystem;


class MagnetoContactContainer{
    public:
        MagnetoContactContainer(RobotSystem* robot);
        ~MagnetoContactContainer() {}
        
        void updateContactList(int new_contact, int next_moving_contact);
        void updateContactSpec(int foot,
                                const double& _fmax,
                                const double& _mu);
        ContactSpec* getContact(int foot);

        std::vector <ContactSpec*> contact_list_;
        std::vector <int> foot_list_;

    private:
        ContactSpec* alfoot_contact_;
        ContactSpec* arfoot_contact_;
        ContactSpec* blfoot_contact_;
        ContactSpec* brfoot_contact_;

};