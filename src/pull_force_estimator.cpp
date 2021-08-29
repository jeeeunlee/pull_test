#include "pull_test/pull_force_estimator.hpp"

#include <pull_test/pull_test_interface.hpp>
#include <pull_test/magneto/magneto_contact_container.hpp>
#include <pull_test/basic_contact_specs.hpp>
#include <pull_test/my_robot_system/robot_system.hpp>
#include <pull_test/my_geometry/polytope/polytope.h>
#include <pull_test/my_utils/math/math_utilities.hpp>

PullForceEstimator::PullForceEstimator(RobotSystem* robot,
                MagnetoContactContainer* contact_container) {
    my_utils::pretty_constructor(1, "PullForceEstimator");
    robot_ = robot;
    contact_container_ = contact_container;
    gravity_ << 0.0, 0.0, -9.81;
    mass_ = robot_->getRobotMass();  
    std::cout << " robot mass = " << mass_ << std::endl;
}

void PullForceEstimator::getPullForceThreshold(void* cmd){    
    // include new contact
    // A = [A1 A2 A3], where Ai = [[pi]xRi; Ri]
    // U = diag(U1 U2 U3)        
    // V = Vrep(U), Uav_ = Hrep(AV)

    _buildAstanceMatrix(); // update Astance_
    // my_utils::pretty_print(Astance_, std::cout, "Astance_");
    _buildUMatrix(); // update U_
    // my_utils::pretty_print(U_, std::cout, "U_");
    if(!_updateConvexHull()) // update Uav_(U_,Astance_), where Uav_=Hrep(AV) & V=vrep(U_)  
    {
        ((PullTestCommandData*)cmd)->pull_force_threshold = 0.0;
        std::cout << "no available double description method under the given condition" << std::endl;
        return;
    }  
    // my_utils::pretty_print(Uav_, std::cout, "Uav_");

    _updateGravityWrench(); // update Fg_
    // my_utils::pretty_print(Fg_, std::cout, "Fg_");

    // u1,u2,u3 = U_av*A
    int nc = contact_container_->contact_list_.size();    
    Eigen::MatrixXd UavA = Uav_*Astance_; //mx(3*nc)
    int rowsize = UavA.rows();
    Eigen::MatrixXd u_1 = UavA.block(0,0, rowsize, 3);
    Eigen::MatrixXd u_23 = UavA.block(0,3, rowsize, 3*(nc-1));
    Eigen::VectorXd fm_23 = Eigen::VectorXd::Zero(3*(nc-1));
    for(int i(1);i<nc;++i){
        // z direction max force
        fm_23(3*(i-1)+2) = ((BodyFramePointContactSpec*)contact_container_->contact_list_[i])->getMaxAdhesionForce();
    }
    Eigen::VectorXd rhs_vec = -Uav_*Fg_ - u_23*fm_23;
    // my_utils::pretty_print(u_23, std::cout, "u_23");
    // my_utils::pretty_print(fm_23, std::cout, "fm_23");
    // std::cout<<" findMinForcePullOnly : u1 fm1 >= rhs_vec " << std::endl;
    // my_utils::pretty_print(u_1, std::cout, "u_1");
    // my_utils::pretty_print(rhs_vec, std::cout, "rhs_vec");

    // check max
    double _eps = 1e-3;
    std::vector<double> fpull_min;
    std::vector<double> fpull_max;
    Eigen::VectorXd u_1_vec = u_1.col(2);
    for(int i(0); i<u_1_vec.size(); ++i){
        if(u_1_vec(i) > _eps){
            fpull_min.push_back( rhs_vec(i)/u_1_vec(i) );
        } else if(u_1_vec(i) < -_eps){
            fpull_max.push_back( rhs_vec(i)/u_1_vec(i) );
        }
    } // fpull_min < fpull < fpull_max
    // double min = *min_element(fpull_min.begin(), fpull_min.end());
    // std::cout <<"fpull_max size: " << fpull_max.size() << ", fpull_min size: "<<fpull_min.size()<<std::endl;
    double f_threshold = *min_element(fpull_max.begin(), fpull_max.end());
    // std::cout<<" min value: " << f_threshold << std::endl;
    
    ((PullTestCommandData*)cmd)->pull_force_threshold = f_threshold;
}

void PullForceEstimator::_updateGravityWrench() {
    // Fg_ = m*([pcom]xg; g)
    Eigen::Vector3d pcom = robot_->getCoMPosition();
    Fg_ = Eigen::VectorXd::Zero(6);
    Fg_.head(3) = mass_ * my_utils::skew3(pcom) * gravity_;
    Fg_.tail(3) = mass_ * gravity_;
}


void PullForceEstimator::_buildAstanceMatrix(){
    int n_contact =  contact_container_->contact_list_.size();
    Astance_ = Eigen::MatrixXd::Zero(6, 3*n_contact);
    for(int i(0); i<n_contact; ++i) { 
        Astance_.block(0,3*i,6,3) = ((BodyFramePointContactSpec*)contact_container_
                ->contact_list_[i])->getConfigurationMtx();  
    }    
}

void PullForceEstimator::_buildUMatrix(){
    if(contact_container_->contact_list_.size() < 1)
        return;
    
    Eigen::MatrixXd U_i;
    int dim_rows, dim_cols, dim_rows_i, dim_cols_i;
    U_ = - contact_container_->contact_list_[0]->getRFConstraintMtx();
    dim_rows = U_.rows();
    dim_cols = U_.cols(); 
    for(uint i(1); i<contact_container_->contact_list_.size(); ++i) {
        U_i = - contact_container_->contact_list_[i]->getRFConstraintMtx();
        dim_rows_i = U_i.rows();
        dim_cols_i = U_i.cols();
        U_.conservativeResize(dim_rows + dim_rows_i, dim_cols + dim_cols_i);
        U_.block(0, dim_cols, dim_rows, dim_cols_i).setZero();
        U_.block(dim_rows, 0, dim_rows_i, dim_cols).setZero();
        U_.block(dim_rows, dim_cols, dim_rows_i, dim_cols_i) = U_i;
        dim_rows += dim_rows_i;
        dim_cols += dim_cols_i;
    }
}
 

bool PullForceEstimator::_updateConvexHull() {
    // check empty
    if(U_.rows() == 0 || U_.cols() == 0)
        return false;

    // double description convex hull
    Polyhedron poly_1, poly_2;
    // 1. face(Uf_)->span(Uf_S=Vf_)
    if( !poly_1.setHrep(U_, Eigen::VectorXd::Zero(U_.rows())) )
        return false;

    Eigen::MatrixXd V = (poly_1.vrep().first).transpose();    

    // 2. span(A_stance * Vf_)->face(U_st)
    // poly.vrep() return VrepXd:(first:Matrix V, second:Vector index(0:Ray, 1:vertex)) 
    Eigen::MatrixXd Vst = Astance_ * V;
    // my_utils::pretty_print(U_, std::cout, "U_");
    // my_utils::pretty_print(V, std::cout, "V");
    // my_utils::pretty_print(Astance_, std::cout, "Astance_");
    // my_utils::pretty_print(Vst, std::cout, "Vst");
    if (!poly_2.setVrep( Vst.transpose(), Eigen::VectorXd::Zero( Vst.cols()) ))
        return false;
    Uav_ = poly_2.hrep().first;
}