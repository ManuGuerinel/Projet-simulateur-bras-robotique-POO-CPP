#include <pinocchio/multibody/fwd.hpp>
#include <pinocchio/spatial/se3.hpp>
#include <iostream>

#include "CJoint.hpp"
#include "CjointPrismatic.hpp"
#include "CjointRevolut.hpp"

double const PI = 3.14159265;

int main() {
    double theta = PI/4;
    double dx = 0.5;

    CJoint* pJoint;
    CJointRevolute Q1(theta,dx);
    pJoint = &Q1;

    Mat4 T = pJoint->getTransform();
    std::cout << "getTransform : " << std::endl << T << std::endl;

    // Transform de reference avec Pinocchio
    // SE3 constructor takes a 3x3 rotation (Matrix or Quaternion) and a translation vector.
    Eigen::Matrix3d R = Eigen::AngleAxisd(theta, Eigen::Vector3d::UnitZ()).toRotationMatrix();
    pinocchio::SE3 ref(R, Eigen::Vector3d(dx,0,0));

    Mat4 T_ref = ref.toHomogeneousMatrix();
    std::cout << "Pinocchio : " << std::endl << T_ref << std::endl;

    //test unitaire
    Q1.setQ(PI/2);
    std::cout << "q = " << Q1.getQ() << std::endl;
    // Q1.setQMin(0);
    // Q1.setQ(-PI/2);
    // std::cout << "q = " << Q1.getQ() << std::endl;
    Q1.setQ(0);
    Q1.setDx(0);
    T = Q1.getTransform();
    std::cout << "getTransform (theta = 0 et dx = 0) : " << std::endl << T << std::endl;

    CJointPrismatic Q2(1);
    T = Q2.getTransform();
    std::cout << "getTransform (d = 1) : " << std::endl << T << std::endl;
}