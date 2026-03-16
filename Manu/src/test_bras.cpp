#include <pinocchio/multibody/fwd.hpp>
#include <pinocchio/spatial/se3.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/algorithm/kinematics.hpp>

#include <iostream>

#include "CJoint.hpp"
#include "CjointPrismatic.hpp"
#include "CjointRevolut.hpp"
#include "Cbras.hpp"

using Mat4 = Eigen::Matrix4d;

int main(){
    CBras bras;
    bras.addJoint(std::make_unique<CJointRevolute>(-M_PI, M_PI, 0.0, 0.5));
    bras.addJoint(std::make_unique<CJointRevolute>(-M_PI, M_PI, 0.3, 0.3));
    bras.addJoint(std::make_unique<CJointPrismatic>(0.0, 0.5, 0.2));
    std::cout << bras;

    CBras bras2;
    Mat4 T_test = bras2.computeFK();
    std::cout << "Bras vide :\n" << T_test << std::endl;
    bras2.addJoint(std::make_unique<CJointRevolute>(-M_PI, M_PI, 0.0, 0.5));
    T_test = bras2.computeFK();
    std::cout << "Bras avec 1 joint :\n" << T_test << std::endl;
    std::cout << "Vecteur translation :\n" << T_test.block<3,1>(0,3) << std::endl;
    bras2.getJoint(1);

    return 0;
}
// pourquoi A*B avec eigen et pas boucle triple : plus rapide(optimisation(vectorisation, optimisation cache)), plus lisible

