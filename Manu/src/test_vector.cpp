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

    bras.setQ(bras.getQ());
    std::cout << bras;

    CBras bras2 = bras;
    CBras bras3(bras);
    std::cout << bras2;
    std::cout << bras3;

    bras3.randomQ();
    std::cout << bras3;

    return 0;
}
