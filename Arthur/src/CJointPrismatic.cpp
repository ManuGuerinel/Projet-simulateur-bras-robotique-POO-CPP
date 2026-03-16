#include "CJointPrismatic.hpp"


//=====CLASSE PRISMATIC======================================
//===========================================================
Mat4 CJointPrismatic::getTransform() const {
    Mat4 T = Mat4::Identity(); //on fait une Id 4*4 et on ajoute le d 
    T(2,3) = q_;

    return T;
}

std::string CJointPrismatic::getTypeName() const {
    return "Prismatic";
}

std::unique_ptr<CJoint> CJointPrismatic::clone() const {
    return std::make_unique<CJointPrismatic>(*this); 
}
