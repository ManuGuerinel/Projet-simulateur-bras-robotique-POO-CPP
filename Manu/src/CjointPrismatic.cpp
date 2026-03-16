#include "CjointPrismatic.hpp"

Mat4 CJointPrismatic::getTransform() const {
    Mat4 T = Mat4::Identity();
    double d = getQ();
    T << 1, 0, 0, 0,
         0, 1, 0, 0,
         0, 0, 1, d,
         0, 0, 0, 1;

    return T;
}

std::string CJointPrismatic::getTypeName() const { 
    return "Prismatic"; 
}

std::unique_ptr<CJoint> CJointPrismatic::clone() const {
    return std::make_unique<CJointPrismatic>(*this);
}

CJointPrismatic::~CJointPrismatic(){
}