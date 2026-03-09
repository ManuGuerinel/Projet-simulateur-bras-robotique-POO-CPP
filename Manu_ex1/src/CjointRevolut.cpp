#include "CjointRevolut.hpp"

Mat4 CJointRevolute::getTransform() const {
    Mat4 T = Mat4::Identity();
    double theta = getQ();
    T << cos(theta), -sin(theta), 0, dx_,
         sin(theta),  cos(theta), 0, 0,
                  0,           0, 1, 0,
                  0,           0, 0, 1;

    return T;
}

std::string CJointRevolute::getTypeName() const { 
    return "Revolute"; 
}

std::unique_ptr<CJoint> CJointRevolute::clone() const {
    return std::make_unique<CJointRevolute>(*this);
}

void CJointRevolute::setDx(double dx){
    dx_ = dx;
}

CJointRevolute::~CJointRevolute(){
}