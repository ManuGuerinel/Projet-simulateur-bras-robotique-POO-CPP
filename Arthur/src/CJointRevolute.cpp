#include "CJointRevolute.hpp"

//======CLASSE REVOLUTE======================================
//===========================================================
Mat4 CJointRevolute::getTransform() const {
    Mat4 T;
    double Co = std::cos(q_);
    double Si = std::sin(q_);

    T << Co, -Si, 0, dx_,
         Si,  Co, 0,  0,
          0,  0,  1,  0,
          0,  0,  0,  1;
          
    return T;
}

std::string CJointRevolute::getTypeName() const {
    return "Revolute";
}

std::unique_ptr<CJoint> CJointRevolute::clone() const {
    return std::make_unique<CJointRevolute>(*this); // make_unique utilise un constructeur de copie par défaut dans le compilateur 
}
