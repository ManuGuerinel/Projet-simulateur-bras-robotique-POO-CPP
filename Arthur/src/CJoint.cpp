#include "CJoint.hpp"

//Constructeur================
CJoint::CJoint(double q_, double qMin_, double qMax_) {
    this->q_ = q_;
    this->qMin_ = qMin_;
    this->qMax_ = qMax_;
}

//Getters=====================
double CJoint::getQ() const {
    return q_;
}

double CJoint::getQMin() const {
    return qMin_;
}

double CJoint::getQMax() const {
    return qMax_;
}


//Setters=====================
void CJoint::setQ(double q) {
    if ((q<qMin_)||(q>qMax_)) {
        throw std::out_of_range("q hors limites!");
    }
    q_ = q;
}
