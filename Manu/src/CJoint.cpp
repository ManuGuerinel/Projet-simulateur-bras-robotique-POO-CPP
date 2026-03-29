#include "CJoint.hpp"

//======== [ constructeur ]===============
CJoint::CJoint(){
    q_ = 0;
    qMax_ = 100000;
    qMin_ = -100000;
}

CJoint::CJoint(double qMin, double qMax, double q){
    q_ = q;
    qMax_ = qMax;
    qMin_ = qMin;
}
//======== [ getters ] ===================
double CJoint::getQ() const{
    return q_;
}

double CJoint::getQMin() const{
    return qMin_;
}

double CJoint::getQMax() const{
    return qMax_;
}

//======== [ setters ] ===================
void CJoint::setQ(double q){
    if(q <= qMax_ && q >= qMin_){
        q_ = q;
    }else{
        throw std::out_of_range("q > qmax ou q < qmin");
    }
}

void CJoint::setQMin(double qmin){
    qMin_ = qmin;
}

void CJoint::setQMax(double qmax){
    qMax_ = qmax;
}
