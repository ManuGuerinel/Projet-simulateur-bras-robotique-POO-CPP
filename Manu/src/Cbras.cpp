#include "../include/Cbras.hpp"
#include <Eigen/Dense>
#include <vector>
#include <memory>
#include "../include/CJoint.hpp"

using Mat4 = Eigen::Matrix4d;

// //=========== [ constructeur ] =============
// CBras::CBras(const CBras& other){
//     // constructeur de copie, dynamique
//     for(int i=0; i<getNbJoints(); i++){
//         addJoint(other.getJoint(i)->clone());
//     }
//     return other;
// }

// CBras::CBras(CBras&& other) noexcept {
//     // constructeur de deplacement
//     joints_ = std::move(other.joints_);
// }

//============ [ methode ] =================
void CBras::addJoint(std::unique_ptr<CJoint> joint){
    joints_.push_back(std::move(joint));    //move car pointeur intelligent ne sont pas copiable par defaut
}

Mat4 CBras::computeFK() {
    Mat4 T = Mat4::Identity();
    for(int i=0; i < this->getNbJoints(); i++){
        T = T * this->getJoint(i).getTransform();
    }
    return T;
}

//============= [ getters ] =================
int CBras::getNbJoints() const {
    return joints_.size();
}

CJoint& CBras::getJoint(int i) const {
    if(i >= this->getNbJoints()){
        char msg[50];
        sprintf(msg, "Index joint invalide (%d > %d)", i, this->getNbJoints()-1);
        throw std::out_of_range(msg);
    }
    return *joints_[i];
}

//============ [ surcharge operateur ]========

std::ostream& operator<<(std::ostream& os, const CBras& bras)
{
    os << "Bras [" << bras.getNbJoints() << " DDL]\n";

    for(int i=0;i<bras.getNbJoints();i++)
    {
        CJoint& j = bras.getJoint(i);

        os << "[" << i << "] "
           << j.getTypeName();

        if(j.getTypeName() == "Revolute")
            os << "  q = " << j.getQ()
               << " rad ";
        else
            os << " q = " << j.getQ()
               << " m ";

        os << "bornes=["
           << j.getQMin() << ", "
           << j.getQMax() << "]\n";
    }

    return os;
}

// CBras& CBras::operator=(CBras&& other) noexcept{
//     // operateur de deplacement
//     if(this != &other)
//     {
//         joints_ = std::move(other.joints_);
//     }
//     return *this;
// }