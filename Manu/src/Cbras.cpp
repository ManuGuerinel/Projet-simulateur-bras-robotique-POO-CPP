#include "../include/Cbras.hpp"
#include <Eigen/Dense>
#include <vector>
#include <memory>
#include <random>
#include "../include/CJoint.hpp"

using Mat4 = Eigen::Matrix4d;

//=========== [ constructeur ] =============
CBras::CBras(const CBras& other){
    // constructeur de copie, dynamique
    for(int i=0; i<other.getNbJoints(); i++){
        addJoint(other.getJoint(i).clone());        // clone permet de copier le bon type dynamiquement, il est fait sur chaque joint car il y a un smartpointeur par joint (non copiable)
    }
}

// pas necessaire????
// CBras::CBras(CBras&& other) {
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

Eigen::VectorXd CBras::randomQ() const{ // q aleatoire dans les bornes
    Eigen::VectorXd Q(this->getNbJoints());

    std::random_device rd;
    std::mt19937 gen(rd());

    for(size_t i = 0; i < this->joints_.size(); i++){
        double qmin = joints_[i]->getQMin();
        double qmax = joints_[i]->getQMax();

        std::uniform_real_distribution<double> q(qmin, qmax);
        Q[i] = q(gen);
    }
    return Q;
}


//============= [ getters ] =================
int CBras::getNbJoints() const {
    return joints_.size();
}

CJoint& CBras::getJoint(int i) const {
    if(i >= this->getNbJoints() || i<0){
        char msg[50];
        sprintf(msg, "Index joint invalide (%d > %d)", i, this->getNbJoints()-1);
        throw std::out_of_range(msg);
    }
    return *joints_[i];
}

Eigen::VectorXd CBras::getQ() const{   // q courants
    int n = this->getNbJoints();
    Eigen::VectorXd q(n);
    for(int i=0; i<n; i++){
        q[i] = this->joints_[i]->getQ();
    }
    return q;
} 

// =========== [ setters ] ===================
void CBras::setQ(const Eigen::VectorXd& q){ // leve invalid_argument si taille incorrecte
    if(q.size() != this->getNbJoints()){
        throw std::invalid_argument("Taille de q incorrecte");
    }
    for(size_t i = 0; i < this->joints_.size(); i++){
        this->joints_[i]->setQ(q[i]);
    }
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

CBras& CBras::operator=(const CBras& other) {
    CBras tmp(other);                // copie via constructeur de copie
    std::swap(joints_, tmp.joints_); // swap
    return *this;
}
