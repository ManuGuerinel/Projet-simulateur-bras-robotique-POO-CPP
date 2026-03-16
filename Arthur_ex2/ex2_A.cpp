#include "ex2_A.hpp"




//======CLASSE CBras========================================
//==========================================================

Mat4 CBras::computeFK() const {
    Mat4 transform = Mat4::Identity();
    for (const auto& joint : joints_) {
        transform *= joint->getTransform();
    }
    return transform;
}


std::ostream& operator<<(std::ostream& os, const CBras& bras) {
    //affichage nb de DDL et type de chaque DDL
    os << "Nombre d'articulations : " << bras.getNbJoints() << endl;
    for (size_t i = 0; i < bras.getNbJoints(); i++) {
        const auto& joint = bras.joints_[i];
        os << "Joint [" << i << "]: " << joint->getTypeName() << "   ";

        os << fixed << setprecision(3); // Affichage avec 3 décimales
        std::string unite = (joint->getTypeName() == "Prismatic") ? " m,   " : " rad,   ";
        os << " q = " << joint->getQ() << unite;
        os << "bornes=[" << joint->getQMin() << ", " << joint->getQMax() << "]" << endl;
    }
    return os;
}


