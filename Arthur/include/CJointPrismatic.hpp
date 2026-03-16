#ifndef CJointPrismatic_HPP
#define CJointPrismatic_HPP

#include "CJoint.hpp"

class CJointPrismatic : public CJoint {
    // liaison prismatique : translation selon z
    public:
        //Constructeur=====
        CJointPrismatic(double q, double qMin, double qMax) : CJoint(q, qMin, qMax) {}

        //Destructeur=====

        //Methodes=====
        virtual Mat4 getTransform() const override;
        virtual std::string getTypeName() const override;
        virtual std::unique_ptr<CJoint> clone() const override;
};

#endif
