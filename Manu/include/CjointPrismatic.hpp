#ifndef CjointPrismatic_HPP
#define CjointPrismatic_HPP

#include "CJoint.hpp"

class CJointPrismatic : public CJoint{
    private:
        double dx_;

    public:
        //======== [constructeur ]===============
        CJointPrismatic(double q=0, double qMin=-1e6, double qMax=1e6) : CJoint(q,qMin,qMax) {}

        //======== [ methode ]===================
        Mat4 getTransform() const override;
        std::string getTypeName() const override;
        std::unique_ptr<CJoint> clone() const override;

        ~CJointPrismatic();
};

#endif