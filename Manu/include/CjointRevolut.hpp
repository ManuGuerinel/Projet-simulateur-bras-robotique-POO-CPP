#ifndef CjointRevolut_HPP
#define CjointRevolut_HPP

#include "CJoint.hpp"

class CJointRevolute : public CJoint{
    private:
        double dx_;

    public:

        //======== [constructeur ]===============
        CJointRevolute(double q=0, double dx=0, double qMin=-1e6, double qMax=1e6) : CJoint(q,qMin,qMax), dx_(dx) {}

        //======== [ methode ]===================
        Mat4 getTransform() const override;
        std::string getTypeName() const override;
        std::unique_ptr<CJoint> clone() const override;

        //======== [ setters ] =================
        void setDx(double dx);

        //======== [ destructeur ]==============
        ~CJointRevolute();
};

#endif