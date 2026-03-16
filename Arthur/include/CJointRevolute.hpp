#ifndef CJointRevolute_HPP
#define CJointRevolute_HPP

#include "CJoint.hpp"

class CJointRevolute : public CJoint {
    // liaison pivot : rotation autour de z, lien dx_
    private:
        double dx_;

    public:
        //Constructeur=====
        CJointRevolute(double q, double qMin, double qMax, double dx) : CJoint(q, qMin, qMax), dx_(dx) {}
        
        //Destructeur=====
        
        //Methodes=====
        virtual Mat4 getTransform() const override;
        virtual std::string getTypeName() const override;
        virtual std::unique_ptr<CJoint> clone() const override;

};

#endif
