#ifndef CjointPrismatic_HPP
#define CjointPrismatic_HPP

#include "CJoint.hpp"
/**
 * @class CJointPrismatic
 * @brief Joint de type prismatic (translation)
 *
 * @details Hérite de CJoint. Translation le long de l'axe z.
 */
class CJointPrismatic : public CJoint{
    private:
        // double dx_;  // Not used for prismatic joint

    public:
        //======== [constructeur ]===============
        CJointPrismatic(double qMin=-1e6, double qMax=1e6,double q=0) : CJoint(qMin,qMax,q) {}

        //======== [ methode ]===================
        Mat4 getTransform() const override;
        std::string getTypeName() const override;
        std::unique_ptr<CJoint> clone() const override;

        ~CJointPrismatic();
};

#endif