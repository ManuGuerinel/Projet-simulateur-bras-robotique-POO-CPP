#ifndef CjointRevolut_HPP
#define CjointRevolut_HPP

#include "CJoint.hpp"
/**
 * @class CJointRevolute
 * @brief Joint de type revolute (rotation)
 *
 * @details Hérite de CJoint. Rotation autour de l'axe z.
 * Possède un attribut supplémentaire dx_ pour la longueur du lien.
 */
class CJointRevolute : public CJoint{
    private:
        double dx_;

    public:

        //======== [constructeur ]===============
        CJointRevolute(double qMin=-1e6, double qMax=1e6, double q=0, double dx=0) : CJoint(qMin,qMax,q), dx_(dx) {}

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