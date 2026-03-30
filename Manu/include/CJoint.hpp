#ifndef Cjoint_HPP
#define Cjoint_HPP

#include <pinocchio/spatial/se3.hpp>
#include <Eigen/Dense>

using Mat4 = Eigen::Matrix4d;
/**
 * @class CJoint
 * @brief Classe abstraite représentant un joint générique
 */
class CJoint {
    private:
        double qMin_;
        double qMax_;
        double q_;

    public:
        //======== [constructeur ]===============
        CJoint();
        CJoint(double qMin, double qMax, double q);

        //======== [ methode ]===================
        virtual Mat4 getTransform() const = 0;
        virtual std::string getTypeName() const = 0;
        virtual std::unique_ptr<CJoint> clone() const = 0;

        //======== [ accesseur ] ================
        double getQ() const;
        double getQMin() const;
        double getQMax() const;

        //======== [ setters ] ==================
        void setQ(double q);
        void setQMin(double qmin);
        void setQMax(double qmax);

        //======== [ destructeur ]===============
        virtual ~CJoint() = default;    //geré par les classe enfant
};

#endif
