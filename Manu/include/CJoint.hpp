#ifndef Cjoint_HPP
#define Cjoint_HPP

#include <pinocchio/spatial/se3.hpp>
#include <Eigen/Dense>

using Mat4 = Eigen::Matrix4d;

class CJoint {
    private:
        double q_;
        double qMin_;
        double qMax_;

    public:
        //======== [constructeur ]===============
        // Cjoint();
        CJoint(double q=0, double qMin=-1e6, double qMax=1e6) : q_(q), qMin_(qMin), qMax_(qMax) {}

        //======== [ methode ]===================
        virtual Mat4 getTransform() const = 0;
        virtual std::string getTypeName() const = 0;
        virtual std::unique_ptr<CJoint> clone() const = 0;

        //======== [ accesseur ] ================
        double getQ() const;

        //======== [ setters ] ==================
        void setQ(double q);
        void setQMin(double qmin);
        void setQMax(double qmax);

        //======== [ destructeur ]===============
        virtual ~CJoint() = default;    //geré par les classe enfant
};

#endif
