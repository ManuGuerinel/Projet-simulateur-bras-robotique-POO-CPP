#ifndef Cbras_HPP
#define Cbras_HPP

#include <Eigen/Dense>
#include <vector>
#include <memory>
#include "CJoint.hpp"

using Mat4 = Eigen::Matrix4d;

class CBras {
    private:
        std::vector<std::unique_ptr<CJoint>> joints_;

    public:
        //============ [ constructeur ] ============
        CBras() = default;

        //============ [ methode ] =================
        void addJoint(std::unique_ptr<CJoint> joint);
        Mat4 computeFK();

        friend std::ostream& operator<<(std::ostream& os, const CBras& bras);

        //============= [ getters ] =================
        int getNbJoints() const;
        CJoint& getJoint(size_t) const;
};

#endif
