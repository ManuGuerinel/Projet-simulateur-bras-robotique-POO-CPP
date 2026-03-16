#include <iostream>
#include <vector>
#include <memory>
#include <stdexcept> //pour std::out_of_range
#include <cmath> //pour std::cos et sin 


#include "pinocchio/parsers/urdf.hpp"      // Pour charger un modèle de robot
#include "pinocchio/multibody/model.hpp"    // Structure du robot
#include "pinocchio/multibody/data.hpp"     // Données de calcul (cache)
#include "pinocchio/algorithm/kinematics.hpp" // Algorithmes de cinématique

#include <Eigen/Dense>
#include <Eigen/Core>

#include <gtest/gtest.h>


//namespace pin = pinocchio;
using namespace std;
using Mat4 = Eigen::Matrix4d;


class CJoint {
    protected: 
        double q_;
        double qMin_;
        double qMax_;

    public:
        //Constructeur================
        CJoint(double q_, double qMin_, double qMax_) {
            this->q_ = q_;
            this->qMin_ = qMin_;
            this->qMax_ = qMax_;
        }

        //Destructeur=================
        virtual ~CJoint() = default;

        //Getters=====================
        double getQ() const {
            return q_;
        }

        double getQMin() const {
            return qMin_;
        }

        double getQMax() const {
            return qMax_;
        }


        //Getters=====================
        void setQ(double q) {
            if ((q<qMin_)||(q>qMax_)) {
                throw std::out_of_range("q hors limites!");
            }
            q_ = q;
        }

        //Methodes====================
        virtual Mat4 getTransform() const = 0;
        virtual std::string getTypeName() const = 0;
        virtual std::unique_ptr<CJoint> clone() const = 0;


};


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



