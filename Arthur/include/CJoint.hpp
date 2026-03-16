#ifndef CJoint_HPP
#define CJoint_HPP

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
        CJoint(double q_, double qMin_, double qMax_) ;

        //Destructeur=================
        virtual ~CJoint() = default;

        //Getters=====================
        double getQ() const ;
        double getQMin() const ;
        double getQMax() const ;


        //Setters=====================
        void setQ(double q) ;

        //Methodes====================
        virtual Mat4 getTransform() const = 0;
        virtual std::string getTypeName() const = 0;
        virtual std::unique_ptr<CJoint> clone() const = 0;


};
#endif
