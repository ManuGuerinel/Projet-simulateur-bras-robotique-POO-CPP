#ifndef CBras_HPP
#define CBras_HPP

#include <iomanip>
#include "CJoint.hpp"

class CBras {
    protected:
        vector<unique_ptr<CJoint>> joints_;
    
    public:
        //Constructeur================
        CBras() = default;
        //Destructeur=================
        ~CBras() = default;
        //Getters=====================
        size_t getNbJoints() const {
            return joints_.size();
        }
        const CJoint& getJoint(size_t i) const {
            if (i >= joints_.size()) {
                throw std::out_of_range("Index de joint hors limites!");
            }
            return *joints_[i];
        }
        //Setters=====================
        void addJoint(std::unique_ptr<CJoint> joint) {
            joints_.push_back(std::move(joint));
        }
        //Methodes====================
        Mat4 computeFK() const;

        friend std::ostream& operator<<(std::ostream& os, const CBras& bras);
};

#endif
