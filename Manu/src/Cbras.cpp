#include "../include/Cbras.hpp"
#include <Eigen/Dense>
#include <vector>
#include <memory>
#include "../include/CJoint.hpp"

using Mat4 = Eigen::Matrix4d;

//============ [ methode ] =================
void CBras::addJoint(std::unique_ptr<CJoint> joint){
    joints_.push_back(std::move(joint));
}

Mat4 CBras::computeFK() const {
}

//============= [ getters ] =================
int CBras::getNbJoints(){
    return joints_.size();
}

CJoint& CBras::getJoint(size_t){

}