
#include "ex2_A.hpp"

#include <gtest/gtest.h>

// Test 3.a : computeFK() sur un bras vide retourne I_4 
TEST(BrasTest, ComputeFKEmpty) {
    CBras bras;
    Mat4 expected = Mat4::Identity();
    Mat4 result = bras.computeFK();
    
    // On vérifie que la matrice retournée est bien l'identité
    EXPECT_TRUE(result.isApprox(expected, 1e-10));
}

// Test 3.b : 1 joint rotatif (theta=0, dx=0.5m) 
TEST(BrasTest, ComputeFKOneRevolute) {
    CBras bras;
    // Ajout d'un joint avec q=0, bornes [-pi, pi], et dx=0.5
    bras.addJoint(std::make_unique<CJointRevolute>(0.0, -M_PI, M_PI, 0.5));
    
    Mat4 T = bras.computeFK();
    
    // Extraction de la translation avec block<3, 1>(0,3) 
    Eigen::Vector3d translation = T.block<3, 1>(0, 3);
    Eigen::Vector3d expected(0.5, 0.0, 0.0);
    
    // Vérification de l'effecteur en (0.5; 0; 0) 
    EXPECT_TRUE(translation.isApprox(expected, 1e-10));
}

// Test 3.c : getJoint(i) lève std::out_of_range pour i >= N 
TEST(BrasTest, GetJointOutOfRange) {
    CBras bras;
    bras.addJoint(std::make_unique<CJointPrismatic>(0.0, -1.0, 1.0)); // N = 1, seul l'index 0 existe
    
    // Pour i >= N (ici i=1), on s'attend à une exception 
    EXPECT_THROW(bras.getJoint(1), std::out_of_range);
    
    // Pour un index valide, pas d'exception
    EXPECT_NO_THROW(bras.getJoint(0));
}

// Point d'entrée GTest
int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}