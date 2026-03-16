#include "ex1_A.hpp"


/*

export PKG_CONFIG_PATH=/home/polytech/miniconda3/lib/python3.13/site-packages/cmeel.prefix/lib/pkgconfig:$PKG_CONFIG_PATH

*/






// Test 3.a : setQ() hors bornes -> std::out_of_range
TEST(JointTest, ExceptionHorsBornes) {
    CJointRevolute joint(0.0, -3.14, 3.14, 0.5);
    
    // On s'attend à ce que setQ(5.0) lève une exception
    EXPECT_THROW(joint.setQ(5.0), std::out_of_range);
    
    // On s'attend à ce qu'une valeur valide fonctionne sans erreur
    EXPECT_NO_THROW(joint.setQ(1.0));
}

// Test 3.b : Transform de Revolute doit donner I_4 pour theta = 0, dx = 0
TEST(JointTest, TransformRevoluteIdentity) {
    CJointRevolute joint(0.0, -1.0, 1.0, 0.0); // q_ = 0, qMin = -1, qMax = 1, dx_ = 0
    
    Mat4 expected = Mat4::Identity();
    Mat4 result = joint.getTransform();
    
    EXPECT_TRUE(result.isApprox(expected, 1e-10)); // isApprox gère les problèmes d'arrondis des flottants (tol 1e-10)
}

// Test 3.c : Transform Prismatic, translation d = 1 doit etre correcte
TEST(JointTest, TransformPrismaticTranslation) {
    CJointPrismatic joint(1.0, -5.0, 5.0); // q_ = 1 (notre d), qMin = -5, qMax = 5
    
    Mat4 expected = Mat4::Identity();
    expected(2, 3) = 1.0; // Translation Z = 1
    
    Mat4 result = joint.getTransform();
    EXPECT_TRUE(result.isApprox(expected, 1e-10));
}

/*
// Test 3.d : Bonus Pinocchio - Comparaison avec le robot UR5
TEST(JointTest, BonusPinocchioUR5) {
    // 1. Initialisation de Pinocchio
    pinocchio::Model model;
    
    
    std::string urdf_path = "/home/polytech/Documents/4A/C++/TP/TP_Projet/ur5.urdf"; 
    
    try {
        pinocchio::urdf::buildModel(urdf_path, model);
    } catch (const std::invalid_argument& e) {
        // Si le fichier n'est pas trouvé, on saute le test plutôt que de faire crasher GTest
        GTEST_SKIP() << "Fichier URDF UR5 introuvable au chemin : " << urdf_path;
    }
    
    pinocchio::Data data(model);

    // 2. Configuration : on tourne le premier joint d'un angle theta = pi/4
    Eigen::VectorXd q = Eigen::VectorXd::Zero(model.nq);
    double theta = M_PI / 4.0;
    q[0] = theta; // q[0] correspond au premier degré de liberté

    // 3. Calcul de la cinématique (met à jour data.oMi)
    pinocchio::forwardKinematics(model, data, q);

    // 4. Récupération de la matrice de transformation de Pinocchio
    // Dans Pinocchio, l'index 0 est l'univers (la table), le joint 1 est le premier axe motorisé.
    pinocchio::SE3 pose_pinocchio = data.oMi[1];
    Mat4 mat_pinocchio = pose_pinocchio.toHomogeneousMatrix();

    // 5. Création de NOTRE joint pour comparer
    // Pour que la comparaison soit juste, il faut que notre 'dx_' corresponde à la géométrie de l'UR5.
    // L'énoncé place la translation sur la composante (0,3) de la matrice (l'axe X).
    // On extrait donc le 'dx' du URDF pour cette position (pour q=0, on pourrait l'extraire directement, 
    // ici on triche un peu en regardant la matrice Pinocchio si elle n'a qu'une translation pure sur cet axe).
    double dx_urdf = model.jointPlacements[1].translation().x(); 
    
    CJointRevolute mon_joint(theta, -M_PI, M_PI, dx_urdf);
    Mat4 ma_matrice = mon_joint.getTransform();

    // 6. La comparaison (Tolérance de 1e-10)
    EXPECT_TRUE(ma_matrice.isApprox(mat_pinocchio, 1e-10));
}
*/


// Point d'entrée GTest par défaut
int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}