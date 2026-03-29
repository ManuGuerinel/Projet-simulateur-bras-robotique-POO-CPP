#include "CJoint.hpp"
#include "CjointPrismatic.hpp"
#include "CjointRevolut.hpp"
#include "Cbras.hpp"
#include "CVector.hpp"

CBras instancier_bras(){
    CBras bras;
    bras.addJoint(std::make_unique<CJointRevolute>(-M_PI, M_PI, -M_PI, 0.3));
    bras.addJoint(std::make_unique<CJointRevolute>(-M_PI/2, M_PI/2, 0.0, 0.25));
    bras.addJoint(std::make_unique<CJointPrismatic>(0.0, 0.20, 0.10));
    bras.addJoint(std::make_unique<CJointRevolute>(-M_PI, M_PI, -0.0, 0.05));
    std::cout << bras;

    return bras;
}

int main(){
    CBras bras = instancier_bras();

    // balayage de l'epaule d'un pas de pi/8 (mettre i<18 pour out_of_range)
    for(int i=0; i<17 ; i++){
        Eigen::VectorXd q_new(4);
        double q_epaule = -M_PI + (M_PI*i)/8;
        q_new << q_epaule, 0.0, 0.10, 0.0;
        bras.setQ(q_new);

        // affichage effecteur
        Mat4 T = bras.computeFK();
        std::cout << "Affichage Bras, position Epaule : " << q_epaule << std::endl;
        //std::cout << "MGD : \n" << T << std::endl;
        std::cout << "Position effecteur : " << T.block<3,1>(0,3).transpose() << std::endl << std::endl;
    }

    // test sur copie profonde  (modifier l’original ne doit pas affecter la copie),
    CBras copieP_bras = bras;   //copie profonde
    Eigen::VectorXd q_new(4);
    q_new << 0.4, 0.10, 0.0, 1.2;
    copieP_bras.setQ(q_new);
    std::cout << "Bras original:\n" << bras << std::endl;
    std::cout << "Copie Bras:\n" << copieP_bras << std::endl;

    // test avec std::move() (l’original doit être vide après déplacement).
    CBras copieM_bras = std::move(bras);   //copie move
    Eigen::VectorXd q_newb(4);
    q_newb << 0.4, 0.10, 0.0, 1.2;
    copieP_bras.setQ(q_new);
    std::cout << "Bras original:\n" << bras << std::endl;
    std::cout << "Copie Bras:\n" << copieP_bras << std::endl;

    return 0;
}