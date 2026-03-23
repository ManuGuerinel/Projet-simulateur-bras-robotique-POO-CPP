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
        CBras(const CBras& other);      //constructeur par copie
        CBras(CBras&& other) noexcept = default; //constructeur de deplacement
        //= default suffit car std::unique_ptr est déplaçable (move-only). Le std::vector de unique_ptr est donc lui aussi déplaçable, ce qui permet un transfert de propriété correct sans implémentation manuelle.

        //============ [ methode ] =================
        void addJoint(std::unique_ptr<CJoint> joint);
        Mat4 computeFK();
        Eigen::VectorXd randomQ() const; // q aleatoire dans les bornes

        friend std::ostream& operator<<(std::ostream& os, const CBras& bras);
        CBras& operator=(const CBras& other);                          // operateur d'affectation par copie
        CBras& operator=(CBras&& other) noexcept = default;     // operateur d'affectation de deplacement

        //============= [ getters ] =================
        int getNbJoints() const;
        CJoint& getJoint(int i) const;
        Eigen::VectorXd getQ() const; // q courants

        //============= [ setters ] =================
        void setQ(const Eigen::VectorXd& q); // leve invalid_argument si taille incorrecte

        //============= [ destructeur ] =============
        ~CBras() = default;
        // = default suffit car std::unique_ptr gère automatiquement la destruction des objets pointés. Les joints sont détruits dans l’ordre inverse de leur insertion (LIFO), lors de la destruction du std::vector.
};

#endif
