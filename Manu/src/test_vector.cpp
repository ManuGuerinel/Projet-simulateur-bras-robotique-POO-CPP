#include <iostream>
#include <cassert>
#include <cmath>
#include <utility>

#include "CJoint.hpp"
#include "CjointPrismatic.hpp"
#include "CjointRevolut.hpp"
#include "Cbras.hpp"

// ============================================================
// Utilitaires
// ============================================================
static int tests_passed = 0;
static int tests_failed = 0;

#define TEST(name, expr) \
    do { \
        bool _ok = (expr); \
        std::cout << (_ok ? "  [PASS] " : "  [FAIL] ") << name << "\n"; \
        if (_ok) tests_passed++; else tests_failed++; \
    } while(0)

static bool vecEqual(const Eigen::VectorXd& a, const Eigen::VectorXd& b, double tol = 1e-9) {
    if (a.size() != b.size()) return false;
    return (a - b).cwiseAbs().maxCoeff() < tol;
}

// Fabrique un bras de référence avec 3 joints et q initiaux connus
static CBras makeBras() {
    CBras b;
    b.addJoint(std::make_unique<CJointRevolute>(-M_PI, M_PI, 1.0, 0.5));
    b.addJoint(std::make_unique<CJointRevolute>(-M_PI, M_PI, 0.5, 0.3));
    b.addJoint(std::make_unique<CJointPrismatic>(0.0, 0.5, 0.2));
    return b;
}

// ============================================================
// Règle 1 — Constructeur de copie  CBras(const CBras&)
// ============================================================
void test_constructeur_copie() {
    std::cout << "\n[Règle 1] Constructeur de copie  CBras(const CBras&)\n";

    CBras original = makeBras();
    CBras copie(original);

    // --- Même état initial
    TEST("même nb de joints",
         copie.getNbJoints() == original.getNbJoints());

    TEST("mêmes q initiaux",
         vecEqual(copie.getQ(), original.getQ()));

    TEST("même type joint 0 (Revolute)",
         copie.getJoint(0).getTypeName() == original.getJoint(0).getTypeName());

    TEST("même type joint 2 (Prismatic)",
         copie.getJoint(2).getTypeName() == original.getJoint(2).getTypeName());

    // --- Indépendance (copie profonde, pas de partage de pointeurs)
    Eigen::VectorXd q_new(3);
    q_new << 0.1, -0.1, 0.3;
    copie.setQ(q_new);

    TEST("modifier la copie ne change pas l'original",
         vecEqual(original.getQ(), (Eigen::VectorXd(3) << 1.0, 0.5, 0.2).finished()));

    Eigen::VectorXd q_new2(3);
    q_new2 << -0.5, 0.2, 0.1;
    original.setQ(q_new2);

    TEST("modifier l'original ne change pas la copie",
         vecEqual(copie.getQ(), q_new));
}

// ============================================================
// Règle 2 — Constructeur de déplacement  CBras(CBras&&)
// ============================================================
void test_constructeur_deplacement() {
    std::cout << "\n[Règle 2] Constructeur de déplacement  CBras(CBras&&)\n";

    CBras source = makeBras();
    Eigen::VectorXd q_avant = source.getQ();
    int n_avant = source.getNbJoints();

    CBras dest(std::move(source));

    // --- La destination a récupéré le contenu
    TEST("nb joints transféré",
         dest.getNbJoints() == n_avant);

    TEST("q transférés",
         vecEqual(dest.getQ(), q_avant));

    // --- La source est dans un état valide (vide après move)
    TEST("source vidée après move",
         source.getNbJoints() == 0);

    // --- Indépendance : modifier dest ne touche pas à source (trivial mais explicite)
    Eigen::VectorXd q_new(3);
    q_new << 0.1, -0.1, 0.3;
    dest.setQ(q_new);

    TEST("dest modifiable après construction par déplacement",
         vecEqual(dest.getQ(), q_new));
}

// ============================================================
// Règle 3 — Opérateur d'affectation par copie  operator=(const CBras&)
//            (implémenté via copy-and-swap dans votre code)
// ============================================================
void test_operateur_affectation_copie() {
    std::cout << "\n[Règle 3] Opérateur d'affectation par copie  operator=(CBras)\n";

    CBras a = makeBras();
    CBras b;
    b.addJoint(std::make_unique<CJointRevolute>(-1.0, 1.0, 0.0));

    b = a;  // copy-and-swap

    // --- Même état
    TEST("nb joints égaux après affectation",
         b.getNbJoints() == a.getNbJoints());

    TEST("mêmes q après affectation",
         vecEqual(b.getQ(), a.getQ()));

    // --- Indépendance
    Eigen::VectorXd q_new(3);
    q_new << 0.2, -0.2, 0.1;
    b.setQ(q_new);

    TEST("modifier b ne change pas a",
         vecEqual(a.getQ(), (Eigen::VectorXd(3) << 1.0, 0.5, 0.2).finished()));

    // --- Auto-affectation (ne doit pas crasher ni corrompre)
    CBras c = makeBras();
    Eigen::VectorXd q_c = c.getQ();
    c = c;  // auto-affectation

    TEST("auto-affectation ne corrompt pas le bras",
         c.getNbJoints() == 3 && vecEqual(c.getQ(), q_c));
}

// ============================================================
// Règle 4 — Opérateur d'affectation par déplacement  operator=(CBras&&)
//            (= default dans votre .hpp)
// ============================================================
void test_operateur_affectation_deplacement() {
    std::cout << "\n[Règle 4] Opérateur d'affectation par déplacement  operator=(CBras&&)\n";

    CBras source = makeBras();
    Eigen::VectorXd q_avant = source.getQ();
    int n_avant = source.getNbJoints();

    CBras dest;
    dest.addJoint(std::make_unique<CJointRevolute>(-1.0, 1.0, 0.0));

    dest = std::move(source);

    // --- La destination a récupéré le contenu de source
    TEST("nb joints transférés dans dest",
         dest.getNbJoints() == n_avant);

    TEST("q transférés dans dest",
         vecEqual(dest.getQ(), q_avant));

    // --- La source est dans un état valide après le move
    TEST("source valide (vide) après move-assign",
         source.getNbJoints() == 0);

    // --- dest est bien utilisable
    Eigen::VectorXd q_new(3);
    q_new << 0.1, -0.1, 0.3;
    dest.setQ(q_new);

    TEST("dest utilisable après move-assign",
         vecEqual(dest.getQ(), q_new));
}

// ============================================================
// Règle 5 — Destructeur
//            Vérifié indirectement : pas de leak, pas de double-free.
//            On le valide en détruisant des bras copiés/déplacés.
// ============================================================
void test_destructeur() {
    std::cout << "\n[Règle 5] Destructeur — pas de leak / double-free\n";

    bool ok = true;
    try {
        // Bloc 1 : copie puis destruction normale
        {
            CBras a = makeBras();
            CBras b(a);           // copy ctor
            CBras c(std::move(a)); // move ctor
            // a, b, c détruits ici
        }

        // Bloc 2 : affectation puis destruction
        {
            CBras a = makeBras();
            CBras b;
            b = a;                // copy-assign
            CBras c;
            c = std::move(b);     // move-assign
            // a, b (vide), c détruits ici
        }

        // Bloc 3 : bras vide détruit
        {
            CBras vide;
        }
    } catch (...) {
        ok = false;
    }

    TEST("aucune exception lors de destructions en cascade (copie + move)",  ok);
    TEST("bras vide détruit sans erreur", ok);

    // Vérifie que les joints polymorphiques sont bien détruits (via valgrind si dispo)
    // Ici on teste que les types Revolute et Prismatic se clonent et se détruisent
    bool clone_ok = true;
    try {
        CBras src = makeBras();
        {
            CBras clone_bras(src);  // clone() appelé sur chaque joint
            // clone_bras détruit ici : ~CJointRevolute et ~CJointPrismatic appelés
        }
    } catch (...) {
        clone_ok = false;
    }
    TEST("joints polymorphiques (Revolute+Prismatic) clonés et détruits sans erreur", clone_ok);
}

// ============================================================
// Test croisé — enchaînement des 5 règles
// ============================================================
void test_enchainement_regles() {
    std::cout << "\n[Bonus] Enchaînement complet des 5 règles\n";

    CBras a = makeBras();

    // copy ctor → b est une copie indépendante de a
    CBras b(a);

    // copy-assign → c reçoit le contenu de b
    CBras c;
    c = b;

    // move ctor → d vole le contenu de c
    CBras d(std::move(c));

    // move-assign → e vole le contenu de d
    CBras e;
    e = std::move(d);

    TEST("après copy+copy-assign+move ctor+move-assign : nb joints correct",
         e.getNbJoints() == 3);

    TEST("après chaîne complète : q identiques à l'original",
         vecEqual(e.getQ(), a.getQ()));

    TEST("sources intermédiaires vidées",
         c.getNbJoints() == 0 && d.getNbJoints() == 0);

    TEST("a (original) inchangé tout au long",
         vecEqual(a.getQ(), (Eigen::VectorXd(3) << 1.0, 0.5, 0.2).finished()));
}

// ============================================================
// main
// ============================================================
int main() {
    std::cout << "=============================================\n";
    std::cout << "  Tests Règle des 5 — CBras\n";
    std::cout << "=============================================\n";

    test_constructeur_copie();
    test_constructeur_deplacement();
    test_operateur_affectation_copie();
    test_operateur_affectation_deplacement();
    test_destructeur();
    test_enchainement_regles();

    std::cout << "\n=============================================\n";
    std::cout << "  Résultat : " << tests_passed << " passé(s), "
              << tests_failed << " échoué(s)\n";
    std::cout << "=============================================\n";

    return tests_failed > 0 ? 1 : 0;
}