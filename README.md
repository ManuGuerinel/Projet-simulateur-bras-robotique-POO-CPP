# Projet-simulateur-bras-robotique-POO-CPP


# exercice 1 : Séance 1 – Conception UML et hiérarchie de joints
## 1.a
Le destructeur doit etre virtual car le destructeur de la classe dérivée ne sera pas appelé : fuite mémoire ou comportement indéfini.

## 1.b
les setters comme setQ() ne peuvent pas etre const car ils modifient des attribues, contrairement au accesseurs comme les getQ(), ils ne modifient rien donc doivent etre const pour la securité.

