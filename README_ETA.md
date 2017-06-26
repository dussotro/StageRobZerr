READ ME --- ETANAO

###############################################################################
# Fonctions prédéfinies pour le NAO "Eta"                                     #
###############################################################################

Comme sur les autres NAO, deux fonctions de mouvement sont définies avec une
configuration spécifique à Eta.

Ces paramètres sont définis et modifiable dans le fichier :
	moveCustom.py

Vous avez le droit de modifier ces paramètres, ce ne sont pas forcément les meilleurs.
Ce sont juste des paramètres qui permettent de faire fonctionner le robot approximativement.

Les fonctions concernant Eta sont:
	moveToEta()
	moveTowardEta()

###############################################################################
# Appel des fonctions - Eta                                                   #
###############################################################################

-------------------------------------
 -> moveToEta(X, Y, Theta, Frequency)
-------------------------------------

X = déplacement à faire vers l'avant (X positif) ou l'arrière (X négatif)
	La vitesse de déplacement est défini par une fonction mathématique qui
	permet de moduler l'amplitude la vitesse en fonction de la distance à 
	parcourir.

Y = déplacement à faire latéralement sur la gauche (Y positif) ou sur la droite
	(Y négaitif)
	La vitesse de déplacement est défini par une fonction mathématique qui
	permet de moduler l'amplitude la vitesse en fonction de la distance à 
	parcourir.

Theta = angle à faire avec le robot

Frequency = la valeur appartient à [0.0, 1.0] et correspond à la fréquence depas
	du robot donc indirectement à sa vitesse. 0.0 est la vitesse minimale
	et 1.0 est la vitesse maximale.

-------------------------------------------
 -> moveTowardEta(vX, vY, omega, Frequency)
-------------------------------------------

vX = vitessedu robot vers l'avant (X positif) ou l'arrière (X négatif)

vY = vitesse latérale du robot sur la gauche (Y positif) ou sur la droite
	(Y négatif)

omega = vitesse angulaire du robot robot

Frequency = la valeur appartient à [0.0, 1.0] et correspond à la fréquence depas
	du robot donc indirectement à sa vitesse. 0.0 est la vitesse minimale
	et 1.0 est la vitesse maximale.


Précautions:
-----------

Le robot Eta a un problème au niveau de la hanche gauche. Ce qui rend sa démarche
instable par défaut, c'est pourquoi il faut utiliser les paramètres définis dans
moveCustom.py.

Autres conseils :
	- éviter de dépasser la fréquence de 0.5 en arrière (chute 1 fois sur 2)
	- éviter de dépasser la fréquence de 0.8 en avant   (idem)
	- la hauteur des pieds changent légèrement la démarche et lui permet de 
	stabiliser son problème; celui-ci a la jambe gauche qui fait un arc de 
	cercle quand il marche, c'est la cause de son instabilité; 
	régler la hauteur permet de limiter ce problème.


