# Moteur BLDC
Utilisation des moteur Maxon Maxon EC45 flat 70W réduit à 1:14.
Durant l'année 2022, un asserivssement en vitesse de ces moteurs à été réalisé par Vincent sous Cube IDE.
Le but de ce projet est d'implémenter cette asservissement en vitesse sur Mbed un RTOS que nous voulons démocratiser à Eirbot.
L'utilisation de CubeMX reste tout de même obligatoire pour que la PWM sur chaque phase soit configurée en "Hard" pour éviter les retard possible de l'ordonnanceur Mbed.

# Structure de l'asservissement
Dans le diagramme UML *Structure asservissement*, il y a un schéma d'implémentation pour l'asservissement en position.
