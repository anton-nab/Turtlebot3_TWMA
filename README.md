# Turtlebot3_TWMA
Project of follow me and go to with a turtlebot 

Le but de ce projet est de réaliser une étape de la compétition de robotique "Robocup"

FOLLOW_ME: 

Le Follow_me est un code qui va permettre au robot de pouvoir suivre une personne. 
Celui-ci poursuivra une personne même si il y a des obstacles de part et d'autre de la personne le robot continuera de suivre la personne. 

Pour lancer le programme Follow_me: 

Il faut ce connecter au préalable au Turtlebot3: 
  Il faut ce connecter au même réseau wifi que le robot
  Ce connecter en SSH au robot en ayant son adresse IP et ces identifiants
  Il faudra lancer sur le robot une ligne de commande qui va entammer la captations des différentes Nodes
  Ensuite on lance le programme "Follow_me" sur notre PC pour initier la Node de ce programme. 

TUNING CODE

Le Tuning Code contient toute les modification de trajectoire de captation etc... 
qui nous ont permis de pouvoir avoir une circulation et un déplacement du robot plus fluide et moins saccadé.

Celui-ci a été modifié dans le fichier de configuration dédié dans les dossiers de la Rpi. 

GOTO_turtlebot && Nav_go_V3:

  Le Nav_go va permettre de faire revenir le robot à une coordonnée enregistrée en amont
  En fin du follow_me le robot va automotiquement revenir à ces coordonnées de départ.

  Pour lancer les programmes: 

  Les même étapes que le follow me s'appliquent pour pouvoir lancer le programme 

  MAP: 

   La map est présente dans ce Git, celle-ci contient la cartographie du couloir du 3éme étage d'ynov. 
  C'est la map qui a été cartographiée avec le turtlebot sous Rviz. 
  C'est elle qui nous a permis de pouvoir faire les Navigation Goal. 

  Follow_me_controled_stopped: 

  Ce code est complémentaire au code du Follow_me, 
  celui-ci permet d'intérrompre le follow me sans qu'il garde ces dernière paramètre de déplacement
  Ce code permet de stopper les moteurs du robot peut importe comment l'intérruption ce fait. 

  

