
# CO_BIEN_Konectik
Software for CO-BIEN Konectik board




## Dépendances
	a) Common/common_data.h
	b) Adafruit_BNO08x
	

a) le fichier permettant l'échange de paramètre avec les boutons tactiles

b) la bibliothèque permettant le dialogue avec l'IMU BNO085


## Préparation de la carte Nucleo32 G431

1) Enlever les straps 0 ohms notés SB2 & SB3
    PQ ?
    PB7 et PA15 court-cicuités avec PA5 et PA6 pour assurer la compatibilité avec Arduino,
    inutile pour nous, et posent problème pour le lecteur RFID
    
2) Poser le strap 0 ohms noté SB8 (il n'y a que "8" de sérigaphié)
    PQ ?
    Utilisation de PF0 pour une sortie TIMER vers un strip led, les autres fonctions de PFO sont désactivées 

3) Poser le strap 0 ohms noté SB11 (il n'y a que "11" de sérigaphié)
    PQ ?
    Utilisation de PF1 pour une sortie GPIO vers l'IMU, les autres fonctions de PF1 sont désactivées 


