/*
  Description :   Permet de générer des signaux PWM de fréquence "modifiable" depuis un Arduino équipé d'un microcontrôleur ATmega328P (Uno, Nano, …),
                  sur la sortie D3, dépendante du timer 2 du µC
                  
  Auteur :        Jérôme TOMSKI (https://passionelectronique.fr/)
  Créé le :       11.11.2021
*/

// **************************************************
// Trois derniers bits du registre de contrôle TCCR2B
// **************************************************
// CS22 | CS21 | CS20 | RÉSULTAT
//   0  |   0  |   0  | Timer arrêté
//   0  |   0  |   1  | Division de fréquence par 1
//   0  |   1  |   0  | Division de fréquence par 8
//   0  |   1  |   1  | Division de fréquence par 32
//   1  |   0  |   0  | Division de fréquence par 64
//   1  |   0  |   1  | Division de fréquence par 128
//   1  |   1  |   0  | Division de fréquence par 256
//   1  |   1  |   1  | Division de fréquence par 1024
// ***************************************************
#define frequencePWMde31372hz 0b00000001
#define frequencePWMde3921hz  0b00000010
#define frequencePWMde980hz   0b00000011
#define frequencePWMde490hz   0b00000100
#define frequencePWMde245hz   0b00000101
#define frequencePWMde122hz   0b00000110
#define frequencePWMde30hz    0b00000111
// Nota : ces fréquences sont celles obtenues avec un µC fonctionnant sur un quartz de 16MHz, tout en laissant le Prescaler sur "1" (pas de division de fréquence globale, donc)

void setup()  
{ 

  // Déclaration de la broche d'E/S D3 en sortie
  pinMode(3, OUTPUT);

  
  // Sélection du rapport de division de fréquence du timer 2
  
  TCCR2B &= 0b11111000;               // <===== à ne pas toucher
  TCCR2B |= frequencePWMde31372hz;    // <===== à changer, selon la fréquence que vous souhaitez en sortie

    // Nota 1 : l'opérateur "&=" constitue un "ET logique". Il applique le masque "0b11111000", afin de mettre à 0 les 3 derniers bits du registre TCCR2B, tout en laissant les autres bits intacts
    // Nota 2 : la fonction "|=" constitue un "OU logique". Il applique notre valeur à 8 bits, comme définie tout en haut, afin de modifier les 3 derniers bits du registre TCCR2B, précédemment mis à zéro


  // Génération d'un signal PWM sur la sortie D3, avec un rapport cyclique arbitraire, fixé à 33% (en prenant la valeur 85 sur 255, donc)
  analogWrite(3, 85);
  
}

void loop()  
{ 

}