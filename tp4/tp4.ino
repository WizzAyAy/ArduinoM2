#include <SoftwareSerial.h>
#include <EEPROM.h>
#include <avr/sleep.h>
#include <avr/interrupt.h>


// j'ai effectué les tests avec le mode idle et sans le mode idle et en effet on gagne
// en performance. On passe de 0.5v a 0.3v


// bluetooth
const byte rxPin = 2; // à relier au TxD du HM-10
const byte txPin = 3; // à relier au RxD du HM-10
SoftwareSerial BT(rxPin, txPin);

//general
unsigned int delayTime = 1000; // temps entre chaque capture
unsigned int incomingByte = 0; // serial data
unsigned int adress = 0; // adress EEPROM

// temperature
unsigned int tempVolt = 0; // variable qui va lire la valeur recu du capteur de temperature
int temp = 0; // temperature calculée
int tempArray[400]; // array avec toutes les temperatures stockées
unsigned char indiceTempArray = 0; // indice de notre array
unsigned char analogPinTmp = 4; // pin analogique sur lequel on va connecter notre capteur

// porte
unsigned char pinPorte = 4;
unsigned char etatPorte = 0;

// capteur lumiere
unsigned char pinAlimLumiere = 6;
unsigned char pinCapteurLumiere = 7;
unsigned char etatCapteurLumiere = 0;

// etat boite aux lettres
#define VIDE 0
#define REMPLI 1
unsigned char etat ; // variable qui prendra les valeurs VIDE ou REMPLI au cours du temps
unsigned int compteur;


// led
unsigned char pinLed = 5;
unsigned char pinLedTemp = 13;
bool blinkLedBlink = false;

//temps
unsigned long timeOldTemp = 0;
unsigned long timeOldCapteur = 0;
unsigned long time = 0;


// ------- SETUP ------- //

void setup() {
//  Serial.begin(9600);
  BT.begin(9600);
  // delay pour que le bt se connect bien 
  delay(1000);
  BT.print("AT+NAMEQuentin"); //module renommé BTM21 par la commande AT+NAME
  delay(1000);
  analogReference(INTERNAL); //VREF = 1.1V interne
  delayTime = readIntFromEEPROM(adress); // lit la valeur stocké en mémoire
  if (delayTime) {
    BT.print("Delay fixé grace a l'eeprom à ");
    BT.println(delayTime);
  }
  else {
    delayTime = 1000;
    BT.print("Delay fixé à ");
    BT.println(delayTime);
  }

  pinMode(pinPorte, INPUT_PULLUP);
  pinMode(pinCapteurLumiere, INPUT_PULLUP);
  pinMode(pinLed, OUTPUT);
  pinMode(pinLedTemp, OUTPUT);
  pinMode(pinAlimLumiere, OUTPUT);


  //setup de la boite aux letrre
  etat = REMPLI;
}

// ------- LOOP ------- //

void loop() {
  set_sleep_mode(SLEEP_MODE_IDLE); // configurer en mode IDLE
  sleep_enable(); // valider le mode veille
  sleep_mode(); // activer la veille
  
  time = millis();

  // TEMPERATURE
  if ( time - timeOldTemp > delayTime) {
    timeOldTemp = time;
    blinkLedBlink = !blinkLedBlink;
    //    fait blink la led inter
    if (blinkLedBlink) {
      digitalWrite(pinLedTemp, HIGH);
    } else {
      digitalWrite(pinLedTemp, LOW);
    }

    tempVolt = analogRead(analogPinTmp); // lit la valeur analogique sur le pin analogPinTmp
    
    temp = (tempVolt - 500); // convertion des volts en celcuis
    tempArray[indiceTempArray] = temp; // stocke les temps dans un tab
    indiceTempArray++; // incrementation du nombre de temps aqueries
  }


  // CAPTEUR
  if ( time - timeOldCapteur > 20) {
    timeOldCapteur = time;


    // SERIAL
    if (Serial.available() > 0) {
      incomingByte = Serial.read();
      manageInputTemp(incomingByte);
      manageInputData(incomingByte);
      askBoxState(incomingByte, etat, compteur);
      if (manageVideBoite(incomingByte)) {
        etat = VIDE;
      }
    }

    // BLUETOOTH
    if (BT.available()) {
      incomingByte = BT.read();
      manageInputTemp(incomingByte);
      manageInputData(incomingByte);
      askBoxState(incomingByte, etat, compteur);
      if (manageVideBoite(incomingByte)) {
        etat = VIDE;
      }
    }

    // ETAT DE LA PORT
    etatPorte = EtatPorte();
    if (etatPorte == 0) {
      //      Serial.println("Porte fermée");
    }
    else if (etatPorte == 1) {
      //      Serial.println("Porte Ouverte");
      etat = VIDE;
    }

    //CAPTEUR DE LA LUMIERE
    digitalWrite(pinAlimLumiere, HIGH);
    delay(100);
    etatCapteurLumiere = EtatCapteurLumiere();
    digitalWrite(pinAlimLumiere, LOW);
    if (etatCapteurLumiere == 0) {
      //     Serial.println("Lumiere Ouverte");
    }
    else if (etatCapteurLumiere == 1) {
      //      Serial.println("Lumiere Fermée");
      etat = REMPLI;
    }

    CommandeSortie(etat, pinLed);
  }
}


// ------- FUCNTIONS ------- //

// fonction qui gere les entrées + et -
void manageInputTemp(int incomingByte) {
  // 43 = "+"
  if (incomingByte == 43) {
    delayTime = delayTime + 100;
    BT.println("on augmente le rythme de prise de tmp");
  }

  // 45 = "-"
  else if (incomingByte == 45) {
    if (delayTime > 150) {
      delayTime = delayTime - 100;
      BT.println("on diminue le rythme de prise de tmp");
    }
    else {
      BT.println("delay deja trop bas ...");
    }
  }
  if (incomingByte == 43 or incomingByte == 45) {
    BT.print("le nouveau rythme est de : ");
    BT.print(delayTime);
    BT.println("ms");
  }
}

// fonction qui gere les entrées e et m
void manageInputData(int incomingByte) {
  // touche e
  if (incomingByte == 101) {
    BT.println("voici toutes les temperatures : ");
    for (int i = 0; i < indiceTempArray; i++) {
      BT.print(tempArray[i] / 10);
      BT.print(".");
      BT.println(tempArray[i] % 10);
    }
    indiceTempArray = 0;
  }

  //touche m
  if (incomingByte == 109) {
    writeIntIntoEEPROM(adress, delayTime);
  }
}


void askBoxState(int incomingByte, unsigned char etat, unsigned int & compteur) {
  //touche ?
  if (incomingByte == 63) {
    compteur++;
    if (etat == VIDE) {
      BT.println("n");
    } else {
      BT.println("o");
      BT.println(compteur % 65536);
    }
  }
}

bool manageVideBoite(int incomingByte) {
  //touche r
  if (incomingByte == 114) {
    return true;
  }
  return false;
}

// fonction qui ecrit dans eeprom un nombre à l'adr adress
void writeIntIntoEEPROM(int address, int number) {
  BT.println("ECRITURE DANS L EEPROM");
  byte byte1 = number >> 8;
  byte byte2 = number & 0xFF;
  EEPROM.write(address, byte1);
  EEPROM.write(address + 1, byte2);
}

// fonction qui lit dans eeprom un nombre à l'adr adress
unsigned int readIntFromEEPROM(unsigned int address) {
  BT.println("LECTURE DE L EEPROM");
  byte byte1 = EEPROM.read(address);
  byte byte2 = EEPROM.read(address + 1);
  return (byte1 << 8) + byte2;
}

// 1 si porte ouverte 0 sinon
int EtatPorte(void) {
  return digitalRead(pinPorte);
}


// 1 si porte lettre 0 sinon
int EtatCapteurLumiere(void) {
  return digitalRead(pinCapteurLumiere);
}

void CommandeSortie (unsigned char Etat, int ledPin) {
  switch (etat) {
    case VIDE :
      digitalWrite(ledPin, LOW);
      break;
    case REMPLI :
      digitalWrite(ledPin, HIGH);
      break;
  }
}
