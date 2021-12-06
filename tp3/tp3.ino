#include <SoftwareSerial.h>
#include <EEPROM.h>

// bluetooth
const byte rxPin = 2; // à relier au TxD du HM-10
const byte txPin = 3; // à relier au RxD du HM-10
SoftwareSerial BT(rxPin, txPin);

// temperature
int tempVolt = 0; // variable qui va lire la valeur recu du capteur de temperature
float temp = 0; // temperature calculée
float tempArray[200]; // array avec toutes les temperatures stockées
int indiceTempArray = 0; // indice de notre array
int analogPinTmp = 4; // pin analogique sur lequel on va connecter notre capteur
int delayTime = 1000; // temps entre chaque capture
int incomingByte = 0; // serial data
int adress = 0; // adress EEPROM

// porte
int pinPorte = 4;
int etatPorte = 0;

// capteur lumiere 
int pinCapteurLumiere = 7;
int etatCapteurLumiere = 0;


void setup() {
  Serial.begin(9600);
  //BT.begin(9600);
  delay(1000);
  BT.print("AT+NAMEQuentin"); //module renommé BTM21 par la commande AT+NAME
  delay(1000);
  analogReference(INTERNAL); //VREF = 1.1V interne
  delayTime = readIntFromEEPROM(adress); // lit la valeur stocké en mémoire
  if (delayTime) {
    Serial.print("Delay fixé grace a l'eeprom à ");
    Serial.println(delayTime);
  }
  else {
    delayTime = 1000;
    Serial.print("Delay fixé à ");
    Serial.println(delayTime);
  }
  
  pinMode(pinPorte, INPUT_PULLUP);
  pinMode(pinCapteurLumiere, INPUT_PULLUP);
}

void loop() {
  tempVolt = analogRead(analogPinTmp); // lit la valeur analogique sur le pin analogPinTmp
  temp = (float) (tempVolt - 500) / 10; // convertion des volts en celcuis
  tempArray[indiceTempArray] = temp; // stocke les temps dans un tab
  indiceTempArray++; // incrementation du nombre de temps aqueries
  delay(delayTime);
  if (Serial.available() > 0) {
    incomingByte = Serial.read();
    manageInputTemp(incomingByte);
    manageInputData(incomingByte);
  }
  if (BT.available()) {
    incomingByte = BT.read();
    manageInputTemp(incomingByte);
    manageInputData(incomingByte);
  }

  etatPorte = EtatPorte();
  if (etatPorte == 0) {
    Serial.println("Porte fermée");
  }
  else if (etatPorte == 1) {
    Serial.println("Porte Ouverte");
  }

  etatCapteurLumiere = EtatCapteurLumiere();
  if (etatCapteurLumiere == 0) {
    Serial.println("Lumiere fermée");
  }
  else if (etatCapteurLumiere == 1) {
    Serial.println("Lumiere Ouverte");
  }
}


// fonction qui gere les entrées + et -
void manageInputTemp(int incomingByte) {
  // 43 = "+"
  if (incomingByte == 43) {
    delayTime = delayTime + 100;
    Serial.println("on augmente le rythme de prise de tmp");
  }

  // 45 = "-"
  else if (incomingByte == 45) {
    if (delayTime > 150) {
      delayTime = delayTime - 100;
      Serial.println("on diminue le rythme de prise de tmp");
    }
    else {
      Serial.println("delay deja trop bas ...");
    }
  }
  if (incomingByte == 43 or incomingByte == 45) {
    Serial.print("le nouveau rythme est de : ");
    Serial.print(delayTime);
    Serial.println("ms");
  }
}

// fonction qui gere les entrées e et m
void manageInputData(int incomingByte) {
  // touche e
  if (incomingByte == 101) {
    Serial.println("voici toutes les temperatures : ");
    for (int i = 0; i < indiceTempArray; i++) {
      Serial.println(tempArray[i]);
    }
    indiceTempArray = 0;
  }

  //touche m
  if (incomingByte == 109) {
    writeIntIntoEEPROM(adress, delayTime);
  }
}

// fonction qui ecrit dans eeprom un nombre à l'adr adress
void writeIntIntoEEPROM(int address, int number)
{
  Serial.println("ECRITURE DANS L EEPROM");
  byte byte1 = number >> 8;
  byte byte2 = number & 0xFF;
  EEPROM.write(address, byte1);
  EEPROM.write(address + 1, byte2);
}

// fonction qui lit dans eeprom un nombre à l'adr adress
int readIntFromEEPROM(int address)
{
  Serial.println("LECTURE DE L EEPROM");
  byte byte1 = EEPROM.read(address);
  byte byte2 = EEPROM.read(address + 1);
  return (byte1 << 8) + byte2;
}

//1 si porte ouverte 0 sinon
int EtatPorte(void) {
  return digitalRead(pinPorte);
}


//1 si porte lettre 0 sinon
int EtatCapteurLumiere(void) {
  return digitalRead(pinCapteurLumiere);
}
