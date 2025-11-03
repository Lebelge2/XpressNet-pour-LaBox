/*
   LaBox Project
   XpressNet part

   @Author : lebelge2@yahoo.fr
   @Organization : Locoduino.org
*/
//=========================================== XpressNetESP V.4 Full-Duplex=====================================================
//
// Dernière modif: 03-11-25

#include <Arduino.h>
#include <driver/uart.h>
#include "DCC.h"

#ifdef ENABLE_XPRESSNET
#include "XpressNetESP.h"
#include "TrackManager.h"
#include "DCCWaveform.h"
#include "DCCEXParser.h"
#include "hmi.h"
#include "EXComm.h"
#include "EXCommItems.h"
#include "LaboxModes.h"

int OffsetAdress = 1;   // Offset numéro de décodeur de fonctions
int OffsetPort = 0;     // Offset numéro de sortie (Décodeur de fonctions)

// Adresses d'appel périphériques:                p + 0x40 + 000x xxxx
uint8_t UserOps[35] = {68, 65, 66, 195, 68, 197, 198, 71, 75, 72, 201, 202, 75, 204, 77, 78, 210, 207, 80, 209, 210, 83, 212, 85, 89, 86, 215, 216, 89, 90, 219, 92, 221, 222, 95};
// Adresses de réponses aux périphériques:        p + 0x60 + 000x xxxx
uint8_t  DirectedOps[35] = {228, 225, 226, 99, 228, 101, 102, 231, 235, 232, 105, 106, 235, 108, 237, 238, 114, 111, 240, 113, 114, 143, 116, 245, 249, 246, 119, 120, 249, 250, 123, 252, 125, 126, 255};

byte BufXpress[16];                     // Buffer reception datas XpressNet
uint8_t Railpower;
unsigned long currentTime = 0;
unsigned long previousTime = 0;
int Bi;                                // Buffer Index
int Ti;
//int t;
uint8_t xpSpeed;
bool dir;
bool Bit9;
int Xor;                               // Xor
int xpAdr;
uint8_t Gr1, Gr2, Gr3, Gr4, Gr5; // Groupes fonctions
uint8_t fonction;
unsigned long xpRegFonc;
int rxPin, txPin, dirPin;
int8_t xpCvValue = 0;
int8_t xpCvAddress = 0;
//--------------------------------------------------------------------------------
XPressNetESP::XPressNetESP(int inRxPin, int inTxPin, int inDirPin) : EXCommItem("XPressNetESP") {
  rxPin = inRxPin;
  txPin = inTxPin;
  dirPin = inDirPin;
  this->MainTrackEnabled = true;
  this->ProgTrackEnabled = true;
  this->AlwaysLoop = true;
}
//------------------------------------------------------------------------------------------
bool XPressNetESP::begin() {
  Serial2.begin(62500, SERIAL_8N1, rxPin, txPin);               // Connecté au port série2
  pinMode(dirPin, OUTPUT);
  DIAG(F("[XPRESSNET] Serial2 Txd:%d   Rxd:%d   Dir:%d"), txPin, rxPin, dirPin);
  XPressNetESP::setPower(csEmergencyStop);
  return true;
}
//---------------------------------------------------------------------------------
bool XPressNetESP::loop() {
  currentTime = micros();
  if (currentTime < (previousTime + 3000))
    return false;
  previousTime = currentTime;
  XPressNetESP::Update();
  return true;
}
//-------------------------------------------------------------------
void XPressNetESP::Update() {
  Ti++;                                           // Adresse périphérique suivante
  if (Ti > 34)  Ti = 0;                           // Plage d'adresse
  digitalWrite(dirPin, HIGH);                     // Dir Haut
  Serial2.write(UserOps[Ti]);                     // Envoi adresse au périphérique (A = adresse, P = Parité.   P10A AAAA)
  delayMicroseconds(150);                         // Attendre envoi complet.
  digitalWrite(dirPin, LOW);                      // Dir Bas
  for (int n = 0; n < 80; n++) {                  // Le périphérique a une fenêtre de 110 µs pour répondre.
    if (digitalRead(rxPin) == LOW) goto A;        // Attend réponse du périphérique (Start Bit, niveau bas pin rx)
    delayMicroseconds(1);
  }
  // Marqueur();                                   // Pour analyseur logique
  return ;                                         // Temps dépassé, retour. (Périférique suivant)
A:                                                 // Réception en cours...
  for (int n = 0; n < 100; n++) {                      // 100 ms
    if (Serial2.available()) goto B;               // Attend datas disponibles
    delay(1);
  }
  //Marqueur();                                    // Pour analyseur logique
  return ;                                         // Temps dépassé, retour. (Périférique suivant)
B:                                                 // Datas disponibles
  //Marqueur();
  Bi = 0;                                          // Buffer Index 0
  Xor = 0;
  while (Serial2.available() > 0) {                // Lire tous les Datas
    BufXpress[Bi] = Serial2.read();                // Lecture et mémorisation des datas reçu
    delayMicroseconds(200);
    Xor ^= BufXpress[Bi++];                        // Xor des datas
  }
  if (Xor != 0) {                                  // Si pas d'erreur, Xor = 0
    DIAG(F("[XPRESSNET] Wrong checksum"));         // Mauvais checksum, retour
    //Serial.println("Mauvais checksum");            // Mauvais checksum, retour
    return ;
  }
  for (int n = 0; n < 32; n += 8) {                // Pour adressage prioritaire,
    UserOps[n] = UserOps[Ti];                      // 4x plus rapide.
    DirectedOps[n] = DirectedOps[Ti];
  }
  Decodage();                                      // Décodage de la trame reçue
}
//-------------------------------------------------------------------
void xpCvValueCallback(int16_t inValue) {
  if (inValue >= 0)
    stateCV = StateCV::Reading_OK;
  else
    stateCV = StateCV::Reading_FAIL;
  xpCvValue = inValue;
}
//-----------------------------------------------------------------------
void XpressCvWriteValueCallback(int16_t inValue) {
  if (inValue >= 0)
    stateCV = StateCV::Written_OK;
  else
    stateCV = StateCV::Written_FAIL;
}
//------------------------------------------------------------
void XPressNetESP::Decodage() {
  if (DIAG_XPNET) {
    Serial.print("=> ");
    for (int n = 0; n < Bi; n++) {                         // DEBUG
      if (BufXpress[n] < 10) Serial.print("0");
      Serial.print(BufXpress[n], HEX);
      Serial.print(" ");
    }
    Serial.println();
  }
  Bit9 = false;
  switch (BufXpress[0]) {
    case 0xE6:  {                                        //   POM CV write MultiMaus
        switch (BufXpress[1]) {
          case 0x30:
            uint16_t Adr = ((BufXpress[2] & 0x3F) << 8) + BufXpress[3];
            uint16_t CVAdr = (((BufXpress[4] & B11) << 8) + BufXpress[5]);
            if ((BufXpress[4] & 0xFC) == 0xEC)  {                              //set byte
              DCC::writeCVByteMain(Adr, CVAdr, BufXpress[6]);
            }
            if ((BufXpress[4] & 0xFC) == 0xE8)  {                              //set byte
              byte bitNum = BufXpress[6] & B00000011 << 8;
              bool value = BufXpress[6] & B00000100;
              DCC::writeCVBitMain(Adr, CVAdr, bitNum, value);
            }
            break;
        }
      } break;
    case 0xE4:
      xpAdr = word(BufXpress[2] & 0x3F, BufXpress[3]);
      dir = BufXpress[4] & 0x80;
      xpSpeed = BufXpress[4];
      fonction = BufXpress[4];
      switch (BufXpress[1]) {
        case 0x10:                                         // Contrôle vitesse loco. 14 crans
        case 0x11:                                         // Contrôle vitesse loco. 27 crans
        case 0x12:                                         // Contrôle vitesse loco. 28 crans
        case 0x13:                                         // Contrôle vitesse loco. 128 crans
          DCC::setThrottle(xpAdr, xpSpeed, dir);
          break;
        case 0x20:                                // Contrôle F0 à F4 loco
          fonction = (fonction & 0b00010000) >> 4 | (fonction & 0b00001111) << 1 ;      // F0 F4 F3 F2 F1   moveBit4to0  F4 F3 F2 F1 F0
          XPressNetESP::Fonction(5, 0); break;             // Contrôle F0 à F4 loco
        case 0x21: XPressNetESP::Fonction(4, 5); break;    // Contrôle F5 à F8 loco
        case 0x22: XPressNetESP::Fonction(4, 9); break;    // Contrôle F9 à F12 loco
        case 0x23: XPressNetESP::Fonction(8, 13); break;   // Contrôle F13 à F20 loco
        case 0x28: XPressNetESP::Fonction(5, 21); break;   // Contrôle F21 à F28 loco
      }
      break;
    case 0xE3:
      switch (BufXpress[1]) {
        case 0x00: {
            xpAdr = word(BufXpress[2] & 0x3F, BufXpress[3]);
            xpSpeed = DCC::getThrottleSpeed(xpAdr);
            XPressNetESP::getfonction(xpAdr);
            uint8_t LocoInfo[] = {DirectedOps[Ti], 0xE4, 0x04, xpSpeed, Gr1, Gr2, 0x00 };   // 0xE4  0x04  (F0 à F9)
            XPressNetESP:: getXOR(LocoInfo, 7);
            XNetsend (LocoInfo, 7);
          } break;
        case 0x08:                   // 3.6  Requête d'état de fonction F13-F28
        case 0x09:
          Serial.println("Non supporté 0xE3 0x09");
          break;
        case 0xF0:                   // Multimaus
          XPressNetESP::notifyXNetgiveLocoMM(DirectedOps[Ti], word(BufXpress[2] & 0x3F, BufXpress[3])); break;
      }
      break;
    case 0x91:                                          // Requête de demande d’arrêt de d'urgence d’une locomotive adresse courte
      DCC::setThrottle(BufXpress[1] , 1, 1); break;
    case 0x92:                                          // Requête de demande d’arrêt de d'urgence d’une locomotive adresse longue
      xpAdr = word( BufXpress[2] & 0x1F,  BufXpress[3]);
      DCC::setThrottle(xpAdr, 1, 1); break;
    case 0x80:                                          // Arrêt urgence toutes loco. coupure électrique
      if (BufXpress[1] == 0x80) {
        Railpower = csEmergencyStop;
        notifyXNetPower(Railpower);
      }
      break;
    case 0x52: {                                         // Requête de commande à un décodeur d’accessoire.
        uint16_t Address((BufXpress[1]  << 2) | (BufXpress[2] & B110) >> 1);
        int data = BufXpress[2];
        Address = (Address >> 2) + OffsetAdress;                   // 00AAAAAA
        uint8_t port = ((data & 0x06) >> 1) + OffsetPort;          // 000000BB
        bool gate = bitRead(data, 0);                              // 0000000D
        bool OnOff =  bitRead(data, 3);                            // 0000d000
        DCC::setAccessory(Address, port, gate, OnOff);
        byte pos1 = bitRead(data, 2);
        byte pos2 = data & B11;
        if (pos2 == 0) data = 0x05;
        if (pos2 > 0) data = 0x06;
        if (pos2 == 3) data = 0x0A;
        if (pos1 == 1)
          bitSet(data, 4);
        Address -= 1;
        uint8_t TrntInfo[] = {DirectedOps[Ti], 0x42, 0x00, 0x00, 0x00 }; // Feedback (0x42,MOD,DATA,...,XOR) Accessory decoder information response
        TrntInfo[2] = Address;
        TrntInfo[3] = data;
        getXOR(TrntInfo, 5);
        XNetsend(TrntInfo, 5);
      } break;
    case 0x42: {                                          // 0x42 Demande informations sur le décodeur d'accessoires
        int Address = ((BufXpress[1] << 2) | (BufXpress[2] & B110) >> 1);
        int data = BufXpress[2];
        Address = (Address >> 2); // + OffsetAdress;
        Address -= 1;
        uint8_t TrntInfo[] = {DirectedOps[Ti], 0x42, 0x00, 0x00, 0x00 }; // Feedback (0x42,MOD,DATA,...,XOR) Accessory decoder information response
        TrntInfo[2] = Address;
        TrntInfo[3] = data;
        getXOR(TrntInfo, 5);
        XNetsend(TrntInfo, 5);
      } break;
    case 0x23:                                           // Requête d'écriture CV en Mode Direct (CV mode (CV=1..256))
      if (BufXpress[1] == 0x12) {                        // Register Mode write request (Register Mode)
        Serial.println("Non supporté 0x23 0x12");
      }
      if (BufXpress[1] == 0x16) {                        // Direct Mode CV write request (CV mode)
        xpCvAddress = BufXpress[2];
        xpCvValue = BufXpress[3];
        if (DIAG_XPNET)
          DIAG(F("[XPRESSNET] Write CV %d  value:%d."), xpCvAddress, xpCvValue);               // DEBUG
        void (*ptr)(int16_t) = &XpressCvWriteValueCallback;
        DCC::writeCVByte(xpCvAddress, xpCvValue, ptr);
        stateCV = Writing;
      }
      if (BufXpress[1] == 0x17) {                        // Paged Mode write request(Paged mode)
        Serial.println("Non supporté 0x23 0x17");
      }
      break;
    case 0x22:                                           // Requête de lecture CV
      if (BufXpress[1] == 0x11) {                        //Register Mode read request
        Serial.println("Non supporté 0x22 0x11");
      }
      if (BufXpress[1] == 0x14) {                        //Paged Mode read request
        Serial.println("Non supporté 0x22 0x14");
      }
      if (BufXpress[1] == 0x15) {                        //Direct Mode CV read request
        Serial.println("Demande Cv");                    // DEBUG
        xpCvAddress = BufXpress[2];
        xpCvValue = 0;
        if (DIAG_XPNET)
          DIAG(F("[XPRESSNET] Read CV %d."), xpCvAddress);               // DEBUG
        void (*ptr)(int16_t) = &xpCvValueCallback;
        DCC::readCV(xpCvAddress, ptr);
        stateCV = Reading;
      }
      break;
    case 0x21:
      switch (BufXpress[1] ) {
        case 0x80:                                             // Requête de commande d’arrêt d’urgence
          Railpower = csTrackVoltageOff;
          XPressNetESP::notifyXNetPower(Railpower);             // 0x60, 0x61, 0x00, 0x61     Broadcast Track power OFF
          break;
        case 0x81:                                              // Requête de commande de reprise des opérations
          Railpower = csNormal;
          XPressNetESP::notifyXNetPower(Railpower);             // 0x60, 0x61, 0x01, 0x60      Broadcast Normal operation
          break;
        case 0x24: {                                            // Demande de status LaBox (0x21 0x24 0x05)
            uint8_t   sendStatus[] = { DirectedOps[Ti], 0x62, 0x22, Railpower, 0x00 };
            XPressNetESP::getXOR(sendStatus, 5);
            XNetsend(sendStatus, 5);
            Ti--;
          } break;
        case 0x21: {                                              // Demande Version Centrale XpressNet    (0x21 0x21 0x00)
            uint8_t  sendVersion[] = { DirectedOps[Ti], 0x63, 0x21, XNetVersion, XNetID, 0x00 };   //63 21 36 0 74
            XPressNetESP::getXOR(sendVersion, 6);
            XNetsend(sendVersion, 6);
            Ti--;
          } break;
        case 0x10:                                         // Lecture du CV Service mode
          if (stateCV == StateCV::Reading_OK) {
            stateCV = StateCV::Ready;
            if (DIAG_XPNET)
              DIAG(F("[XPRESSNET] Reading answer Cv."), xpCvAddress, xpCvValue);               // DEBUG
            uint8_t  sendReading_OK[] = { DirectedOps[Ti], 0x63, 0x14, xpCvAddress, xpCvValue, 0x00 };   //63 14 ou 15 ?
            XPressNetESP::getXOR(sendReading_OK, 6);
            XNetsend(sendReading_OK, 6);
          }
          else if (stateCV == StateCV::Reading) {
            DIAG(F("[XPRESSNET] Reading..."));               // DEBUG
            uint8_t  sendReading[] = { DirectedOps[Ti], 0x61, 0x1F, 0x00 };   //63 1F
            XPressNetESP::getXOR(sendReading, 4);
            XNetsend(sendReading, 4);
          }
          else {

          }
          break;
      }
      break;
    default:
      unknown();                                          // Commande non supportée. {0x61, 0x82, 0xE3}
      DIAG(F("[XPRESSNET] Commande non supportée."));
      break;
  }
}
//------------------------------------------------------
void XPressNetESP::notifyXNetPower(uint8_t State) {
  if (State == 0)
    TrackManager::setMainPower(POWERMODE::ON);      // Requête de commande de reprise des opérations
  else if (State == 1)
    TrackManager::setMainPower(POWERMODE::OFF);     // Requête de commande d’arrêt d’urgence
  else if (State == 2)
    TrackManager::setMainPower(POWERMODE::OFF);     // Requête de commande court circuit
  //else                                              // ServiceMode 0x08
  XPressNetESP::setPower(State);                        // Envoi l'état de la centrale aux périphériques
}
//-------------------------------------------------------
void XPressNetESP::setPower(uint8_t Power) {
  switch (Power) {
    case csNormal: {
        uint8_t NormalPower[] = { GENERAL_BROADCAST, 0x61, 0x01, 0x60 };
        XNetsend(NormalPower, 4);
        break;
      }
    case csEmergencyStop: {
        uint8_t EStopPower[] = { GENERAL_BROADCAST, 0x81, 0x00, 0x81 };
        XNetsend(EStopPower, 4);
        break;
      }
    case csTrackVoltageOff: {
        uint8_t OffPower[] = { GENERAL_BROADCAST, 0x61, 0x00, 0x61 };
        XNetsend(OffPower, 4);
        break;
      }
    case csShortCircuit: {
        uint8_t ShortPower[] = { GENERAL_BROADCAST, 0x61, 0x08, 0x69 };
        XNetsend(ShortPower, 4);
        break;
      }
    case csServiceMode: {
        uint8_t ServicePower[] = { GENERAL_BROADCAST, 0x61, 0x02, 0x63 };
        XNetsend(ServicePower, 4);
        break;
      }
  }

  Railpower = Power;  //Save new Power State
}
//--------------------------------------------------------
void XPressNetESP::notifyXNetgiveLocoMM(uint8_t UserOps, word Address) {       // Requête d’information d'une locomotive (Multimaus) E3 F0
  uint8_t Speed = DCC::getThrottleSpeed(Address);
  bool dir = DCC::getThrottleDirection(Address);
  if (dir)
    Speed += 0x80;
  XPressNetESP::getfonction(Address);
  SetLocoInfoMM(UserOps, 4, Speed, 0, Gr1, Gr2, Gr3);
}
//-----------------------------------------------------------------------------
void XPressNetESP::SetLocoInfoMM(uint8_t UserOps, uint8_t Steps, uint8_t Speed, uint8_t F0, uint8_t F1, uint8_t F2, uint8_t F3) {
  //E7 0x04=128 Steps||0x0C=128+Busy (Dir,Speed) (F0,F4,F3,F2,F1) F5-F12 F13-F20 0x00 0x00 CRC
  byte v = Speed;
  if (Steps == Loco28 || Steps == Loco27) {
    v = (Speed & 0x0F) << 1;                                   //Speed Bit
    v |= (Speed >> 4) & 0x01;                                  //Addition Speed Bit
    v |= 0x80 & Speed;                                         //Dir
  }
  uint8_t LocoInfo[] = {UserOps,  0xE7, Steps, v, 0x20, F1, F2, F3, 0x00, 0x00 };
  LocoInfo[4] |= F0;
  XPressNetESP::getXOR(LocoInfo, 10);
  XNetsend(LocoInfo, 10); // XNetsend
}
//-----------------------------------------------------
void  XPressNetESP::Fonction(int a, int b) {                  // 28 fonctions
  for (int i = 0; i < a; i++) {
    if (bitRead(xpRegFonc, i + b) !=  bitRead(fonction, i)) {
      DCC::setFn(xpAdr, i + b, bitRead(fonction, i));
      bitWrite( xpRegFonc, i + b, bitRead(fonction, i));
    }
  }
}
//-------------------------------------------------
void XPressNetESP::getXOR (uint8_t *data, byte length) {       // calculate the XOR
  byte XOR = 0x00;
  data++;
  for (int i = 0; i < (length - 2); i++) {
    XOR = XOR ^ *data;
    data++;
  }
  *data = XOR;
}
//--------------------------------------------------------------------------------------------
void XPressNetESP::XNetsend(unsigned char *dataString, byte byteCount) {
  gpio_matrix_out(txPin, 0x100, false, false);           // Déconnecte la pin Tx du module UART2
  digitalWrite (dirPin, HIGH);
  Bit9 = false;
  for (int i = 0; i < byteCount; i++) {
    Tx9(*dataString);
    dataString ++;
  }
  digitalWrite (dirPin, LOW);
  gpio_matrix_out(txPin, U2TXD_OUT_IDX, false, false);   // Reconnecte la pin Tx au module UART2
  //pinMode(txPin, IN
}
//--------------------------------------------------------------------------------------------
void XPressNetESP::Tx9(int dTx) {  // Envoi d'un octet 62500 Bauds   9 bits  1 stop
  digitalWrite(txPin, LOW);        // Bit Start
  delayMicroseconds(15);           // Largeur 16µs
  cli();
  for (int m = 0; m < 8; m++) {    // 8 bits
    if (bitRead(dTx, m))           // Test état du bit
      digitalWrite(txPin, HIGH);   // Etat haut
    else                           // Si non
      digitalWrite(txPin, LOW);    // Etat bas
    delayMicroseconds(15);         // Largeur 16µs
  }                                // Boucle
  sei();
  if (Bit9 == false)               // Bit 9
    digitalWrite(txPin, HIGH);     // Bit 9 Haut
  else
    digitalWrite(txPin, LOW);      // Bit 9 Bas
  delayMicroseconds(15);           // 16µs
  digitalWrite(txPin, HIGH);       // 1 Bit Stop
  delayMicroseconds(15);           // 16µs
  Bit9 = true;
}
//---------------------------------------------
void XPressNetESP::getfonction(uint8_t adr) {
  uint32_t FonctionMap = DCC::getFunctionMap(adr);
  uint8_t moveBit0to4 = FonctionMap & 0x1F ;     // 5 bits
  Gr1 = (moveBit0to4 & 0b00000001) << 4 | (moveBit0to4 & 0b00011111) >> 1 ;   // F4 F3 F2 F1 F0    moveBit0to4  F0 F4 F3 F2 F1
  //Gr1 = (FonctionMap & 0x1F) ;                 // Groupe 1: 0 0 0 F0 F4 F3 F2 F1              bit0 à bit4
  Gr2 = (FonctionMap & 0xF00) >> 8;              // Groupe 2: 0 0 0 0 F8 F7 F6 F5               bit8 à bit11
  Gr3 = (FonctionMap & 0xF000) >> 12;            // Groupe 3: 0 0 0 0 F12 F11 F10 F9            bit12 à bit15
  Gr4 = (FonctionMap & 0xFF0000) >> 16;          // Groupe 4: F20 F19 F18 F17 F16 F15 F14 F13   bit16 à bit23
  Gr5 = (FonctionMap & 0xFF000000) >> 24;        // Groupe 5: F28 F27 F26 F25 F24 F23 F22 F21   bit24 à bit31
  // Serial.print("Groupe 1 ");Serial.println(Gr1,BIN);
}
//--------------------------------------------------------------
void XPressNetESP::unknown(void) {
  uint8_t NotIn[] = {DirectedOps[Ti], 0x61, 0x82, 0xE3 };
  XNetsend(NotIn, 4);
}
//--------------------------------------------------------------
void XPressNetESP::Marqueur(int a) {       // Pour analyseur logique
  digitalWrite(dirPin, HIGH);              // Haut __-
  delayMicroseconds(a);
  digitalWrite(dirPin, LOW);               // Bas  -__
}
#endif
