









/*
   _____     ____      __    _    ____    _    _   _     _ 
  |  __ \   / __ \    |  \  | |  / __ \  | |  | | | |   | |
  | |__| | | /  \_|   | . \ | | / /  \ \ | |  | |  \ \ / /
  |  _  /  | |   _    | |\ \| | | |__| | | |  | |   \ ' /
  | | \ \  | \__/ |   | | \ ' | |  __  |  \ \/ /     | |
  |_|  |_|  \____/    |_|  \__| |_|  |_|   \__/      |_|(C)2020

                http://p.loussouarn.free.fr

ENGLISH:
========

Aim of the sketch:
=================
This sketch demontrates how to command a Futaba S3003 servo which has been modified for continuous rotation in both directions.
See tuto for the servo modification: http://www.youtube.com/watch?v=SK8mhnEzcvY
The idea, is to use this kind of modified servo as a winch: push/stop/pull.
This sketch is intended to be run on an UNO, LEONARDO or MEGA arduino which have native USB serial console interface.

Behaviour of the servo:
======================

  Rotation speed
       | _________           _________
       |          \         /
       |           \       /
       |   Dir 1    \     /  Dir 2
       |             \   /
       |              \ /
       '_______________V________________ Pulse width(us)
       1000     1400 1500 1600     2000
         <-----------Stop----------->
         
Pulse width < 1500us --> The Servo rotates in one direction       (with a kind of proportionnality from 1500 down to 1400 us)
Pulse width = 1500us --> Servo is stopped                         (the pulse width shall be carefully found: '-' and '+' keys will help you to find the exact value)
Pulse width > 1500us --> The Servo rotates in the other direction (with a kind of proportionnality from 1500 to 1600 us)

The direction and the rotation speed are commanded by using the '-' and '+' keys of gthe keyboard
To make it more easier, (to avoid pressing "enter" after each press on '-' or '+'), do not use the built-in Serial Console (close it)
and open an external terminal such as HyperTerminal, TeraTerm, GtkTerm, CoolTerm.


FRANCAIS:
========

But du sketch:
=============
Ce sketch demontre comment commander un servo Futaba S3003 modifie pour une rotation continue dans les 2 sens.
Voir tuto pour la modification du servo: http://www.youtube.com/watch?v=SK8mhnEzcvY
L'idee est d'en faire, par exemple, des treuils filer/stop/virer pour modelisme naval pilotes par un simple ATtiny85.
Ce sketch est prévu pour tourner sur un arduino UNO, LEONARDO ou MEGA qui disposent d'une interface USB/Serie native pour la console.

Comportement du servo:
=====================

    Vitesse 
       | _________           _________
       |          \         /
       |           \       /
       |  Sens 1    \     /  Sens 2
       |             \   /
       |              \ /
       '_______________V________________ Impulsion(us)
       1000     1400 1500 1600     2000
         <-----------Stop----------->
         
Impulsion < 1500us --> Rotation du Servo dans un sens      (avec une certaine proportionnalite de 1500 a 1400 us)
Impulsion = 1500us --> Arret du Servo                      (la largeur d'impulsion doit être trouvée avec précision: utilisez les touches '-' et '+' du clavier pour trouver la valeur exacte)
Impulsion > 1500us --> Rotation du Servo dans l'autre sens (avec une certaine proportionnalite de 1500 a 1600 us)

Le sens et la vitesse se commandent par les touches '-' et '+' du clavier
Pour plus de facilite (pour eviter de presser "enter" apres chaque appui sur '-' ou '+'), fermer la serial console
et ouvrir un terminal externe (HyperTerminal, TeraTerm, GtkTerm, CoolTerm).

*/

#include <SoftRcPulseOut.h>
#include <Rcul.h> /* Necessary for last version of SoftRcPulseOut */
#include <Wire.h>
#include <FastPID.h>

/* /!\ Do NOT touch below /!\ */
#define PWM  0
#define CPPM 1
/* /!\ Do NOT touch above /!\ */

/* vvv Sketch configuration vvv */
#define RC_MODE         PWM /* <-- Chose here CPPM to Rx CPPM frame or PPM to Rx a receiver channel output */
#define DATA_RC_CHANNEL 7    /* If Mode = CPPM, define here the channel to Rx */
/* ^^^ Sketch configuration ^^^ */

#if (RC_MODE == CPPM)
#include <TinyCppmReader.h>
#else
#if (RC_MODE == PWM)
#include <SoftRcPulseIn.h>
#else
#error Define RC_MODE = CPPM (#define RC_MODE CPPM) or RC_MODE = PPM (#define RC_MODE PPM)
#endif
#endif

#include <RcRxSerial.h>

/* I2C Pro Micro
  SCL = 3
  SDA = 2
*/

/* I2C Pro Mini
  SCL = A5
  SDA = A4
*/

#define xPPM_INPUT_PIN       9// 0 bloque le port série

#ifndef RC_RX_SERIAL_SYNCH
#define RC_RX_SERIAL_SYNCH  RC_RX_SERIAL_NO_FILTER
#endif

#if (RC_MODE == CPPM)
TinyCppmReader TinyCppmReader;
RcRxSerial MyRcRxSerial(&TinyCppmReader, RC_RX_SERIAL_SYNCH,   DATA_RC_CHANNEL); /* Create a Rx serial port on the channel#5 of the TinyPpmReader */
#else
static SoftRcPulseIn PwmRcSignal;
RcRxSerial MyRcRxSerial(&PwmRcSignal, RC_RX_SERIAL_SYNCH,   DATA_RC_CHANNEL); /* Create a Rx serial port on the channel#5 of the TinyPpmReader */
#endif

#define INACTIVITY_MS        2000UL
#define RX_MSG_LEN_MAX       6

static uint8_t RxMsg[RX_MSG_LEN_MAX];
void PrintBin(uint8_t *Buf, uint8_t BufSize, uint8_t RemainingNibble = 0);
void PrintByteBin(uint8_t Byte, uint8_t RemainingNibble = 0);
void PrintBinln(uint8_t *Buf, uint8_t BufSize, uint8_t RemainingNibble = 0);

#define I2C_ADDR_OFFSET      0 // Shall match with the offset of the sensor
#define I2C_SLAVE_7B_ADDR    (0x0C + I2C_ADDR_OFFSET)
#define I2C_REG_ADDR         0x20

#define UNIT_FOR_360_DEG     4096UL
#define UNIT_PER_DEG(Deg)    (((Deg) * UNIT_FOR_360_DEG) / 360UL)

SoftRcPulseOut GisServo;
SoftRcPulseOut ThrServo;

#define SERIAL_BAUD_RATE     115200 /* Use the same data rate in the terminal serial / Utiliser le meme debit dans le terminal serial */

#define GIS_SERVO_PIN        4      /* Set here the pin use to control the servo / déclarer ici la broche qui controle le servo */
#define THR_SERVO_PIN        6

#define GIS_NEUTRAL_US       1305//1500
#define GIS_EXC_MAX_US       250

#define THR_MONO_DIR_STOP_US 1000
#define THR_BI_DIR_STOP_US   1500

#define THR_STOP_US          THR_MONO_DIR_STOP_US /* Chose here the type of ESC for Throttle */

#define THR_STEP_US          (4000 / THR_STOP_US)/* 2 for MONO-DIR, 4 for BI-DIR */

#define THR_ACCEL_STEP_US    16

int16_t  GisExc_us0 = 0, GisExc_us = 0; /* Excursion will go from -250us to +250us / L'excursion ira de -250us a +250us */
int16_t  ThrExc_us = 0;

#define EXC_TO_US(Exc)       (GIS_NEUTRAL_US + (Exc)) 
#define US_TO_EXC(us)        ((us) - GIS_NEUTRAL_US) 

#define CfgSerial            Serial
#define CARRIAGE_RETURN      0x0D /* '\r' = 0x0D (code ASCII) */
#define RUB_OUT              0x08
#define CFG_MSG_MAX_LENGTH   22 /* Longest Rx or Tx Message */
static char                  CfgMessage[CFG_MSG_MAX_LENGTH + 1];/* + 1 pour fin de chaine */

/* http://www.ferdinandpiette.com/blog/2011/08/implementer-un-pid-sans-faire-de-calculs/ */

#if 1
float Kp=0.5/*0.175*/; // P = 0.5 x Kp quand atteinte de la limite de l'ocillation continue (0.41)
float Ki=0.190; // I reduit l'erreur statique
float Kd=0.015; // D calme les oscillations résiduelles
#else
/* Tres bonne base */
float Kp=0.205; // P = 0.5 x Kp quand atteinte de la limite de l'ocillation continue (0.41)
float Ki=0.150; // I reduit l'erreur statique
float Kd=0.010; // D calme les oscillations résiduelles
#endif

float Hz=50;

#define PID_BIT_NB   6
#define PID_MAX_OUT_VAL ((1 << (PID_BIT_NB - 1)) - 1)
int  output_bits = PID_BIT_NB; // -256 to +255 ==> signed 9 bit value
bool output_signed = true;

FastPID myPID(Kp, Ki, Kd, Hz, output_bits, output_signed);

void setup()
{
  //myPID.setOutputRange(-PID_MAX_OUT_VAL*2/3, +PID_MAX_OUT_VAL*2/3);
  myPID.setOutputRange(-PID_MAX_OUT_VAL*3/3, +PID_MAX_OUT_VAL*3/3);
  
  GisServo.attach(GIS_SERVO_PIN);
  GisServo.write_us(EXC_TO_US(GisExc_us));
  
  ThrServo.attach(THR_SERVO_PIN);
  ThrServo.write_us(THR_STOP_US);
  
  Serial.begin(SERIAL_BAUD_RATE);
  while(!Serial); /* Only needed in the Leonardo Arduino: can be kept on other arduino / Seulement necessaire pour l'arduino Leonardo, peut rester pour les autres arduinos */
#if (RC_MODE == CPPM)
  Serial.print(F("Sketch configured to spy X-Any message on channel "));Serial.print(DATA_RC_CHANNEL);Serial.println(F(" of CPPM signal of the transmitter (Connect CPPM signal to Arduino pin0)"));
  TinyCppmReader.attach(xPPM_INPUT_PIN); /* Attach MyCppmReader to xPPM_INPUT_PIN pin */
#else
  Serial.println(F("Sketch configured to spy X-Any message on PWM signal (output channel) of the receiver (Connect PWM signal to Arduino pin0)"));
  PwmRcSignal.attach(xPPM_INPUT_PIN);   /* Attach PwmRcSignal to xPPM_INPUT_PIN pin */
#endif
  Serial.println(F("Test I2C slave: 0-360° angular sensor"));
  Wire.begin(); // join i2c bus (address optional for master)
  Serial.print(F("UNIT_PER_DEG(180)="));Serial.println(UNIT_PER_DEG(180));
  Serial.print(F("UNIT_PER_DEG(360)="));Serial.println(UNIT_PER_DEG(360));
  Serial.println(PID_MAX_OUT_VAL);
}
typedef struct{
  uint8_t AddUs[4];
}UsUStep_t;
                    /*  0 us offset,  1 us offset,  2 us offset,  3 us offset */
UsUStep_t UsUStep[] = {{0, 0, 0, 0}, {0, 0, 0, 4}, {0, 4, 0, 4}, {4, 4, 4, 0}};
uint8_t  RemainUs = 0;
uint8_t  RemainIdx = 0;
uint16_t OrderAngle = 0;
uint16_t OrderThrottle = 0;

void loop()
{
  static uint8_t PulseCnt = 0;
  uint8_t        RxMsgAttr, RxNibbleLen, RxMsgChecksum, ComputedChecksum, *BytePtr;
  char           RxChar;
  uint16_t       CurrentAngle, PulseUs;
  int16_t        AzError;
  int16_t        ThrExcDiff_us;
  static uint32_t LastActivityStartMs = millis(), LastMsgStartMs = millis();
#if 0
  if(Serial.available())
  {
    RxChar = Serial.read();
    
    switch(RxChar)
    {
      case '-':
      if(GisExc_us > -GIS_EXC_MAX_US) GisExc_us--; /* Using '-' key, reduce the excursion (deviation from Neutral position (1500us)) */
      break;
      
      case'+':
      if(GisExc_us < +GIS_EXC_MAX_US) GisExc_us++; /* Using '+' key, increase the excursion (deviation from Neutral position (1500us)) */
      break;
      
      default:
      /* Ignore other caracters */
      break;
    }
    
    if((RxChar == '-') || (RxChar == '+'))
    {
      RemainUs = EXC_TO_US(GisExc_us) % 4;
      Serial.print(F("Pulse = "));Serial.print(EXC_TO_US(GisExc_us));Serial.print(F(" us -> Angle = "));Serial.print(GisExc_us/10);Serial.println(F(" deg")); /* For a display from -90° to +90° with a stop at 0° */
      /* Here, to compute the displayed angle, we assume a variation of 10us generates a variation of 1 degree */
    }
  }
#else
  /* Get Order */
  if(CfgMessageAvailable() >= 0)
  {
    OrderAngle = atoi(CfgMessage);
    //Serial.println("J'ai un message");
  }
#endif
    if((RxMsgAttr = MyRcRxSerial.msgAvailable((char *)&RxMsg, RX_MSG_LEN_MAX)))
    {
      //Serial.println("RxMsgAttr valide");
//      if(RxMsgChecksumIsValid((char *)&RxMsg, RxMsgAttr))
      if(RcRxSerial::msgChecksumIsValid((char *)&RxMsg, RxMsgAttr))
      {
        //Serial.println("msgChecksumIsValid valide");
        //OK, on peut faire confiance au contenu du message
        //LastActivityStartMs = millis();
        //Serial.print(F("X-Any Msg[TotNbl="));Serial.print(RxNibbleLen);Serial.print(F("]="));PrintBin((uint8_t *)&RxMsg, (RxNibbleLen / 2) - 1, RxNibbleLen & 1);Serial.print(F(" T="));Serial.println(millis() - LastMsgStartMs);
        OrderAngle    = GetAz(RxMsg);
        OrderThrottle = GetThr(RxMsg);
        //Serial.print(F("Az="));Serial.print(OrderAngle);Serial.print(F(" Gaz="));Serial.println(GetThr(RxMsg));
        //LastMsgStartMs = millis();
      }
      else
      {
        Serial.println(F("X-Any Length or Checksum error!"));
      }
    }
  
  if(SoftRcPulseOut::refresh())
  {
    /* Update Az Servo with 1 µs pseudo resolution */
    PulseUs = (EXC_TO_US(GisExc_us) & ~0x0003) + UsUStep[RemainUs].AddUs[RemainIdx];
    //Serial.print(F("PulseUs="));Serial.println(PulseUs);
    GisServo.write_us(PulseUs); /* Use write_us() rather than write() to get more accuracy */
#if 0
Serial.print(EXC_TO_US(GisExc_us));Serial.print(" ");
Serial.print(RemainUs);Serial.print(" ");
Serial.print(PulseUs);Serial.print(" ");
Serial.println(GisExc_us);//Serial.print(" ");
#endif
    RemainIdx++;
    if(RemainIdx >= 4) RemainIdx = 0;
    PulseCnt++;
    //CurrentAngle = 4095 - getCurrentAngle();//si engrenage ==> inversion
    CurrentAngle = getCurrentAngle();// si courrroie
    AzError = DeltaAngle(OrderAngle, CurrentAngle);
    Serial.print(F("Ord="));Serial.print(OrderAngle);
    Serial.print(F(" -> Cur="));Serial.print(CurrentAngle);Serial.print(F(" -> "));Serial.print(((float)CurrentAngle*360.0)/4096.0, 1);Serial.print(F("°"));
    Serial.print(F(" Err="));Serial.print(AzError);
    GisExc_us0 = myPID.step(0, +AzError);
    Serial.print(F(" Exc="));Serial.println(GisExc_us);
    if(!(PulseCnt % 4))
    {
      GisExc_us = GisExc_us0;
    }
    /* Throttle Management */
    ThrExcDiff_us = abs((int16_t)(OrderThrottle * THR_STEP_US) - ThrExc_us);
    if(ThrExcDiff_us > (THR_ACCEL_STEP_US / 2))
    {
      if((int16_t)(OrderThrottle * THR_STEP_US) > ThrExc_us)
      {
        ThrExc_us += THR_ACCEL_STEP_US;
        if(ThrExc_us > (255 * THR_STEP_US)) ThrExc_us = (255 * THR_STEP_US);
      }
      else
      {
        ThrExc_us -= THR_ACCEL_STEP_US;
        if(ThrExc_us < 0) ThrExc_us = 0;
      }
    }
    else ThrExc_us = OrderThrottle * THR_STEP_US;
//    Serial.print(F(" OrderThrottle="));Serial.println(OrderThrottle);Serial.print(F(" ThrExc="));Serial.println(ThrExc_us);
    ThrServo.write_us(THR_STOP_US + ThrExc_us);
  }
}
uint16_t GetAz(uint8_t *RxMsg)
{
  uint16_t Az;
  Az =(RxMsg[0] << 4) + (RxMsg[1] >> 4);
  return(Az);
}

uint16_t GetThr(uint8_t *RxMsg)
{
  uint8_t Thr;
  Thr = (RxMsg[1] << 4) + (RxMsg[2] >> 4);
  
  return(Thr);
}

uint8_t RxMsgChecksumIsValid(char *Msg, uint8_t MsgAttr)
{
  uint8_t RxMsgByteLen, RxMsgNibbleLen, Idx, RxMsgChecksum, ComputedChecksum = 0;

  if(MsgAttr & RC_RX_SERIAL_PENDING_NIBBLE_INDICATOR)
  {
    RxMsgByteLen   = (MsgAttr & ~RC_RX_SERIAL_PENDING_NIBBLE_INDICATOR);
    RxMsgNibbleLen = (RxMsgByteLen * 2) + 1;
  }
  else
  {
    RxMsgNibbleLen = (RxMsgByteLen * 2);
  }

  /* Extract received checksum */
  if(RxMsgNibbleLen & 1)
  {
    RxMsgChecksum = ( ((Msg[RxMsgByteLen - 1] & 0x0F) << 4) | ((Msg[RxMsgByteLen] & 0xF0) >> 4) );
  }
  else
  {
    RxMsgChecksum = Msg[RxMsgByteLen - 1];
  }
  /* Compute checksum */
  for(Idx = 0; Idx < RxMsgByteLen - 1; Idx++)
  {
    ComputedChecksum ^= Msg[Idx];
  }
  if(RxMsgNibbleLen & 1)
  {
    ComputedChecksum ^= (Msg[Idx] & 0xF0);
  }
  ComputedChecksum ^= 0x55;
  
  return(RxMsgChecksum == ComputedChecksum);
}

static int16_t DeltaAngle(uint16_t AngleOrder, uint16_t CurrentAngle)
{
  int16_t Order, Current, AzError;

  Order   = (int16_t)AngleOrder;
  Current = (int16_t)CurrentAngle;
  AzError = Order - Current;
  //Serial.print(F("Order="));Serial.print(Order);
  //Serial.print(F(" Current="));Serial.println(Current);
  //Serial.print(F(" AzError="));Serial.println(AzError);
  if(abs(AzError) > UNIT_PER_DEG(180))
  {
    if(AzError > 0) AzError -= UNIT_PER_DEG(360);
    else            AzError += UNIT_PER_DEG(360);
  }
//Serial.print(F(" AzError="));Serial.println(AzError);
  return(AzError);
}

static uint16_t getCurrentAngle(void)
{
  uint8_t  ValMsb, ValLsb;
  uint16_t Angle;

  Wire.beginTransmission(I2C_SLAVE_7B_ADDR); // transmit to slave device
  Wire.write(I2C_REG_ADDR);                  // sends one byte to select first register
  Wire.endTransmission();                    // stop transmitting

  while(Wire.requestFrom(I2C_SLAVE_7B_ADDR, 2) != 2); // Request 2 bytes from slave
  ValMsb = Wire.read(); // MSB first
  ValLsb = Wire.read(); // LSB
  Angle = (ValMsb << 8) +  ValLsb;
  Angle &= 0x0FFF;
  
  return(Angle);
}

static char CfgMessageAvailable(void)
{
  char Ret = -1;
  char RxChar;
  static uint8_t Idx = 0;

  if(CfgSerial.available() > 0)
  {
    RxChar = CfgSerial.read();
    switch(RxChar)
    {
      case CARRIAGE_RETURN: /* Si retour chariot: fin de message */
      CfgMessage[Idx] = 0;/* Remplace CR character par fin de chaine */
      Ret = Idx;
      Idx = 0; /* Re-positionne index pour prochain message */
      break;
            
      case RUB_OUT:
      if(Idx) Idx--;
      break;
            
      default:
      if(Idx < CFG_MSG_MAX_LENGTH)
      {
        CfgMessage[Idx] = RxChar;
        Idx++;
      }
      else Idx = 0; /* Re-positionne index pour prochain message */
      break;
    }
  }
  return(Ret); 
}

void PrintBin(uint8_t *Buf, uint8_t BufSize, uint8_t RemainingNibble/* = 0*/)
{
  uint8_t ByteIdx;
  for(ByteIdx =0; ByteIdx < BufSize; ByteIdx++)
  {
    if(ByteIdx) Serial.print(F("."));
    PrintByteBin(Buf[ByteIdx]);
  }
  if(RemainingNibble)
  {
    if(BufSize) Serial.print(F("."));
    PrintByteBin(Buf[ByteIdx], RemainingNibble);
  }
}

void PrintByteBin(uint8_t Byte, uint8_t RemainingNibble/* = 0*/)
{
  for(uint8_t Idx = 0; Idx < 8; Idx++)
  {
    Serial.print(bitRead(Byte, 7 - Idx));
    if(Idx == 3)
    {
      if(RemainingNibble) break;
      Serial.print(F("."));
    }
  }
}
