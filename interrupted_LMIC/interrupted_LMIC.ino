/*
   .code for blackout detection or case opening.
   TODO: make a board
*/

#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>
#include <LowPower.h>

#define interruptPin 2
volatile bool interrupted = false;
volatile bool door = 0;


static const PROGMEM u1_t NWKSKEY[16] = { 0xD3, 0xAC, 0xBE, 0x62, 0x04, 0x49, 0x7A, 0xFC, 0xFD, 0xA1, 0x95, 0xF2, 0x5C, 0x08, 0x6A, 0x33 };
static const u1_t PROGMEM APPSKEY[16] = { 0xED, 0x1C, 0x8D, 0x48, 0x96, 0x9C, 0xDD, 0x6D, 0xBD, 0x9C, 0xBA, 0x59, 0x0A, 0x53, 0x00, 0xDE  };
static const u4_t DEVADDR = 0x00C7C270;

void os_getArtEui (u1_t* buf) { }
void os_getDevEui (u1_t* buf) { }
void os_getDevKey (u1_t* buf) { }

int sleepTime = 4;
int week = 3;//450*24*7;
int openTime = 2;//225;

uint8_t mydata[1] = {0x01};

static osjob_t sendjob;

const lmic_pinmap lmic_pins =
{
  .nss = 10,
  .rxtx = LMIC_UNUSED_PIN,
  .rst = 5,
  .dio = {6, 7, LMIC_UNUSED_PIN},
};

void sleeping(int cycles)
{
  for (int i = 0; i < cycles; i++)
  {
    Serial.println(F("Sleep..."));
    delay(100);
    LowPower.powerDown(SLEEP_8S, ADC_OFF, BOD_OFF);
    if (interrupted) //leave sleep cycle if interrutred
    {
      interrupted = false;
      return;
    }
  }
}

void onEvent (ev_t ev) {
  Serial.print(os_getTime());
  Serial.print(F(": "));
  switch (ev) 
  {
    /*
    case EV_SCAN_TIMEOUT:
      Serial.println(F("EV_SCAN_TIMEOUT"));
      break;
    case EV_BEACON_FOUND:
      Serial.println(F("EV_BEACON_FOUND"));
      break;
    case EV_BEACON_MISSED:
      Serial.println(F("EV_BEACON_MISSED"));
      break;
    case EV_BEACON_TRACKED:
      Serial.println(F("EV_BEACON_TRACKED"));
      break;
    case EV_JOINING:
      Serial.println(F("EV_JOINING"));
      break;
    case EV_JOINED:
      Serial.println(F("EV_JOINED"));
      break;
    case EV_RFU1:
      Serial.println(F("EV_RFU1"));
      break;
    case EV_JOIN_FAILED:
      Serial.println(F("EV_JOIN_FAILED"));
      break;
    case EV_REJOIN_FAILED:
      Serial.println(F("EV_REJOIN_FAILED"));
      break;
      */
    case EV_TXCOMPLETE:
      Serial.println(F("EV_TXCOMPLETE (includes waiting for RX windows)"));
      if (LMIC.txrxFlags & TXRX_ACK)
        Serial.println(F("Received ack"));
      if (LMIC.dataLen)
      {
        Serial.print(F("Received "));
        Serial.print(LMIC.dataLen);
        Serial.println(F(" bytes of payload : "));
        Serial.println(LMIC.frame[LMIC.dataBeg]); // LMIC.dataBeg : start of data ..... LMIC.dataBeg-1 : port
        switch (LMIC.frame[LMIC.dataBeg - 1])
        {
          case 128:
            openTime = LMIC.frame[LMIC.dataBeg];
            Serial.print(F("Alert time set to: "));
            Serial.print(openTime*8);
            Serial.println(F(" s"));
            break;
          case 129:
            week = LMIC.frame[LMIC.dataBeg];
            Serial.print(F("Sleeptime set to: "));
            Serial.print(week*8);
            Serial.println(F(" s"));
            break;
          case 130:
            reset();
            break;
        }
        /*
        if (LMIC.opmode & OP_TXRXPEND)
        {
          Serial.println(F("OP_TXRXPEND, not sending"));
        }
        else
        {
          //LMIC_setTxData2(1, mystring, sizeof(mystring) - 1, 0);
          LMIC_setTxData2(192, (uint8_t*)mydata, sizeof(mydata), 0); //u1_t port, xref2u1_t data, u1_t dlen, u1_t confirmed
          Serial.println(F("Packet queued"));
        }
        */
      }
      if (door == true) //OPEN
      {
        sleepTime = openTime;
        mydata[0] = 0x00;
      }
      else // CLOSE
      {
        sleepTime = week;
        mydata[0] = 0x01;
      }
      Serial.println(mydata[0]);
      os_setCallback(&sendjob, do_send);
      sleeping(sleepTime);
      break;
      /*
    case EV_LOST_TSYNC:
      Serial.println(F("EV_LOST_TSYNC"));
      break;
    case EV_RESET:
      Serial.println(F("EV_RESET"));
      break;
    case EV_RXCOMPLETE:
      Serial.println(F("EV_RXCOMPLETE"));
      break;
    case EV_LINK_DEAD:
      Serial.println(F("EV_LINK_DEAD"));
      break;
    case EV_LINK_ALIVE:
      Serial.println(F("EV_LINK_ALIVE"));
      break;
      */
    default:
      Serial.println(F("Unknown event"));
      break;
  }

}

void do_send(osjob_t* j)
{
  if (LMIC.opmode & OP_TXRXPEND)
  {
    Serial.println(F("OP_TXRXPEND, not sending"));
  }
  else
  {
    LMIC_setTxData2(192, (uint8_t*)mydata, sizeof(mydata), 0); //u1_t port, xref2u1_t data, u1_t dlen, u1_t confirmed
    Serial.println(F("Packet queued"));
  }
}

void setup()
{
  Serial.begin(9600);
  Serial.println(F("Starting"));
  pinMode(2, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(interruptPin), isrDoor, FALLING);
  noInterrupts();
  os_init();
  LMIC_reset();

#ifdef PROGMEM

  uint8_t appskey[sizeof(APPSKEY)];
  uint8_t nwkskey[sizeof(NWKSKEY)];
  memcpy_P(appskey, APPSKEY, sizeof(APPSKEY));
  memcpy_P(nwkskey, NWKSKEY, sizeof(NWKSKEY));
  LMIC_setSession (0x1, DEVADDR, nwkskey, appskey);
#else
  LMIC_setSession (0x1, DEVADDR, NWKSKEY, APPSKEY);
#endif

  LMIC_setupChannel(0, 868100000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
  LMIC_setupChannel(1, 868300000, DR_RANGE_MAP(DR_SF12, DR_SF7B), BAND_CENTI);      // g-band
  LMIC_setupChannel(2, 868500000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
  LMIC_setupChannel(3, 867100000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
  LMIC_setupChannel(4, 867300000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
  LMIC_setupChannel(5, 867500000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
  LMIC_setupChannel(6, 867700000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
  LMIC_setupChannel(7, 867900000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
  LMIC_setupChannel(8, 868800000, DR_RANGE_MAP(DR_FSK,  DR_FSK),  BAND_MILLI);      // g2-band
  LMIC_setLinkCheckMode(0);
  LMIC.dn2Dr = DR_SF9;
  LMIC_setDrTxpow(DR_SF7, 14);
  interrupts();
  do_send(&sendjob); //START
}

void loop()
{
  os_runloop_once();
}

void isrDoor()
{
  detachInterrupt(digitalPinToInterrupt(interruptPin));
  door = true;
  interrupted = true;
  sleepTime = openTime;
  Serial.println(F("#Door opened"));
  Serial.println(F("'door flag' 1"));
  Serial.println(F("'interrupt' flag 1"));
}

void reset()
{
  door = false;
  interrupted = false;
  Serial.println(F("#Reset..."));
  Serial.println(F("'door flag' 1"));
  Serial.println(F("'interrupt' flag 1"));
  EIFR = 1; //clear interrupt flag for INT0
  Serial.println(F("INT0 status register cleared! "));
  attachInterrupt(digitalPinToInterrupt(interruptPin), isrDoor, FALLING);
}
