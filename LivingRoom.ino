// vim:sw=4:expandtab:ts=4
#include <stdlib.h>
#include <string.h>
#include "lr.h"
#define DHT22_NO_FLOAT
#include <DHT22.h>

#include <IRremote.h>
#include <XBee.h>

static const int DHT22_PIN = 5;
DHT22 *dht; //(DHT_PIN);
static ZbMsg zmsg; 
static IrCmd *irCmd;

static int tempC; // degrees x 10
static int humidPct; // degrees x 10
static DHT22_ERROR_t dhtErr;
static char *dhtStatus = NULL;
static int dhtMinSampPeriod = 5000;
byte rxPayloadSize = 84;

long dhtReadFreqMillis = 10000;

IRsend irsend; // uses pin 3 on the UNO
XBee xbee = XBee();
XBeeResponse response = XBeeResponse();
// create reusable response objects for responses we expect to handle 
ZBRxResponse rx = ZBRxResponse();
ModemStatusResponse msr = ModemStatusResponse();

// SH + SL Address of receiving XBee
XBeeAddress64 addr64_coord = XBeeAddress64(0, 0);
XBeeAddress64 addr64_bcast = XBeeAddress64(0, 0xffff);
ZBTxStatusResponse txStatus = ZBTxStatusResponse();

int statusLed = 13;
int errorLed = 13;
int dataLed = 13;
int toggle = 0;
char txMsg[76]; // shortened by 4 for encryption and 4 for routing
char rxMsg[76]; // shortened by 4 for encryption and 4 for routing
char *pc;
// TODO figure out how to query the ATNP register to get the real value
long dhtLastMillis=0;
long diff;

static inline int toFahrenheit(int cx10) { 
    float celsius = (float) cx10 / 10.0;
    // F = (9.0/5.0) * C + 32.0
    // F - 32 = (9/5) * C
    // C = (F - 32) * (5/9)
    return (int) ((10.0) * (1.8f * celsius + 32.0)); 
}

void setup() {
    pinMode(DHT22_PIN, INPUT_PULLUP);
    pinMode(statusLed, OUTPUT);
    pinMode(errorLed, OUTPUT);
    pinMode(dataLed,  OUTPUT);
    Serial.begin(9600);
    xbee.begin(Serial);

    dht = new DHT22(DHT22_PIN);

    flashLed(statusLed, 3, 50);
}

void flashLed(int pin, int times, int wait) {    
    for (int i = 0; i < times; i++) {
        digitalWrite(pin, HIGH);
        delay(wait);
        digitalWrite(pin, LOW);

        if (i + 1 < times) {
            delay(wait);
        }
    }
}


void doDht() {
    long mils = millis();
    if (mils < 6000) { // allow it to calibrate on powerup
        return;
    }
    diff = mils - dhtLastMillis;
    if (diff < 0) {
        dhtLastMillis = 0;
    }
    else {
        if (0 == dhtLastMillis || (diff > dhtMinSampPeriod && diff > dhtReadFreqMillis)) {
            // Send the DHT data as a text string
            // Note that arduino's sprintf doesn't support floats
            // To keep 1 digit of decimal precision, I multiply by 10, then use modulo to get the mantissa
            dhtErr = dht->readData();
            switch(dhtErr) {
                case DHT_ERROR_NONE:
                    humidPct = dht->getHumidityInt();
                    tempC = dht->getTemperatureCInt();
                    dhtStatus = "OK";
                    break;
                case DHT_BUS_HUNG:
                    dhtStatus = "BUS_HUNG";
                    break;
                case DHT_ERROR_NOT_PRESENT:
                    dhtStatus = "NOT_PRESENT";
                    break;
                case DHT_ERROR_ACK_TOO_LONG:
                    dhtStatus = "ACK_TOO_LONG";
                    break;
                case DHT_ERROR_SYNC_TIMEOUT:
                    dhtStatus = "SYNC_TIMEOUT";
                    break;
                case DHT_ERROR_DATA_TIMEOUT:
                    dhtStatus = "DATA_TIMEOUT";
                    break;
                case DHT_ERROR_CHECKSUM:
                    dhtStatus = "CHECKSUM";
                    break;
                case DHT_ERROR_TOOQUICK:
                    dhtStatus = "TOO_QUICK";
                    break;
                default:
                    break;
            }

            snprintf(txMsg, sizeof(txMsg), "dht-status:%s\ndht-C:%d.%d\ndht-RH:%d.%d\n"
                    , dhtStatus
                    , tempC/10
                    , tempC%10
                    , humidPct/10
                    , humidPct%10); 

            if (*txMsg) {
                uint8_t broadcastRadius = 0;
                uint8_t frameId = 0; // no response required
                uint8_t option = 0;
                ZBTxRequest zbTx = ZBTxRequest(addr64_coord, 0xfffe, broadcastRadius, option, (uint8_t *)txMsg, strnlen(txMsg, sizeof(txMsg)), frameId);
                xbee.send(zbTx); // after sending a tx request, we expect a status response      
            }
            *txMsg = 0;     
            dhtLastMillis = mils;
        }
#if 0
        if (0 == dhtLastMillis || (diff > dhtMinSampPeriod && diff > dhtReadFreqMillis)) {
            int itempf = toFahrenheit(itemp);
            snprintf(txMsg, sizeof(txMsg), "dht-status:%s\ndht-F:%d.%d\ndht-C:%d.%d\ndht-RH:%d.%d\n"
                    , worked ? "OK" : "NOK"
                    , itempf/10
                    , itempf%10
                    , itemp/10
                    , itemp%10
                    , ihumi/10
                    , ihumi%10
                    );

            if (*txMsg) {
                uint8_t broadcastRadius = 0;
                uint8_t frameId = 0; // no response required
                uint8_t option = 0;
                ZBTxRequest zbTx = ZBTxRequest(addr64_coord, 0xfffe, broadcastRadius, option, (uint8_t *)txMsg, strnlen(txMsg, sizeof(txMsg)), frameId);
                xbee.send(zbTx); // after sending a tx request, we expect a status response      
            }
            *txMsg = 0;     
            dhtLastMillis = mils;
        }
#endif

    }
}


void loop() {   
    long mils = millis();

    if (mils > 30000) {
        doDht();
    }

    xbee.readPacket(); // always returns quickly

    if (xbee.getResponse().isAvailable()) {
        // got something        
        if (xbee.getResponse().getApiId() == ZB_RX_RESPONSE) {
            xbee.getResponse().getZBRxResponse(rx);              
            if (rx.getOption() == ZB_PACKET_ACKNOWLEDGED) {
                // the sender got an ACK
                flashLed(statusLed, 10, 10);
            } else {
                // we got it (obviously) but sender didn't get an ACK
                flashLed(errorLed, 2, 20);
            }
            handleRxResponse();        
        } 
        else if (xbee.getResponse().getApiId() == MODEM_STATUS_RESPONSE) {
            xbee.getResponse().getModemStatusResponse(msr);
            // the local XBee sends this response on certain events, like association/dissociation

            if (msr.getStatus() == ASSOCIATED) {
                // yay this is great.  flash led
                flashLed(statusLed, 10, 10);
            } else if (msr.getStatus() == DISASSOCIATED) {
                // this is awful.. flash led to show our discontent
                flashLed(errorLed, 10, 10);
            } else {
                // another status
                flashLed(statusLed, 5, 10);
            }
        } else {
            // not something we were expecting
            flashLed(errorLed, 1, 25);    
        }
    } else if (xbee.getResponse().isError()) {
        flashLed(errorLed, 1, 25);    
        //nss.print("Error reading packet.  Error code: ");  
        //nss.println(xbee.getResponse().getErrorCode());
    }    
}

void handleRxResponse() {

    int d = 0;
    *txMsg = 0;
    int datLen = rx.getDataLength();
    int repeats = 0;

    memset(&zmsg, 0, sizeof(zmsg));
    for (int i=0; i < datLen && i < sizeof(zmsg); i++) {
        zmsg.data_buf[i] = rx.getData(i);
    }

    LrMsg *lrMsg = &zmsg.lrMsg;
    IrCmd *irCmd = NULL;

    handleLrMsg(lrMsg, txMsg, sizeof(txMsg));

    if (*txMsg) {
        uint8_t broadcastRadius = 0;
        uint8_t frameId = 0; // no response required
        uint8_t option = 0;
        ZBTxRequest zbTx = ZBTxRequest(addr64_coord, 0xfffe, broadcastRadius, option, (uint8_t *)txMsg, strnlen(txMsg, sizeof(txMsg)), frameId);
        xbee.send(zbTx); // after sending a tx request, we expect a status response      
    }
    *txMsg = 0;     
}


void sony(long command, int nbits) {
    irsend.sendSony(command, nbits);
}

void nec(long command, int nbits) {
    irsend.sendNEC(command, nbits);
}
