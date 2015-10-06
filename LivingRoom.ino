#include <IRremote.h>
#include <XBee.h>
#include <DHT.h>

//#define PIR_PIN 2
#define DHT_PIN 5
//#define ACRELAY_PIN 7
//#define PHOTORESISTOR_PIN 0
//#define MOTION_EXPIRY_THRESH_MILLIS 10L * 60L * 1000L

//volatile boolean motionDetected;
//volatile long motionMillis;

byte rxPayloadSize = 84;

//long motionReportedMillis = 0;
long dhtReadFreqMillis = 120000;
//long motionReportFreqMillis = 60000;
//int lightLevel;
DHT dht;

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
// TODO figure out how to query the ATNP register to get the real value
long dhtLastMillis=0;
long diff;

uint16_t command16 = 0;
uint64_t command64 = 0; 

short tempF;
short tempC;
short relHumid;
        
void setup() {
  pinMode(statusLed, OUTPUT);
  pinMode(errorLed, OUTPUT);
  pinMode(dataLed,  OUTPUT);
//  pinMode(ACRELAY_PIN,  OUTPUT);
//  pinMode(PIR_PIN, INPUT);
  
  
//  digitalWrite(ACRELAY_PIN, LOW);  
  
//  attachInterrupt(digitalPinToInterrupt(PIR_PIN), isrPirChanged, CHANGE);
//  motionDetected = false;
  Serial.begin(9600);
  xbee.begin(Serial);
  
  flashLed(statusLed, 3, 50);
  dht.setup(DHT_PIN); 
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
  if (mils < 60000) { // allow it to calibrate on powerup
    return;
  }
  diff = mils - dhtLastMillis;
  if (diff < 0) {
    dhtLastMillis = 0;
  }
  else {
    if (0 == dhtLastMillis || (diff > dht.getMinimumSamplingPeriod() && diff > dhtReadFreqMillis)) {
      // Send the DHT data as a text string
      // Note that arduino's sprintf doesn't support floats
      // To keep 1 digit of decimal precision, I multiply by 10, then use modulo to get the mantissa
      noInterrupts();
      float c = dht.getTemperature();
      tempF = (short) (dht.toFahrenheit(c)*10);
      relHumid = (short) (dht.getHumidity()*10);
      interrupts();
      snprintf(txMsg, sizeof(txMsg), "dht-status:%s\ndht-F:%d.%d\ndht-RH:%d.%d\n", 
          dht.getStatusString(),
          tempF/10, tempF%10, relHumid/10, relHumid%10); 
     
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
  }
}

//void reportMotion() {  
//  long mils = millis();
//  if (mils < 30000) {
//    return; // allow it to calibrate
//  }
//  if (mils - motionReportedMillis >= motionReportFreqMillis) { //rate-limit the motion reports
//    lightLevel = analogRead(PHOTORESISTOR_PIN);  
//    snprintf(txMsg, sizeof(txMsg), "pir-presence:%s\nlight:%d\n", motionDetected ? "Y" : "N", lightLevel);     
//    if (*txMsg) {
//      uint8_t broadcastRadius = 0;
//      uint8_t frameId = 0; // no response required
//      uint8_t option = 0;
//      ZBTxRequest zbTx = ZBTxRequest(addr64_bcast, 0xfffe, broadcastRadius, option, (uint8_t *)txMsg, strnlen(txMsg, sizeof(txMsg)), frameId);
//      xbee.send(zbTx); 
//      motionReportedMillis = mils;
//    }
//    *txMsg = 0;
//  }
//}


void loop() {   
    long mils = millis();
//    if(motionDetected && mils - motionMillis > MOTION_EXPIRY_THRESH_MILLIS) {
//      motionDetected = false;
//    }
//  
//    if (!motionDetected) {
//      digitalWrite(ACRELAY_PIN, LOW);
//    }
    
    if (mils > 30000) {
      doDht();
//      reportMotion();
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
  int nbits;
  int i=0;
  *txMsg = 0;
  int datLen = rx.getDataLength();
  int repeats = 0;
  switch(rx.getData(d++)) {
    case 'S': // SONY
      command16 = 0;
      nbits = rx.getData(d++); // should be 12 (0x0c) 
      for (int shiftbytes = nbits/8; shiftbytes >=0; shiftbytes--) {
        command16 |= rx.getData(d++) << (8 * shiftbytes);
      }
      repeats = 0xff && rx.getData(d++);
      if (command16 > 0) {
        do {
          for (int i = 0; i < 3; i++) {
            irsend.sendSony(command16, nbits); // Sony TV power code
            delay(40);
          }   
          if (repeats) {
            delay(150); // not sure how long to wait before it considers it another button press
          }
        } while(--repeats > 0);        
      }
      snprintf(txMsg, sizeof(txMsg), "sony-cmd:0x%x\n", command16);            
      break;
      
    case 'X': // XBOX (RC6)            
      command64 = 0;
      nbits = rx.getData(d++); // should be 36 (0x24)              
      for (int shiftbytes = 7; shiftbytes >=0; shiftbytes--) {
        command64 |= rx.getData(d++) << (8 * shiftbytes);
      }
      
      // In RC6, every other button press must have a toggle bit set
      // It's how the receiver differentiates between discrete presses and
      // holding down the button
      command64 = toggle ? command64 ^ 0x8000LL : command64;
      irsend.sendRC6(command64, nbits);
      snprintf(txMsg, sizeof(txMsg), "rc6-cmd:%x\n", command64);
      
      toggle = 1 - toggle;
      break;   
      
    case 'D': // reset dht timeout millis
      // Message is 'D' followed by decimal digits
      // any data following decimal digits is ignored
      // if the first byte after 'D' is not a decimal digit, the whole message is ignored
      i=0;
      for (char c = rx.getData(d++); i < sizeof(txMsg)-1 && isdigit(c); c = rx.getData(d++)) {
        txMsg[i++] = c;
      }
      txMsg[i] = 0;
      if (strlen(txMsg) > 0) {
        dhtReadFreqMillis = strtol(txMsg, NULL, 10);
      }
      break;
      
    default:
      break;
  } 
}

 
//// interrupt handler 
//void isrPir() {
//  motionMillis = millis();
//  // allow PIR sensor to calibrate before using its readings  
//  if (motionMillis > 30000) {
//    motionDetected = true;
//    digitalWrite(ACRELAY_PIN, HIGH);  
//  }
//}


//// interrupt handler 
//void isrPirChanged() {
//  if (digitalRead(PIR_PIN) == LOW) {
//    digitalWrite(ACRELAY_PIN, LOW); 
//  }
//  else {
//    motionMillis = millis();
//    // allow PIR sensor to calibrate before using its readings  
//    if (motionMillis > 30000) {
//      motionDetected = true;
//      digitalWrite(ACRELAY_PIN, HIGH);  
//    }
//  }
//}

