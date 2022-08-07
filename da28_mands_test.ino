#include <Wire.h>
#include "pm_struct.h"

#ifdef MEGACOREX
#pragma message "Compiled using MegaCoreX!"
#endif

int I2C_CLIENT_ADDR = 0x34;       // base address, modified by pins PF0 / PF2

void scanI2C(void);

void receiveEvent(size_t howMany);

// function that executes whenever data is requested by Host
// this function is registered as an event, see setup()
void requestEvent() ;

HardwareSerial &ser = Serial1;  // setup ser to point to serial1 uart

#if defined(MORS_BOTH)
  HardwareI2C &i2c_host = TWI1;
  HardwareI2C &i2c_client = TWI0;
#endif

void setup() {
  delay(2000);

  pinMode(20, INPUT_PULLUP); // pins for client address configuration
  pinMode(21, INPUT_PULLUP);

  bool addr0 = digitalRead(20); // see what our hardwired address is
  bool addr1 = digitalRead(21);

  #ifndef PM_CLIENT_ADDRESS
  I2C_CLIENT_ADDR |= (addr0 | (addr1<<1)); // compute address
  #else
  I2C_CLIENT_ADDR = PM_CLIENT_ADDRESS;     // use what was sent during compile
  #endif

  //ser.pins(PIN_PA0, PIN_PA1);
  ser.begin(115200); 

  Wire.enableDualMode(false);                // enable fmp+ is false
  Wire.begin();                              // setup host default pins SDA PA2, SCL PA3
  Wire.begin(I2C_CLIENT_ADDR, false);        // setup client with address, ignore broadcast, default pins SDA PC2, SCL PC3
  Wire.setClock(100000);                     // bus speed 100khz
  ser.println("TWI0 dual-mode setup");
  ser.printf("\n\nHello, world!\nClient address: 0x%X\nDUALCTRL: 0x%X\n", I2C_CLIENT_ADDR, TWI0_DUALCTRL);

  ser.flush();

  #ifdef MANDS_SINGLE 
    #pragma message "MANDS_SINGLE defined!"
  #endif

  delay(2000);

  scanI2C();  // scan bus?!

  Wire.onRequest(requestEvent); // register requestEvent interrupt handler
  Wire.onReceive(receiveEvent); // register receiveEvent interrupt handler
}

uint16_t      i=0;
uint16_t      x=0;
uint8_t       ledX=0;
uint8_t       adcUpdateCnt      = 0;
const uint8_t adcUpdateInterval = 20;

// the loop function runs over and over again forever
void loop() {
  i++;
  adcUpdateCnt++;

  if (i>1000){
    i=0;
    // if (timeStatus()==timeSet) {             // print timestamps once time is set
      ledX = ledX ^ 1;              // xor previous state
      digitalWrite(LED_BUILTIN, ledX);     // turn the LED on (HIGH is the voltage level)
      // ser.printf("Timestamp: %lu\n", now());
    // } 
    
  }

  delay(1);
}

// function that executes whenever data is received from Host
// this function is registered as an event, see setup()
void receiveEvent(size_t howMany) {
  uint8_t   _isr_HostByte  = 0;
  uint16_t  _isr_HostUint  = 0;
  int16_t   _isr_HostInt   = 0;
  uint32_t  _isr_HostUlong = 0;
  int32_t   _isr_HostLong  = 0;
  uint32_t  _isr_timeStamp   = 0;


  Wire.readBytes( (uint8_t *) &rxData,  howMany);                  // transfer everything from buffer into memory
  rxData.dataLen = howMany - 1;                                    // save the data length for future use
  ser.printf("RX cmd 0x%X plus %u data bytes\n", rxData.cmdAddr, rxData.dataLen);
  
  rxData.cmdData[howMany] = '\0'; // set the Nth byte as a null
}

void scanI2C() {
  byte error, address;
  int nDevices;

  ser.println("Scanning...");

  nDevices = 0;
  for(address = 1; address < 127; address++ ) {
    Wire.beginTransmission(address);
    error = Wire.endTransmission();

    if (error == 0)
    {
      ser.print("I2C device found at address 0x");
      if (address<16) 
      ser.print("0");
      ser.print(address, HEX);
      ser.println(" !");

      nDevices++;
    }
     else if (error==4) 
    {
      ser.print("Unknow error at address 0x");
      if (address<16) 
      ser.print("0");
      ser.println(address, HEX);
    } 
  }
  
  if (nDevices == 0)
    ser.println("No I2C devices found");
  else
    ser.println("done.");
 
}

// function that executes whenever data is requested by Host
// this function is registered as an event, see setup()
void requestEvent() {   
  char reqBuff[80];                          // Host has requested data
  sprintf(txData.cmdData, "Client 0x%X ready!", I2C_CLIENT_ADDR);
  for (int i=0; i<16; i++) {
    Wire.write(txData.cmdData[i]);                         // didn't have anything to send? respond with message of 6 bytes
  }
  
  ser.printf("Wrote to host: %s", txData.cmdData);

}
