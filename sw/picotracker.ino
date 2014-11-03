/*
Picotracker 3.2 SW
ATMEGA 328p@4MHz and 1.8V
RFM22 430 MHz beacon CW and RTTY packet compatible with  
NMEA rate 2 Hz

by OM2AMR and OK1CDJ
parts from AVA code by M0UPU

Changes:
GPS power safe mode when gps_sat >5 and max. pref. <5


*/

#if ARDUINO < 22
#error "Oops! We need Arduino 22 or later"
#endif

// custom libs

//#include "debug.h"
#include "gps.h"

#define GPS_BAUDRATE  9600
#define RTTY_PERIOD   1000UL
#define DEBUG_RESET  // AVR reset
#define DEBUG_RTTY //RTTY packet dump
#define DEBUG_CW   // CW packet dump
#define RADIO_FREQUENCY 434.690  
#define RADIO_POWER  0x05
/*
 0x02  5db (3mW)
 0x03  8db (6mW)
 0x04 11db (12mW)
 0x05 14db (25mW)
 0x06 17db (50mW)
 0x07 20db (100mW)
 */
#define RADIO_REBOOT 20  // Reboot Radio every X telemetry lines, Anthony to robi zahybanim SDN pinom, ja ho mam GND natvrdo, skusit SW reset ?
#define BATTVOLTPIN 0

//CW TIMING 
// 40 WPM
//#define DOT 40
//#define DASH 120
//#define INTERDD 40
//#define ENDCHAR 80
//#define INTERWORD 200
// 20 WPM
#define DOT 80
#define DASH 240
#define INTERDD 80
#define ENDCHAR 160
#define INTERWORD 400


// Arduino/AVR libs
#include <Wire.h>
#include <Arduino.h>
#include <avr/power.h>
#include <avr/sleep.h>
#include <SPI.h>
#include <RFM22.h>
#include <util/crc16.h>

rfm22 radio1(10);
char datastring[200];
char datastring2[200];
unsigned int pkt_num;
unsigned int sentence_counter;
float battvolt=0;
float tempin=0;
unsigned long rtty_next_tx_millis;
int firstlock = 0;



void disable_bod_and_sleep()
{
  /* This will turn off brown-out detection while
   * sleeping. Unfortunately this won't work in IDLE mode.
   * Relevant info about BOD disabling: datasheet p.44
   *
   * Procedure to disable the BOD:
   *
   * 1. BODSE and BODS must be set to 1
   * 2. Turn BODSE to 0
   * 3. BODS will automatically turn 0 after 4 cycles
   *
   * The catch is that we *must* go to sleep between 2
   * and 3, ie. just before BODS turns 0.
   */
  unsigned char mcucr;

  cli();
  mcucr = MCUCR | (_BV(BODS) | _BV(BODSE));
  MCUCR = mcucr;
  MCUCR = mcucr & (~_BV(BODSE));
  sei();
  sleep_mode();    // Go to sleep
}

void power_save()
{
  /* Enter power saving mode. SLEEP_MODE_IDLE is the least saving
   * mode, but it's the only one that will keep the UART running.
   * In addition, we need timer0 to keep track of time, timer 1
   * to drive the buzzer and timer2 to keep pwm output at its rest
   * voltage.
   */
 

  set_sleep_mode(SLEEP_MODE_IDLE);
  sleep_enable();
  power_adc_disable();
  power_spi_disable();
  power_twi_disable();

 

  //digitalWrite(LED_PIN, HIGH);
  sleep_mode();    // Go to sleep

  
  sleep_disable();  // Resume after wake up
  power_all_enable();

}

void setup()
{
  setupRADIO(); 
  Serial.begin(GPS_BAUDRATE);
  resetGPS();
  setupGPS(); 
  gps_parser_setup();
  wait(500);
  
  //doplnit reset radia po 20 vetach !
  morse("DE OK1CDJ BALLOON TRACKER");
  wait(1000);

  radio1.write(0x07, 0x08); // turn tx on
  //radioON=true;
  
#ifdef DEBUG_RESET
  Serial.println("RESET");
#endif
  
  
  rtty_next_tx_millis = millis() + 2000;

  
}

//////////////////////////////////////////////////////
//                                                  //
//                                                  //
//                     MAIN LOOP                    //
//                                                  //
//////////////////////////////////////////////////////

void loop()
{
  int cc;
  
  //int temperature; 

   
   
  //rtty
  //rtty_send();
  /*read temp from RFM
          radio1.write(0x0f,10000000); //ADCSTART
          radio1.write(0x10,0x00);
          while (!(radio1.read(0x0f) & 0x80)); // wait for ADC conversion finish
          
          temperature = radio1.read(0x11) /2 ;
          temperature = temperature - 64 - 10;

          Serial.print(temperature);
          Serial.print("\n");
 */
  if (millis() >= rtty_next_tx_millis) {
   sensors(); 
   rtty_send();
   rtty_next_tx_millis = millis() + RTTY_PERIOD;
   
  }

  while (Serial.available()) {
    cc = Serial.read();
    gps_decode(cc);
   //Serial.print("X") ;
   // power save GPS
    if(firstlock == 0 && gps_sat > 5){
          setupGPSpower();
          firstlock = 1;
          //Serial.print("GPS power save") ;
      }    
    if(firstlock == 1 && gps_sat < 5){
          setGps_MaxPerformanceMode();
          firstlock = 0;
          //Serial.print("GPS Max performance") ;
      }
  }
            
  power_save();
}





void rtty_txbit (int bit)
{
  if (bit)
  {
    radio1.write(0x73,0x03); // High0x03
  }
  else
  {
    radio1.write(0x73,0x00); // Low
  }

 
  delayMicroseconds(1685); // 300 baud
                    //delayMicroseconds(843); // 600 baud
  //delayMicroseconds(10000); // For 50 Baud uncomment this and the line below. 
  //delayMicroseconds(10150); // For some reason you can't do 20150 it just doesn't work.
 
}
void rtty_txbyte (char c)
{
  /* Simple function to sent each bit of a char to 
   	** rtty_txbit function. 
   	** NB The bits are sent Least Significant Bit first
   	**
   	** All chars should be preceded with a 0 and 
   	** proceded with a 1. 0 = Start bit; 1 = Stop bit
   	**
   	*/
 
  int i;
 
  rtty_txbit (0); // Start bit
 
  // Send bits for for char LSB first	
 
  for (i=0;i<7;i++) // Change this here 7 or 8 for ASCII-7 / ASCII-8
  {
    if (c & 1) rtty_txbit(1); 
 
    else rtty_txbit(0);	
 
    c = c >> 1;
 
  }
 
  rtty_txbit (1); // Stop bit
  rtty_txbit (1); // Stop bit
}

void rtty_txstring (char * string)
{
 
  
  /* Simple function to sent a char at a time to 
   	** rtty_txbyte function. 
   	** NB Each char is one byte (8 Bits)
   	*/
 
  char c;
 
  c = *string++;
 
  while ( c != '\0')
  {
    rtty_txbyte (c);
    c = *string++;
  }
  
}
 
uint16_t gps_CRC16_checksum (char *string)
{
  size_t i;
  uint16_t crc;
  uint8_t c;
 
  crc = 0xFFFF;
 
  // Calculate checksum ignoring the first four $s
  for (i = 4; i < strlen(string); i++)
  {
    c = string[i];
    crc = _crc_xmodem_update (crc, c);
  }
 
  return crc;
}     

void rtty_send() {
 
  
  char temp1[12];  
  //char gps_rtty_time[10];
  char h_str[3];
  char m_str[3];
  char s_str[3];
  
  
  
  //modify time to HH:MM:SS format
  //strcpy(gps_rtty_time,"123456");  //temp, only for testing
   
  //strncpy(h_str, gps_rtty_time+0, 2);
  //strncpy(m_str, gps_rtty_time+2, 2);
  //strncpy(s_str, gps_rtty_time+4, 2);

  strncpy(h_str, gps_time+0, 2);
  strncpy(m_str, gps_time+2, 2);
  strncpy(s_str, gps_time+4, 2);
 
// $$<CALL SIGN>,<COUNTER D>,<TIME HH:MM:SS>,<LATITUDE DD.DDDDDD>,<LONGITUDE DD.DDDDDD>,<ALTITUDE METRES MMMMM>,
//<O SPEED KM/H DDDD.DD>,<O BEARING DDD.DD>,<O TEMPERATURE INTERNAL C D.DD>,<O TEMPERATURE EXTERNAL C D.DD>,
//<O TEMPERATURE CAMERA C D.DD>,<O BAROMETRIC PRESSURE hPa(millibars)>,<O CUSTOM DATA>*<CHECKSUM><NEWLINE> 
   pkt_num++;

   strcpy(datastring,"$$$$CDJ-1,"); //CALLSIGN
   
   strcpy(datastring2,"OK1CDJ ");
   
   //sprintf(temp1,"%u",millis()/1000); //packet number
   sprintf(temp1,"%u",pkt_num); 
   strcat(datastring,temp1);
   
   strcat(datastring2,temp1); // pkt num do cw message
   
   strcat(datastring, ",");
   strcat(datastring2, " ");
   
   snprintf(temp1, 3, "%s", h_str);
   strcat(datastring, temp1);
   strcat(datastring2,temp1); // hodiny do cw message
   
   strcat(datastring, ":");
   snprintf(temp1, 3, "%s", m_str);
   strcat(datastring, temp1);
   strcat(datastring2,temp1); // minuty do cw message
   
   strcat(datastring, ":");
   snprintf(temp1, 3, "%s", s_str);
   strcat(datastring, temp1);
   strcat(datastring, ",");
   strcat(datastring2, " ");
   
   dtostrf(gps_lat, 7, 6, temp1); // DD.DDDDDD
   strcat(datastring,temp1);
   strcat(datastring, ",");
   dtostrf(gps_lon, 7, 6, temp1); // DD.DDDDDD
   strcat(datastring,temp1);
   strcat(datastring, ",");
   //sprintf(temp1,"%s", gps_altitude);
   snprintf(temp1, 6, "%05ld", (long)(gps_altitude + 0.5));
   strcat(datastring,temp1);
   strcat(datastring, ",");
   snprintf(temp1, 4, "%03d", (int)(gps_speed + 0.5));
   strcat(datastring,temp1);
   strcat(datastring, ",");
   //snprintf(temp1, 4, "%03d", (int)(gps_course + 0.5));
   //strcat(datastring,temp1); 
   //strcat(datastring, ",");
     
   dtostrf(battvolt, 4, 2, temp1);
   strcat(datastring,temp1);
   strcat(datastring, ",");
   dtostrf(tempin, 3, 1, temp1); //Ti
   strcat(datastring,temp1);
   strcat(datastring, ",");
   snprintf(temp1, 3, "%02d", (int)gps_sat);
   strcat(datastring,temp1);
   strcat(datastring, ",");
   snprintf(temp1, 3, "%02d", (int)firstlock);
   //strcat(datastring, ",OM2AMR");
   strcat(datastring,temp1);
  
  unsigned int CHECKSUM = gps_CRC16_checksum(datastring);  // Calculates the checksum for this datastring
  char checksum_str[6];
  sprintf(checksum_str, "*%04X\n", CHECKSUM);
  strcat(datastring,checksum_str);
  
  noInterrupts();
  rtty_txstring (datastring);
  interrupts();

  #ifdef DEBUG_RTTY
  Serial.print(datastring);
  #endif
  
    
  // vypocet lokatoru
  float gps_lat2=gps_lat*100000;
  float gps_lon2=gps_lon*100000;
  long lat,lon,lat1,lon1;
  lat1 = (long) gps_lat2;
  lon1 = (long) gps_lon2;
   //lat1 = 5002410; 
  // lon1 = 1578116;                           
  lon = lon1 + 18000000;                            
  lat = lat1 +  9000000;  
  char loc[8] = {'A', 'A', '0', '0', 'A', 'A','0','0'};     // Initialise string
  loc[0] +=  lon / 2000000;                       // Field
  loc[1] +=  lat / 1000000;
  loc[2] += (lon % 2000000) / 200000;             // Square
  loc[3] += (lat % 1000000) / 100000;
  loc[4] += (lon %  200000) /   8333;             // Subsquare .08333 ==  5/60 
  loc[5] += (lat %  100000) /   4166;  
  loc[6] += (lon %  8333) /   833;             // extende Square
  loc[7] += 10-(lat %  4166) /   417;
  

  // lokator do cw msg
  char temp2[12];
  strlcpy(temp2,loc,9);
  strcat(datastring2,temp2);
  //Serial.println(loc);
  strcat(datastring2, " ");
  
  // vyska do cw mesg
  snprintf(temp2, 6, "%05ld", (long)(gps_altitude + 0.5));
  strcat(datastring2,temp2);
   
  
  #ifdef DEBUG_CW
  Serial.println(datastring2);
  #endif
  
  
  sentence_counter++;
   //
   if (sentence_counter >= RADIO_REBOOT ) {
    sentence_counter = 0;
    radio1.write(0x07, 0x80); // radio soft reset
    wait(500);
    setupRADIO();
    morse(datastring2);
    wait(1000);
    radio1.write(0x07, 0x08); // turn tx on
    wait(2000);
    
      
   }
  
  
  //delay(2000);
}


void sendUBX(uint8_t *MSG, uint8_t len) {
  Serial.flush();
  Serial.write(0xFF); //preco toto na zaciatku spravy ?
  wait(100);
  for(int i=0; i<len; i++) {
    Serial.write(MSG[i]);
  }
}
 
void resetGPS() {
  /*
  Forced (Watchdog)
  Coldstart
  */
  uint8_t set_reset[] = {0xB5, 0x62, 0x06, 0x04, 0x04, 0x00, 0xFF, 0x87, 0x00, 0x00, 0x94, 0xF5};
  sendUBX(set_reset, sizeof(set_reset)/sizeof(uint8_t));
}

void setupGPS() {
  morse("V");
  
  //setGPS_NMEAoff(); test only
  setGPS_GLLoff();
  wait(1000);
  setGPS_GSVoff();
  wait(1000); 
  setGPS_VTGoff();
  wait(1000);
  setGPS_GSAoff();
  wait(1000);
  setGPS_rate();
  wait(1000);
  setGPS_DynamicModel6();
  wait(1000);
  setGps_MaxPerformanceMode();
  wait(1000); 
  
  morse("V");
  
  //turn off GSV
   
  
}

void setupGPSpower() {
  //Set GPS ot Power Save Mode
  uint8_t setPSM[] = { 0xB5, 0x62, 0x06, 0x11, 0x02, 0x00, 0x08, 0x01, 0x22, 0x92 }; // Setup for Power Save Mode (Default Cyclic 1s)
 
  sendUBX(setPSM, sizeof(setPSM)/sizeof(uint8_t));
}

void wait(unsigned long delaytime) // Arduino Delay doesn't get CPU Speeds below 8Mhz
{
  unsigned long _delaytime=millis();
  while((_delaytime+delaytime)>=millis()){
  }
}



void setGPS_GSVoff()
{
  
    int gps_set_sucess=0;
  uint8_t setGSVoff[] = {0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 0x38};
  
  while(!gps_set_sucess)
  {
   sendUBX(setGSVoff, sizeof(setGSVoff)/sizeof(uint8_t));
    gps_set_sucess=getUBX_ACK(setGSVoff);
  }
  
    morse("OK");
}

void setGPS_GLLoff()
{
  
    int gps_set_sucess=0;
  uint8_t setGLLoff[] = {0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x2A};
  
  while(!gps_set_sucess)
  {
   sendUBX(setGLLoff, sizeof(setGLLoff)/sizeof(uint8_t));
    gps_set_sucess=getUBX_ACK(setGLLoff);
  }
  
    morse("OK");
}


void setGPS_VTGoff()
{
  
    int gps_set_sucess=0;
  uint8_t setVTGoff[] = {0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x05, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x04, 0x46};
  
  while(!gps_set_sucess)
  {
   sendUBX(setVTGoff, sizeof(setVTGoff)/sizeof(uint8_t));
    gps_set_sucess=getUBX_ACK(setVTGoff);
  }
  
    morse("OK");
}

void setGPS_GSAoff()
{
  
    int gps_set_sucess=0;
  uint8_t setGSAoff[] = {0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x31};
  
  while(!gps_set_sucess)
  {
   sendUBX(setGSAoff, sizeof(setGSAoff)/sizeof(uint8_t));
    gps_set_sucess=getUBX_ACK(setGSAoff);
  }
  
    morse("OK");
}

void setGPS_NMEAoff()
{
  
    int gps_set_sucess=0;
  uint8_t setNMEAoff[] = {0xB5, 0x62, 0x06, 0x00, 0x14, 0x00, 0x01, 0x00, 0x00, 0x00, 0xD0, 0x08, 0x00, 0x00, 0x80, 0x25, 0x00, 0x00, 0x07, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0xA0, 0xA9};
  
  while(!gps_set_sucess)
  {
   sendUBX(setNMEAoff, sizeof(setNMEAoff)/sizeof(uint8_t));
    gps_set_sucess=getUBX_ACK(setNMEAoff);
  }
  
    morse("OK");
}

void setGPS_rate()
{

    int gps_set_sucess=0;
  uint8_t setrate[] = {0xB5, 0x62, 0x06, 0x08, 0x06, 0x00, 0xF4, 0x01, 0x01, 0x00, 0x01, 0x00, 0x0B, 0x77 };
  
  while(!gps_set_sucess)
  {
   sendUBX(setrate, sizeof(setrate)/sizeof(uint8_t));
    gps_set_sucess=getUBX_ACK(setrate);
  }
  
    morse("OK");  
  
}

void setGPS_DynamicModel6()
{
  /*
  CFG-NAV5

Header: 0xB5, 0x62, 
ID: 0x06, 0x24, 
Length 0x24, 0x00, 
mask 0xFF, 0xFF, 
dynModel:  0x06, (Airborne <1g)
fixMode: 0x03, 
fixedAlt: 0x00, 0x00, 0x00, 0x00, 
fixedAltVar: 0x10, 0x27, 0x00, 0x00, 
minElev 0x05, 
drLimit 0x00, 
pDop 0xFA, 0x00, 
tDop 0xFA, 0x00, 
pAcc 0x64, 0x00, 
tAcc 0x2C, 0x01, 
staticHoldThresh 0x00, 
dgpsTimeOut 0x00, 
0x00, 0x00, 0x00, 0x00, 
0x00, 0x00, 0x00, 0x00, 
0x00, 0x00, 0x00, 0x00,
CK_A 0x16, 
CK_B 0xDC  
  
  */
  int gps_set_sucess=0;
  uint8_t setdm6[] = {
    0xB5, 0x62, 0x06, 0x24, 0x24, 0x00, 0xFF, 0xFF, 0x06,
    0x03, 0x00, 0x00, 0x00, 0x00, 0x10, 0x27, 0x00, 0x00,
    0x05, 0x00, 0xFA, 0x00, 0xFA, 0x00, 0x64, 0x00, 0x2C,
    0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x16, 0xDC                                                             };
  while(!gps_set_sucess)
  {
    sendUBX(setdm6, sizeof(setdm6)/sizeof(uint8_t));
    gps_set_sucess=getUBX_ACK(setdm6);
  }
    morse("OK");
}


void setGps_MaxPerformanceMode() {
  /*
  UBX-CFG-RMX - 0 Continuous Mode (Max Performance Mode)
  */
  //Set GPS for Max Performance Mode
  uint8_t setMax[] = { 
    0xB5, 0x62, 0x06, 0x11, 0x02, 0x00, 0x08, 0x00, 0x21, 0x91                                                                             }; // Setup for Max Power Mode
  sendUBX(setMax, sizeof(setMax)/sizeof(uint8_t));
}


boolean getUBX_ACK(uint8_t *MSG) {
  uint8_t b;
  uint8_t ackByteID = 0;
  uint8_t ackPacket[10];
  unsigned long startTime = millis();

  // Construct the expected ACK packet    
  ackPacket[0] = 0xB5;	// header
  ackPacket[1] = 0x62;	// header
  ackPacket[2] = 0x05;	// class
  ackPacket[3] = 0x01;	// id
  ackPacket[4] = 0x02;	// length
  ackPacket[5] = 0x00;
  ackPacket[6] = MSG[2];	// ACK class
  ackPacket[7] = MSG[3];	// ACK id
  ackPacket[8] = 0;		// CK_A
  ackPacket[9] = 0;		// CK_B

  // Calculate the checksums
  for (uint8_t ubxi=2; ubxi<8; ubxi++) {
    ackPacket[8] = ackPacket[8] + ackPacket[ubxi];
    ackPacket[9] = ackPacket[9] + ackPacket[8];
  }

  while (1) {

    // Test for success
    if (ackByteID > 9) {
      // All packets in order!
      return true;
    }

    // Timeout if no valid response in 3 seconds
    if (millis() - startTime > 3000) { 
      return false;
    }

    // Make sure data is available to read
    if (Serial.available()) {
      b = Serial.read();

      // Check that bytes arrive in sequence as per expected ACK packet
      if (b == ackPacket[ackByteID]) { 
        ackByteID++;
      } 
      else {
        ackByteID = 0;	// Reset and look again, invalid order
      }

    }
  }
}

void setupRADIO()
{
 rfm22::initSPI();
 
  radio1.init();
 
  radio1.write(0x71, 0x00); // unmodulated carrier
 
  //This sets up the GPIOs to automatically switch the antenna depending on Tx or Rx state, only needs to be done at start up
  radio1.write(0x0b,0x12);
  radio1.write(0x0c,0x15);
  
  /*radio module temp sensor setup
  
  To use the Temp Sensor:
1. Set the input for ADC to the temperature sensor, "Register 0Fh. ADC Configuration"—adcsel[2:0] = 000 
2. Set the reference for ADC, "Register 0Fh. ADC Configuration"—adcref[1:0] = 00
3. Set the temperature range for ADC, "Register 12h. Temperature Sensor Calibration"—tsrange[1:0]
4. Set entsoffs = 1, "Register 12h. Temperature Sensor Calibration"
5. Trigger ADC reading, "Register 0Fh. ADC Configuration"—adcstart = 1
6. Read temperature value—Read contents of "Register 11h. ADC Value"
  
 */
  
  radio1.write(0x0f,0x00);
  radio1.write(0x12,0x20); //tsrange
  radio1.write(0x13,0x00); //tvoffs
 
  radio1.setFrequency(RADIO_FREQUENCY);
  radio1.write(0x6D, RADIO_POWER);
}

void sensors()
{
  battvolt = analogRead(BATTVOLTPIN);
  battvolt = (battvolt/1024)*1.77;
  
  //radio temp sensor read
  
    radio1.write(0x0f,0x80); //triggerADC
    wait(100);
    tempin = (radio1.read(0x11))*0.5 - 64;
    
}

// MORSE ENGINE :-)

void morse(char string[])
{

 unsigned int i;
  for (i = 0; i < strlen(string); i++)
    {
      char c = string[i];
      morseLetter(c);
    }
  
 

}

void morseTone(int length)
{
  radio1.write(0x07, 0x08); // turn tx on
  wait(length);
  radio1.write(0x07, 0x01); // turn tx off
  wait(INTERDD);
}


void morseLetter(char x)
{
switch (x)
 {
  case ' ':
    wait(INTERWORD);
  return;
  case 'A':
    morseTone(DOT);
    morseTone(DASH);
    wait(ENDCHAR);
  return;
  case 'B':
    morseTone(DASH);
    morseTone(DOT);
    morseTone(DOT);
    morseTone(DOT);
    wait(ENDCHAR);
  return;
  case 'C':
    morseTone(DASH);
    morseTone(DOT);
    morseTone(DASH);
    morseTone(DOT);
    wait(ENDCHAR);
  return;
  case 'D':
    morseTone(DASH);
    morseTone(DOT);
    morseTone(DOT);
    wait(ENDCHAR);
  return;
  case 'E':
    morseTone(DOT);
    wait(ENDCHAR);
  return;
  case 'F':
    morseTone(DOT);
    morseTone(DOT);
    morseTone(DASH);
    morseTone(DOT);
    wait(ENDCHAR);
  return;
  case 'G':
    morseTone(DASH);
    morseTone(DASH);
    morseTone(DOT);
    wait(ENDCHAR);
  return;
  case 'H':
    morseTone(DOT);
    morseTone(DOT);
    morseTone(DOT);
    morseTone(DOT);
    wait(ENDCHAR);
  return;
  case 'I':
    morseTone(DOT);
    morseTone(DOT);
    wait(ENDCHAR);
  return;
  case 'J':
    morseTone(DOT);
    morseTone(DASH);
    morseTone(DASH);
    morseTone(DASH);
    wait(ENDCHAR);
  return;
  case 'K':
    morseTone(DASH);
    morseTone(DOT);
    morseTone(DASH);
    wait(ENDCHAR);
  return;
  case 'L':
    morseTone(DOT);
    morseTone(DASH);
    morseTone(DOT);
    morseTone(DOT);
    wait(ENDCHAR);
  return;
  case 'M':
    morseTone(DASH);
    morseTone(DASH);
    wait(ENDCHAR);
  return;
  case 'N':
    morseTone(DASH);
    morseTone(DOT);
    wait(ENDCHAR);
  return;
  case 'O':
    morseTone(DASH);
    morseTone(DASH);
    morseTone(DASH);
    wait(ENDCHAR);
  return;
  case 'P':
    morseTone(DOT);
    morseTone(DASH);
    morseTone(DASH);
    morseTone(DOT);
    wait(ENDCHAR);
  return;
  case 'Q':
    morseTone(DASH);
    morseTone(DASH);
    morseTone(DOT);
    morseTone(DASH);
    wait(ENDCHAR);
  return;
  case 'R':
    morseTone(DOT);
    morseTone(DASH);
    morseTone(DOT);
    wait(ENDCHAR);
  return;
  case 'S':
    morseTone(DOT);
    morseTone(DOT);
    morseTone(DOT);
    wait(ENDCHAR);
  return;
  case 'T':
    morseTone(DASH);
    wait(ENDCHAR);
  return;
  case 'U':
    morseTone(DOT);
    morseTone(DOT);
    morseTone(DASH);
    wait(ENDCHAR);
  return;
  case 'V':
    morseTone(DOT);
    morseTone(DOT);
    morseTone(DOT);
    morseTone(DASH);
    wait(ENDCHAR);
  return;
  case 'X':
    morseTone(DASH);
    morseTone(DOT);
    morseTone(DOT);
    morseTone(DASH);
    wait(ENDCHAR);
  return;
  case 'Y':
    morseTone(DASH);
    morseTone(DOT);
    morseTone(DASH);
    morseTone(DASH);
    wait(ENDCHAR);
  return;
  case 'Z':
    morseTone(DASH);
    morseTone(DASH);
    morseTone(DOT);
    morseTone(DOT);
    wait(ENDCHAR);
  return;
  case '0':
    morseTone(DASH);
    morseTone(DASH);
    morseTone(DASH);
    morseTone(DASH);
    morseTone(DASH);
    wait(ENDCHAR);
  return;
  case '1':
    morseTone(DOT);
    morseTone(DASH);
    morseTone(DASH);
    morseTone(DASH);
    morseTone(DASH);
    wait(ENDCHAR);
  return;
  case '2':
    morseTone(DOT);
    morseTone(DOT);
    morseTone(DASH);
    morseTone(DASH);
    morseTone(DASH);
    wait(ENDCHAR);
  return;
  case '3':
    morseTone(DOT);
    morseTone(DOT);
    morseTone(DOT);
    morseTone(DASH);
    morseTone(DASH);
    wait(ENDCHAR);
  return;
  case '4':
    morseTone(DOT);
    morseTone(DOT);
    morseTone(DOT);
    morseTone(DOT);
    morseTone(DASH);
    wait(ENDCHAR);
  return;
  case '5':
    morseTone(DOT);
    morseTone(DOT);
    morseTone(DOT);
    morseTone(DOT);
    morseTone(DOT);
    wait(ENDCHAR);
  return;
  case '6':
    morseTone(DASH);
    morseTone(DOT);
    morseTone(DOT);
    morseTone(DOT);
    morseTone(DOT);
    wait(ENDCHAR);
  return;
  case '7':
    morseTone(DASH);
    morseTone(DASH);
    morseTone(DOT);
    morseTone(DOT);
    morseTone(DOT);
    wait(ENDCHAR);
  return;
  case '8':
    morseTone(DASH);
    morseTone(DASH);
    morseTone(DASH);
    morseTone(DOT);
    morseTone(DOT);
    wait(ENDCHAR);
  return;
  case '9':
    morseTone(DASH);
    morseTone(DASH);
    morseTone(DASH);
    morseTone(DASH);
    morseTone(DOT);
    wait(ENDCHAR);
  return;
  case '/':
    morseTone(DASH);
    morseTone(DOT);
    morseTone(DOT);
    morseTone(DASH);
    morseTone(DOT);
    wait(ENDCHAR);
  return;
  case '=':
    morseTone(DASH);
    morseTone(DOT);
    morseTone(DOT);
    morseTone(DOT);
    morseTone(DASH);
    wait(ENDCHAR);
  return;
 }
}

