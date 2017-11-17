/**************************************************************************
    Souliss - DHTxx
    
    Sensor
      - DHTxx
      // Connect pin 1 (on the left) of the sensor to +5V
      // NOTE: If using a board with 3.3V logic like an Arduino Due connect pin 1
      // to 3.3V instead of 5V!
      // Connect pin 2 of the sensor to whatever your DHTPIN is
      // Connect pin 4 (on the right) of the sensor to GROUND
      // Connect a 10K resistor from pin 2 (data) to pin 1 (power) of the sensor      
     
    Run this code on one of the following boards:
      - Arduino Ethernet (W5100) 
      - Arduino with Ethernet Shield (W5100)
      
    As option you can run the same code on the following, just changing the
    relevant configuration file at begin of the sketch
      - Arduino with ENC28J60 Ethernet Shield
      - Arduino with W5200 Ethernet Shield
      - Arduino with W5500 Ethernet Shield 
      
***************************************************************************/

// Let the IDE point to the Souliss framework
#include "SoulissFramework.h"

// Configure the framework
#include "bconf/StandardArduino.h"          // Use a standard Arduino
#include "conf/ethW5100.h"                  // Ethernet through Wiznet W5100
#include "conf/Gateway.h"                   // The main node is the Gateway, we have just one node
#include "conf/SmallNetwork.h"
#include "user/float16.h"
                // Enable DHCP and DNS

// Include framework code and libraries
#include <SPI.h>
#include <DHT.h>


// Include sensor libraries (from Adafruit) Uncomment whatever type you're using!
//#define DHTTYPE DHT11   // DHT 11
#define DHTTYPE DHT22     // DHT 22 (AM2302), AM2321
//#define DHTTYPE DHT21   // DHT 21 (AM2301)

#define DHTPIN  2         // what digital pin we're connected to

/*** All configuration includes should be above this line ***/ 
#include "Souliss.h"

#define HUMIDITY              0               // Leave 2 slots for T53
#define TEMP0                 2               // Leave 2 slots for T52
#define FAN_LOW               4
#define FAN_HIGH              5
#define LIGHT                 6
#define HUMISET               7



// DHT sensor
DHT dht(DHTPIN, DHTTYPE); // for ESP8266 use dht(DHTPIN, DHTTYPE, 11)

uint8_t ip_address[4] = { 192, 168, 0, 77 };
uint8_t subnet_mask[4] = { 255, 255, 255, 0 };
uint8_t ip_gateway[4] = { 192, 168, 0, 1 };
uint8_t hour;

float humidity = 0;
float humidity_prev = 0;
#define myvNet_address  ip_address[3]       // The last byte of the IP address (77) is also the vNet address
#define myvNet_subnet   0xFF00
EthernetUDP Udp;
char timeServer[] = "time.nist.gov"; // time.nist.gov NTP server

const int NTP_PACKET_SIZE = 48; // NTP time stamp is in the first 48 bytes of the message

byte packetBuffer[NTP_PACKET_SIZE]; //buffer to hold incoming and outgoing packets

void setup()
{   
    Initialize();

    // Get the IP address from DHCP
	Souliss_SetIPAddress(ip_address, subnet_mask, ip_gateway);

	
	SetAsGateway(myvNet_address);                 // Set this node as gateway for SoulissApp
	Udp.begin(8888);
	Serial.begin(9600);
    dht.begin();                                // initialize temperature sensor
	pinMode(9, OUTPUT);
	pinMode(4, OUTPUT);
	pinMode(5, INPUT);
    Set_Humidity(HUMIDITY);
    Set_Temperature(TEMP0);
	Set_SimpleLight(FAN_HIGH);
	Set_SimpleLight(FAN_LOW);
	Set_DigitalInput(LIGHT);
	Set_Humidity_Setpoint(HUMISET);


	//получаем время


	sendNTPpacket(timeServer);

	Serial.println("packet sent");

	delay(3000);

	if (Udp.parsePacket()) {
		// We've received a packet, read the data from it
		Udp.read(packetBuffer, NTP_PACKET_SIZE);
		// read the packet into the buffer

		// the timestamp starts at byte 40 of the received packet and is four bytes,
		// or two words, long. First, extract the two words:

		unsigned long highWord = word(packetBuffer[40], packetBuffer[41]);
		unsigned long lowWord = word(packetBuffer[42], packetBuffer[43]);
		// combine the four bytes (two words) into a long integer
		// this is NTP time (seconds since Jan 1 1900):
		unsigned long secsSince1900 = highWord << 16 | lowWord;
		Serial.print("Seconds since Jan 1 1900 = ");
		Serial.println(secsSince1900);

		// now convert NTP time into everyday time:
		Serial.print("Unix time = ");
		// Unix time starts on Jan 1 1970. In seconds, that's 2208988800:
		const unsigned long seventyYears = 2208988800UL;
		// subtract seventy years:
		unsigned long epoch = secsSince1900 - seventyYears;
		// print Unix time:
		Serial.println(epoch);


		// print the hour, minute and second:
		Serial.print("The houre is ");
		// UTC is the time at Greenwich Meridian (GMT)
		hour = (epoch % 86400L) / 3600;
		// print the hour (86400 equals secs per day)
		Serial.println(hour + 3);

	}





	
	


}

void loop()
{




	// Here we start to play
	EXECUTEFAST() {
		UPDATEFAST();

		// Execute the code every 1 time_base_fast      
		FAST_10ms() {

			// Just process communication as fast as the logics
			ProcessCommunication();
			DigInHoldCustom(5, Souliss_T1n_OffCmd, 150, FAN_HIGH,10000);
			//DigIn(5, Souliss_T1n_OnCmd, FAN_HIGH);
		//	DigInHold(5, Souliss_T1n_RstCmd, Souliss_T1n_Timed, FAN_HIGH);
			

			//LowDigIn(5, Souliss_T1n_OffCmd, LIGHT);
		}

		FAST_50ms() {

			

		
			//Souliss_DigInHold(5, Souliss_T1n_OffCmd, Souliss_T1n_OnCmd, LIGHT, 10000);
		
			Logic_SimpleLight(FAN_HIGH);
			Logic_SimpleLight(FAN_LOW);
			Logic_T13(LIGHT);
			Logic_Humidity_Setpoint(HUMISET);
			Logic_Humidity(HUMIDITY);
			Logic_Temperature(TEMP0);
			DigOut(3, Souliss_T1n_Coil, FAN_LOW);
			DigOut(9, Souliss_T1n_Coil, FAN_HIGH);	

			

		}

		FAST_90ms() {


			

		}


		FAST_510ms() {

			Timer_SimpleLight(FAN_HIGH);
		

		}


		// Process the other Gateway stuffs
		FAST_GatewayComms();
	} 
	EXECUTESLOW()
	{
		UPDATESLOW();

		SLOW_10s() {







			Serial.print("LIGHT_STATUS=");
			Serial.println(mInput(FAN_HIGH));
			Serial.println(mOutput(FAN_HIGH));
			Serial.println(mAuxiliary(FAN_HIGH));
			Serial.println(hour);


		


			SLOW_50s() {

				humidity = dht.readHumidity();
				float temperature = dht.readTemperature(false);
				//if (!isnan(humidity) || !isnan(temperature)) {
				ImportAnalog(HUMIDITY, &humidity);
				ImportAnalog(TEMP0, &temperature);
				Serial.println(temperature);
				Serial.println(humidity);
				Logic_Humidity(HUMIDITY);
				Serial.println(Souliss_SinglePrecisionFloating(&mOutput((HUMIDITY))));

			}


			SLOW_30m() {


				sendNTPpacket(timeServer);

				Serial.println("packet sent");

				delay(3000);

				if (Udp.parsePacket()) {
					// We've received a packet, read the data from it
					Udp.read(packetBuffer, NTP_PACKET_SIZE);
					// read the packet into the buffer

					// the timestamp starts at byte 40 of the received packet and is four bytes,
					// or two words, long. First, extract the two words:

					unsigned long highWord = word(packetBuffer[40], packetBuffer[41]);
					unsigned long lowWord = word(packetBuffer[42], packetBuffer[43]);
					// combine the four bytes (two words) into a long integer
					// this is NTP time (seconds since Jan 1 1900):
					unsigned long secsSince1900 = highWord << 16 | lowWord;
					Serial.print("Seconds since Jan 1 1900 = ");
					Serial.println(secsSince1900);

					// now convert NTP time into everyday time:
					Serial.print("Unix time = ");
					// Unix time starts on Jan 1 1970. In seconds, that's 2208988800:
					const unsigned long seventyYears = 2208988800UL;
					// subtract seventy years:
					unsigned long epoch = secsSince1900 - seventyYears;
					// print Unix time:
					Serial.println(epoch);


					// print the hour, minute and second:
					Serial.print("The houre is ");
					// UTC is the time at Greenwich Meridian (GMT)
					 hour = (epoch % 86400L) / 3600+3;
					// print the hour (86400 equals secs per day)
					Serial.println(hour);

				}



			}





		}

	}
}



void sendNTPpacket(char* address) {
	// set all bytes in the buffer to 0
	memset(packetBuffer, 0, NTP_PACKET_SIZE);
	IPAddress ntp(194,54,80,30);
	// Initialize values needed to form NTP request
	// (see URL above for details on the packets)
	packetBuffer[0] = 0b11100011;   // LI, Version, Mode
	packetBuffer[1] = 0;     // Stratum, or type of clock
	packetBuffer[2] = 6;     // Polling Interval
	packetBuffer[3] = 0xEC;  // Peer Clock Precision
							 // 8 bytes of zero for Root Delay & Root Dispersion
	packetBuffer[12] = 49;
	packetBuffer[13] = 0x4E;
	packetBuffer[14] = 49;
	packetBuffer[15] = 52;

	// all NTP fields have been given values, now
	// you can send a packet requesting a timestamp:
	Udp.beginPacket(ntp, 123); //NTP requests are to port 123
	Udp.write(packetBuffer, NTP_PACKET_SIZE);
	Udp.endPacket();
}
 
