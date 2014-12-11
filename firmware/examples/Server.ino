/*
 *  PietteTech_Remote: Server
 *
 *  Server example for supporting network based sensors
 *
 *  Created on: Nov 14, 2014
 *      Author: Scott Piette
 *
 *  Written by Scott Piette (Piette Technologies, LTD)
 *  Copyright (c) 2014 Scott Piette (scott.piette@gmail.com)
 *
 *  Released under the following license:
 *	GPL v3 (http://www.gnu.org/licenses/gpl.html)
 *
 *  NOTES:
 *		This example demonstrates the server interface on PietteTech_Remote
 *			configure the PietteTech_Remote class in server mode
 *			when sensor data is received update the PietteTech_NET_U object
 *			dump the data out the serial port every two seconds
 *
 *		TODO:  Future versions will use UDB to publish server IP address
 *
 *  Revision History
 *
 *    0.1   Initial commit
 *
 */

#include <application.h>
#include "PietteTech_Remote.h"

SYSTEM_MODE(MANUAL)

/**************************************************************************/
/*!
    variables and defines
*/
/**************************************************************************/
PietteTech_NET_U _nSensors[10];

// PietteTech_Remote::Stream remote();
PietteTech_Remote::Stream stream1(PIETTETECH_REMOTE_DEFAULT_PORT);

/**************************************************************************/
/*!
        Program debug options
*/
/**************************************************************************/
#define WAIT_FOR_KEYPRESS		 // wait for keypress in setup
#define SERIAL_DEBUG  1		         // 1 = timing, 2 = sensor
#if defined(SERIAL_DEBUG)
#define D(x) x
#else
#define D(x)
#endif
#if (SERIAL_DEBUG > 1)
#define DD2(x) x
#else
#define DD2(x)
#endif

/**************************************************************************/
/*!
    Spark_Net variables and defines
*/
/**************************************************************************/
unsigned long _curTime, _coreUpTime, _lastTime;
unsigned long SparkSensorNext;	           // next time to update the sensor data
unsigned long SparkPublishNext;	           // next time to publish the sensor data
unsigned long SparkLastTimeSync;	   // next time to read sensors
#define ONE_DAY_MILLIS (24 * 60 * 60 * 1000)
#define SPARK_SENSOR_INTERVAL  5000	   // how often to read sensors
#define SPARK_PUBLISH_INTERVAL  20000	   // how often to publish sensor data
unsigned long lastSync = millis();	   // last time we sync'd time

/**************************************************************************/
/*!
    Heartbeat variables and defines
*/
/**************************************************************************/
#define HEARTBEAT_DELAY 1000
unsigned long SparkHeartBeatNext;	   // next time to toggle heartbeat led
int heartbeat_led = D7;
bool heartbeat_state = false;
TCPServer server = TCPServer(PIETTETECH_REMOTE_DEFAULT_PORT);

//#define DISCONNECT_WIFI_AFTER_SEND
#define PT_RECONNECT_LIMIT 10
int _wifistate;
enum {
    WIFI_ON             	= (1),
    WIFI_OFF			= (2),
    WIFI_CONNECTING		= (3),
    WIFI_CLOUD_CONNECTING	= (4),
    WIFI_READY			= (5)
};
unsigned long _start_net_connect;       // DEBUG
unsigned long _totalConnectTime;
int _j;

int startWiFi() {

	// startup wifi
	switch (_wifistate) {
		case WIFI_OFF:
            D(_start_net_connect = millis(); _j = 0;)
           	D(Serial.print("Starting WiFi\n\r");)
            WiFi.on();            // Turn on the WiFi radio
            WiFi.connect();       // Start WiFi connection
            _wifistate = WIFI_CONNECTING;
            D(Serial.print("Enabling WiFi");)
            break;
		case WIFI_CONNECTING:
            // Wait for WiFi DHCP to get setup
            D(if (!(++_j % 5000)) Serial.print(".");)
            if (!WiFi.connecting() && WiFi.ready()) {
		IPAddress _addr = WiFi.localIP();
            	if (_addr[0] != 0) {
#if defined(ENABLE_SPARK_CLOUD)
            		_wifistate = WIFI_ON;
#else
            		_wifistate = WIFI_READY;
#endif
            		Serial.println(WiFi.localIP());
            		_j = 0;
            	}
            }
			break;
		case WIFI_ON:
            // Start the connection with the Spark Cloud
			D(Serial.print("\n\rConnecting to Cloud\n\r");)
            Spark.connect();
    		_wifistate = WIFI_CLOUD_CONNECTING;
            break;
		case WIFI_CLOUD_CONNECTING:
            // Wait for WiFi DHCP to get setup
			Spark.process();
			D(if (!(++_j % 5000)) Serial.print(".");)
            if (++_j > 50 && Spark.connected())
                _wifistate = WIFI_READY;
            break;
		default:
			D(Serial.println("Invalid WiFi state");)
			break;
	}

	if (_wifistate == WIFI_READY) {
        // We have working WiFi connection
        D(Serial.print("WiFi connected: time to connect = ");)
        D(float _t = (millis() - _start_net_connect) / 1000;)
        _totalConnectTime += _t;
        D(Serial.print(_t, 2);)
        D(Serial.print("s.\n\r");)
	}

	return _wifistate;
}

/**************************************************************************/
/*!
    Function:  IPAddress2Str
*/
/**************************************************************************/
char _strIPAddress[20];
char *IPAddress2Str(IPAddress _address)
{
	sprintf(_strIPAddress, "%d.%d.%d.%d", _address[0], _address[1], _address[2], _address[3]);
	return &_strIPAddress[0];
}

/**************************************************************************/
/*
        Function:  process_sensors
 */
/**************************************************************************/
void process_sensors()
{
	int numSensors = 0;

    // Get the sensor data and print its value.
    sensors_event_t event;
    sensor_t sensor;
    for(int n = 0; n < 10.; n++) {
        if (_nSensors[n].initialized()) {
            _nSensors[n].getSensor(&sensor);
//            _nSensors[n].printSensorDetail(&sensor);
            _nSensors[n].getEvent(&event);
            _nSensors[n].printSensorEvent(&event, sensor.name);
        	numSensors++;
        }
    }
    Serial.print("Sensors allocated ");
    Serial.print(numSensors);

    SparkSensorNext = _curTime + 2000;
}

/**************************************************************************/
/*
        Function:  SerialCurrentTime
        This fixes a bug in Spark Time where there is a line feed
        on the end of the string.
 */
/**************************************************************************/
void SerialCurrentTime(bool bNewLine = false) {
    D(char buf [50];)
    D(char *_c;)
    D(sprintf(buf, "[%s", Time.timeStr().c_str());)
    D(for(_c = buf; ((*_c != '\r') && (*_c != '\n') && (*_c != 0)); _c++) ;)
    D(*_c++ = ']';)
    D(*_c = 0;)
	D(if (bNewLine))
		D(Serial.println(buf);)
	D(else)
		D(Serial.print(buf);)
}

/**************************************************************************/
/*
        Function:  setup
 */
/**************************************************************************/
void setup()
{
    // Configure the LED heartbeat pin
    pinMode(heartbeat_led, OUTPUT);
    digitalWrite(heartbeat_led, LOW);
    heartbeat_state = false;
    SparkHeartBeatNext = millis() + HEARTBEAT_DELAY;

    Serial.begin(9600);
#if defined(WAIT_FOR_KEYPRESS)
    while(!Serial.available()) {
	delay(500);
	SerialCurrentTime();
	Serial.println(" press any key to begin");
	Spark.process();  // keep the spark happy
	delay(500);
    }
#endif

    // register the Spark function
    _wifistate = WIFI_OFF;
//    while (startWiFi() != WIFI_READY) { Spark.process(); }

    Serial.print("setup:Server IP address : ");
    Serial.println(IPAddress2Str(WiFi.localIP()));

    // provide the server with objects to store remote sensor data into
    stream1.server(_nSensors, 10);

    Serial.println("setup:Starting TCPServer.\n");
    server.begin();

    // Lets wait until we are connected and then sync our time with the Cloud
//    while (!Spark.connected()) Spark.process();
//    Spark.syncTime();       // Lets get the time from the cloud

    SparkSensorNext = millis() + SPARK_SENSOR_INTERVAL;

    _lastTime = millis();
}

/**************************************************************************/
/*
        Function:  loop
 */
/**************************************************************************/
void loop()
{
    _curTime = millis();

    /*
     * Flash the heartbeat led
     */
    if (_curTime > SparkHeartBeatNext)
    {
        digitalWrite(heartbeat_led, heartbeat_state?LOW:HIGH);       // Turn LED either on or off
    	delay(20);
    	digitalWrite(heartbeat_led, heartbeat_state?HIGH:LOW);       // Turn LED either on or off
    	SparkHeartBeatNext = millis() + HEARTBEAT_DELAY;
    }

    if (_curTime > SparkSensorNext)
    {
    	_coreUpTime += (_curTime - _lastTime)/1000UL;
        _lastTime = _curTime;

        // dump time
        Serial.print(IPAddress2Str(WiFi.localIP()));
        Serial.print(":");
        SerialCurrentTime(true);

        // Process the local sensor data
        process_sensors();

        Serial.println();

        // Setup the next time we collect sensor data
        SparkSensorNext = millis() + SPARK_SENSOR_INTERVAL;
    }

    // Here is where the network action happens
    if (_wifistate == WIFI_READY)
	stream1.listen(&server);
    else
      startWiFi();

    // Lets sync with the network time once a day
//     if (_curTime - SparkLastTimeSync > ONE_DAY_MILLIS) {
//         Spark.syncTime();
//         SparkLastTimeSync = millis();
//     }
}



