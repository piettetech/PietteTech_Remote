/*
 *  PietteTech_Remote
 *
 *  PietteTech Network driver for remote sensors
 *
 *  Written by Scott Piette (Piette Technologies, LTD)
 *  Copyright (c) 2014 Scott Piette (scott.piette@gmail.com)

 *  Released under the following license:
 *	GPL v3 (http://www.gnu.org/licenses/gpl.html)
 *
 *  October 3, 2014
 *  This class supports remote network sensors
 *
 *  NOTES:
 *		There are two modes of operation - server & client  YOU CAN ONLY USE ONE MODE
 *		SERVER:  the server will listen for connections and allocate a PietteTech_NET_U object for each sensor received
 *		CLIENT:  the client will package up the contents of a PietteTech_Sensor and send it over to the listening server
 *
 *  Revision History
 *
 *    0.1   Initial commit
 *
 */

#include "PietteTech_Remote.h"
#include "jsmnSpark.h"

#define SERIAL_DEBUG  1		// 1 = timing, 2 = JSON

#if defined(SERIAL_DEBUG)
#define D(x) x
#else
#define D(x)
#endif
#if (SERIAL_DEBUG > 1)
#define DJ(x) x
#else
#define DJ(x)
#endif

#define TOKEN_STRING(js, t, s) \
	(strncmp(js+(t).start, s, (t).end - (t).start) == 0 \
	 && strlen(s) == (t).end - (t).start)

#define TOKEN_GETINT(js, t, i)        \
	{ char c = *(js+(t).end);         \
	  *(js+(t).end) = 0;              \
	  i = atoi(js+(t).start);         \
	  *(js+(t).end) = c; }

#define TOKEN_GETFLOAT(js, t, f)      \
	{ char c = *(js+(t).end);         \
	  *(js+(t).end) = 0;              \
	  f = atof(js+(t).start);         \
	  *(js+(t).end) = c; }

#define TOKEN_GETSTRING(js, t, s)     \
	{ char c = *(js+(t).end);         \
	  *(js+(t).end) = '\0';           \
	  strlcpy(s, js+(t).start, ((t).end - (t).start + 1));     \
	  *(js+(t).end) = c; }

/**************************************************************************/
/*
	Function:  coreID2hostIP(char *coreID)

	Returns the uint32_t value of the coreID  (it's our pseudo hostIP)
 */
/**************************************************************************/
uint32_t coreID2hostIP(char *coreID) {
	union {
		char _coreID[5];
		uint32_t _id;
	};

	// use the union to convert the string to an pseudo IP
    strlcpy(_coreID, coreID, sizeof(_coreID));
    return _id;
}

/**************************************************************************/
/*!
    @brief  Setups the HW
*/
/**************************************************************************/
// Constructor
PietteTech_Remote::Stream::Stream(uint16_t port) {
	_port = port;
}

/**************************************************************************/
/*!
    init - sets up the host and port to use for sending data
    	   and initializes a TCP client object if not already created
*/
/**************************************************************************/
int PietteTech_Remote::Stream::init(IPAddress host, unsigned int port)
{
    _host = host;
    _port = port;

    // create the TCPclient if none exist
    if (!_client) _client = TCPClient();

    return 1;
}

/**************************************************************************/
/*!
	Function:  client

	Sets up this object for acting as a client, call after init of class

	Checks for UDP broadcast of the server IP address. Once server IP
	address has been obtained we configure this object for client
	operation and initialize variables and create the TCP client object.

	Following a success, the first set of calls to addData will build
	create json string of the sensor object definitions.

	Call sendData to forward the definitions to the server.  Then all
	future addData calls build a json string of sensor data values.

	Returns:
		1	Success, we have a server IP and are ready to forward sensor data
		0	Failure, no server IP address is found
*/
/**************************************************************************/
int PietteTech_Remote::Stream::client(unsigned int port){

    // if we have already been initialized as server, return with error
    if (_mode == PIETTETECH_REMOTE_MODE_SERVER)
	return 0;

    // set mode to client
    _mode = PIETTETECH_REMOTE_MODE_CLIENT;

    // check for server IP address
//	if (_hostIP)

    // set the port
    _port = port;

    // set the flag for sending object definition information
    _bDefined = false;

    // clear out the buffers
    memset(_params, 0, sizeof(_params));
    memset(_response, 0, sizeof(_response));

    /*
     * Extract the ID for this core
     * we use pairs 8 & 9 in the ID string
     */
    Spark.deviceID().toCharArray(_response, 24);
    strncpy(_params, &_response[18], 4);        // Copy over the first 4 bytes of the last 6 bytes
    _params[4] = 0;        // Null terminate the string

    // lets start the json message with the core ID
    sprintf(_response, "\"%s\":[", _params);
    _params[0] = 0;
    if (!addBuf())
	Serial.println("AddBuf failed adding core name.");

    return 1;
}

/**************************************************************************/
/*!
    Function:	addBuf()

    Adds the text in _response into _params checking if there is space

	Returns:
	 	>0	Success, number of bytes added to _params
		0	Failure, no space left in _params buffer for _response
*/
/**************************************************************************/
int PietteTech_Remote::Stream::addBuf() {
    unsigned int _len = strlen(_params);
    unsigned int _size = strlen(_response);
    if (_size + _len > (sizeof(_params) - 1)) return 0;
    strcat(_params, _response);
    return _size;
}

/**************************************************************************/
/*!
    Function:	addData(PietteTech_Sensor*)

    Add the information from the sensor object to the data buffer
	_bDefined = false
		add the definition information about the object
	_bDefined = true
		add the data value from the object

	Returns:
	 	1	Success, information from object was added
		0	Failure, most likely out of buffer space
*/
/**************************************************************************/
int PietteTech_Remote::Stream::addData(PietteTech_Sensor *sensor, bool _bDefine) {

    if (_bDefine) {
	sensor_t _sensor;
	// get the sensor information
	sensor->getSensor(&_sensor);

	// start with the sensor name
	sprintf(_response, "{\"name\":\"%s\",", _sensor.name);
	if (!addBuf())
	    Serial.println("AddBuf failed adding sensor name.");

	// add sensor version
//	sprintf(_response, "\"ver\":%ld,", _sensor.version);
//	if (!addBuf())
//	    Serial.println("AddBuf failed adding sensor version.");

	// add sensor id
	sprintf(_response, "\"id\":%ld,", _sensor.sensor_id);
	if (!addBuf())
	    Serial.println("AddBuf failed adding sensor id.");

	// add sensor type
	sprintf(_response, "\"type\":%ld,", _sensor.type);
	if (!addBuf())
	    Serial.println("AddBuf failed adding sensor type.");

	// add max_value
	sprintf(_response, "\"max\":%1.4f,", _sensor.max_value);
	if (!addBuf())
	    Serial.println("AddBuf failed adding sensor max_value.");

	// add min_value
	sprintf(_response, "\"min\":%1.4f,", _sensor.min_value);
	if (!addBuf())
	    Serial.println("AddBuf failed adding sensor min_value.");

	// add resolution
	sprintf(_response, "\"res\":%1.4f,", _sensor.resolution);
	if (!addBuf())
	    Serial.println("AddBuf failed adding sensor resolution.");

	// add sensor data
	sensors_event_t _event;
	sensor->getEvent(&_event);
	sprintf(_response, "\"data\":%1.4f}\r\n", _event.data[0]);
	if (!addBuf())
	    Serial.println("AddBuf failed adding sensor event data.");
    }
    else {
	sensors_event_t _event;
	// get the sensor event data
	sensor->getEvent(&_event);

	// add sensor id
	sprintf(_response, "{\"id\":%ld,", _event.sensor_id);
	if (!addBuf())
	    Serial.println("AddBuf failed adding sensor id.");

	// add sensor type
	sprintf(_response, "\"type\":%ld,", _event.type);
	if (!addBuf())
	    Serial.println("AddBuf failed adding sensor type.");

	// add data
	sprintf(_response, "\"data\":%1.4f}\r\n", _event.data[0]);
	if (!addBuf())
	  Serial.println("AddBuf failed adding sensor event data.");
    }

    return 1;
}


/**************************************************************************/
/*
	Function:  sendData()

	Opens the connection, waits to connect, sends all sensor data and waits
	for the response.  It we are sending definition data then set bDefine
	to false after successful send.

	The timeout delay can be adjusted shorter if needed.
 */
/**************************************************************************/

#define MAX_CONNECT_TIMEOUT 100

int PietteTech_Remote::Stream::sendData() {
    int _ret;
    int _timeout = 0;
    int _status = -1;
    size_t numBytes;
    int _endOfParams;

    D(Serial.print("Starting client.connect(");)
    D(Serial.print(_host);)
    D(Serial.print(":");)
    D(Serial.print(_port);)
    D(Serial.println(")");)

    // Open the connection to the host
    if(_client.connect(_host,_port)) {
    	D(Serial.println("After client.connect()");)
	while ( _timeout < MAX_CONNECT_TIMEOUT && !_client.connected()) {
	    D(if (!_timeout) Serial.print("Waiting for connect");)
	    D(else Serial.print(".");)
	    delay(10);
	    ++_timeout;
	}

    // send the sensor data
    D(if (_timeout) Serial.println("timeout");)

    // save the location of the end of _params
    _endOfParams = strlen(_params);

    // lets close the json
    strcpy(_response, "]\r\n");
    if (!addBuf())
    	Serial.println("AddBuf failed closing json.");

    // send the data
    numBytes = _client.write((uint8_t*)_params, strlen(_params));
    D(Serial.print("Message write sent [");)
    D(Serial.print(numBytes);)
	D(Serial.print("/");)
	D(Serial.print(strlen(_params));)
	D(Serial.println("]");)
	D(Serial.print(_params);)

	// wait for response
	_timeout = 0;
    	while (_timeout < MAX_CONNECT_TIMEOUT && !(_ret = _client.available())) {
    	    D(if (!_timeout) Serial.print("\r\nWaiting for response");)
	    D(else Serial.print(".");)
	    delay(100);
    	    ++_timeout;
    	}
    	D(Serial.println();)

    	// If we have a response from the server lets process it
    	if (_ret) {
    	    bool _found = false;  // set to true when we have reached the end of the http response
    	    int _count = 0; // This keeps track of how many /n /r we have received
    	    memset(_response, 0, sizeof(_response));  // empty the response buffer

    	    // Search thru the response reading one byte at a time
    	    // and keep the response code
    	    char _c; // character read from stream
    	    char *_r = _response; // beginning of response buffer
    	    char *_q = _r + sizeof(_response) - 1;  // end of response buffer
    	    while (int8_t(_c = _client.read()) != -1) {
    		D(Serial.print(_c);)
		if (_c == '\r' || _c == '\n') {
		    _count++;
		    if (_count == 4) _found = true;	// we have found the end of http response
		}
		else _count = 0;
		if (_found && (_status == -1) && (_c == '0' || _c == '1')) // we have found the phant response
		    _status = _c - '0';
		if (_status != -1) {
		    // Copy the error text message to our response buffer
		    if ((_c != '\n' || _c != '\r') && _r < _q) *_r++ = _c;
		}
    	    }; // keep reading until no data is available

    	    D(Serial.print("Return Status = ");)
    	    D(Serial.println(_status);)
    	    D(Serial.println(_response);)
    	}
    	else {
    	    D(Serial.println("\r\nNo Response.");)
	    _status = 0;
    	}
    	Serial.println("Client.flush");
    	_client.flush();
    	Serial.println("Client.stop");
    	_client.stop();
    }
    else {
	_status = 0;	// error on connect
	D(Serial.println("Failed to connect");)
    }

    // Empty the params array of we had success
    if (_status)
        _params[0] = 0;
    else
	// remove the closing json
      _params[_endOfParams] = 0;

    return _status;
}

/**************************************************************************/
/*
	Function:  getError

	Returns the pointer to the response error message
	The contents of this buffer is only valid after a call to
	sendData and before any subsequent calls to addData
 */
/**************************************************************************/
char *PietteTech_Remote::Stream::getError() {
    return _response;
}

/**************************************************************************/
/*
	Function:  server(PietteTech_NET_U *, int)

	Sets up the object for server mode, call after init
	Pass in an array of PietteTech_NET_U objects and the count of objects.
	Publishes the IP address of	this core via UDP

	Call listen method to process any remote connections.

 */
/**************************************************************************/
int PietteTech_Remote::Stream::server(PietteTech_NET_U *sensors, int numSensors) {

	if (_mode == PIETTETECH_REMOTE_MODE_CLIENT)
		return 0;

	// save the PietteTech_NET_U sensor array and the number of objects in that array
	_mode = PIETTETECH_REMOTE_MODE_SERVER;
	_sensors = sensors;
	_numSensors = numSensors;
	_nextSensor = 0;

	D(Serial.println("Remote:Server called");)

	return 1;

}

/**************************************************************************/
/*
	Function:  remote_alloc_sensor()

	Allocates a fresh unused sensor from the sensor array
 */
/**************************************************************************/
PietteTech_NET_U *PietteTech_Remote::Stream::remote_alloc_sensor() {
    PietteTech_NET_U *sensor;
    if (_nextSensor >= _numSensors) {
	return NULL;
    }
    sensor = &_sensors[_nextSensor++];
    return sensor;
}

/**************************************************************************/
/*
	Function:  remote_find_sensor(uint32_t hostIP, int32_t type, int32_t id)

	Returns the sensor that matches hostIP, type, and id from the sensor array
 */
/**************************************************************************/
PietteTech_NET_U *PietteTech_Remote::Stream::remote_find_sensor(uint32_t hostIP, int32_t type, int32_t id) {

    // starting with the first sensor look for a match
    for (int i = 0; i < _nextSensor; i++) {
	if (_sensors[i].isMatch(hostIP, type, id))
	    return &_sensors[i];
    }
    return NULL;
}

/**************************************************************************/
/*
	Function:  listen()

	Checks for any incoming connections and reads data from socket
	Parses the data, if the object a definition object we search for
	this object in our list of NET_U objects, if not found we allocate
	one of the unused objects and sets the configuration data.
	When we have received sensor data information we search for the object
	and update the data and timestamp.  Otherwise we generate an error

	After we have processed the data we close the connection

 */
/**************************************************************************/
int PietteTech_Remote::Stream::listen(TCPServer *server) {
    int _ret;
    int _timeout = 0;
//    size_t numBytes;
    int numBytes;

    // check if we have any client connections to service
    if (_client.connected()) {

    	// process data from client
    	_timeout = 0;
    	while (_timeout < MAX_CONNECT_TIMEOUT && !(_ret = _client.available())) {
    		D(if (!_timeout) Serial.print("\r\nWaiting for data");)
	    	D(else Serial.print(".");)
			delay(100);
    		++_timeout;
    	}
    	D(Serial.println();)

    	// process remote data from client
    	if (_ret) {
    		// allocate storage for the coreID
    		char _coreID[5];

    		// use the _params buffer
    		int _size = sizeof(_params);
    		int _len;
    		char *_buf = _params;

    		// zero out buffer
    		memset(_params, 0, sizeof(_params));

    		// read all available data from the TCP stream a block at a time
    		while ((_len = _client.read((uint8_t *)_buf, _size)) != -1) {
    			D(Serial.print("JSON Read: buffer len = ");)
    			D(Serial.println(_len);)
				_buf += _len;
    			_size -= _len;
    		}

    		// calculate the actual length read from TCP stream
    		_len = sizeof(_params) - _size;

    		// null terminate the string
    		_params[_len] = 0;

    		jsmn_parser p;
    		jsmntok_t tok[PIETTETECH_JSON_TOKENS];

    		D(Serial.print("listen:processing ");)
    		D(Serial.print(_len);)
			D(Serial.print(":");)
			D(Serial.print(strlen(_params));)
			D(Serial.println(" bytes.");)

			D(Serial.print("Data received = \"");)
			D(Serial.print(_params);)
			D(Serial.println("\"");)

			D(if (_len == _size))
				D(Serial.print("Full buffer");)

			// parse the json data
			jsmn_init(&p);
    		_ret = jsmn_parse(&p, _params, tok, PIETTETECH_JSON_TOKENS);

    		if (_ret != JSMN_SUCCESS) {
    			DJ(Serial.print("parse failed");)
				DJ(Serial.println(_ret);)
				return 0;
    		}
    		DJ(else)
				DJ(Serial.println("parsed successfuly");)

			// sanity check the json data
			if (tok[0].type == JSMN_STRING) {
				TOKEN_GETSTRING(_params, tok[0], _coreID);
				DJ(Serial.print("JSON Found: coreID = ");)
				DJ(Serial.println(_coreID);)
			}
    		DJ(else)
    			DJ(Serial.println("JSON Err: missing core name");)

			DJ(if (tok[1].type != JSMN_ARRAY))
				DJ(Serial.println("JSON Err: invalid array");)
			DJ(else {)
				DJ(Serial.print("JSON Parse: found array of ");)
				DJ(Serial.println(tok[1].size);)
			DJ(})

			uint8_t j = 2;	// first object index

    		// check each object in array
    		for (uint8_t i = 0; i < tok[1].size; i++) {
    			if (tok[j].type == JSMN_OBJECT) {
    				DJ(Serial.print("JSON Parse: size = ");)
        	    	DJ(Serial.println(tok[j].size);)

					// check for valid object size
					if (!(tok[j].size == 6 || tok[j].size == 14)) {
						DJ(Serial.print("JSON Err: invalid object size");)
       		    		break;
					}

    				// are we defining a sensor or just updating it's value
    				if (tok[j].size == 14) {
    					// define a new sensor, parse the data
    					short items = 0;
    					char _name[12];
    					int32_t _sensorID, _senseType;
    					float _maxValue, _minValue, _resolution, _data;

    					// parse the contents
    					for (uint8_t jj = j + 1; jj < (j + tok[j].size + 1); jj++) {
    						if (TOKEN_STRING(_params, tok[jj], "name") && tok[jj + 1].type == JSMN_STRING) {
    							// sensor_name
    							jj++;	// advance to next token
    							TOKEN_GETSTRING(_params, tok[jj], _name);
    							items++;
    							DJ(Serial.print("JSON Found: name = ");)
    							DJ(Serial.println(_name);)
    						} else if (TOKEN_STRING(_params, tok[jj], "id") && tok[jj + 1].type == JSMN_PRIMITIVE) {
    							// sensor id
    							jj++;	// advance to next token
    							TOKEN_GETINT(_params, tok[jj], _sensorID);
    							items++;
    							DJ(Serial.print("JSON Found: id = ");)
    							DJ(Serial.println(_sensorID);)
    						} else if (TOKEN_STRING(_params, tok[jj], "type") && tok[jj + 1].type == JSMN_PRIMITIVE) {
    							// sensor type
    							jj++;	// advance to next token
    							TOKEN_GETINT(_params, tok[jj], _senseType);
    							items++;
    							DJ(Serial.print("JSON Found: type = ");)
    							DJ(Serial.println(_senseType);)
    						} else if (TOKEN_STRING(_params, tok[jj], "max") && tok[jj + 1].type == JSMN_PRIMITIVE) {
    							// sensor maximum
    							jj++;	// advance to next token
    							TOKEN_GETFLOAT(_params, tok[jj], _maxValue);
    							items++;
    							DJ(Serial.print("JSON Found: max = ");)
    							DJ(Serial.println(_maxValue);)
    						} else if (TOKEN_STRING(_params, tok[jj], "min") && tok[jj + 1].type == JSMN_PRIMITIVE) {
    							// sensor minimum
    							jj++;	// advance to next token
    							TOKEN_GETFLOAT(_params, tok[jj], _minValue);
    							items++;
    							DJ(Serial.print("JSON Found: min = ");)
    							DJ(Serial.println(_minValue);)
    						} else if (TOKEN_STRING(_params, tok[jj], "res") && tok[jj + 1].type == JSMN_PRIMITIVE) {
    							// sensor resolution
    							jj++;	// advance to next token
    							TOKEN_GETFLOAT(_params, tok[jj], _resolution);
    							items++;
    							DJ(Serial.print("JSON Found: resolution = ");)
    							DJ(Serial.println(_resolution);)
    						} else if (TOKEN_STRING(_params, tok[jj], "data") && tok[jj + 1].type == JSMN_PRIMITIVE) {
    							jj++;	// advance to next token
    							TOKEN_GETFLOAT(_params, tok[jj], _data);
    							items++;
    							DJ(Serial.print("JSON Found: data = ");)
    							DJ(Serial.println(_data);)
						DJ(} else {)
							DJ(char obj[10];)
							DJ(Serial.print("JSON Err: Unknown token ");)
							DJ(strlcpy(obj, &_params[tok[jj].start], (tok[jj].end - tok[jj].start + 1));)
							DJ(Serial.println(obj);)
						}
    					}

    					if (items == 7) {
    						// allocate a sensor NET object and copy contents over
    						PietteTech_NET_U *newSensor;

    						DJ(Serial.print("JSON Parse: looking for sensor object ");)
    						newSensor = remote_find_sensor(coreID2hostIP(_coreID), _senseType, _sensorID);
    						if (!newSensor) {
    							DJ(Serial.println("JSON Parse: allocating new sensor object");)
    			    			newSensor = remote_alloc_sensor();
    							if (newSensor) {
    								newSensor->begin(coreID2hostIP(_coreID), _senseType, _sensorID, _name);
    								newSensor->setMinMaxDelay(_minValue, _maxValue, _resolution, 0);
    	        						newSensor->setData(_data);
   							}
    						}
    					DJ(} else {)
       		    			DJ(Serial.print("JSON Err: missing parameters, expected 7 found ");)
							DJ(Serial.println(items);)
    					}

    				} else {

    					// we are updating the value in an existing sensor
    					short items = 0;
    					int32_t _sensorID, _senseType;
    					float _data;

    					// parse the contents
    					for (uint8_t jj = j + 1; jj < (j + tok[j].size + 1); jj++) {
    						if (TOKEN_STRING(_params, tok[jj], "id") && tok[jj + 1].type == JSMN_PRIMITIVE) {
    							// get the sensor_id
    							jj++;	// advance to next token
    							TOKEN_GETINT(_params, tok[jj], _sensorID);
    							items++;
    							DJ(Serial.print("JSON Found: id = ");)
    							DJ(Serial.println(_sensorID);)
    						} else if (TOKEN_STRING(_params, tok[jj], "type") && tok[jj + 1].type == JSMN_PRIMITIVE) {
    							jj++;	// advance to next token
    							TOKEN_GETINT(_params, tok[jj], _senseType);
    							items++;
    							DJ(Serial.print("JSON Found: type = ");)
    							DJ(Serial.println(_senseType);)
    						} else if (TOKEN_STRING(_params, tok[jj], "data") && tok[jj + 1].type == JSMN_PRIMITIVE) {
    							jj++;	// advance to next token
    							TOKEN_GETFLOAT(_params, tok[jj], _data);
    							items++;
    							DJ(Serial.print("JSON Found: data = ");)
    							DJ(Serial.println(_data);)
							DJ(} else {)
								DJ(char obj[10];)
								DJ(Serial.print("JSON Err: Unknown token ");)
								DJ(strlcpy(obj, &_params[tok[jj].start], (tok[jj].end - tok[jj].start + 1));)
								DJ(Serial.println(obj);)
							}
    					}

    					if (items == 3) {
    						// locate the sensor in the NET object array using CORE, ID, and Type
    						PietteTech_NET_U *newSensor;

    						DJ(Serial.print("JSON Parse: looking for sensor object ");)
    						newSensor = remote_find_sensor(coreID2hostIP(_coreID), _senseType, _sensorID);
    						if (newSensor) {
    							DJ(Serial.println("- found");)
        						newSensor->setData(_data);
    						}
    						DJ(else {)
        						DJ(Serial.println("- unable to find sensor object");)
    						}
    					} DJ(else)
        		    		DJ(Serial.println("JSON Err: missing parameters in sensor update.");)
    				}
    			} DJ(else)
        			DJ(Serial.println("JSON Err: expecting object");)

				DJ(Serial.print("Advancing to next object: from ");)
				DJ(Serial.print(j);)
				DJ(Serial.print(" to ");)
				DJ(Serial.println((j + tok[j].size + 1));)

				j += tok[j].size + 1;	// move index to next object

    		}	// loop thru objects in array
    	}	// client data was available

    	// send the response
    	char _responseMessage[] = "\r\n\r\n 1 Success\r\n";
        D(Serial.println("Sending response message");)
        numBytes = _client.write((uint8_t*)_responseMessage, strlen(_responseMessage));
        D(Serial.print("Message write sent [");)
        D(Serial.print(numBytes);)
        D(Serial.print("/");)
        D(Serial.print(strlen(_responseMessage));)
        D(Serial.println("]");)
        D(Serial.print(_responseMessage);)

        // delay for the data to get sent
        delay(100);

        // discard any additional data
        _client.flush();
        // close the client connection
        _client.stop();

        // Workaround from gorsat Re: [firmware] design flaw in TCPServer/TCPClient (#358)
        //server->begin();
	}
    else
        _client = server->available();

	return 1;
}
