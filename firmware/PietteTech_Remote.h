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

#ifndef __PIETTETECH_REMOTE_H__
#define __PIETTETECH_REMOTE_H__

#include <application.h>
#include "PietteTech_NET_U.h"

//#define PIETTETECH_REMOTE_DEFAULT_PORT		80	// default port
#define PIETTETECH_REMOTE_DEFAULT_PORT		5005	// default port
#define PIETTETECH_REMOTE_SOCKET_NUMBER		6	// default socket

#define PIETTETECH_REMOTE_MODE_CLIENT		1	// client mode
#define PIETTETECH_REMOTE_MODE_SERVER		2	// server mode

#define PIETTETECH_REMOTE_MAX_SENSORS		6	// How many sensors can we parse from one remote core
												// this determine memory allocation each sensor = 8 bytes
												// +16 bytes overhead.  Each token = 8 bytes
												// 15 tokens for definition, 7 for update
#define PIETTETECH_JSON_TOKENS				PIETTETECH_REMOTE_MAX_SENSORS * 15 + 2

namespace PietteTech_Remote
{
    class Stream
    {
        private:

    		// common variables
    		uint8_t 			_mode;			// are we a client or server
    		unsigned int 		_port;			// TCP port to use
    		TCPClient 			_client;		// client object
//    		TCPServer			_server(PIETTETECH_REMOTE_DEFAULT_PORT);		// server object

    		// client variables
    		IPAddress 			_host;			// IP address of server
    		char 				_params[512];	// buffer for data name & value fields in message body
    		char 				_response[64];  // buffer for response text message
    		int					addBuf();		// adds to the _params buffer checking to make sure it fits
    		bool				_bDefined;		// flag for if we have already sent object definition data

    		// server variables
    		PietteTech_NET_U	*_sensors;		// PietteTech_NET_U object array
    		uint8_t				_numSensors;	// number of objects in array
    		uint8_t				_nextSensor;	// next available object
    		PietteTech_NET_U	*remote_alloc_sensor();
    		PietteTech_NET_U	*remote_find_sensor(uint32_t hostIP, int32_t type, int32_t id);

        public:

    		// common methods

    		Stream(uint16_t port);
//    		Stream(unsigned int port = PIETTETECH_REMOTE_DEFAULT_PORT);

    		// server methods
    		int					server(PietteTech_NET_U *sensors, int numSensors);
    		int					listen(TCPServer *server);


    		// client methods
      		int					init(IPAddress host, unsigned int port = PIETTETECH_REMOTE_DEFAULT_PORT);
      		int					client(unsigned int port = PIETTETECH_REMOTE_DEFAULT_PORT);
    		int 				addData(PietteTech_Sensor *sensor, bool _bDefine = false);
    		int 				sendData();
    		char 				*getError();

    };
}
#endif
