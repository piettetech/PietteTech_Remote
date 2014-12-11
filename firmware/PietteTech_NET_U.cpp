/*
 *  PietteTech_NET_U
 *
 *  PietteTech Unified driver for Network based sensors
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
 *
 *  Revision History
 *
 *    0.1   Initial commit
 *
 */


#include "PietteTech_NET_U.h"

/***************************************************************************
 *
 *  Constructor: PietteTech_NET_U class
 *
 ***************************************************************************/
PietteTech_NET_U::PietteTech_NET_U() {}

/***************************************************************************
 *
 * PUBLIC FUNCTIONS
 *
 ***************************************************************************/

 /***************************************************************************
 *
 * Setups the HW
 *
 ***************************************************************************/
void PietteTech_NET_U::begin(uint32_t hostIP, int32_t senseType, int32_t sensorId, char *sensorName)
{
	_hostIP = hostIP;
    _sensorID = sensorId;
    _senseType = senseType;

    if (sensorName) {
        strlcpy(_name, sensorName, sizeof(_name));
    }
}

/***************************************************************************
 *
 *  returns true if object has been used (hostIP is populated)
 *
 ***************************************************************************/
bool PietteTech_NET_U::initialized()
{
	return (!(_hostIP == 0));
}

/***************************************************************************
 *
 *  returns true if object has been defined
 *
 ***************************************************************************/
void PietteTech_NET_U::define()
{
	_bDefined = true;
}

/***************************************************************************
 *
 *  returns true if object has been defined
 *
 ***************************************************************************/
bool PietteTech_NET_U::defined()
{
	return (_bDefined);
}

/***************************************************************************
 *
 *  Stores the data value into the object
 *
 ***************************************************************************/
void PietteTech_NET_U::setData(float data)
{
	_data = data;
	_timestamp = millis();
}

/**************************************************************************/
/*!
        Provides the min, max, and delay sensor_t data
 */
/**************************************************************************/
void PietteTech_NET_U::setMinMaxDelay(float min, float max, float res, int32_t delay) {
	_maxValue   = max;
	_minValue   = min;
	_resolution  = res;
	_minDelay   = delay;
}

/***************************************************************************
 *
 *  returns the _hostIP value of the object
 *
 ***************************************************************************/
uint32_t PietteTech_NET_U::getHostIP()
{
	return _hostIP;
}

/***************************************************************************
 *
 *  returns true of the hostIP, type and id match this object
 *
 ***************************************************************************/
bool PietteTech_NET_U::isMatch(uint32_t hostIP, int32_t type, int32_t id)
{
	return (_hostIP == hostIP && _senseType == type && _sensorID == id);
}

/***************************************************************************
 *
 *  Populates the sensor_t name for this sensor
 *
 ***************************************************************************/
void PietteTech_NET_U::setName(sensor_t* sensor) {
    if (_name) {
        strncpy(sensor->name, _name, sizeof(sensor->name) - 1);
    } else {
        strncpy(sensor->name, "NET*", sizeof(sensor->name) - 1);
    }
    sensor->name[sizeof(sensor->name)- 1] = 0;
}

/***************************************************************************
 *
 *  Provides the sensor_t data for this sensor
 *
 ****************************************************************************/
void PietteTech_NET_U::getSensor(sensor_t *sensor)
{
    /* Clear the sensor_t object */
    memset(sensor, 0, sizeof(sensor_t));
    
    // Set sensor name
    setName(sensor);
    // Set version and ID
    sensor->version     = 1;
    sensor->sensor_id   = _sensorID;
//    sensor->host_ip     = _hostIP;
    // Set type and characteristics.
    sensor->type        = _senseType;
    sensor->min_delay   = _minDelay;
    sensor->max_value   = _maxValue;
    sensor->min_value   = _minValue;
    sensor->resolution  = _resolution;
}

/***************************************************************************
 *
 * Reads the sensor and returns the data as a sensors_event_t
 *
 ***************************************************************************/
void PietteTech_NET_U::getEvent(sensors_event_t *event)
{
    // Clear event definition.
    memset(event, 0, sizeof(sensors_event_t));

    // Populate sensor reading values.
    event->version     = sizeof(sensors_event_t);
    event->sensor_id   = _sensorID;
    event->type        = _senseType;
    event->timestamp   = _timestamp;

    // store the data value in the proper variable.
    switch (_senseType) {
    	case SENSOR_TYPE_LIGHT:
    		event->light = _data;
    		break;
    	case SENSOR_TYPE_PRESSURE:
    		event->pressure = _data;
    		break;
    	case SENSOR_TYPE_PROXIMITY:
    		event->distance = _data;
    		break;
    	case SENSOR_TYPE_GRAVITY:
    		event->data[0] = _data;				//  NOTE - this was not specified
    		break;
    	case SENSOR_TYPE_RELATIVE_HUMIDITY:
    		event->relative_humidity = _data;
    		break;
    	case SENSOR_TYPE_AMBIENT_TEMPERATURE:
    		event->temperature = _data;
    		break;
    	case SENSOR_TYPE_VOLTAGE:
    		event->voltage = _data;
    		break;
    	case SENSOR_TYPE_CURRENT:
    		event->current = _data;
    		break;
    }
}
