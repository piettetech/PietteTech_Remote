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

#ifndef __PIETTETECH_NET_U__
#define __PIETTETECH_NET_U__

#include "PietteTech_Sensor.h"

class PietteTech_NET_U : public PietteTech_Sensor
{
public:
    PietteTech_NET_U();
    
    void		begin(uint32_t hostIP, int32_t senseType, int32_t sensorId = -1, char *sensorName = NULL);
    void		setData(float data);
    void		setMinMaxDelay(float min, float max, float res, int32_t delay);
    uint32_t	getHostIP();
    void		getEvent(sensors_event_t*);
    void		getSensor(sensor_t*);
    bool		initialized();
    bool		defined();
    void		define();
    bool		isMatch(uint32_t hostIP, int32_t type, int32_t id);

private:
    void		setName(sensor_t*);
    bool 		_bDefined;						// have we transmitted sensor definition
    char		_name[12];                      // sensor name
    uint32_t	_hostIP;						// remote IP address
    uint32_t	_timestamp;						// last time updated
    int32_t		_senseType;						// sensor data type
    float		_data;							// sensor data value
    int32_t		_sensorID;						// sensor ID
    float	    _maxValue;                      // maximum value of this sensor's value in SI units
    float    	_minValue;                      // minimum value of this sensor's value in SI units
    float    	_resolution;                    // smallest difference between two values reported by this sensor
    int32_t  	_minDelay;                      // min delay in microseconds between events. zero = not a constant rate
};

#endif
