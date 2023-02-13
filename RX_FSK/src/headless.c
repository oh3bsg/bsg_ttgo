#include <Arduino.h>
#include <math.h>
#include "headless.h"

//#define EARTH_RADIUS (6371000.0F)
#ifndef PI
#define  PI  (3.1415926535897932384626433832795)
#endif
// defined by Arduino.h   #define radians(x) ( (x)*180.0F/PI )

struct GpsPos gpsPos;

/*
float calcLatLonDist(float lat1, float lon1, float lat2, float lon2) {
        float x = radians(lon1-lon2) * cos( radians((lat1+lat2)/2) );
        float y = radians(lat2-lat1);
        float d = sqrt(x*x+y*y)*EARTH_RADIUS;
	return d;
}
*/