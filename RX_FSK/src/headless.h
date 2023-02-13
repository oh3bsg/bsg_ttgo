#ifndef HEADLESS_H
#define HEADLESS_H

struct GpsPos {
	double lat;
	double lon;
	int alt;
	int course;
	float speed;
	int sat;
	int accuracy;
	int hdop;
	int valid;
};
extern struct GpsPos gpsPos;

//float calcLatLonDist(float lat1, float lon1, float lat2, float lon2);

#endif // HEADLESS_H