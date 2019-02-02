#ifndef UTILITIES_H
#define UTILITIES_H

#include "json.hpp"
#include <math.h>
#include <vector>

using namespace std;

// For converting back and forth between radians and degrees.
double deg2rad(double x);
double rad2deg(double x);

// For converting back and forth between mph and mps.
double mph_to_mps(double mph); // m.s-1
double mps_to_mph(double ms);

// calculation for euclidean distance
double distance(double x1, double y1, double x2, double y2);

int ClosestWaypoint(double x, double y, const vector<double>& maps_x, const vector<double>& maps_y);

int NextWaypoint(double x, double y, double theta, const vector<double>& maps_x, const vector<double>& maps_y);

vector<double> getFrenet(double x, double y, double theta, const vector<double>& maps_x, const vector<double>& maps_y);

vector<double> getXY(double s, double d, const vector<double>& maps_s, const vector<double>& maps_x, const vector<double>& maps_y);

vector<double> global2localCarCoord(double ref_x, double ref_y, double ref_yaw, double point_x, double point_y);

vector<double> localCar2globalCoord(double ref_x, double ref_y, double ref_yaw, double point_x, double point_y);

string hasData(string s);

#endif // UTILITY_H