//
// Created by david on 7/24/17.
//

#ifndef PATH_PLANNING_UTILS_H
#define PATH_PLANNING_UTILS_H

#include <cmath>
#include <vector>
#include "json.hpp"
#include "Eigen-3.3/Eigen/Dense"
#include "constants.h"

using namespace std;

// for convenience
using json = nlohmann::json;

// For converting back and forth between radians and degrees.
inline constexpr double pi() { return M_PI; }
inline double deg2rad(double x) { return x * pi() / 180; }
inline double rad2deg(double x) { return x * 180 / pi(); }
inline double lane2d(int lane) { return lane * LANE_WIDTH + HALF_WIDTH; }
inline int d2lane(double d) {
  if (d < 0) return -1;
  for (int i = 0; i < LANES; ++i) if (abs(d - LANE_WIDTH*i - HALF_WIDTH) <= HALF_WIDTH) return i;
  return LANES;
}

inline double distance(double x1, double y1, double x2, double y2)
{
  return sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1));
}

inline double normalizeS(double s) {
  if (s < 0) s += MAX_S;
  if (s >= MAX_S) s -= MAX_S;
  return s;
}



// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(string s);

vector<double> solveQuintic(const vector<double> &start,
                            const vector<double> &end,
                            double delta_T);

vector<double> solveQuartic(const vector<double> &start,
                            const vector<double> &end,
                            double delta_T);

vector<double> forwardSSearch(const vector<double> &start,
                              const vector<double> &end,
                              double T_offset);

bool SFeasible(const vector<double> &s_curve, double delta_T);

vector<double> polyDiff(const vector<double> &curve);

double polyEval(const vector<double> &curve, double T);
#endif //PATH_PLANNING_UTILS_H
