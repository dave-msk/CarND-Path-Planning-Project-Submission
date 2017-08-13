//
// Created by david on 7/24/17.
//

#ifndef PATH_PLANNING_PTG_H
#define PATH_PLANNING_PTG_H

#include <vector>
#include <queue>
#include <map>
#include <string>
#include "spline.h"

using namespace std;

class PTG {
public:

  tk::spline map_x;
  tk::spline map_y;
  tk::spline map_dx;
  tk::spline map_dy;

  enum State {FL, FF, LC};

  PTG(const vector<double> &map_waypoints_x,
      const vector<double> &map_waypoints_y,
      const vector<double> &map_waypoints_s,
      const vector<double> &map_waypoints_dx,
      const vector<double> &map_waypoints_dy);

  virtual ~PTG();

  pair<vector<double>,vector<double>> update_trajectory(const vector<double> &local_data,
                                                        const vector<double> &previous_path_x,
                                                        const vector<double> &previous_path_y,
                                                        const vector<vector<double>> &sensor_fusion);



private:

  State state;

  double start_s, start_sd, start_sdd, start_d;
  int from_step;
  bool initialized;

  vector<double> getXY(double s, double d);
  vector<vector<int>> getClosestVehicles(const vector<vector<double>> &sensor_fusion, double s);
  pair<vector<double>, vector<double>> getInLanePath(const vector<double> &s_curve, vector<double> &ret_path_x, vector<double> &ret_path_y);
  pair<vector<double>, vector<double>> getLaneChangePath(const vector<double> &s_curve, double s_end_t, int target_lane, vector<double> &ret_path_x, vector<double> &ret_path_y);
};

#endif //PATH_PLANNING_PTG_H
