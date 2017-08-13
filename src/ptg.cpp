//
// Created by david on 7/24/17.
//

// A* may not be applicable
// cuz the road structure is not taken into account.

#include "ptg.h"
#include "constants.h"
#include "spline.h"
#include "utils.h"

using namespace std;

PTG::PTG(const vector<double> &map_waypoints_x, const vector<double> &map_waypoints_y,
         const vector<double> &map_waypoints_s, const vector<double> &map_waypoints_dx,
         const vector<double> &map_waypoints_dy) {
  initialized = false;
  vector<double> X;
  X.reserve(map_waypoints_x.size()+1);
  X.insert(X.end(), map_waypoints_x.begin(), map_waypoints_x.end());
  X.push_back(map_waypoints_x[0]);

  vector<double> Y;
  Y.reserve(map_waypoints_y.size()+1);
  Y.insert(Y.end(), map_waypoints_y.begin(), map_waypoints_y.end());
  Y.push_back(map_waypoints_y[0]);

  vector<double> DX;
  DX.reserve(map_waypoints_dx.size()+1);
  DX.insert(DX.end(), map_waypoints_dx.begin(), map_waypoints_dx.end());
  DX.push_back(map_waypoints_dx[0]);

  vector<double> DY;
  DY.reserve(map_waypoints_dy.size()+1);
  DY.insert(DY.end(), map_waypoints_dy.begin(), map_waypoints_dy.end());
  DY.push_back(map_waypoints_dy[0]);

  vector<double> S;
  S.reserve(map_waypoints_s.size()+1);
  S.insert(S.end(), map_waypoints_s.begin(), map_waypoints_s.end());
  S.push_back(MAX_S);

  map_x.set_points(S, X);
  map_y.set_points(S, Y);
  map_dx.set_points(S, DX);
  map_dy.set_points(S, DY);
}

PTG::~PTG() {}

// Transform from Frenet s,d coordinates to Cartesian x,y
vector<double> PTG::getXY(double s, double d) {
  if (s > MAX_S) s -= MAX_S;
  double base_x = map_x(s);
  double base_y = map_y(s);
  double dx = map_dx(s);
  double dy = map_dy(s);
  return {base_x+dx*d, base_y+dy*d};
}

pair<vector<double>,vector<double>> PTG::update_trajectory(const vector<double> &local_data,
                                                           const vector<double> &previous_path_x,
                                                           const vector<double> &previous_path_y,
                                                           const vector<vector<double>> &sensor_fusion) {
  vector<double> ret_path_x, ret_path_y;

  if (!initialized) {
    start_s = local_data[2];
    start_sd = 0;
    start_sdd = 0;
    start_d = local_data[3];
    from_step = 0;
    state = FL;
    initialized = true;
  } else if (state != LC){
    from_step = TRACK_LIMIT + previous_path_x.size() - STEPS;
  } else {
    if (previous_path_x.size() > TRACK_LIMIT)
      return make_pair(previous_path_x, previous_path_y);
    from_step = previous_path_x.size();
    state = FL;
  }

  if (from_step > 0) {
    ret_path_x.reserve(from_step);
    ret_path_x.insert(ret_path_x.end(), previous_path_x.begin(), previous_path_x.begin() + from_step);
    ret_path_y.reserve(from_step);
    ret_path_y.insert(ret_path_y.end(), previous_path_y.begin(), previous_path_y.begin() + from_step);
  }

  double t_offset = from_step * STEP_T;
  int lane = d2lane(start_d);
  vector<vector<int>> lanes = getClosestVehicles(sensor_fusion, local_data[2]);

  vector<double> s_free = forwardSSearch({start_s, start_sd, start_sdd}, {TARGET_SPEED, 0}, t_offset);
  double s_end_t = s_free.back();
  s_free.pop_back();
  double pred_s = normalizeS(polyEval(s_free, PRED_LIMIT-t_offset));

  if (state == FL) {
    if (lanes[lane][0] == -1)
      return getInLanePath(s_free, ret_path_x, ret_path_y);

    double ts = sensor_fusion[lanes[lane][0]][5];
    if ((local_data[2] < pred_s && pred_s < ts) ||
        (ts < local_data[2] && (pred_s > local_data[2] || pred_s < ts)))
      return getInLanePath(s_free, ret_path_x, ret_path_y);
    state = FF;
  }

  // See if left lane change is possible
  if (lane > 0) {
    int target_lane = lane-1;
    if (lanes[target_lane][0] == -1) {
      // perform left lane change
      state = LC;
      return getLaneChangePath(s_free, s_end_t, target_lane, ret_path_x, ret_path_y);
    }

    const vector<double> &f = sensor_fusion[lanes[target_lane][0]];

    double f_s = f[5];
    if ((local_data[2] < pred_s && pred_s < f_s) ||
        (f_s < local_data[2] && (pred_s > local_data[2] || pred_s < f_s))) {

      const vector<double> &b = sensor_fusion[lanes[target_lane][1]];
      double b_s = b[5];
      double b_dx = map_dx(b_s);
      double b_dy = map_dy(b_s);
      double b_sd = -b_dy*b[3] + b_dx*b[4];

      if (b_s + b_sd < local_data[2]) {
        // perform left lane change
        state = LC;
        return getLaneChangePath(s_free, s_end_t, target_lane, ret_path_x, ret_path_y);
      }
    }
  }

  // See if right lane change is possible
  if (lane < LANES-1) {
    int target_lane = lane+1;
    if (lanes[target_lane][0] == -1) {
      // perform right lane change
      state = LC;
      return getLaneChangePath(s_free, s_end_t, target_lane, ret_path_x, ret_path_y);
    }

    const vector<double> &f = sensor_fusion[lanes[target_lane][0]];

    double f_s = f[5];
    if ((local_data[2] < pred_s && pred_s < f_s) ||
        (f_s < local_data[2] && (pred_s > local_data[2] || pred_s < f_s))) {

      const vector<double> &b = sensor_fusion[lanes[target_lane][1]];
      double b_s = b[5];
      double b_dx = map_dx(b_s);
      double b_dy = map_dy(b_s);
      double b_sd = -b_dy*b[3] + b_dx*b[4];

      if (b_s + b_sd < local_data[2]) {
        // perform right lane change
        state = LC;
        return getLaneChangePath(s_free, s_end_t, target_lane, ret_path_x, ret_path_y);
      }
    }
  }

  // Follow the front vehicle.
  const vector<double> &f = sensor_fusion[lanes[lane][0]];
  double f_s = f[5];
  double f_dx = map_dx(f_s);
  double f_dy = map_dy(f_s);
  double f_sd = -f_dy*f[3] + f_dx*f[4];
  double diff_s = normalizeS(f_s - start_s);
  if (diff_s < CAR_SAFE_DIST_2) f_sd *= .6;
  else if (diff_s < CAR_SAFE_DIST_1) f_sd *= .8;
  else if (diff_s < CAR_SAFE_DIST_0) f_sd *= 0.95;

  if ((f_sd >= TARGET_SPEED) ||
      (local_data[2] < pred_s && pred_s < f_s) ||
      (f_s < local_data[2] && (pred_s > local_data[2] || pred_s < f_s)))
    state = FL;

  vector<double> s_follow = forwardSSearch({start_s, start_sd, start_sdd},
                                           {f_sd, 0}, t_offset);
  return getInLanePath(s_follow, ret_path_x, ret_path_y);
}

/**
 * Get the closest vehicles (vehicle at direct front and back of the main car) from each lane
 * @param sensor_fusion
 * @param s current main car's s coordinate
 * @return list of pair of indices. Index 0 is the vehicle at direct front, index 1 is that at direct back
 */
vector<vector<int>> PTG::getClosestVehicles(const vector<vector<double>> &sensor_fusion, double s) {

  vector<vector<int>> lanes;
  for (int i = 0; i < LANES; ++i) lanes.push_back({-1, -1});

  for (int i = 0; i < sensor_fusion.size(); ++i) {
    const vector<double> &v = sensor_fusion[i];
    double v_s = v[5];
    double v_d = v[6];
    int lane = d2lane(v_d);
    if (lane < 0 || lane > LANES-1) continue;
    if (lanes[lane][0] == -1) {
      lanes[lane][0] = i;
    } else {
      const vector<double> &f = sensor_fusion[lanes[lane][0]];
      double f_s = f[5];
      if ((v_s < f_s && (v_s >= s || f_s <= s)) || (f_s < s && s <= v_s)) lanes[lane][0] = i;
    }

    if (lanes[lane][1] == -1) lanes[lane][1] = i;
    else {
      const vector<double> &b = sensor_fusion[lanes[lane][1]];
      double b_s = b[5];
      if ((v_s > b_s && (v_s <= s || b_s >= s)) || (v_s <= s && b_s > s)) lanes[lane][1] = i;
    }
  }
  return lanes;
}

/**
 * Get the path in (x,y) coordinates for lane-following, and update the status being tracked
 * @param s_curve
 * @param ret_path_x
 * @param ret_path_y
 * @return
 */
pair<vector<double>, vector<double>> PTG::getInLanePath(const vector<double> &s_curve, vector<double> &ret_path_x,
                                                        vector<double> &ret_path_y) {
  for (int i = 0; i < STEPS-from_step; ++i) {
    double s = normalizeS(polyEval(s_curve, (i+1)*STEP_T));
    vector<double> xy = getXY(s, start_d);
    ret_path_x.push_back(xy[0]);
    ret_path_y.push_back(xy[1]);
  }
  vector<double> s1 = polyDiff(s_curve);
  vector<double> s2 = polyDiff(s1);
  double track_t = (TRACK_LIMIT-from_step)*STEP_T;
  start_s = normalizeS(polyEval(s_curve, track_t));
  start_sd = polyEval(s1, track_t);
  start_sdd = polyEval(s2, track_t);
  return make_pair(ret_path_x, ret_path_y);
}

/**
 * Get the path in (x,y) coordinates for lane-chanding, and update the status being tracked
 * @param s_curve
 * @param s_end_t
 * @param target_lane
 * @param ret_path_x
 * @param ret_path_y
 * @return
 */
pair<vector<double>, vector<double>> PTG::getLaneChangePath(const vector<double> &s_curve, double s_end_t, int target_lane,
                                                            vector<double> &ret_path_x, vector<double> &ret_path_y) {
  double target_d = lane2d(target_lane);
  vector<double> d_curve = solveQuintic({start_d, 0, 0}, {lane2d(target_lane), 0, 0}, LANE_CHANGE_T);
  vector<double> s1 = polyDiff(s_curve);
  vector<double> s2 = polyDiff(s1);

  const double s_end = polyEval(s_curve, s_end_t);
  const double sd_end = polyEval(s1, s_end_t);

  double s = start_s, d = start_d;
  for (double t = STEP_T; t <= LANE_CHANGE_T; t += STEP_T) {
    if (t <= s_end_t) s = polyEval(s_curve, t);
    else s = s_end + sd_end*(t-s_end_t);
    s = normalizeS(s);
    d = polyEval(d_curve, t);
    vector<double> xy = getXY(s, d);
    ret_path_x.push_back(xy[0]);
    ret_path_y.push_back(xy[1]);
  }

  if (LANE_CHANGE_T <= s_end_t) {
    start_s = normalizeS(polyEval(s_curve, LANE_CHANGE_T-STEP_T));
    start_sd = polyEval(s1, LANE_CHANGE_T-STEP_T);
    start_sdd = polyEval(s2, LANE_CHANGE_T-STEP_T);
  } else {
    start_s = normalizeS(s_end + sd_end*(LANE_CHANGE_T-s_end_t-STEP_T));
    start_sd = sd_end;
    start_sdd = polyEval(s2, s_end_t);
  }
  start_d = target_d;
  return make_pair(ret_path_x, ret_path_y);
}