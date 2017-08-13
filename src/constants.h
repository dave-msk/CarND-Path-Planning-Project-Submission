//
// Created by david on 7/25/17.
//

#ifndef PATH_PLANNING_CONSTANTS_H
#define PATH_PLANNING_CONSTANTS_H

const double MAX_S = 6945.554;
const double TARGET_SPEED = 20.8;
const double ROAD_SPEED_LIMIT = 22;
const double S_ACC_LIMIT = 5;
const double S_JERK_LIMIT = 5;
const int STEPS = 50;
const double STEP_T = 0.02;
const double PRED_LIMIT = 1.5;
const double LANE_CHANGE_T = 2;
const double S_SEARCH_START_T = PRED_LIMIT;
const double S_SEARCH_RES = 0.5;
const int TRACK_LIMIT = 20;

const int LANES = 3;
const double LANE_WIDTH = 4;
const double HALF_WIDTH = LANE_WIDTH/2;
const double CAR_SAFE_DIST_0 = 20;
const double CAR_SAFE_DIST_1 = 15;
const double CAR_SAFE_DIST_2 = 10;


#endif //PATH_PLANNING_CONSTANTS_H
