//
// Created by david on 7/26/17.
//

#include "utils.h"

string hasData(string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_first_of("}");
  if (found_null != string::npos) {
    return "";
  } else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 2);
  }
  return "";
}

vector<double> solveQuintic(const vector<double> &start,
                            const vector<double> &end,
                            double delta_T) {
  double a0 = start[0];
  double a1 = start[1];
  double a2 = start[2]*0.5;

  double b0 = (((end[0] - a0)/delta_T - a1)/delta_T - a2)/delta_T;
  double b1 = ((end[1] - a1)/delta_T - 2*a2)/delta_T;
  double b2 = (0.5*end[2] - a2)/delta_T;

  double T2 = delta_T*delta_T;
  Eigen::MatrixXd A(3,3);
  A << 1, delta_T, T2,
          3, 4*delta_T, 5*T2,
          3, 6*delta_T, 10*T2;

  Eigen::VectorXd b(3);
  b << b0, b1, b2;
  Eigen::VectorXd sol = A.inverse() * b;
  return {a0, a1, a2, sol(0), sol(1), sol(2)};
}

vector<double> solveQuartic(const vector<double> &start,
                            const vector<double> &end,
                            double delta_T) {
  double a0 = start[0];
  double a1 = start[1];
  double a2 = start[2]*0.5;

  double b1 = ((end[0] - a1)/delta_T - 2*a2)/delta_T;
  double b2 = (0.5*end[1] - a2)/delta_T;

  Eigen::MatrixXd A(2,2);
  A << 3, 4*delta_T,
       3, 6*delta_T;

  Eigen::VectorXd b(2);
  b << b1, b2;
  Eigen::VectorXd sol = A.inverse() * b;
  return {a0, a1, a2, sol(0), sol(1)};
}

vector<double> forwardSSearch(const vector<double> &start, const vector<double> &end, double T_offset) {
  double t = S_SEARCH_START_T - T_offset;
  while (1) {
    if (t < 0) {
      t += S_SEARCH_RES;
      continue;
    }
    vector<double> s_curve;
    if (end.size() == 2) s_curve = solveQuartic(start, end, t);
    else s_curve = solveQuintic(start, end, t);
    if (SFeasible(s_curve, t)) {
      s_curve.push_back(t);
      return s_curve;
    }
    t += S_SEARCH_RES;
  }
}

bool SFeasible(const vector<double> &s_curve, double delta_T) {
  vector<double> s1 = polyDiff(s_curve);
  vector<double> s2 = polyDiff(s1);
  vector<double> s3 = polyDiff(s2);

  for (double t = 0; t <= delta_T; t += STEP_T)
    if (abs(polyEval(s1, t)) >= ROAD_SPEED_LIMIT ||
        abs(polyEval(s2, t)) >= S_ACC_LIMIT ||
        abs(polyEval(s3, t) >= S_JERK_LIMIT))
      return false;
  return true;
}

vector<double> polyDiff(const vector<double> &curve) {
  vector<double> diff;
  for (int i = 1; i < curve.size(); ++i)
    diff.push_back(i*curve[i]);
  if (diff.empty()) diff.push_back(0);
  return diff;
}

double polyEval(const vector<double> &curve, double T) {
  double value = 0;
  for (int i = curve.size()-1; i >= 0; --i)
    value = value*T + curve[i];
  return value;
}