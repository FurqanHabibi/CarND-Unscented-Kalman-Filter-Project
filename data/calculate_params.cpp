#include <iostream>
#include <string>
#include <fstream>
#include <vector>
#include <cmath>
#include <sstream>

using namespace std;

int main() {
  ifstream dataFile("obj_pose-laser-radar-synthetic-input.txt");
  string line;
  vector<float> longitudinal_accels;
  vector<float> yaw_accels;
  float last_longitudinal_speed = nanf("");
  float last_yaw_speed = nanf("");
  long long last_timestamp = 0;
  if (dataFile.is_open()) {
    while (getline(dataFile, line)) {
      istringstream iss(line);
      string temp;

      iss >> temp; // sensor_type
      if (temp.compare("L") == 0) {
        iss >> temp; // x_measured
        iss >> temp; // y_measured
      }
      else {
        iss >> temp; // rho_measured
        iss >> temp; // phi_measured
        iss >> temp; // rhodot_measured
      }
      long long timestamp;
      iss >> timestamp; // timestamp

      iss >> temp; // x_groundtruth
      iss >> temp; // y_groundtruth

      float vx;
      iss >> vx; // vx_groundtruth
      float vy;
      iss >> vy; // vy_groundtruth
      float longitudinal_speed = sqrt(vx * vx + vy * vy);

      float yaw;
      iss >> yaw; // yaw_groundtruth
      float yawrate;
      iss >> yawrate; // yawrate_groundtruth

      if (!isnan(last_longitudinal_speed)) {
        longitudinal_accels.push_back((longitudinal_speed - last_longitudinal_speed) / ((timestamp - last_timestamp) * 0.000001));
        yaw_accels.push_back((yawrate - last_yaw_speed) / ((timestamp - last_timestamp) * 0.000001));
      }

      last_longitudinal_speed = longitudinal_speed;
      last_yaw_speed = yawrate;
      last_timestamp = timestamp;
    }
  }
  dataFile.close();

  std::vector<float>::iterator it;

  float sum_longitudinal_accel = 0;
  for (it = longitudinal_accels.begin(); it != longitudinal_accels.end(); ++it) {
    sum_longitudinal_accel += *it;
  }
  float mean_longitudinal_accel = sum_longitudinal_accel / longitudinal_accels.size();
  
  float sum_longitudinal_accel_squared_difference = 0;
  for (it = longitudinal_accels.begin(); it != longitudinal_accels.end(); ++it) {
    sum_longitudinal_accel_squared_difference += pow((*it - mean_longitudinal_accel), 2);
  }
  float stddev_longitudinal_accel = sqrt(sum_longitudinal_accel_squared_difference / longitudinal_accels.size());

  float sum_yaw_accel = 0;
  for (it = yaw_accels.begin(); it != yaw_accels.end(); ++it) {
    sum_yaw_accel += *it;
  }
  float mean_yaw_accel = sum_yaw_accel / yaw_accels.size();

  float sum_yaw_accel_squared_difference = 0;
  for (it = yaw_accels.begin(); it != yaw_accels.end(); ++it) {
    sum_yaw_accel_squared_difference += pow((*it - mean_yaw_accel), 2);
  }
  float stddev_yaw_accel = sqrt(sum_yaw_accel_squared_difference / yaw_accels.size());

  cout << "Longitudinal acceleration mean = " << mean_longitudinal_accel << endl;
  cout << "Longitudinal acceleration stddev = " << stddev_longitudinal_accel << endl;
  cout << endl;
  cout << "Yaw acceleration mean = " << mean_yaw_accel << endl;
  cout << "Yaw acceleration stddev = " << stddev_yaw_accel << endl;

  ofstream accelFile("accel-data.txt", ios::trunc);
  if (accelFile.is_open()) {
    for (int i = 0; i < longitudinal_accels.size(); ++i) {
      accelFile << longitudinal_accels[i] << "    " << yaw_accels[i] << endl;
    }
    accelFile.close();
  }

  return 0;
}
