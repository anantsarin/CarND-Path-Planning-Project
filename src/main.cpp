#include <uWS/uWS.h>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "helpers.h"
#include "json.hpp"
#include "spline.h"

// for convenience
using nlohmann::json;
using std::string;
using std::vector;

double get_cost(float dist) {
  /* code */
  double cost  = (1 - exp(-1/dist));
  return cost;
}

int find_next_lane(vector<vector<double >> sensor_fusion, double car_s, int path_size, int lane, double same_lane_cost){
  int my_lane = lane;
  double cost = 0.0;
  vector<double> cost_total;
  for(int j=0; j<2 ; j++){
    cost = 0.0;
    if(my_lane==0 || my_lane==2){
      lane = 1;
      j++;
    }
    else{
      lane = j*2;
    }
    // int count = 0;
    for(int i =0; i<sensor_fusion.size(); i++){
      float d = sensor_fusion[i][6]; //ith cars
      if(d < (2+4*lane+2) && d > (2+4*lane-2)){
          //my lane
        double vx = sensor_fusion[i][3];
              double vy = sensor_fusion[i][4];
              double check_speed = sqrt(vx*vx+vy*vy);
              double check_car_s = sensor_fusion[i][5];

              check_car_s += ((double)path_size*0.02*check_speed);
              if((check_car_s > car_s)&& ((check_car_s - car_s)< 30)){
                //remain in the lane 2
                cost += get_cost((check_car_s - car_s));
                // count++;
              }
              if((check_car_s < car_s)&& ((car_s - check_car_s)< 10)) {
                // there is a car at the back
                cost += get_cost((car_s - check_car_s));
                // count++;
              }
        }
    }
    cost_total.push_back(cost);
    // std::cout << count << std::endl;
    if(lane==1){ // if destination is lane 1
      if(same_lane_cost <= cost_total[0]){
        // std::cout << "current lane  == 0/2 no lane chage" << my_lane << std::endl;
        return my_lane;
      }
      else{
        // std::cout << "current lane  == 0/2 chage late to 1" << "1" << std::endl;
        return 1;
      }
    }
    else{
      //return 0 lane number
      int ans  = (cost_total[0] < cost_total[1]) ? ((cost_total[0] < same_lane_cost)?0:1):((cost_total[1] < same_lane_cost)?2:1) ;
      // std::cout << "lane not = 1 " << ans << std::endl;
      return ans;
    }
  }
}


int main() {
  uWS::Hub h;

  // Load up map values for waypoint's x,y,s and d normalized normal vectors
  vector<double> map_waypoints_x;
  vector<double> map_waypoints_y;
  vector<double> map_waypoints_s;
  vector<double> map_waypoints_dx;
  vector<double> map_waypoints_dy;

  // Waypoint map to read from
  string map_file_ = "../data/highway_map.csv";
  // The max s value before wrapping around the track back to 0
  double max_s = 6945.554;

  std::ifstream in_map_(map_file_.c_str(), std::ifstream::in);

  string line;
  while (getline(in_map_, line)) {
    std::istringstream iss(line);
    double x;
    double y;
    float s;
    float d_x;
    float d_y;
    iss >> x;
    iss >> y;
    iss >> s;
    iss >> d_x;
    iss >> d_y;
    map_waypoints_x.push_back(x);
    map_waypoints_y.push_back(y);
    map_waypoints_s.push_back(s);
    map_waypoints_dx.push_back(d_x);
    map_waypoints_dy.push_back(d_y);
  }

  int lane = 1; // init lane
  double ref_vel = 0.0;

  h.onMessage([&lane,&ref_vel,&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,&map_waypoints_dx,&map_waypoints_dy](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {

      auto s = hasData(data);

      if (s != "") {
        auto j = json::parse(s);

        string event = j[0].get<string>();

        if (event == "telemetry") {
          // j[1] is the data JSON object

          // Main car's localization Data
          double car_x = j[1]["x"];
          double car_y = j[1]["y"];
          double car_s = j[1]["s"];
          double car_d = j[1]["d"];
          double car_yaw = j[1]["yaw"];
          double car_speed = j[1]["speed"];

          // Previous path data given to the Planner
          auto previous_path_x = j[1]["previous_path_x"];
          auto previous_path_y = j[1]["previous_path_y"];
          // Previous path's end s and d values
          double end_path_s = j[1]["end_path_s"];
          double end_path_d = j[1]["end_path_d"];

          // Sensor Fusion Data, a list of all other cars on the same side
          //   of the road.
          auto sensor_fusion = j[1]["sensor_fusion"];

          json msgJson;



          /**
           * TODO: define a path made up of (x,y) points that the car will visit
           *   sequentially every .02 seconds
           */

          int path_size = previous_path_x.size();

          if (path_size>0){
            car_s = end_path_s;
          }
          bool too_close = false;
          bool car_in_left_lane = false;
          bool car_in_right_lane = false;

          //find ref_v to use
          for(int i =0; i<sensor_fusion.size(); i++){
            //car in my lane
            float d = sensor_fusion[i][6]; //ith cars
            //////////////////////
            if(d < (2+4*lane+2) && d > (2+4*lane-2)){
              double vx = sensor_fusion[i][3];
              double vy = sensor_fusion[i][4];
              double check_speed = sqrt(vx*vx+vy*vy);
              double check_car_s = sensor_fusion[i][5];

              check_car_s += ((double)path_size*0.02*check_speed);
              if((check_car_s > car_s)&& ((check_car_s - car_s)< 30)){
                //ref_vel = 29.5;
                // check the cost of right lane and left lane change
                // if not slow down

                double same_lane_cost  = get_cost((check_car_s - car_s));
                int next_lane =   find_next_lane(sensor_fusion, car_s, path_size, lane,same_lane_cost);

                if(next_lane == lane){
                  too_close = true;
                }
                else{
                  lane = next_lane;
                }
              }
            }

          }

          if(too_close){
            ref_vel -= 0.224;
          }
          else if(ref_vel < 49.0){
            ref_vel += 0.224;
          }

          std::vector<double> ptsx;
          std::vector<double> ptsy;

          double ref_x = car_x;
          double ref_y = car_y;
          double ref_yaw = deg2rad(car_yaw);

          if( path_size < 2){
            double prev_car_x = car_x - cos(car_yaw);
            double prev_car_y = car_y - sin(car_yaw);

            ptsx.push_back(prev_car_x);
            ptsx.push_back(car_x);

            ptsy.push_back(prev_car_y);
            ptsy.push_back(car_y);

          }
          else {
            ref_x = previous_path_x[path_size-1];
            ref_y = previous_path_y[path_size-1];


            double ref_x2 = previous_path_x[path_size-2];
            double ref_y2 = previous_path_y[path_size-2];
            double angle = atan2(ref_y-ref_y2,ref_x-ref_x2);

            ptsx.push_back(ref_x2);
            ptsx.push_back(ref_x);
            ptsy.push_back(ref_y2);
            ptsy.push_back(ref_y);
          }


          std::vector<double> next_wp0 = getXY(car_s+30, (2+4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
          std::vector<double> next_wp1 = getXY(car_s+60, (2+4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
          std::vector<double> next_wp2 = getXY(car_s+90, (2+4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);

          ptsx.push_back(next_wp0[0]);
          ptsx.push_back(next_wp1[0]);
          ptsx.push_back(next_wp2[0]);

          ptsy.push_back(next_wp0[1]);
          ptsy.push_back(next_wp1[1]);
          ptsy.push_back(next_wp2[1]);

          for(int i =0; i<ptsx.size(); i++){
            double shift_x = ptsx[i] - ref_x;
            double shift_y = ptsy[i] - ref_y;

            ptsx[i] = (shift_x * cos(0-ref_yaw)-shift_y*sin(0-ref_yaw));
            ptsy[i] = (shift_x * sin(0-ref_yaw)+shift_y*cos(0-ref_yaw));
          }

          tk::spline s;

          s.set_points(ptsx, ptsy);

          vector<double> next_x_vals;
          vector<double> next_y_vals;



          for (int i = 0; i < path_size; ++i) {
            next_x_vals.push_back(previous_path_x[i]);
            next_y_vals.push_back(previous_path_y[i]);
          }
          double target_x = 30.0;
          double target_y = s(target_x);
          double target_dist = sqrt(target_x*target_x+target_y*target_y);

          double x_add_on = 0;

          for (int i = 0; i < 50-path_size; i++){
            double N = target_dist/(0.02*ref_vel/2.24);
            double x_point = x_add_on+(target_x/N );
            double y_point = s(x_point);

            x_add_on = x_point;

            double x_ref = x_point;
            double y_ref = y_point;

            x_point = ref_x + (x_ref * cos(ref_yaw)-y_ref*sin(ref_yaw));
            y_point = ref_y + (x_ref * sin(ref_yaw)+y_ref*cos(ref_yaw));

            next_x_vals.push_back(x_point);
            next_y_vals.push_back(y_point);
          }

          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;

          auto msg = "42[\"control\","+ msgJson.dump()+"]";

          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }  // end "telemetry" if
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }  // end websocket if
  }); // end h.onMessage

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
                         char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen(port)) {
    std::cout << "Listening to port " << port << std::endl;
  } else {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }

  h.run();
}