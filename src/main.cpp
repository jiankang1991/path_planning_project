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

#include "constants.h"
#include "vehicle.h"

// for convenience
using nlohmann::json;
using std::string;
using std::vector;
using std::cout;
using std::endl;

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
  
  
  
  double ref_vel = 0.0;
  int car_lane = 1;
  vector<int> lanes = {0,1,2};

  vehicle mycar = vehicle();

  mycar.lane = car_lane;

  h.onMessage([&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,
               &map_waypoints_dx,&map_waypoints_dy, &ref_vel, &lanes, &car_lane, &mycar]
              (uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
               uWS::OpCode opCode) {
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
          
        //   cout<<"car_s " << car_s <<endl;

          double car_d = j[1]["d"];
          double car_yaw = j[1]["yaw"];
          double car_speed = j[1]["speed"];
        //   cout<< "car_speed: " << car_speed << endl;

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

          vector<double> next_x_vals;
          vector<double> next_y_vals;

          //log
          ofstream single_iteration_log;
          single_iteration_log.open("path_planning_log-single_iteration.csv");
        
          /**
           * TODO: define a path made up of (x,y) points that the car will visit
           *   sequentially every .02 seconds
           */
          
          // define the start point of the car

          mycar.x = car_x;
          mycar.y = car_y;
          mycar.yaw = deg2rad(car_yaw);
          mycar.speed = car_speed;


          // reference points for coordinate transformation (shift and rotation)
          double ref_x = mycar.x;
          double ref_y = mycar.y;
          double ref_yaw = mycar.yaw;

          // anchor points set to construct smooth spline curve
          vector<double> ptsx;
          vector<double> ptsy;
          
          
          int prev_size = previous_path_x.size();

          // set the car location of s in frenet coordinate system
          if (prev_size>0) {
              mycar.s = end_path_s;
          } else {
              mycar.s = car_s;
          }
          // define six cars we should pay attention to, front and behind cars in each lane
          vehicle left_lane_front_vehicle = vehicle();
          vehicle left_lane_behind_vehicle = vehicle();

          vehicle mid_lane_front_vehicle = vehicle();
          vehicle mid_lane_behind_vehicle = vehicle();

          vehicle right_lane_front_vehicle = vehicle();
          vehicle right_lane_behind_vehicle = vehicle();


          vector<vehicle> left_lane_vehicles = {left_lane_front_vehicle, left_lane_behind_vehicle};
          vector<vehicle> mid_lane_vehicles = {mid_lane_front_vehicle, mid_lane_behind_vehicle};
          vector<vehicle> right_lane_vehicles = {right_lane_front_vehicle, right_lane_behind_vehicle};


          double left_front_s = 999999;
          double left_behind_s = -99999;
          double mid_front_s = 999999;
          double mid_behind_s = -99999;
          double right_front_s = 999999;
          double right_behind_s = -99999;

          // select the nearest cars (front and behind) in each lane 
          for (int j=0; j<sensor_fusion.size(); j++) {

              float d = sensor_fusion[j][6];
              double vx = sensor_fusion[j][3];
              double vy = sensor_fusion[j][4];
              double check_speed = sqrt(vx*vx+vy*vy);

              double check_car_s = sensor_fusion[j][5];
              check_car_s += (double)prev_size*TIME_STEP*check_speed;

              // double check_car_behind_s = sensor_fusion[j][5];
              // check_car_behind_s -= (double)prev_size*0.02*check_speed;


              if (fabs(d-2-4*lanes[0])<2) {

                if ((check_car_s<left_front_s)&&(mycar.s<=check_car_s)) {

                  left_front_s = check_car_s;
                  
                  left_lane_vehicles[0].d = d;
                  left_lane_vehicles[0].lane = lanes[0];
                  left_lane_vehicles[0].s = sensor_fusion[j][5];
                  left_lane_vehicles[0].speed = check_speed;

                  left_lane_vehicles[0].x = sensor_fusion[j][1];
                  left_lane_vehicles[0].y = sensor_fusion[j][2];
                  left_lane_vehicles[0].vx = sensor_fusion[j][3];
                  left_lane_vehicles[0].vy = sensor_fusion[j][4];
                  left_lane_vehicles[0].yaw = atan2(left_lane_vehicles[0].vy, left_lane_vehicles[0].vx);

                } else if ((check_car_s>left_behind_s)&&(check_car_s<mycar.s)) {

                  left_behind_s = check_car_s;

                  left_lane_vehicles[1].d = d;
                  left_lane_vehicles[1].lane = lanes[0];
                  left_lane_vehicles[1].s = sensor_fusion[j][5];
                  left_lane_vehicles[1].speed = check_speed;

                  left_lane_vehicles[1].x = sensor_fusion[j][1];
                  left_lane_vehicles[1].y = sensor_fusion[j][2];
                  left_lane_vehicles[1].vx = sensor_fusion[j][3];
                  left_lane_vehicles[1].vy = sensor_fusion[j][4];
                  left_lane_vehicles[1].yaw = atan2(left_lane_vehicles[1].vy, left_lane_vehicles[1].vx);

                }

              }

              if (fabs(d-2-4*lanes[1])<2) {

                if ((check_car_s<mid_front_s)&&(mycar.s<=check_car_s)) {

                  mid_front_s = check_car_s;
                  
                  mid_lane_vehicles[0].d = d;
                  mid_lane_vehicles[0].lane = lanes[1];
                  mid_lane_vehicles[0].s = sensor_fusion[j][5];
                  mid_lane_vehicles[0].speed = check_speed;

                  mid_lane_vehicles[0].x = sensor_fusion[j][1];
                  mid_lane_vehicles[0].y = sensor_fusion[j][2];
                  mid_lane_vehicles[0].vx = sensor_fusion[j][3];
                  mid_lane_vehicles[0].vy = sensor_fusion[j][4];
                  mid_lane_vehicles[0].yaw = atan2(mid_lane_vehicles[0].vy, mid_lane_vehicles[0].vx);

                } else if ((check_car_s>mid_behind_s)&&(check_car_s<mycar.s)) {

                  mid_behind_s = check_car_s;

                  mid_lane_vehicles[1].d = d;
                  mid_lane_vehicles[1].lane = lanes[1];
                  mid_lane_vehicles[1].s = sensor_fusion[j][5];
                  mid_lane_vehicles[1].speed = check_speed;

                  mid_lane_vehicles[1].x = sensor_fusion[j][1];
                  mid_lane_vehicles[1].y = sensor_fusion[j][2];
                  mid_lane_vehicles[1].vx = sensor_fusion[j][3];
                  mid_lane_vehicles[1].vy = sensor_fusion[j][4];
                  mid_lane_vehicles[1].yaw = atan2(mid_lane_vehicles[1].vy, mid_lane_vehicles[1].vx);

                }

              }

              if (fabs(d-2-4*lanes[2])<2) {

                if ((check_car_s<right_front_s)&&(mycar.s<=check_car_s)) {

                  right_front_s = check_car_s;
                  
                  right_lane_vehicles[0].d = d;
                  right_lane_vehicles[0].lane = lanes[2];
                  right_lane_vehicles[0].s = sensor_fusion[j][5];
                  right_lane_vehicles[0].speed = check_speed;

                  right_lane_vehicles[0].x = sensor_fusion[j][1];
                  right_lane_vehicles[0].y = sensor_fusion[j][2];
                  right_lane_vehicles[0].vx = sensor_fusion[j][3];
                  right_lane_vehicles[0].vy = sensor_fusion[j][4];
                  right_lane_vehicles[0].yaw = atan2(right_lane_vehicles[0].vy, right_lane_vehicles[0].vx);

                } else if ((check_car_s>right_behind_s)&&(check_car_s<mycar.s)) {

                  right_behind_s = check_car_s;

                  right_lane_vehicles[1].d = d;
                  right_lane_vehicles[1].lane = lanes[2];
                  right_lane_vehicles[1].s = sensor_fusion[j][5];
                  right_lane_vehicles[1].speed = check_speed;

                  right_lane_vehicles[1].x = sensor_fusion[j][1];
                  right_lane_vehicles[1].y = sensor_fusion[j][2];
                  right_lane_vehicles[1].vx = sensor_fusion[j][3];
                  right_lane_vehicles[1].vy = sensor_fusion[j][4];
                  right_lane_vehicles[1].yaw = atan2(right_lane_vehicles[1].vy, right_lane_vehicles[1].vx);

                }

              }
          }
          // cout << "prev size: " << prev_size << endl;
          // cout << "mid_front_s dist: " << mid_lane_vehicles[0].s + (double)prev_size*0.02*mid_lane_vehicles[0].speed - mycar.s << endl;
          // cout << "cur_car_s: " << mycar.s << endl;
          // cout << "mid_behind_s dist: " << mycar.s - mid_lane_vehicles[1].s - (double)prev_size*TIME_STEP*mid_lane_vehicles[0].speed << endl;

          // cout << "mycar_ref_vel: " << mycar.ref_vel << endl;
          // next lane label generation
          mycar.next_lane(left_lane_vehicles, mid_lane_vehicles, right_lane_vehicles, prev_size, lanes, map_waypoints_x, map_waypoints_y);

          // cout << "mycar_lane: " << mycar.lane << endl;


          //log
          single_iteration_log << "left_lane_front_vehicle_predictions" << endl;
          for (auto sd_vec: mycar.left_lane_front_car_feature_sd) {
            single_iteration_log << sd_vec[0] << "," << sd_vec[1]; 
          }
          single_iteration_log << endl;

          single_iteration_log << "mid_lane_front_vehicle_predictions" << endl;
          for (auto sd_vec: mycar.mid_lane_front_car_feature_sd) {
            single_iteration_log << sd_vec[0] << "," << sd_vec[1]; 
          }
          single_iteration_log << endl;

          single_iteration_log << "right_lane_front_vehicle_predictions" << endl;
          for (auto sd_vec: mycar.right_lane_front_car_feature_sd) {
            single_iteration_log << sd_vec[0] << "," << sd_vec[1]; 
          }
          single_iteration_log << endl;

          single_iteration_log << "my_car_position" << endl;
          single_iteration_log << mycar.s << "," << 2 + 4*mycar.lane << endl;
          //log



        //   if (ref_vel<49.5) {
        //       ref_vel += 0.224;
        //   }

        //   if (too_close) {
        //       ref_vel -= 0.224;
        //   } else if (ref_vel<49.5) {
        //       ref_vel += 0.224;
        //   }



          // using previous path points for trajectory generation
          if (prev_size < 2) {
            // tangent to the path
            double ref_x_prev = mycar.x - cos(mycar.yaw);
            double ref_y_prev = mycar.y - sin(mycar.yaw);

            // cout<<"ref_x_prev "<< ref_x_prev <<endl;
            // cout<<"ref_x "<< ref_x <<endl;
            
            ptsx.push_back(ref_x_prev);
            ptsx.push_back(ref_x);

            ptsy.push_back(ref_y_prev);
            ptsy.push_back(ref_y);
          } else {

            ref_x = previous_path_x[prev_size-1];
            ref_y = previous_path_y[prev_size-1];

            double ref_x_prev = previous_path_x[prev_size-2];
            double ref_y_prev = previous_path_y[prev_size-2];
            ref_yaw = atan2(ref_y - ref_y_prev, ref_x - ref_x_prev);

            ptsx.push_back(ref_x_prev);
            ptsx.push_back(ref_x);

            ptsy.push_back(ref_y_prev);
            ptsy.push_back(ref_y);

          }

          // add other anchor points into the anchor point set for the spline fitting
          vector<double> next_wp0 = getXY(mycar.s+30, (2+4*mycar.lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
          vector<double> next_wp1 = getXY(mycar.s+60, (2+4*mycar.lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
          vector<double> next_wp2 = getXY(mycar.s+90, (2+4*mycar.lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);

          ptsx.push_back(next_wp0[0]);
          ptsx.push_back(next_wp1[0]);
          ptsx.push_back(next_wp2[0]);

          ptsy.push_back(next_wp0[1]);
          ptsy.push_back(next_wp1[1]);
          ptsy.push_back(next_wp2[1]);

          // coordinate system transformation from global map system to local car system
          for (int i=0; i<ptsx.size(); i++) {
            // MPC
            //shift car reference angle to 0 degree
            double shift_x = ptsx[i] - ref_x;
            double shift_y = ptsy[i] - ref_y;

            ptsx[i] = (shift_x*cos(0-ref_yaw) - shift_y*sin(0-ref_yaw));
            ptsy[i] = (shift_x*sin(0-ref_yaw) + shift_y*cos(0-ref_yaw));
          }


          tk::spline s_xy;
        //   for(int i=0; i<ptsx.size();i++) {
        //     cout<<"ptsx " <<ptsx[i]<<endl;
        //   }
          s_xy.set_points(ptsx, ptsy);

          for (int i=0; i<prev_size; i++) {

            next_x_vals.push_back(previous_path_x[i]);
            next_y_vals.push_back(previous_path_y[i]);

          }

          // linearized target distance for points generation on the spline
          double target_x = 30.0;
          double target_y = s_xy(target_x);
          double target_dist = sqrt(target_x*target_x+target_y*target_y);

          double x_add_on = 0;

          
          // generate new points on the spline trajectory and transform back to the global map system
          for (int i=1; i<=50-prev_size;i++) {

            double N=(target_dist/(TIME_STEP*mycar.ref_vel/2.24));
            double x_point = x_add_on + (target_x)/N;
            double y_point = s_xy(x_point);

            x_add_on = x_point;

            double x_ref = x_point;
            double y_ref = y_point;
        
            //rotate back to normal 
            x_point = x_ref*cos(ref_yaw) - y_ref*sin(ref_yaw);
            y_point = x_ref*sin(ref_yaw) + y_ref*cos(ref_yaw);

            x_point += ref_x;
            y_point += ref_y;

            next_x_vals.push_back(x_point);
            next_y_vals.push_back(y_point);

          }



          // driving with constant velocity and stays in the middle of the lane
          // for (int i=0; i<50-path_size; i++) {

          //   vector<double> frenet_sd = getFrenet(pos_x, pos_y, angle, map_waypoints_x, map_waypoints_y);
          //   double next_s_val = frenet_sd[0] + dist_inc*i;
          //   // double next_s_val = frenet_sd[0] + car_speed * time_step * i;
          //   // double next_d_val = frenet_sd[1];
          //   double next_d_val = 2.0;
          //   vector<double> vec_xy = getXY(next_s_val, next_d_val, map_waypoints_s, map_waypoints_x, map_waypoints_y);

          //   next_x_vals.push_back(vec_xy[0]);
          //   next_y_vals.push_back(vec_xy[1]);

          //   // next_x_vals.push_back(car_x+(dist_inc*i)*cos(deg2rad(car_yaw)));
          //   // next_y_vals.push_back(car_y+(dist_inc*i)*sin(deg2rad(car_yaw)));
          // }


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