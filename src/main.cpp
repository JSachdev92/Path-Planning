#include <uWS/uWS.h>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "helpers.h"
#include "json.hpp"
#include <algorithm>
#include "spline.h"


// for convenience
using nlohmann::json;
using std::string;
using std::vector;

double ref_vel = 0.0;
double Max_accel = 5; //set to 5 m/s
double Max_vel = 49/2.237; //50 mph to m/s conversion for max_vel
double T_sample = 0.02; // 0.02s iteration time. 
double desired_vel = Max_vel;
int lane = 1; // starting in the middle lane  
double dV_max = T_sample * Max_accel;
double LaneChangeOccured;

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

  h.onMessage([&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,
               &map_waypoints_dx,&map_waypoints_dy]
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

          vector<double> next_x_vals;
          vector<double> next_y_vals;

          /**
           * TODO: define a path made up of (x,y) points that the car will visit
           *   sequentially every .02 seconds
           */
      
          
          int path_size = previous_path_x.size();
           /**
           * Behavioural Planning
           */
          

          bool obstacle = false;
          bool lc_la = true; //lane change towards left available
          bool lc_ra = true; //lane change towards right available
          double warning_time = 2.5; //safety margin to account for amount of space between cars as a time to collision value (adjusts for velocity) for approaching vehicles
          double distance_danger_time = 2; //amount of space between cars as a time to collision value 
          double lc_warning_time = 1.1; // amount of space needed for lane change in terms of time
          double lc_rel_vel_warning_time = 7; // amount of space needed for lane change in terms of time
          car_speed /= 2.237;//convert to m/s
          LaneChangeOccured -= 1.0;
          LaneChangeOccured = std::max(LaneChangeOccured, 0.0);
          double left_lane_vel = Max_vel;
          double right_lane_vel = Max_vel;

          
          // Go through other visible cars tracked by sensor fusion
          for(int i=0; i<sensor_fusion.size(); i++){
            
            double vx = sensor_fusion[i][3];
            double vy = sensor_fusion[i][4];
            double check_car_speed = sqrt(vx*vx + vy*vy); // calculate the magnitude velocity
            double check_car_s = sensor_fusion[i][5];
            float check_car_d = sensor_fusion[i][6]; //

            
            //check if there is a slow car infront of me
            if(check_car_d < (2+4*lane+2) && check_car_d > (2+4*lane-2)){            
              //if vehicle is in the same lane as me, ahead of me and the relative distance is less than a threshold calculated by relative velocity
              //OR if vehicle is ahead of me by a danger threshold of relative distance then vehicle is an obstacle
              if(check_car_s > car_s && ((((check_car_s - car_s)< (warning_time*(abs(check_car_speed - car_speed))))&&check_car_speed<car_speed)||((check_car_s - car_s)< (distance_danger_time*(abs(car_speed)))))){
                obstacle = true;  
                desired_vel = check_car_speed;    //adjust velocity so we track the vehicle ahead while it is an obsticle             
              }else{
                desired_vel = Max_vel;
              }              
            }
            
            
            //check if left lane change is available
            int left_lane = lane - 1;
            // if left lane valid and the vehicle is in the left lane, but has more than a calibratable time before collision with my vehicle OR if
            // the new lane car has a high relative velocity to our vehicle which would bring it closer to us in a calibratable threshold then left lane change is not available
            if( left_lane >= 0 && (check_car_d < (2+4*left_lane+2) && check_car_d > (2+4*left_lane-2))){   
              if(check_car_s > car_s && ((((check_car_s - car_s)< (2.5*distance_danger_time*(abs(car_speed))))))){
                 double left_lane_vel = check_car_speed;
              }
               if((abs(check_car_s - car_s)<abs(lc_warning_time*((car_speed))))||(abs(check_car_s - car_s)<abs(lc_rel_vel_warning_time*((check_car_speed - car_speed))))){
                lc_la = false;               
              }              
            }
            //check if right lane change is available
            int right_lane = lane + 1;
             // if right lane valid and the vehicle is in the left lane, but has more than a calibratable time before collision with my vehicle OR if
            // the new lane car has a high relative velocity to our vehicle which would bring it closer to us in a calibratable threshold then right lane change is not available
            if( right_lane >= 0 && (check_car_d < (2+4*right_lane+2) && check_car_d > (2+4*right_lane-2))){
              if(check_car_s > car_s && ((((check_car_s - car_s)< (2.5*distance_danger_time*(abs(car_speed))))))){
                double right_lane_vel = check_car_speed;
              }
              if((abs(check_car_s - car_s)<abs(lc_warning_time*((car_speed))))||(abs(check_car_s - car_s)<abs(lc_rel_vel_warning_time*((check_car_speed - car_speed))))){
                lc_ra = false;     
              }
            }         
          }
            
            if(obstacle == true){ //obstacle detected, either move to left lane, right lane, or slow down  
              //first check if you are in middle or right lane and left lane change is availabe, do lane change if right lane change is not available and we did not do a lane change in last 3 seconds
              if(lane >0 && lc_la && (left_lane_vel>right_lane_vel) && (left_lane_vel>desired_vel) && LaneChangeOccured == 0.0){ 
                lane -= 1;
                ref_vel -= dV_max;
                LaneChangeOccured = 3.0/T_sample;
              }
              //then check if you are in left or middle lane and right lane change is availabe, do lane change if left lane change is not available and we did not do a lane change in last 3 seconds   
              else if (lane <2 && lc_ra  && (right_lane_vel>left_lane_vel) && (right_lane_vel>desired_vel) && LaneChangeOccured == 0.0){
                lane += 1;
                ref_vel -= dV_max; 
                LaneChangeOccured = 3.0/T_sample;                
              }
              // if both left and right lane change is available, first change to left, then change to right but only if no lange change has occured in last 3 seconds.
              else if (lane >0 && lc_la && LaneChangeOccured == 0.0){
                lane -= 1;
                ref_vel -= dV_max; 
                LaneChangeOccured = 3.0/T_sample;
              }else if (lane <2 && lc_ra  && LaneChangeOccured == 0.0){
                lane += 1;
                ref_vel -= dV_max; 
                LaneChangeOccured = 3.0/T_sample;
              }
              //otherwise you cannot change lanes so slow down at the max rate
              else{
                ref_vel -= dV_max; 
              }

            }else if (ref_vel > desired_vel){           
              ref_vel -= dV_max;            
            } else if (ref_vel < desired_vel){           
              ref_vel += dV_max;            
            } 
          
		  //generate a vector of x and y points for spline generation
          vector<double> xpts;
          vector<double> ypts;
          double pos_x, pos_y, car_angle, prev_pos_x, prev_pos_y;
          
          //start by using previous points as start point of spline
          pos_x = car_x;
          pos_y = car_y;
          car_angle = deg2rad(car_yaw);


          if (path_size <= 2) {
 
            prev_pos_x = car_x - cos(car_angle);
            prev_pos_y = car_y - sin(car_angle);
            
            xpts.push_back(prev_pos_x);
            xpts.push_back(car_x);
            
            ypts.push_back(prev_pos_y);
            ypts.push_back(car_y);
            
          } else {
            pos_x = previous_path_x[path_size-1];
            pos_y = previous_path_y[path_size-1];

            prev_pos_x = previous_path_x[path_size-2];
            prev_pos_y = previous_path_y[path_size-2];
            
            car_angle = atan2(pos_y-prev_pos_y,pos_x-prev_pos_x);
            
            xpts.push_back(prev_pos_x);
            xpts.push_back(pos_x);
            
            ypts.push_back(prev_pos_y);
            ypts.push_back(pos_y);            
		  }
          // Setting up target points in the future as a function of LA time

          double T_LA_1 = 2.2; // 1s Look ahead point for target point 1.
          double T_LA_2 = 4; // 2.5s Look ahead point for target point 2.
          double T_LA_3 = 6; // 5 s Look ahead point for target point 3.

          
          
          vector<double> wp0 = getXY(car_s + Max_vel*T_LA_1, (2 + 4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);//desired_vel*T_LA_1
          vector<double> wp1 = getXY(car_s + Max_vel*T_LA_2, (2 + 4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);//desired_vel*T_LA_2
          vector<double> wp2 = getXY(car_s + Max_vel*T_LA_3, (2 + 4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);//desired_vel*T_LA_3

          xpts.push_back(wp0[0]);
          xpts.push_back(wp1[0]);
          xpts.push_back(wp2[0]);

          ypts.push_back(wp0[1]);
          ypts.push_back(wp1[1]);
          ypts.push_back(wp2[1]);
           
          // transform coordinates from global frame to local car frame
          for ( int i = 0; i < xpts.size(); i++ ) {
            double shift_x = xpts[i] - pos_x;
            double shift_y = ypts[i] - pos_y;

            xpts[i] = (shift_x * cos(0 - car_angle)) - (shift_y * sin(0 - car_angle));
            ypts[i] = (shift_x * sin(0 - car_angle)) + (shift_y * cos(0 - car_angle));
          }

            // Create the spline.
          tk::spline s;
          s.set_points(xpts, ypts);
          
          //generate next points for path based on previous points and the spline
          int use_path_size = std::min(path_size, 50);
          
          
          for (int i = 0; i < use_path_size; ++i) {
            next_x_vals.push_back(previous_path_x[i]);
            next_y_vals.push_back(previous_path_y[i]);
          }
          
          
          double T_LA = 1.5; //look ahead time for new points 
          double X_tgt = Max_vel*T_LA;
          double Y_tgt = s(X_tgt);
          double target_dist = sqrt( X_tgt * X_tgt + Y_tgt * Y_tgt);

          double dist_step = target_dist/(std::max(T_sample*ref_vel, 0.0001)); //how many steps needed to maintain velocity based on the reference velocity
          
          double x_step = X_tgt/dist_step;
          double y_step =  Y_tgt/dist_step;
          
          double step = x_step;
          double x_cur = 0;
          double x_point, y_point, x_car, y_car;
          
          for (int i = 0; i < (50 - use_path_size); i++){ // 50: defined total number of points

            x_point = x_cur + step;
		    y_point = s(x_point);

		    x_cur = x_point;

		    x_car = x_point;
		    y_car = y_point;

		    // Transform back to world coordinate after rotating it earlier
		     x_point = x_car * cos(car_angle) - y_car * sin(car_angle);
		     y_point = x_car * sin(car_angle) + y_car * cos(car_angle);

		     x_point += pos_x;
		     y_point += pos_y;

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