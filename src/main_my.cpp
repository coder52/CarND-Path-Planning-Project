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

#include "vehicle.cpp"
#include <map>

// for convenience
using nlohmann::json;
using std::string;
using std::vector;

using std::map;


float MAX_VELOCITY = 0.0;

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
           int LANE =1;

           //if(car_s>200) LANE = 0;

           if(MAX_VELOCITY > 48.0){
             MAX_VELOCITY=49.0;
           }else{
             MAX_VELOCITY+=2.0;
           }

           //predictions [ID, x, y, Vx, Vy, s, d]
           int car_ahead=1; //0 if there is a car front of ego
           int turn_left=1; // 0 if can not turn left
           int turn_right=1; // 0 if can not turn right

           for(int i=0;i<sensor_fusion.size();i++){
             double id = sensor_fusion[i][0];
             double x = sensor_fusion[i][1];
             double y = sensor_fusion[i][2];
             double vx = sensor_fusion[i][3];
             double vy = sensor_fusion[i][4];
             double s = sensor_fusion[i][5];
             double d = sensor_fusion[i][6];

             double v = sqrt(vx*vx + vy*vy);
             double dist = distance(x, y, car_x, car_y);
             std::cout<<d<<"\n";

             if(d<12 && dist<15){
               if(s-car_s<7){
                 car_ahead=0;
                 MAX_VELOCITY = v*2.23;
               }
               if(d-car_d<4){turn_right=0;}
               if(car_d-d<4){turn_left=0;}
             }
           }

           if(car_ahead==0){//There is a car front of ego
             if(turn_left==1){//ego can turn right
               LANE -=1;
             } else if(turn_right==1){//ego can turn right
               LANE +=1;
             }
           }





           std::cout<<"#########################"<<"\n";
           //Start:: Define initial coordinates and angle
           double pos_x;
           double pos_y;
           double angle;
           int path_size = previous_path_x.size();
           //std::cout<<"previous_path_x:"<<previous_path_x.size()<<"\n";
           if (path_size == 0) {
             pos_x = car_x;
             pos_y = car_y;
             angle = deg2rad(car_yaw);
           } else {
             pos_x = previous_path_x[path_size-1];
             pos_y = previous_path_y[path_size-1];

             double pos_x2 = previous_path_x[path_size-2];
             double pos_y2 = previous_path_y[path_size-2];
             angle = atan2(pos_y - pos_y2, pos_x - pos_x2);
           }
           //End:: Define initial coordinates and angle
           //std::cout<<"A"<<"\n";
           //Start:: Create an axis towards the horizon
           vector<double> x_sp;
           vector<double> y_sp;

           x_sp.push_back(pos_x);
           y_sp.push_back(pos_y);

           vector<double> horizons = {30,60,90};

           for(int i=0;i<horizons.size();i++){
             int h = horizons[i];
             vector<double> xy = getXY(car_s+h,(2+4*LANE), map_waypoints_s, map_waypoints_x, map_waypoints_y);

             x_sp.push_back(xy[0]);
             y_sp.push_back(xy[1]);
           }
           //std::cout<<"A1"<<"\n";
           for (int i=0;i<x_sp.size(); i++)
           {
             double shift_x = x_sp[i]-pos_x;
             double shift_y = y_sp[i]-pos_y;

             x_sp[i] = (shift_x *cos(0-angle)-shift_y*sin(0-angle));
             y_sp[i] = (shift_x *sin(0-angle)+shift_y*cos(0-angle));
             //std::cout<<"x_sp[i]:"<<x_sp[i]<<"\n";
             //std::cout<<"y_sp[i]:"<<y_sp[i]<<"\n";
           }
           //std::cout<<"x_sp:"<<x_sp.size()<<"\n";
           std::cout<<"angle:"<<angle<<"\n";
           //std::cout<<"A2"<<"\n";
           // make a spline
           tk::spline sp;
           sp.set_points(x_sp, y_sp);
           //End:: Create an axis towards the horizon
           //std::cout<<"A3"<<"\n";
           //Start:: Create an track towards on the road
           vector<double> track_x;
           vector<double> track_y;

           double goal_x = 10.0;
           double goal_y = sp(goal_x);
           double dist = sqrt((goal_x)*(goal_x)+(goal_y)*(goal_y));
           double N = (dist/(.02*MAX_VELOCITY/2.24));
           //std::cout<<"B"<<"\n";
           double anchor = 0;
           for(int i=1; i<=50-path_size;i++)
           {
             double x = anchor+(goal_x)/N;
             double y = sp(x);
             anchor = x;

             x = (x*cos(angle)-y*sin(angle));
             y = (x*sin(angle)+y*cos(angle));

             x += pos_x;
             y += pos_y;

             //std::cout<<"x_point:"<<x<<"\n";
             //std::cout<<"x_point:"<<y<<"\n";

             track_x.push_back(x);
             track_y.push_back(y);
           }
           //std::cout<<"C"<<"\n";
           //End:: Create an track towards on the road

           //Start:: Prepare Path
           for (int i = 1; i <= path_size-1; ++i) {
             next_x_vals.push_back(previous_path_x[i]);
             next_y_vals.push_back(previous_path_y[i]);
           }

           for(int i=0; i<50-path_size-1;i++){
             next_x_vals.push_back(track_x[i]);
             next_y_vals.push_back(track_y[i]);
           }
           //End:: Prepare Path
           //std::cout<<"D"<<"\n";

           std::cout<<"next_x_vals:"<<next_x_vals.size()<<"\n";
           std::cout<<"previous_path_x:"<<previous_path_x.size()<<"\n";
           //std::cout<<"track_x:"<<track_x.size()<<"\n";
           //std::cout<<"x_sp:"<<x_sp.size()<<"\n";







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
