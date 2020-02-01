#include <uWS/uWS.h>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
//#include "Eigen-3.3/Eigen/Core"
//#include "Eigen-3.3/Eigen/QR"
#include "helpers.h"
#include "json.hpp"
#include "spline.h"

#include "vehicle.cpp"
#include <map>
//#include "Eigen"


// for convenience
using nlohmann::json;
using std::string;
using std::vector;

using std::map;

double speed = 0; // m/s
double lane = 1;

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

           for(int i=1;i<previous_path_x.size(); i++)
           {
             next_x_vals.push_back(previous_path_x[i]);
             next_y_vals.push_back(previous_path_y[i]);
           }



           vector<double> start_s;
           vector<double> end_s;
           vector<double> start_d;
           vector<double> end_d;
           double T;
           double pos_s;
           double pos_d;
           if(previous_path_x.size()==0){
             start_s = {0,0,0};
             end_s = {24.2,22,10};

             start_d = {0,0,0};
             end_d = {50,22,0};

             T = 2.2;
             pos_s = car_s;
             pos_d = car_d;
           } else {
             start_s = {0,20,0};
             end_s = {50,20,0};

             start_d = {0,22,0};
             end_d = {50,22,0};

             T = 2.2;

             pos_s = end_path_s;
             pos_d = end_path_d;
           }

           vector<double> jrk_s = JMT(start_s, end_s, T);
           vector<double> jrk_d = JMT(start_d, end_d, T);

           double dist_s;
           double dist_d;
           double new_d = 6;

           // Test the lane change
           if(car_s>200){new_d = 2;}
           if(car_s>300){new_d = 6;}
           if(car_s>400){new_d = 10;}
           if(car_s>500){new_d = 6;}

           double next_d;

           vector<double> track_x;
           vector<double> track_y;

           for (int i = 0; i < 50; ++i) {
             double t = 0.02*(i+1);
             dist_s = jrk_s[0] + jrk_s[1]*t + jrk_s[2]*t*t +
                        jrk_s[3]*t*t*t + jrk_s[4]*t*t*t*t + jrk_s[5]*t*t*t*t*t;

             double next_s = pos_s+dist_s;


             t = 0.02;
             dist_d = jrk_d[0] + jrk_d[1]*t + jrk_d[2]*t*t +
                        jrk_d[3]*t*t*t + jrk_d[4]*t*t*t*t + jrk_d[5]*t*t*t*t*t;


             if(pos_d<new_d){
               next_d = pos_d+dist_d;
               if(next_d>new_d){
                 next_d = new_d;
               }
             }else if(pos_d>new_d){
               next_d = pos_d-dist_d;
               if(next_d<new_d){
                 next_d = new_d;
               }
             }else{
               next_d = pos_d;
             }

             vector<double> xy = getXY(next_s, next_d, map_waypoints_s,
                                        map_waypoints_x, map_waypoints_y);

             track_x.push_back(xy[0]);
             track_y.push_back(xy[1]);
           }

           ////////////////////////////////////////////////////
           ///////////DEBUG/////////////////////

           std::cout<<"track_x.size()"<<track_x.size()<<"\n";
           std::cout<<"next_d"<<next_d<<"\n";
           std::cout<<"next_d"<<pos_d<<"\n";
           //std::cout<<"end_path_s"<<end_path_s<<"\n";
           //std::cout<<"first_new"<<track_s[0]<<"\n";



           for(int i=0;i<50-previous_path_x.size();i++){
             next_x_vals.push_back(track_x[i]);
             next_y_vals.push_back(track_y[i]);
           }











           /*
           ///////////////////////////////////////////////////////////////////
           //////////////////////////////////////////////////////////////////
           std::cout<<"#####################################"<<"\n";
           std::cout<<"#####################################"<<"\n";
           std::cout<<"car_x:"<<car_x<<"\n";
           std::cout<<"car_y:"<<car_y<<"\n";
           std::cout<<"car_s:"<<car_s<<"\n";
           std::cout<<"car_d:"<<car_d<<"\n";
           std::cout<<"car_yaw:"<<car_yaw<<"\n";
           std::cout<<"car_speed:"<<car_speed<<"\n";
           std::cout<<"car_x:"<<car_x<<"\n";
           std::cout<<"#####################################"<<"\n";
           std::cout<<"previous_path_x[0]:"<<previous_path_x[0]<<"\n";
           std::cout<<"previous_path_y[0]:"<<previous_path_y[0]<<"\n";
           std::cout<<"previous_path_x[-1]:"<<previous_path_x[previous_path_x.size()-1]<<"\n";
           std::cout<<"previous_path_y[-1]:"<<previous_path_y[previous_path_y.size()-1]<<"\n";
           std::cout<<"#####################################"<<"\n";
           std::cout<<"end_path_s:"<<end_path_s<<"\n";
           std::cout<<"end_path_d:"<<end_path_d<<"\n";
           std::cout<<"#####################################"<<"\n";
           std::cout<<"next_x_vals:"<<next_x_vals[0]<<"\n";
           std::cout<<"next_y_vals:"<<next_y_vals[0]<<"\n";
           std::cout<<"next_x_vals:"<<next_x_vals[next_x_vals.size()-1]<<"\n";
           std::cout<<"next_y_vals:"<<next_x_vals[next_y_vals.size()-1]<<"\n";
           std::cout<<"#####################################"<<"\n";
           std::cout<<"next_x_vals.size():"<<next_x_vals.size()<<"\n";
           std::cout<<"previous_path_x.size():"<<previous_path_x.size()<<"\n";
           std::cout<<"sensor_fusion.size():"<<sensor_fusion.size()<<"\n";
           */








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
