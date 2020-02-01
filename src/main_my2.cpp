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



           if(speed<22){ speed += 1;}


           if(car_s>200){ lane=0;}
           if(car_s>250){ lane=1;}
           if(car_s>300){ lane=2;}
           if(car_s>350){ lane=1;}
           if(car_s>400){ lane=0;}
           if(car_s>450){ lane=1;}
           if(car_s>500){ lane=2;}
           if(car_s>550){ lane=1;}
           if(car_s>600){ lane=0;}
           if(car_s>650){ lane=1;}
           if(car_s>700){ lane=2;}
           if(car_s>750){ lane=1;}
           if(car_s>800){ lane=0;}
           if(car_s>850){ lane=1;}
           if(car_s>900){ lane=2;}
           if(car_s>950){ lane=1;}
           if(car_s>1000){ lane=0;}
           if(car_s>1050){ lane=1;}
           if(car_s>1100){ lane=2;}
           if(car_s>1150){ lane=1;}
           if(car_s>1200){ lane=0;}
           if(car_s>1250){ lane=1;}
           if(car_s>1300){ lane=2;}
           if(car_s>1350){ lane=1;}







           vector<double> sp_x;
           vector<double> sp_y;

           int prev_size = previous_path_x.size();

           double pos_x;
           double pos_y;
           double angle;
           int path_size = previous_path_x.size();

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

           sp_x.push_back(pos_x);
           sp_y.push_back(pos_y);

           vector<double> horizons = {30,60,90};

           for(int i=0;i<horizons.size();i++){
             int h = horizons[i];
             vector<double> xy = getXY(car_s+h,(2+4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);

             sp_x.push_back(xy[0]);
             sp_y.push_back(xy[1]);
           }

           for (int i=0;i<sp_x.size(); i++)
           {
             double shift_x = sp_x[i]-pos_x;
             double shift_y = sp_y[i]-pos_y;

             sp_x[i] = (shift_x *cos(0-angle)-shift_y*sin(0-angle));
             sp_y[i] = (shift_x *sin(0-angle)+shift_y*cos(0-angle));
           }

           // make a spline
           tk::spline sp;
           sp.set_points(sp_x, sp_y);


           for(int i=0;i<previous_path_x.size(); i++)
           {
             next_x_vals.push_back(previous_path_x[i]);
             next_y_vals.push_back(previous_path_y[i]);
           }


           double target_x = 30.0;
           double target_y = sp(target_x);
           double target_dist = sqrt((target_x)*(target_x)+(target_y)*(target_y));
           double N = (target_dist/(.02*speed));

           double anchor = 0;
           for(int i=1; i<=50-previous_path_x.size();i++)
           {

             double track_x = anchor+(target_x)/N;
             double track_y = sp(track_x);

             anchor = track_x;

             double x = track_x;
             double y = track_y;

             track_x = pos_x + (x * cos(angle) - y * sin(angle));
             track_y = pos_y + (x * sin(angle) + y * cos(angle));

             next_x_vals.push_back(track_x);
             next_y_vals.push_back(track_y);
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
