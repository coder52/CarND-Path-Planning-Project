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

// globals
float ego_time = 0.0;
float speed = 0.0;
int lane = 1;

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
           //
           // count time
           ego_time += 0.02;
           std::cout<<"time: "<<ego_time<<" / s :"<<car_s<<" / speed :"<<car_speed<<"\n";
           // 1st PART
           // flags for nearby cars
           bool car_ahead = false;
           bool car_right = false;
           bool car_left = false;
           bool slow_down = false;
           // for all other cars on the road
           for(int i=0;i<sensor_fusion.size();i++){
             double id = sensor_fusion[i][0];
             double x = sensor_fusion[i][1];
             double y = sensor_fusion[i][2];
             double vx = sensor_fusion[i][3];
             double vy = sensor_fusion[i][4];
             double s = sensor_fusion[i][5];
             double ln = sensor_fusion[i][6];
             //calculate speed of car (speed will little different after one loop)
             double v = sqrt(vx*vx + vy*vy)-.3;
             if(v<=0){v=0.01;}
             // use cars position after one loop
             double cr_s = car_s+2.5;
             // give number each lane by lane range
             int lane_num;
             if(ln<=4 && ln>=0 ) {lane_num=0;}
             if(ln>=4 && ln<=8) {lane_num=1;}
             if(ln>=8 && ln<=12) {lane_num=2;}
             // find lane of the car
             int lane_check =  lane_num-lane;
             // Check ego's lane
             if(lane_check==0){
               double a=s-cr_s;
               if(a>0 && a<speed*2){//Front
                 car_ahead=true;
               }
             }
             // Check right lane
             if(lane_check==1){
               double a=s-cr_s;
               if(a>0 && a<50){ car_right=true; } // Front
               if(a<0 && a>-20){ car_right=true; } // Back
             }
             // Check left lane
             if(lane_check==-1){
               double a=s-cr_s;
               if(a>0 && a<50){ car_left=true; } // Front
               if(a<0 && a>-20){ car_left=true; } //Back
             }
             // 2nd PART
             // Speed control
             if(car_ahead==1 && lane_check==0 && speed>v){
               slow_down = true;
             }
           }
           // tune the speed of the ego
           if(slow_down){
             speed-=0.5;
           } else if(speed > 21){
             speed=22.0;
           } else {
             speed+=1.0;
           }
           // 3rd PART
           // preferred lane is lane_1 (middle)
           if(car_left==0 && car_ahead==0 && car_right==0){lane=1;}
           // no way to out of lane
           if(lane==0){car_left=true;}
           if(lane==2){car_right=true;}
           // all other conditions
           if(car_ahead==0 && car_left==0 && car_right==0){lane=1;}
           if(car_ahead==0 && car_left==1 && car_right==0){lane=lane;}
           if(car_ahead==0 && car_left==0 && car_right==1){lane=lane;}
           if(car_ahead==0 && car_left==1 && car_right==1){lane=lane;}
           if(car_ahead==1 && car_left==0 && car_right==0){lane -= 1; car_right = false;}
           if(car_ahead==1 && car_left==0 && car_right==1){lane -= 1; car_right = false;}
           if(car_ahead==1 && car_left==1 && car_right==0){lane += 1; car_left = false;}
           if(car_ahead==1 && car_left==1 && car_right==1){lane = lane;}
           // show that: Is there a car nearby
           //std::cout<<car_left<<car_ahead<<car_right<<"\n";
           // 4th PART
           // make a vectors for spline
           vector<double> sp_x;
           vector<double> sp_y;

           double pos_x;
           double pos_y;
           double angle;
           int path_size = previous_path_x.size();
           // set position and angle of the ego at start
           if (path_size == 0) {
             pos_x = car_x;
             pos_y = car_y;
             angle = deg2rad(car_yaw);
             // add position in spline koordinates
             sp_x.push_back(pos_x);
             sp_y.push_back(pos_y);
           } else {
             // add unused last 30 coordinates to spline after motion
             for(int i=30;i>0;i--){
               pos_x = previous_path_x[path_size-(1+i)];
               pos_y = previous_path_y[path_size-(1+i)];

               sp_x.push_back(pos_x);
               sp_y.push_back(pos_y);
             }
             // set position and angle of the ego when it moving
             pos_x = previous_path_x[path_size-1];
             pos_y = previous_path_y[path_size-1];

             double pos_x2 = previous_path_x[path_size-2];
             double pos_y2 = previous_path_y[path_size-2];
             angle = atan2(pos_y - pos_y2, pos_x - pos_x2);
           }
           // set next coordinates at far for making soft shift between lanes.
           vector<double> horizons = {50,80,120};
           for(int i=0;i<horizons.size();i++){
             int h = horizons[i];
             // lane change takes place here
             vector<double> xy = getXY(car_s+h,(2+4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);

             sp_x.push_back(xy[0]);
             sp_y.push_back(xy[1]);
           }
           // 5th PART
           // Shift the coordinates we have created for
           //     the spline to the cartesian coordinates
           for (int i=0;i<sp_x.size(); i++)
           {
             double shift_x = sp_x[i]-pos_x;
             double shift_y = sp_y[i]-pos_y;

             sp_x[i] = (shift_x *cos(0-angle)-shift_y*sin(0-angle));
             sp_y[i] = (shift_x *sin(0-angle)+shift_y*cos(0-angle));
           }
           // 6th PART
           // make a spline
           tk::spline sp;
           sp.set_points(sp_x, sp_y);
           // Add coordinates not used by ego
           for(int i=0;i<previous_path_x.size(); i++)
           {
             next_x_vals.push_back(previous_path_x[i]);
             next_y_vals.push_back(previous_path_y[i]);
           }
           // set x for drawing a line
           // the spline function will return the y coordinate
           double target_x = 40.0;
           double target_y = sp(target_x);
           // find the hypotenuse
           double target_dist = sqrt((target_x)*(target_x)+(target_y)*(target_y));
           // The hypotenuse can be divided into several
           //    equal parts depending on the speed of the ego.
           double N = (target_dist/(.02*speed));
           double anchor = 0;
           for(int i=1; i<=50-previous_path_x.size();i++)
           {
             // set coordinates for spline
             double track_x = anchor+target_x/N;
             double track_y = sp(track_x);
             // move to the last position
             anchor = track_x;
             // 7th PART
             // move the coordinates back to the ego's coordinate system
             double x = track_x;
             double y = track_y;
             track_x = pos_x + (x * cos(angle) - y * sin(angle));
             track_y = pos_y + (x * sin(angle) + y * cos(angle));
             // 8th PART
             // forward the coordinates of the new line created to the simulator
             next_x_vals.push_back(track_x);
             next_y_vals.push_back(track_y);
           }
           // END

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
