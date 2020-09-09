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

int curr_lane=1;
double target_speed=49.0;
double curr_speed=0;

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
		   
		    // Defining variables pos_x, pos_y which stores current coordinates of vehicle
			// and pos_x2, pos_y2 which previous coordinates of the vehicle
			double pos_x, pos_x2;
			double pos_y, pos_y2;
			double angle;           // angle stores yaw angle of the car
			
			// ptsx , ptsy stores x and y coordinates for the anchor points which will be used in trajectory generation.
			vector<double> ptsx;	
			vector<double> ptsy;
			int path_size = previous_path_x.size();  
					
			if (path_size < 2) {
			  /* this part extrapolates trajectory backward to get
			  a point tangent to current yaw angle of the car for trajectory generation*/
			  pos_x = car_x;
			  pos_y = car_y;
			  angle = deg2rad(car_yaw);
			  
			  pos_x2 = pos_x-cos(angle);
			  pos_y2 = pos_y-sin(angle);			  
			  
			} else {
			  /*this part stores previous as well current car co-ordinates
			   for trajectory generation*/
			  pos_x = previous_path_x[path_size-1];
			  pos_y = previous_path_y[path_size-1];

			  pos_x2 = previous_path_x[path_size-2];
			  pos_y2 = previous_path_y[path_size-2];
			  angle = atan2(pos_y-pos_y2,pos_x-pos_x2);
			}

			ptsx.push_back(pos_x2);
			ptsy.push_back(pos_y2);
			
			ptsx.push_back(pos_x);
			ptsy.push_back(pos_y);
			
			// Assigning lane number (to 'curr_lane') based on current car d-coordinate
            if ((car_d>0) && (car_d<4))
              curr_lane=0;
            else if ((car_d>4) && (car_d<8))
              curr_lane=1;
            else
              curr_lane=2;
          
		    // Initializing d coordinate for future anchor points
            vector<double> next_d(3,2+4*curr_lane);
			
			if (path_size>0)
			{
				car_s=end_path_s;
			}
			
			
			double dist_car=30.0;     // s distance from car ahead which is considered too close
			vector<double> min_dist_lane(3,99999.0);  // vector to store minimum distance s from cars in all lanes 
			
			/* vector stores closest cars ids which are both in front and behind the car
			   The 1st three element store cars which are closest in each of the lanes and
			   the next three elements store cars ids which are behind the car*/
            vector<int> lane_closest_cars(6,-1);
			
			for (int i=0;i<sensor_fusion.size();i++)
			{	 
                double other_car_s=sensor_fusion[i][5];
                double other_car_d=sensor_fusion[i][6];
				double vx=sensor_fusion[i][3];
				double vy=sensor_fusion[i][4];
				double speed =sqrt(vx*vx+vy*vy);
				
				other_car_s+=((double)path_size*0.02*speed);
				if (other_car_s-car_s>0)
				{ 
			        // Checks which cars id are in front and stores their minimum their relative s distances from the car
					if ((other_car_s-car_s<min_dist_lane[0]) && (other_car_d>0) && (other_car_d<4))
					{
						// for lane 0 -- leftmost lane
						min_dist_lane[0]=other_car_s-car_s;
						lane_closest_cars[0]=i;
						
					} else if ((other_car_s-car_s<min_dist_lane[1]) && (other_car_d>4) && (other_car_d<8))
					{
						// for lane 1 -- middle lane
						min_dist_lane[1]=other_car_s-car_s;
						lane_closest_cars[1]=i;

					} else if ((other_car_s-car_s<min_dist_lane[2]) && (other_car_d>8) && (other_car_d<12))
					{
						// for lane 2 -- rightmost lane
						min_dist_lane[2]=other_car_s-car_s;
						lane_closest_cars[2]=i;

					}
				} else if (other_car_s-car_s>-10){
					// Storing Cars ids upto 10 units distance which are behind the car in all lanes.
					int lane=other_car_d/4;
					lane_closest_cars[3+lane]=i;
				}
			}
			
			
			int future_lane=curr_lane;  // future lane variable is used to assign future lanes numbers for tajectory generation
			if (min_dist_lane[curr_lane]<dist_car)
			{		
				/* reduction in speed of the car if other car in the same lane 
				   in front is driving slower and change lanes if possible*/
				curr_speed-=0.224;  
				if (curr_lane==1)
				{   
					/* if driving in middle lane ---
					   Checks for both left and right lane by checking which lane is empty for a longer 
					   distance and its distance is more than the distance from the car in front in the same lane*/
					if (min_dist_lane[0]<min_dist_lane[2] && min_dist_lane[curr_lane]<min_dist_lane[2] && (lane_closest_cars[3+2]==-1)) { 
					future_lane=curr_lane+1; } // Right turn
					else if (min_dist_lane[0]>min_dist_lane[2] && min_dist_lane[curr_lane]<min_dist_lane[0] && (lane_closest_cars[3+0]==-1)) {
					future_lane=curr_lane-1; } // Left turn
				} else if ((min_dist_lane[curr_lane]<min_dist_lane[1]) && (lane_closest_cars[3+1]==-1)) {
					future_lane=1; //Come bact to the middle lane
				} 

			} else if (curr_speed<target_speed) {
			  /* else if distance is not less than dist_car(30 units) 
			     increase speed if it is less than 49.5 mph */
              curr_speed+=0.224;
            }
			
			next_d[0]=2+4*future_lane;
			next_d[1]=2+4*future_lane;
			next_d[2]=2+4*future_lane;
			
			// Anchor points to generate a spline for a valid trajectory 
			vector<double> next_xy0 = getXY(car_s + (30), next_d[0], map_waypoints_s, map_waypoints_x,map_waypoints_y);
			ptsx.push_back(next_xy0[0]);
			ptsy.push_back(next_xy0[1]);
			
			vector<double> next_xy1 = getXY(car_s + (60), next_d[1], map_waypoints_s, map_waypoints_x,map_waypoints_y);
			ptsx.push_back(next_xy1[0]);
			ptsy.push_back(next_xy1[1]);
			
			vector<double> next_xy2 = getXY(car_s + (90), next_d[2], map_waypoints_s, map_waypoints_x,map_waypoints_y);
			ptsx.push_back(next_xy2[0]);
			ptsy.push_back(next_xy2[1]);
			
			
			for (int i=0;i<ptsx.size();i++)
			{
				
				ptsx[i]-=pos_x;
				ptsy[i]-=pos_y;
				
				double shift_x = ptsx[i]*cos(angle)+ptsy[i]*sin(angle);
				double shift_y = ptsx[i]*sin(-angle)+ptsy[i]*cos(angle);
				
				ptsx[i]=shift_x;
				ptsy[i]=shift_y;
   
            }
            
			// spline generation
			tk::spline s;
			s.set_points(ptsx,ptsy);
			
			double target_x=30;
			double target_y=s(target_x);
			double dist = sqrt(target_x*target_x+target_y*target_y);
			
			int n = dist/(0.02*curr_speed/2.24);
			
			for (int i = 0; i < path_size; i++) {
			  next_x_vals.push_back(previous_path_x[i]);
			  next_y_vals.push_back(previous_path_y[i]);
			}
			
			double x_add_on = 0;
			for (int i = 1; i <=50-path_size; i++) {
				
				//Interpolation of points within spline for car's movement
				double x_point = x_add_on+target_x/n;
				double y_point = s(x_point);
				
				x_add_on=x_point;
				
				double next_x=x_point*cos(angle)-y_point*sin(angle);
				double next_y=y_point*cos(angle)+x_point*sin(angle);
				
				next_x+=pos_x;
				next_y+=pos_y;
				
				next_x_vals.push_back(next_x);
				next_y_vals.push_back(next_y);
			
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