#include <math.h>
#include <uWS/uWS.h>
#include <iostream>
#include <string>
#include "json.hpp"
#include "PID.h"

// for convenience
using nlohmann::json;
using std::string;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_last_of("]");
  if (found_null != string::npos) {
    return "";
  }
  else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 1);
  }
  return "";
}

int main(int argc, char* argv[]) {
  uWS::Hub h;
  
  PID pid;

  bool twiddle = (argc > 4 && std::string("twiddle").compare(argv[4]) == 0);
  // init twiddle params
  std::vector<double> p = {0,0,0}; 
  if (twiddle) p = {std::stod(argv[1]), std::stod(argv[2]), std::stod(argv[3])};
  std::vector<double> dp = {0.1,0.1,0.1};
  std::vector<double> best_p = {p[0],p[1],p[2]};
  int timestep = 0;
  int max_timestep = (argc == 6) ? std::stoi(argv[5]) : 3200;
  double total_cte = 0.0;
  double error = 0.0;
  double best_error = 10000.00;
  double tol = 0.01;
  int p_iter = 0;
  bool first_run = true;
  bool second_run = true;
  bool let_robot_run = true;
  
  if (twiddle == true) pid.Init(p[0],p[1],p[2]);
  else if (argc == 4) { pid.Init(std::stod(argv[1]), std::stod(argv[2]), std::stod(argv[3]));
  else pid.Init(0.577838,1e-05,7.48411);

  h.onMessage([&pid, &p, &dp, &timestep, &max_timestep, &tol, &error, &best_error, &p_iter, &let_robot_run, &total_cte, &first_run, &second_run, &twiddle, &best_p]
  (uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {

    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {
      auto s = hasData(string(data).substr(0, length));

      if (s != "") {
        auto j = json::parse(s);

        string event = j[0].get<string>();

        if (event == "telemetry") {
          // j[1] is the data JSON object
          double cte = std::stod(j[1]["cte"].get<string>());
          double speed = std::stod(j[1]["speed"].get<string>());
          double angle = std::stod(j[1]["steering_angle"].get<string>());
          double steer_value;
          json msgJson;

          pid.UpdateError(cte);
          steer_value = pid.TotalError();
            
          if (twiddle == true){
            if (timestep == 0) pid.Init(p[0],p[1],p[2]); 
            total_cte = total_cte + pow(cte,2);
            
            timestep = timestep+1;
            if (timestep > max_timestep){ 
              error = total_cte/max_timestep;
              if(first_run) {
                p[p_iter] += dp[p_iter];
                let_robot_run = true;
                first_run = false;
                std::cout << "Candidate 1 p[0] p[1] p[2]: " << p[0] << "," << p[1] << "," << p[2] << std::endl;;
                std::cout << "Current dp[0] dp[1] dp[2]: " << dp[0] << "," << dp[1] << "," << dp[2] << "; " << std::endl;;
              } else if(second_run && error < best_error) {
                best_error = error;
                for (int i = 0; i < 3; i++) best_p[i] = p[i];
                dp[p_iter] *= 1.1;
                let_robot_run = false;
                std::cout << "Best p[0] p[1] p[2]: " << best_p[0] << "," << best_p[1] << "," << best_p[2] << "; ";
                std::cout << "Current dp[0] dp[1] dp[2]: " << dp[0] << "," << dp[1] << "," << dp[2] << "; " << std::endl;;
                std::cout << "Best error: " << best_error << std::endl;;
              } else if(second_run) {
                p[p_iter] -= 2 * dp[p_iter];
                let_robot_run = true;
                second_run = false;
                std::cout << "Candidate 2 p[0] p[1] p[2]: " << p[0] << "," << p[1] << "," << p[2] << std::endl;;
                std::cout << "Current dp[0] dp[1] dp[2]: " << dp[0] << "," << dp[1] << "," << dp[2] << "; " << std::endl;;
              }
              else if(error < best_error) { // first and second run finished
                best_error = error;
                for (int i = 0; i < 3; i++) best_p[i] = p[i];
                dp[p_iter] *= 1.1;
                let_robot_run = false;
                std::cout << "Best p[0] p[1] p[2]: " << best_p[0] << "," << best_p[1] << "," << best_p[2] << "; ";
                std::cout << "Current dp[0] dp[1] dp[2]: " << dp[0] << "," << dp[1] << "," << dp[2] << "; " << std::endl;;
                std::cout << "Best error: " << best_error << std::endl;;
              } else {
                p[p_iter] += dp[p_iter];
                dp[p_iter] *= 0.9;
                let_robot_run = false;
              }

              std::cout << "error: " << error << std::endl;;

              if(!let_robot_run) {
                p_iter = (p_iter+1)%3;
                first_run = true;
                second_run = true;
                let_robot_run = true;
              }

              // reset
              total_cte = 0.0;
              timestep = 0;

              if(dp[0]+dp[1]+dp[2] < tol) {
                std::cout << "Best p[0] p[1] p[2]: " << best_p[0] << "," << best_p[1] << "," << best_p[2] << "; ";
                std::cout << "Current dp[0] dp[1] dp[2]: " << dp[0] << "," << dp[1] << "," << dp[2] << "; " << std::endl;;
              } else {
                std::string reset_msg = "42[\"reset\",{}]";
                ws.send(reset_msg.data(), reset_msg.length(), uWS::OpCode::TEXT);
              }
              
            } else {
              msgJson["steering_angle"] = steer_value;
              msgJson["throttle"] = 0.3;
              auto msg = "42[\"steer\"," + msgJson.dump() + "]";
              ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
            }
           
          } else {
            // DEBUG
            // std::cout << "CTE: " << cte << " Steering Value: " << steer_value << " Throttle Value: " << throttle_value << " Count: " << timestep << std::endl;
            msgJson["steering_angle"] = steer_value;
            msgJson["throttle"] = 0.3;
            auto msg = "42[\"steer\"," + msgJson.dump() + "]";
            // std::cout << msg << std::endl;
            ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
          } //twiddle else
        }  // end "telemetry" if
      } else {
        // Manual driving
        string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }  // end websocket message if
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