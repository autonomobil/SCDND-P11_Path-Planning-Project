#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "json.hpp"
#include <chrono>
#include <fstream>
#include <iostream>
#include <math.h>
#include <thread>
#include <uWS/uWS.h>
#include <vector>

#include "spline.h"
#include "utilities.h"

using namespace std;

// for convenience
using json = nlohmann::json;

// constant time step in seconds
const double dt = 0.02;

int main()
{
    uWS::Hub h;

    // start lane number
    int lane = 1;
    // The max s value before wrapping around the track back to 0
    double MAX_S = 6945.554;
    // max speed
    double MAX_VEL = mph_to_mps(49.85); // mph --> m/s
    // max speed
    double MAX_ACC = 9; // m/s²
    // last  speed
    double ref_vel = 0; // m/s

    // Load up map values for waypoint's x,y,s and d normalized normal vectors
    vector<double> map_waypoints_x;
    vector<double> map_waypoints_y;
    vector<double> map_waypoints_s;
    vector<double> map_waypoints_dx;
    vector<double> map_waypoints_dy;
    // vector<double> last_s_points;
    // vector<double> last_d_points;
    // Waypoint map to read from
    string map_file_ = "../data/highway_map.csv";

    ifstream in_map_(map_file_.c_str(), ifstream::in);

    string line;
    while (getline(in_map_, line)) {
        istringstream iss(line);
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

    h.onMessage([&ref_vel, &MAX_ACC, &MAX_VEL, &lane, &map_waypoints_x, &map_waypoints_y, &map_waypoints_s, &map_waypoints_dx, &map_waypoints_dy](uWS::WebSocket<uWS::SERVER> ws, char* data, size_t length,
                    uWS::OpCode opCode) {
        // "42" at the start of the message means there's a websocket message event.
        // The 4 signifies a websocket message
        // The 2 signifies a websocket event
        //auto sdata = string(data).substr(0, length);
        //cout << sdata << endl;
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

                    // Sensor Fusion Data, a list of all other cars on the same side of the road.
                    auto sensor_fusion = j[1]["sensor_fusion"];

                    json msgJson;

                    size_t prev_size = previous_path_x.size();
                    int use_no_old_points = min(5, (int)prev_size);

                    //////////////////// PREDICTION ////////////////////
                    bool car_ahead = false;
                    bool car_left = false;
                    bool car_right = false;
                    double delta_l = 100000000000000;
                    double delta_r = 100000000000000;

                    bool danger = false;
                    double delta_s = 10000000000000;
                    double ahead_speed;

                    for (int i = 0; i < sensor_fusion.size(); i++) {
                        double check_car_d = sensor_fusion[i][6];
                        double check_car_s = sensor_fusion[i][5];

                        if (abs(check_car_d - car_d) < 2.5 && check_car_s - car_s < 7 && check_car_s - car_s > 0) {
                            danger = true;
                            cout << "DANGER!" << endl;
                            break;
                        }

                        int check_car_lane = -1;
                        // check lane of car
                        if (check_car_d > 0 && check_car_d < 4) {
                            check_car_lane = 0;
                        } else if (check_car_d > 4 && check_car_d < 8) {
                            check_car_lane = 1;
                        } else if (check_car_d > 8 && check_car_d < 12) {
                            check_car_lane = 2;
                        }
                        if (check_car_lane < 0) {
                            continue;
                        }
                        // calculate check car speed
                        double vx = sensor_fusion[i][3];
                        double vy = sensor_fusion[i][4];
                        double check_speed = sqrt(vx * vx + vy * vy);

                        // predict check car s position 50*0.02=1 second in the future after executing previous trajectory
                        double predict_check_car_s = check_car_s + 50.0 * dt * check_speed;
                        double predict_car_s = car_s + 50.0 * dt * ref_vel;

                        if (check_car_lane == lane) {
                            // car same lane infront
                            // cout << "predict_check_car_s: " << predict_check_car_s << "   predict_car_s:" << predict_car_s << endl;
                            // cout << "check_car_s: " << check_car_s << "   car_s:" << car_s << endl;

                            car_ahead |= predict_check_car_s > predict_car_s && predict_check_car_s - predict_car_s < 40;
                            // get id of currently clostest car in same lane
                            if (abs(check_car_s - car_s) < delta_s) {
                                ahead_speed = check_speed; // speed of car in front
                                delta_s = abs(check_car_s - car_s); // use this delta_s instead
                            }

                        } else if (check_car_lane - lane == -1) {
                            // car left
                            car_left |= predict_car_s - 12 < predict_check_car_s && predict_car_s + 40 > predict_check_car_s;
                            if (check_car_s - car_s < delta_l && (check_car_s - car_s) > 0) {
                                delta_l = check_car_s - car_s;
                            }
                        } else if (check_car_lane - lane == 1) {
                            // car right
                            car_right |= predict_car_s - 12 < predict_check_car_s && predict_car_s + 40 > predict_check_car_s;
                            if (check_car_s - car_s < delta_r && (check_car_s - car_s) > 0) {
                                delta_r = check_car_s - car_s;
                            }
                        }
                    }

                    //////////////////// BEHAVIOUR PLANNING ////////////////////
                    double speed_diff = 0;
                    // bool follow_car_ahead = false;
                    bool lane_change_left = true; // check if left lane is best option

                    if (delta_r > delta_l) {
                        lane_change_left = false;
                    }

                    if (danger) {
                        speed_diff -= MAX_ACC * dt;
                    } else if (car_ahead) {
                        // double check_car_s = sensor_fusion[id_car_ahead][5];
                        if (!car_left && lane > 0 && lane_change_left) {
                            // if there is no dangerous car left and there is a left lane
                            lane--; // Change lane left.
                            cout << "Lane change left! delta_l: " << delta_l << "   delta_r:" << delta_r << endl;
                        } else if (!car_right && lane != 2) {
                            // if there is no dangerous car right and there is a right lane
                            lane++; // Change lane right.
                            cout << "Lane change right! delta_l: " << delta_l << "   delta_r:" << delta_r << endl;
                        } else {
                            // slow down
                            cout << "car ahead! delta_s: " << delta_s << endl;
                            if (delta_s < 15 && car_speed > ahead_speed * 0.95) {
                                speed_diff -= MAX_ACC * dt * min(1.0, pow(15, 2) / pow((delta_s), 2));
                                use_no_old_points = 3;
                                // follow_car_ahead = true;
                            }
                        }

                    } else {
                        if (lane != 1) { // go back to center lane if possible, because there are more options
                            if ((lane == 0 && !car_right) || (lane == 2 && !car_left)) {
                                lane = 1;
                                cout << "Take me back to the center " << endl;
                            }
                        }
                        if (ref_vel < MAX_VEL) {
                            speed_diff += MAX_ACC * dt * 0.85;
                        }
                    }
                    ref_vel += speed_diff;

                    if (ref_vel > MAX_VEL) {
                        ref_vel = MAX_VEL;
                    }
                    //////////////////// TRAJECTORY GENERATION ////////////////////
                    vector<double> next_x_vals;
                    vector<double> next_y_vals;

                    size_t vec_size = 100;
                    // size_t use_no_old_points = prev_size;

                    // take previous unused path points for continuity and smoothness
                    if (danger) {
                        use_no_old_points = 3;
                    }
                    // if(follow_car_ahead){
                    //     use_no_old_points = 5;
                    // }

                    for (int i = 0; i < use_no_old_points; i++) {
                        next_x_vals.push_back(previous_path_x[i]);
                        next_y_vals.push_back(previous_path_y[i]);
                    }

                    // these point vectors will be fed to the spline for smooth trajectory generation
                    vector<double> ptsx;
                    vector<double> ptsy;

                    // current global position
                    double ref_x = car_x;
                    double ref_y = car_y;
                    double ref_yaw = deg2rad(car_yaw);

                    // check if previous path is available
                    if (prev_size < 2) {
                        // calculate last position
                        double last_car_x = car_x - cos(car_yaw);
                        double last_car_y = car_y - sin(car_yaw);

                        // push last position and current position
                        ptsx.push_back(last_car_x);
                        ptsx.push_back(car_x);
                        ptsy.push_back(last_car_y);
                        ptsy.push_back(car_y);

                    } else {
                        // use the last two points from previous path
                        ref_x = previous_path_x[use_no_old_points - 1];
                        ref_y = previous_path_y[use_no_old_points - 1];
                        double last_ref_x = previous_path_x[use_no_old_points - 2];
                        double last_ref_y = previous_path_y[use_no_old_points - 2];

                        // calculate tangent yaw between last two points from previous path
                        ref_yaw = atan2(ref_y - last_ref_y, ref_x - last_ref_x);

                        ptsx.push_back(last_ref_x);
                        ptsx.push_back(ref_x);
                        ptsy.push_back(last_ref_y);
                        ptsy.push_back(ref_y);
                    }

                    // setting up target points
                    int n_wp = 3; // number of target points
                    int s_incr = 35; // distance between target points

                    for (int i = 1; i <= n_wp; i++) {
                        vector<double> next_wp = getXY(car_s + s_incr * i, 2 + 4 * lane, map_waypoints_s, map_waypoints_x, map_waypoints_y);
                        ptsx.push_back(next_wp[0]);
                        ptsy.push_back(next_wp[1]);
                    }

                    // global coordinates to local car coordinates
                    for (int i = 0; i < ptsx.size(); i++) {
                        vector<double> local_pts = global2localCarCoord(ref_x, ref_y, ref_yaw, ptsx[i], ptsy[i]);
                        ptsx[i] = local_pts[0];
                        ptsy[i] = local_pts[1];
                    }

                    // create the spline
                    tk::spline spl;
                    // feed the spline
                    try {
                        spl.set_points(ptsx, ptsy);
                    } catch (...) {
                        ptsx.erase(ptsx.begin() + 1);
                        ptsy.erase(ptsy.begin() + 1);

                        spl.set_points(ptsx, ptsy);
                    }

                    // set target x in local car coordinates, calculate target y target and distance
                    double target_x = 60;
                    double target_y = spl(target_x);
                    double target_dist = distance(0, 0, target_x, target_y);

                    double x_add_on = 0;

                    for (int i = 1; i < vec_size - use_no_old_points; i++) {

                        double N = target_dist / (dt * ref_vel);
                        double x_point = x_add_on + target_x / N;

                        // double x_point = x_add_on + (dt * ref_vel);

                        double y_point = spl(x_point);

                        x_add_on = x_point;

                        vector<double> global_pts = localCar2globalCoord(ref_x, ref_y, ref_yaw, x_point, y_point);

                        next_x_vals.push_back(global_pts[0]);
                        next_y_vals.push_back(global_pts[1]);
                    }

                    ////////////////////////////////////////////////////////////////////
                    msgJson["next_x"] = next_x_vals;
                    msgJson["next_y"] = next_y_vals;

                    auto msg = "42[\"control\"," + msgJson.dump() + "]";

                    //this_thread::sleep_for(chrono::milliseconds(1000));
                    ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
                }
            } else {
                // Manual driving
                std::string msg = "42[\"manual\",{}]";
                ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
            }
        }
    });

    // We don't need this since we're not using HTTP but if it's removed the
    // program
    // doesn't compile :-(
    h.onHttpRequest([](uWS::HttpResponse* res, uWS::HttpRequest req, char* data,
                        size_t, size_t) {
        const std::string s = "<h1>Hello world!</h1>";
        if (req.getUrl().valueLength == 1) {
            res->end(s.data(), s.length());
        } else {
            // i guess this should be done more gracefully?
            res->end(nullptr, 0);
        }
    });

    h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
        std::cout << "Connected!!!" << std::endl;
    });

    h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
                          char* message, size_t length) {
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
