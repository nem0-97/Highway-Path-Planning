#include <fstream>
//#include <math.h>
#include <uWS/uWS.h>
//#include <chrono>
//#include <iostream>
//#include <thread>
//#include <vector>
#include "json.hpp"
#include "spline.h"//json.hpp and spline.h aleady include most the necessary headers

using namespace std;

// for convenience
using json = nlohmann::json;

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
    auto b2 = s.find_first_of("}");
    if (found_null != string::npos) {
        return "";
    } else if (b1 != string::npos && b2 != string::npos) {
        return s.substr(b1, b2 - b1 + 2);
    }
    return "";
}

double distance(double x1, double y1, double x2, double y2)
{
    return sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1));
}
int ClosestWaypoint(double x, double y, const vector<double> &maps_x, const vector<double> &maps_y)
{
    
    double closestLen = 100000; //large number
    int closestWaypoint = 0;
    
    for(int i = 0; i < maps_x.size(); i++)
    {
        double map_x = maps_x[i];
        double map_y = maps_y[i];
        double dist = distance(x,y,map_x,map_y);
        if(dist < closestLen)
        {
            closestLen = dist;
            closestWaypoint = i;
        }
        
    }
    
    return closestWaypoint;
    
}

int NextWaypoint(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y)
{
    
    int closestWaypoint = ClosestWaypoint(x,y,maps_x,maps_y);
    
    double map_x = maps_x[closestWaypoint];
    double map_y = maps_y[closestWaypoint];
    
    double heading = atan2((map_y-y),(map_x-x));
    
    double angle = fabs(theta-heading);
    angle = min(2*pi() - angle, angle);
    
    if(angle > pi()/2)
    {
        closestWaypoint++;
        if (closestWaypoint == maps_x.size())
        {
            closestWaypoint = 0;
        }
    }
    
    return closestWaypoint;
}

// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
vector<double> getFrenet(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y)
{
    int next_wp = NextWaypoint(x,y, theta, maps_x,maps_y);
    
    int prev_wp;
    prev_wp = next_wp-1;
    if(next_wp == 0)
    {
        prev_wp  = maps_x.size()-1;
    }
    
    double n_x = maps_x[next_wp]-maps_x[prev_wp];
    double n_y = maps_y[next_wp]-maps_y[prev_wp];
    double x_x = x - maps_x[prev_wp];
    double x_y = y - maps_y[prev_wp];
    
    // find the projection of x onto n
    double proj_norm = (x_x*n_x+x_y*n_y)/(n_x*n_x+n_y*n_y);
    double proj_x = proj_norm*n_x;
    double proj_y = proj_norm*n_y;
    
    double frenet_d = distance(x_x,x_y,proj_x,proj_y);
    
    //see if d value is positive or negative by comparing it to a center point
    
    double center_x = 1000-maps_x[prev_wp];
    double center_y = 2000-maps_y[prev_wp];
    double centerToPos = distance(center_x,center_y,x_x,x_y);
    double centerToRef = distance(center_x,center_y,proj_x,proj_y);
    
    if(centerToPos <= centerToRef)
    {
        frenet_d *= -1;
    }
    
    // calculate s value
    double frenet_s = 0;
    for(int i = 0; i < prev_wp; i++)
    {
        frenet_s += distance(maps_x[i],maps_y[i],maps_x[i+1],maps_y[i+1]);
    }
    
    frenet_s += distance(0,0,proj_x,proj_y);
    
    return {frenet_s,frenet_d};
    
}

// Transform from Frenet s,d coordinates to Cartesian x,y
vector<double> getXY(double s, double d, const vector<double> &maps_s, const vector<double> &maps_x, const vector<double> &maps_y)
{
    int prev_wp = -1;
    
    while(s > maps_s[prev_wp+1] && (prev_wp < (int)(maps_s.size()-1) ))
    {
        prev_wp++;
    }
    
    int wp2 = (prev_wp+1)%maps_x.size();
    
    double heading = atan2((maps_y[wp2]-maps_y[prev_wp]),(maps_x[wp2]-maps_x[prev_wp]));
    // the x,y,s along the segment
    double seg_s = (s-maps_s[prev_wp]);
    
    double seg_x = maps_x[prev_wp]+seg_s*cos(heading);
    double seg_y = maps_y[prev_wp]+seg_s*sin(heading);
    
    double perp_heading = heading-pi()/2;
    
    double x = seg_x + d*cos(perp_heading);
    double y = seg_y + d*sin(perp_heading);
    
    return {x,y};
    
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
    
    int lanes=3;//number lanes going in proper direction
    double laneWidth=4;//lanes are 4m wide
    double speedLim=49.5;//speed limit
    double maxAcc=.224;//acceleration limit
    
    double jerk=.1;//play with this more can probably be higher
    
    double desSpeed=0;//how fast to drive(start at 0)
    double acc=maxAcc;//start at maxAcc since in simulator no cars are around
    
    int lane=1;//lane trying to drive in
    
    //TODO : Chane the "magic number" 15 im using in places like as my closeness threshold to a calculation of how much room in meters I need to decelerate to 0 mph without exceeding acceleration/jerk thresholds + a little extra
    // 15 seems to work well though
    
    h.onMessage([&jerk,&maxAcc,&acc,&desSpeed,&lanes,&lane,&laneWidth,&speedLim,&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,&map_waypoints_dx,&map_waypoints_dy](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
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
                    int prevSize=previous_path_x.size();
                    // Previous path's end s and d values
                    double end_path_s = j[1]["end_path_s"];
                    double end_path_d = j[1]["end_path_d"];
                    
                    // Sensor Fusion Data, a list of all other cars on the same side of the road.
                    auto sensor_fusion = j[1]["sensor_fusion"];
                    
                    
                    //DECIDE ACTION(change/keep lane, change speed)
                    if(prevSize>0){
                        car_s=end_path_s;
                    }
                    bool close=false;
                    bool lLane=(lane>0);//whether lane to leftexists
                    double lClose=50;//closest car in left lane
                    bool rLane=(lane<(lanes-1));//whether lane to right exists
                    double rClose=50;//closest car in left lane
                    
                    //check if coming up on car and get closest car in left and right lanes
                    for(int i=0;i<sensor_fusion.size();i++){
                        vector<double> car=sensor_fusion[i];//[id,x,y,vx,vy,s,d]
                        double speed=sqrt(car[3]*car[3]+car[4]*car[4]);
                        double checkS=car[5]+speed*.02*prevSize;//get cars future s position
                        double sDiff=checkS-car_s;//et difference between car's s and check's s
                        if(car[6]>lane*laneWidth&&car[6]<(lane+1)*laneWidth){//if car is in my lane
                            if(checkS>car_s&&(sDiff<15)){//if theres a chance would hit a car in front of us followin prevpath
                                close=true;
                            }
                        }else if(lLane&&car[6]>(lane-1)*laneWidth&&car[6]<lane*laneWidth){//if lane to my left exists and car is in it
                            if(sDiff>15){//if in front and far enough
                                if(sDiff<lClose){//if closest car ahead in that lane
                                    lClose=sDiff;//keep track
                                }
                            }else if(sDiff>-15){//if car is in way can't change to that lane
                                lLane=false;
                            }
                        }else if(rLane&&car[6]>(lane+1)*laneWidth&&car[6]<(lane+2)*laneWidth){//if lane to my right exists and car is in it
                            if(sDiff>15){//if in front and far enough
                                if(sDiff<rClose){//if closest car ahead in that lane
                                    rClose=sDiff;//keep track
                                }
                            }else if(sDiff>-15){//if car is in way can't change to that lane
                                rLane=false;
                            }
                        }
                    }
                    if(!rLane){
                        rClose=0;
                    }
                    
                    //calculate lane changes and speed to travel
                    if(close){//if too close to a car in same lane
                        if(acc<maxAcc){
                            acc+=jerk;
                        }
                        desSpeed-=acc;
                        if(desSpeed<0){//make it really small but not 0/negative or get error in spline at assert(m_x[i]<m_x[i+1]) in splin.h set_points
                            desSpeed=0.001;
                        }
                        if(lLane&&lClose>rClose){//if can change lanes to left and more room to do so then changing right
                            lane--;
                        }else if(rLane){//if can change lanes to right
                            lane++;
                        }
                    }else if(desSpeed<speedLim){//if can speed up do so
                        if(acc<maxAcc){
                            acc+=jerk;
                        }
                        desSpeed+=acc;
                    }
                    
                    //GENERATE TRAJECTORY FOR ACTION USING SPLINE
                    double refX=car_x;
                    double refY=car_y;
                    double refYaw=deg2rad(car_yaw);//radians
                    
                    //get spaced out waypoints for creating spline(start with where car is/end of previous path generated)
                    vector<double> ptsX;
                    vector<double> ptsY;
                    
                    if(prevSize<2){//if arent enough points passed over from previous path create basis for spline from cars current position
                        ptsX.push_back(refX-cos(refYaw));
                        ptsY.push_back(refY-sin(refYaw));
                    }else{//otherwise use previous path's points for spline's basis
                        refX=previous_path_x[prevSize-1];
                        refY=previous_path_y[prevSize-1];
                        double prev_refX=previous_path_x[prevSize-2];
                        double prev_refY=previous_path_y[prevSize-2];
                        refYaw=atan2(refY-prev_refY,refX-prev_refX);
                        ptsX.push_back(prev_refX);
                        ptsY.push_back(prev_refY);
                    }
                    ptsX.push_back(refX);
                    ptsY.push_back(refY);
                    
                    //add waypoints that describe the lane want to travel to spline's basis
                    vector<double> wp;
                    for(int i=30;i<=90;i+=30){//last point from previous path is at least
                        wp=getXY(car_s+i,laneWidth*(lane+.5),map_waypoints_s,map_waypoints_x,map_waypoints_y);
                        ptsX.push_back(wp[0]);
                        ptsY.push_back(wp[1]);
                    }
                    
                    //shift basis points to reference of car
                    for(int i=0;i<ptsY.size();i++){
                        double shiftX=ptsX[i]-refX;
                        double shiftY=ptsY[i]-refY;
                        ptsX[i]=shiftX*cos(-refYaw)-shiftY*sin(-refYaw);
                        ptsY[i]=shiftX*sin(-refYaw)+shiftY*cos(-refYaw);
                    }
                    //create spline with basis points
                    tk::spline s;
                    s.set_points(ptsX,ptsY);//should have 5 points
                    
                    //sample spline for points
                    vector<double> next_x_vals;
                    vector<double> next_y_vals;
                    
                    //push points left from previous path
                    for(int i=0;i<prevSize;i++){
                        next_x_vals.push_back(previous_path_x[i]);
                        next_y_vals.push_back(previous_path_y[i]);
                    }
                    
                    double targX=15.0;
                    double targY=s(targX);
                    double targDist=sqrt(targX*targX+targY*targY);
                    double step=targX/(targDist/(.02*desSpeed/2.24));
                    double xOff=0;
                    
                    for(int i=0;i<50-prevSize;i++){
                        double x=xOff+step;
                        double y=s(x);
                        xOff=x;
                        double tempX=x;
                        //transform from relative coords to global map coords
                        x=x*cos(refYaw)-y*sin(refYaw)+refX;
                        y=y*cos(refYaw)+tempX*sin(refYaw)+refY;
                        
                        //add them to path
                        next_x_vals.push_back(x);
                        next_y_vals.push_back(y);
                    }
                    json msgJson;
                    msgJson["next_x"] = next_x_vals;
                    msgJson["next_y"] = next_y_vals;
                    
                    auto msg = "42[\"control\","+ msgJson.dump()+"]";
                    
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
    h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data,
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
