#ifndef VEHICLE_H
#define VEHICLE_H

#include <map>
#include <string>
#include <vector>

using namespace std;

// vehicle class

class vehicle
{
    private:
        /* data */
    public:

        double s; 
        double d;
        int lane;
        // int next_lane_;
        double speed;
        double ref_vel;
        double x;
        double y;
        double yaw;
        double vx;
        double vy;

        vector<vector<double>> left_lane_front_car_feature_sd;
        vector<vector<double>> mid_lane_front_car_feature_sd;
        vector<vector<double>> right_lane_front_car_feature_sd;


        
        vehicle();
        vehicle(int lane, double s, double d, double speed, double ref_vel, double x, double y, double yaw);
        virtual ~vehicle();

        void next_lane(vector<vehicle> &left_lane_vehicles, vector<vehicle> &mid_lane_vehicles, vector<vehicle> &right_lane_vehicles, int prev_size, vector<int> lanes,
                       const vector<double> &maps_x, const vector<double> &maps_y);
        
        vector<bool> check_left_lane_vehicle_too_close(vector<vehicle> &left_lane_vehicles, int prev_size);
        vector<bool> check_mid_lane_vehicle_too_close(vector<vehicle> &mid_lane_vehicles, int prev_size);
        vector<bool> check_right_lane_vehicle_too_close(vector<vehicle> &right_lane_vehicles, int prev_size);

        // vector<vector<double>> generate_predictions(double traj_start_time, double duration);

        void generate_predictions(vector<vehicle> &left_lane_vehicles, vector<vehicle> &mid_lane_vehicles, vector<vehicle> &right_lane_vehicles, int prev_size, 
                                  const vector<double> &maps_x, const vector<double> &maps_y);
};

// vehicle::vehicle(/* args */)
// {
// }

// vehicle::~vehicle()
// {
// }




# endif






