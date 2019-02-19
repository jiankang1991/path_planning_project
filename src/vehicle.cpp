
#include <math.h>
#include <iterator>
#include <iostream>

#include "helpers.h"
#include "vehicle.h"
#include "constants.h"


using std::vector;

vehicle::vehicle() {}

// constructor
vehicle::vehicle(int lane, double s, double d, double speed, double ref_vel, double x, double y, double yaw) {

    this->lane = lane;
    this->s = s;
    this->d = d;
    this->speed = speed;
    this->ref_vel = ref_vel;
    this->x = x;
    this->y = y;
    this->yaw = yaw;

}


vehicle::~vehicle() {}

// check the two cars (front and behind) on the left lane, return the flags whether they are too close
vector<bool> vehicle::check_left_lane_vehicle_too_close(vector<vehicle> &left_lane_vehicles, int prev_size) {
    
    double check_car_front_s = left_lane_vehicles[0].s;
    check_car_front_s += (double)prev_size*TIME_STEP*left_lane_vehicles[0].speed;


    double check_behind_vehicle_s = left_lane_vehicles[1].s;
    check_behind_vehicle_s += (double)prev_size*TIME_STEP*left_lane_vehicles[1].speed;

    bool front_too_close = false;
    bool behind_too_close = false;
    vector<bool> too_close_flags;

    if((check_car_front_s >= this->s)&&((check_car_front_s - this->s)<MIN_FRONT_CAR)) {
        front_too_close = true;
    }
    if((check_behind_vehicle_s <= this->s)&&(fabs(check_behind_vehicle_s - this->s)<MIN_BEHIND_CAR)) {
        behind_too_close = true;
    }

    too_close_flags.push_back(front_too_close);
    too_close_flags.push_back(behind_too_close);

    return too_close_flags;

}

// check the two cars (front and behind) on the middle lane, return the flags whether they are too close
vector<bool> vehicle::check_mid_lane_vehicle_too_close(vector<vehicle> &mid_lane_vehicles, int prev_size) {
    
    double check_car_front_s = mid_lane_vehicles[0].s;
    check_car_front_s += (double)prev_size*TIME_STEP*mid_lane_vehicles[0].speed;

    double check_behind_vehicle_s = mid_lane_vehicles[1].s;
    check_behind_vehicle_s += (double)prev_size*TIME_STEP*mid_lane_vehicles[1].speed;

    bool front_too_close = false;
    bool behind_too_close = false;
    vector<bool> too_close_flags;

    if((check_car_front_s >= this->s)&&((check_car_front_s - this->s)<MIN_FRONT_CAR)) {
        front_too_close = true;
    }
    if((check_behind_vehicle_s <= this->s)&&(fabs(check_behind_vehicle_s - this->s)<MIN_BEHIND_CAR)) {
        behind_too_close = true;
    }

    too_close_flags.push_back(front_too_close);
    too_close_flags.push_back(behind_too_close);

    return too_close_flags;
}

// check the two cars (front and behind) on the right lane, return the flags whether they are too close
vector<bool> vehicle::check_right_lane_vehicle_too_close(vector<vehicle> &right_lane_vehicles, int prev_size) {
    
    double check_car_front_s = right_lane_vehicles[0].s;
    check_car_front_s += (double)prev_size*TIME_STEP*right_lane_vehicles[0].speed;

    double check_behind_vehicle_s = right_lane_vehicles[1].s;
    check_behind_vehicle_s += (double)prev_size*TIME_STEP*right_lane_vehicles[1].speed;

    bool front_too_close = false;
    bool behind_too_close = false;
    vector<bool> too_close_flags;

    if((check_car_front_s >= this->s)&&((check_car_front_s - this->s)<MIN_FRONT_CAR)) {
        front_too_close = true;
    }
    if((check_behind_vehicle_s <= this->s)&&(fabs(check_behind_vehicle_s - this->s)<MIN_BEHIND_CAR)) {
        behind_too_close = true;
    }


    too_close_flags.push_back(front_too_close);
    too_close_flags.push_back(behind_too_close);

    return too_close_flags;
}

// next lane label generation according to all the information of the six vehicles
void vehicle::next_lane(vector<vehicle> &left_lane_vehicles, vector<vehicle> &mid_lane_vehicles, vector<vehicle> &right_lane_vehicles, int prev_size, vector<int> lanes,
                        const vector<double> &maps_x, const vector<double> &maps_y) {

    vector<bool> left_lane_too_close_flags = check_left_lane_vehicle_too_close(left_lane_vehicles, prev_size);
    vector<bool> mid_lane_too_close_flags = check_mid_lane_vehicle_too_close(mid_lane_vehicles, prev_size);
    vector<bool> right_lane_too_close_flags = check_right_lane_vehicle_too_close(right_lane_vehicles, prev_size);

    generate_predictions(left_lane_vehicles, mid_lane_vehicles, right_lane_vehicles, prev_size, maps_x, maps_y);

    // cout << "left_lane_too_close_flags: " << left_lane_too_close_flags[0] << left_lane_too_close_flags[1] << endl;
    // cout << "mid_lane_too_close_flags: " << mid_lane_too_close_flags[0] << mid_lane_too_close_flags[1] << endl;
    // cout << "right_lane_too_close_flags: " << right_lane_too_close_flags[0] << right_lane_too_close_flags[1] << endl;

    int next_lane_;

    if (this->lane == lanes[0]) {
        if (left_lane_too_close_flags[0]) {
            
            double decrease_v_step = fabs(this->speed - left_lane_vehicles[0].speed*2.24)/PARA_D;

            vector<double> left_front_car_end_sd = this->left_lane_front_car_feature_sd[this->left_lane_front_car_feature_sd.size()-1];
            vector<double> mid_front_car_end_sd = this->mid_lane_front_car_feature_sd[this->mid_lane_front_car_feature_sd.size()-1];

            if (mid_lane_too_close_flags[0]||mid_lane_too_close_flags[1]) {
                
                next_lane_ = lanes[0];

                if (fabs(mid_front_car_end_sd[1]-2-4*this->lane)<2) {

                    if ((mid_front_car_end_sd[0]-this->s < left_front_car_end_sd[0]-this->s)) {

                        this->ref_vel -= decrease_v_step * 1.5;
                        
                    } else
                    {
                        this->ref_vel -= decrease_v_step;
                        // next_lane_ = lanes[0];
                    }
                    
                } else
                {
                    this->ref_vel -= decrease_v_step;
                    // next_lane_ = lanes[0];
                }
                
            } else
            {
                next_lane_ = lanes[1];
            }
        } else if (!left_lane_too_close_flags[0]) {

            next_lane_ = lanes[0];
            
            if (this->ref_vel < MAX_VEL) {
                this->ref_vel += 0.224;
            }
        }

    } else if (this->lane == lanes[1]) {

        if (mid_lane_too_close_flags[0]) {

            double decrease_v_step = fabs(this->speed - mid_lane_vehicles[0].speed*2.24)/PARA_D;
            
            vector<double> left_front_car_end_sd = this->left_lane_front_car_feature_sd[this->left_lane_front_car_feature_sd.size()-1];
            vector<double> mid_front_car_end_sd = this->mid_lane_front_car_feature_sd[this->mid_lane_front_car_feature_sd.size()-1];
            vector<double> right_front_car_end_sd = this->right_lane_front_car_feature_sd[this->right_lane_front_car_feature_sd.size()-1];


            if (!left_lane_too_close_flags[0] && !left_lane_too_close_flags[1]) {
                next_lane_ = lanes[0];
            } else if ((left_lane_too_close_flags[0] || left_lane_too_close_flags[1]) && !(right_lane_too_close_flags[0] || right_lane_too_close_flags[1])) 
            {
                next_lane_ = lanes[2];
            } else 
            {
                next_lane_ = lanes[1];
                if (fabs(left_front_car_end_sd[1]-2-4*this->lane)<2) {
                    // next_lane_ = lanes[1];

                    if ((left_front_car_end_sd[0]-this->s < mid_front_car_end_sd[0]-this->s)) {

                        this->ref_vel -= decrease_v_step * 1.5;
                        
                    } else
                    {
                        this->ref_vel -= decrease_v_step;
                        // next_lane_ = lanes[1];
                    }
                } else if(fabs(left_front_car_end_sd[1]-2-4*this->lane)>2 && fabs(right_front_car_end_sd[1]-2-4*this->lane)<2) {
                    // next_lane_ = lanes[1];
                    if ((right_front_car_end_sd[0]-this->s < mid_front_car_end_sd[0]-this->s)) {
                        this->ref_vel -= decrease_v_step * 1.5;
        
                    } else
                    {
                        this->ref_vel -= decrease_v_step;
                        // next_lane_ = lanes[1];
                    }
                    
                } else
                {
                    this->ref_vel -= decrease_v_step;
                    // next_lane_ = lanes[1];
                }
                

            }
        } else if (!mid_lane_too_close_flags[0]) {

            next_lane_ = lanes[1];
            
            if (this->ref_vel < MAX_VEL) {
                this->ref_vel += 0.224;
            }
        }

    } else {
        if (right_lane_too_close_flags[0]) {
            
            double decrease_v_step = fabs(this->speed - right_lane_vehicles[0].speed*2.24)/PARA_D;

            vector<double> mid_front_car_end_sd = this->mid_lane_front_car_feature_sd[this->mid_lane_front_car_feature_sd.size()-1];
            vector<double> right_front_car_end_sd = this->right_lane_front_car_feature_sd[this->right_lane_front_car_feature_sd.size()-1];


            if (mid_lane_too_close_flags[0] || mid_lane_too_close_flags[1]) {

                if (fabs(mid_front_car_end_sd[1]-2-4*this->lane)<2) {
                    next_lane_ = lanes[2];

                    if ((mid_front_car_end_sd[0]-this->s < right_front_car_end_sd[0]-this->s)) {
                        
                        this->ref_vel -= decrease_v_step * 1.5;

                    } else {

                        this->ref_vel -= decrease_v_step;
                        // next_lane_ = lanes[2];
                    }

                } else
                {
                    this->ref_vel -= decrease_v_step;
                    next_lane_ = lanes[2];
                }
                
            } else {
                next_lane_ = lanes[1];
            }

        } else if (!right_lane_too_close_flags[0]) {

            next_lane_ = lanes[2];

            if (this->ref_vel < MAX_VEL) {
                this->ref_vel += 0.224;
            }


        }
    }


    // if (this->lane == lanes[0]) {
    //     if (left_lane_too_close_flags[0]) {
            
    //         double decrease_v_step = (this->speed - left_lane_vehicles[0].speed*2.24)/PARA_D;

    //         if (mid_lane_too_close_flags[0]||mid_lane_too_close_flags[1]) {

    //             this->ref_vel -= decrease_v_step;
    //             next_lane_ = lanes[0];

    //         } else {
    //             next_lane_ = lanes[1];
    //         }

    //     } else if (!left_lane_too_close_flags[0]) {
    //         next_lane_ = lanes[0];

    //         if (this->ref_vel < MAX_VEL) {
    //             this->ref_vel += 0.224;
    //         }

    //     }
    // } else if (this->lane == lanes[1]) {

    //     if (mid_lane_too_close_flags[0]) {
            
    //         double decrease_v_step = (this->speed - mid_lane_vehicles[0].speed*2.24)/PARA_D;

    //         if (!left_lane_too_close_flags[0] && !left_lane_too_close_flags[1]) {
    //             next_lane_ = lanes[0];
    //         } else if ((left_lane_too_close_flags[0] || left_lane_too_close_flags[1]) && !(right_lane_too_close_flags[0] || right_lane_too_close_flags[1])) {
    //             next_lane_ = lanes[2];
    //         } else {
    //             this->ref_vel -= decrease_v_step;
    //             next_lane_ = lanes[1];
    //         }

    //     } else if (!mid_lane_too_close_flags[0]) {
    //         next_lane_ = lanes[1];

    //         if (this->ref_vel < MAX_VEL) {
    //             this->ref_vel += 0.224;
    //         }
            
    //     }
    // } else {
    //     if (right_lane_too_close_flags[0]) {
            
    //         double decrease_v_step = (this->speed - right_lane_vehicles[0].speed*2.24)/PARA_D;

    //         if (mid_lane_too_close_flags[0] || mid_lane_too_close_flags[1]) {
                
    //             this->ref_vel -= decrease_v_step;
    //             next_lane_ = lanes[2];

    //         } else {
    //             next_lane_ = lanes[1];
    //         }

    //     } else if (!right_lane_too_close_flags[0]) {
    //         next_lane_ = lanes[2];

    //         if (this->ref_vel < MAX_VEL) {
    //             this->ref_vel += 0.224;
    //         }


    //     }
    // }

    this->lane = next_lane_;

}
// predict the positions of all the surronding cars
void vehicle::generate_predictions(vector<vehicle> &left_lane_vehicles, vector<vehicle> &mid_lane_vehicles, vector<vehicle> &right_lane_vehicles, int prev_size,
                                   const vector<double> &maps_x, const vector<double> &maps_y) {


    double traj_start_time = (double)prev_size*TIME_STEP;

    for (int i=1; i<50-prev_size; i++) {

        double t = traj_start_time + (i*TIME_STEP);

        left_lane_vehicles[0].x += left_lane_vehicles[0].vx * t;
        left_lane_vehicles[0].y += left_lane_vehicles[0].vy * t;

        mid_lane_vehicles[0].x += mid_lane_vehicles[0].vx * t;
        mid_lane_vehicles[0].y += mid_lane_vehicles[0].vy * t;

        right_lane_vehicles[0].x += right_lane_vehicles[0].vx * t;
        right_lane_vehicles[0].y += right_lane_vehicles[0].vy * t;

        vector<double> left_car_sd = getFrenet(left_lane_vehicles[0].x, left_lane_vehicles[0].y, left_lane_vehicles[0].yaw, 
                                               maps_x, maps_y);
        vector<double> mid_car_sd = getFrenet(mid_lane_vehicles[0].x, mid_lane_vehicles[0].y, mid_lane_vehicles[0].yaw, 
                                              maps_x, maps_y);
        vector<double> right_car_sd = getFrenet(right_lane_vehicles[0].x, right_lane_vehicles[0].y, right_lane_vehicles[0].yaw, 
                                                maps_x, maps_y);

        
        this->left_lane_front_car_feature_sd.push_back(left_car_sd);
        this->mid_lane_front_car_feature_sd.push_back(mid_car_sd);
        this->right_lane_front_car_feature_sd.push_back(right_car_sd);



    }





}

















