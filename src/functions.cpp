#include <fstream>
#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "json.hpp"
#include "spline.h"

using namespace std;

// for convenience
using json = nlohmann::json;

/*void print_message(){
  cout << "test fct ok !" << endl;
}
*/
void state_road(bool& too_close, bool& right_lane_free, bool& left_lane_free, vector<vector<double>> sensor_fusion, int& lane, double car_s, int prev_size, double& ref_vel, vector<int>& LCL, vector<int>& LCR){
  /*
  This function search for vehicle in front of our autonomous vehicle
  */

  // check for that the car stay on the right side lanes
  if(lane == 0){
    left_lane_free = false;
  }else if(lane == 2){
    right_lane_free = false;
  }

  // find cars around the vehicle 
  for(int i=0; i<sensor_fusion.size(); i++){
    //car is in my lane
    float d = sensor_fusion[i][6];

    //find the lane in which the "other car" is.
    int other_car_lane = fabs(d/4);

    
    double vx = sensor_fusion[i][3];
    double vy = sensor_fusion[i][4];
    double check_speed = sqrt(vx*vx+vy*vy);
    double check_car_s = sensor_fusion[i][5];

    check_car_s += ((double)prev_size*.02*check_speed); // Estimate where the "other car" will be in the next iteration. Because, action of our autonomous vehicle will take effect only on the next iteration.

    double diff_car_s = check_car_s - car_s;

    // check s value greater than mine and s gap
    if((diff_car_s > 0) && (diff_car_s) < 30){
      
      if(other_car_lane == (lane-1)){//other car in the left lane ?
	left_lane_free = false;
      }else if(other_car_lane == (lane+1)){//other car in the right lane ?
	right_lane_free = false;
      }else if(other_car_lane == (lane)){//other car in the same lane ?
	too_close = true;
      }
      
    }else if((diff_car_s < 0) && (diff_car_s > -10)){
      
      if(other_car_lane == (lane-1)){//other car in the left lane ?
	left_lane_free = false;
      }else if(other_car_lane == (lane+1)){//other car in the right lane ?
	right_lane_free = false;
      }
    }
  }

  LCR.erase(LCR.begin());
  LCR.push_back(right_lane_free);

  LCL.erase(LCL.begin());
  LCL.push_back(left_lane_free);

  bool LCL_state = accumulate(LCL.begin(), LCL.end(), 0) > 7;
  //cout << "LCL state : " << LCL_state << endl;
  bool LCR_state = accumulate(LCR.begin(), LCR.end(), 0) > 7;

  if(too_close){
    if ((lane == 0) && LCR_state){
      lane += 1;
    }else if((lane == 2) && LCL_state){
      lane -= 1;
    }else if(lane == 1){
      if(LCR_state && !LCL_state){
	lane += 1;
      }else if(!LCR_state && LCL_state){
	lane -= 1;
      }else if(LCR_state && LCL_state){
	lane -= 1; // A AMELIORER !!
      }else{
	ref_vel -= .224;
      }
    }else{
      ref_vel -= .224;
    }
  }else if(ref_vel < 49.5){
    ref_vel += .224;
  }

}
