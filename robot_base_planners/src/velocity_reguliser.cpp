#include "robot_planners/velocity_reguliser.h"
#include <cmath>
#include <iostream>

Velocity_reguliser::Velocity_reguliser(double max_speed, double min_speed):
min_speed(min_speed),
max_speed(max_speed)
{

}

void Velocity_reguliser::set_min_speed_ms(double min_speed){
    this->min_speed = min_speed;
}

void Velocity_reguliser::set_max_speed_ms(double max_speed){
    this->max_speed = max_speed;
}

double Velocity_reguliser::bel_shape_curve(double distance_target_m, double beta){

    distance_target_cm = 100 * distance_target_m;

    std::cout<< "distance[cm]:  " << distance_target_cm << std::endl;
    std::cout<< "1-exp:         " << (1.0 - std::exp(-beta * (distance_target_cm * distance_target_cm))) << std::endl;
    std::cout<< "raw speed:     " << (min_speed + (1.0 - std::exp(-beta * (distance_target_cm * distance_target_cm))) * max_speed) << std::endl;
    std::cout<< "speed:         " << scale((1.0 - std::exp(-beta * (distance_target_cm * distance_target_cm))),0,1,min_speed,max_speed) << std::endl;

    return scale((1.0 - std::exp(-beta * (distance_target_cm * distance_target_cm))),0,1,min_speed,max_speed);

}
