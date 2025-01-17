#include <iostream>
#include <cmath>

using namespace std;

#define PI 3.14159
#define DYN_MIN_POS 0
#define DYN_MAX_POS 1000
#define DEGREE_MIN 0
#define DEGREE_MAX 90


int main()
{   
    //Millimeters
    double x_move = 80;  //How far to move the foot forward
    double y_rest = 304; //distance from joint 1 to foot
    double angle_radians;
    double angle_degrees;

    //Only moving in one quadrant
    double old_range = DEGREE_MAX - DEGREE_MIN;
    double new_range = DYN_MAX_POS - DYN_MIN_POS; 
    double new_value;

    angle_radians = atan2(x_move, y_rest);
    printf("angle_radians = %f\n\n", angle_radians);

    //convert to degrees - make code more readable
    angle_degrees = angle_radians * (180/PI);
    printf("angle_degrees = %f\n\n", angle_degrees);

    //convert degrees to dynamixel range
    new_value = (((angle_degrees - DEGREE_MIN) * new_range) / old_range) + DYN_MIN_POS;
    printf("dyanmixel rotation value = %f\n\n", new_value);


    return 0;
}