#include <iostream>
#include <cmath>

using namespace std;

#define PI 3.14159
#define DYN_MIN_POS 0
#define DYN_MAX_POS 1000
#define DEGREE_MIN 0
#define DEGREE_MAX 90

#define Y_REST 127
#define Z_REST -50
#define PI 3.14159
#define R2 127
#define R3 190

void IK(int x, int y, int z, double joint_angles[])
{
  double l1, theta1, theta2, theta3, phi2, phi1;

  y += Y_REST;
  z += Z_REST;
  
  // Joint 1
  theta1 = atan2(x, y);
  
  //Joint 2 and 3
  l1 = sqrt(x*x + y*y);
  printf("l1: %f\n", l1);

  printf("(R2*R2) + (R3*R3) - (l1*l1)) / (2 * R2 * R3): %f\n", (R2*R2) + (R3*R3) - (l1*l1)) / (2 * R2 * R3);

  theta3 = acos(((R2*R2) + (R3*R3) - (l1*l1)) / (2 * R2 * R3));
  printf("theta3: %f\n", theta3);

  phi2 = acos(((R2*R2) + (l1*l1) - (R3*R3)) / (2 * R2 * l1));
  printf("phi2: %f\n", phi2);

  phi1 = atan2(Z_REST, Y_REST);
  printf("phi1: %f\n", phi1);

  theta2 = phi2 - phi1;
  printf("theta2: %f\n", theta2);

}


int main()
{   double joints_angles[3];
    IK(80, 80, 80, joints_angles);
    return 0;
}