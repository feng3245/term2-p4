#include "PID.h"
#include <chrono>
using namespace std;
using namespace std::chrono;
/*
* TODO: Complete the PID class.
*/

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd) {
this->Kp = Kp;
this->Ki = Ki,
this->Kd = Kd;
totalcte = 0;
previouscte = 0;
last_measurement_time = system_clock::now().time_since_epoch().count();
}

void PID::UpdateError(double cte) {
totalcte += cte;
}

double PID::TotalError() {
return totalcte;
}
double PID::GetSteering(double cte){
double dt = last_measurement_time - system_clock::now().time_since_epoch().count();
double current_cte = cte;
double sum_cte = totalcte;
double steering = -Kp*current_cte-Kd*((current_cte-previouscte)/1)-Ki*sum_cte;
previouscte = current_cte;
return steering;
}
