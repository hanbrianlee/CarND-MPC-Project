#ifndef MPC_H
#define MPC_H

#include <vector>
#include "Eigen-3.3/Eigen/Core"

using namespace std;


// Both the reference cross track and orientation errors are 0.
// The reference velocity is set below. The MPC will generate a and delta that can keep the car on track with waypoints
const double ref_v = 100;

// The timesteps shouldn't be too high. The higher the speed, the less the number (can't look too far ahead)
//also 3rd order polynomial can't properly characterize if the road up ahead is too curvy.
const unsigned int N = 10; //number of time-steps.

//take into account 1 cycle of processing latency(approx 15~40ms) and data transfer delay(100ms)
//h message update rate is very fast (3~4ms) so will be ignored
const double dt = 0.13; // 130 milliseconds: 100ms + 15~35ms

// This value assumes the model presented in the classroom is used.
//
// It was obtained by measuring the radius formed by running the vehicle in the
// simulator around in a circle with a constant steering angle and velocity on a
// flat terrain.
//
// Lf was tuned until the the radius formed by the simulating the model
// presented in the classroom matched the previous radius.
//
// This is the length from front to CoG that has a similar radius.
const double Lf = 2.67; //by making this value larger (imagine a train), the solver will try to use more aggressive steering.



class MPC {
 public:
  MPC();

  virtual ~MPC();

  // Solve the model given an initial state and polynomial coefficients.
  // Return the first actuatotions.
  vector<double> Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs);
};

#endif /* MPC_H */
