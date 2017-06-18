#ifndef MPC_H
#define MPC_H

#include <vector>
#include "Eigen-3.3/Eigen/Core"

using namespace std;

class MPC {
public:
  MPC(double Lf) :
    Lf_(Lf)
  {
  }

  // Solve the model given an initial state and polynomial coefficients.
  // Return the first actuatotions.
  vector<double> Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs);
  vector<double> pred_x;
  vector<double> pred_y;

  double Lf() const { return Lf_; }

private:
  double Lf_;
};

#endif /* MPC_H */
