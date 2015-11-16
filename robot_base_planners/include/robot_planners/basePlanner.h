#ifndef BASEPLANNER_H_
#define BASEPLANNER_H_

#include <armadillo>

class Base_EE_Planner {

public:

    virtual void get_ee_linear_velocity(arma::colvec3& linear_velocity) = 0;

    virtual void get_ee_angular_velocity(arma::colvec3& ang_velocity) = 0;

private:

};


#endif
