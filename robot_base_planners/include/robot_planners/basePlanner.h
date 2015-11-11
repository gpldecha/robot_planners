#ifndef BASEPLANNER_H_
#define BASEPLANNER_H_

#include <armadillo>

class BasePlanner {

public:

    virtual void getDirection(arma::vec3& direction) = 0;

    void drawDirection(const arma::vec3& direction,const arma::vec3 &position);

private:

};


#endif
