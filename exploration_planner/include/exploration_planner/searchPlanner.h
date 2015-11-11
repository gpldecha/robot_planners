#ifndef SEARCHCONTROLLER_H_
#define SEARCHCONTROLLER_H_


//mycode

#include "robot_planners/gmmPlanner.h"
#include "robot_planners/discretePlanner.h"

namespace CTRLMODES{
    enum controlModes {NONE=0,SIMPLE=1,GMM=2,HYBRID=3,DISCRETE=4};
}

class SearchPlanner{

public:

   SearchPlanner();

    void  init();

    void GetDirection(arma::vec3 &direction,const arma::vec3& bel_pos,double uncertainty=1.0);

    void setTarget(const arma::vec3& target);

    void setControlMode(CTRLMODES::controlModes mode);

    void draw(const arma::vec3 &beliefPosition, arma::mat33 *const pRot);

private:



private:

    arma::vec obs;

public:

    bool bFirst;
    arma::vec3 mTarget;

 //   double T;
    CTRLMODES::controlModes ctrl_mode;
    bool isInAir;

    // Controllers

    planners::GMAPlanner mGmmPlanner;
    planners::GMRPlanner mGmmAir;
    planners::SimplePlanner mSimplePlanner;
    planners::HybridPlanner mHybridPlanner;

    planners::DiscretePlanner mDiscretePlanner;

};


#endif
