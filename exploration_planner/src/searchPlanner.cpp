#include "exploration_planner/searchPlanner.h"
#include <string>


SearchPlanner::SearchPlanner(){

    ctrl_mode =CTRLMODES::SIMPLE;

}

void SearchPlanner::init(){
 /*   bFirst=true;
    isInAir = true;
    obs = arma::vec(4);

    std::string path = "/home/guillaume/roscode/RobotToolKit/packages/addons/WAMBrain/first_sucess_gmm/";



    mGmmAir.load(path + "air/");
    mGmmPlanner.load(path + "gmm_used_in_report/");
    mHybridPlanner.gmaPlanner.load(path + "gmm_used_in_report/");

    /// Load Discrete plan

    std::string filename ="/home/guillaume/MatlabWorkSpace/AMDP/CostalNavigation/SavePolicy/Policy.txt";
    mDiscretePlanner.load(filename);

    mGmmAir.initConditional();
    mGmmPlanner.initConditional();
    mHybridPlanner.initialise();*/
}

void SearchPlanner::GetDirection(arma::vec3& direction,const arma::vec3& bel_pos,double uncertainty){

/*    switch(ctrl_mode){

    case CTRLMODES::NONE:
    {
        break;
    }
    case CTRLMODES::GMM:
    {
      //  std::cout<< "CTRLMODES::GMM" << std::endl;
        obs(0) = bel_pos(0);
        obs(1) = bel_pos(1);
        obs(2) = bel_pos(2);
        obs(3) = uncertainty;
      //  obs.print(" ----- obs -----");

       if(isInAir){
        //    std::cout<< "-------->   gmmAir" << std::endl;
            mGmmAir.condition(obs);
            mGmmAir.getDirection(direction);

        }else{
          //  std::cout<< "-------->   gmmController" << std::endl;
            mGmmPlanner.condition(obs);
            mGmmPlanner.getDirection(direction);
        }
        break;
    }
    case CTRLMODES::SIMPLE:
    {

        mSimplePlanner.setCurrentPos(bel_pos);
        mSimplePlanner.setTarget(mTarget);
        mSimplePlanner.getDirection(direction);

        break;
    }
    case CTRLMODES::HYBRID:
    {
        obs(0) = bel_pos(0);
        obs(1) = bel_pos(1);
        obs(2) = bel_pos(2);
        obs(3) = uncertainty;
        if(isInAir){
           // std::cout<< "gmmAir" << std::endl;
            mGmmAir.condition(obs);
            mGmmAir.getDirection(direction);
           // direction.print("direction");
        }else{
            // std::cout<< "gmmController" << std::endl;
            mHybridPlanner.gmaPlanner.condition(obs);
            mHybridPlanner.simplePlanner.setCurrentPos(bel_pos);
            mHybridPlanner.simplePlanner.setTarget(mTarget);
            mHybridPlanner.setUncertainty(uncertainty);
            mHybridPlanner.getDirection(direction);
        }
        break;
    }
    case CTRLMODES::DISCRETE:
    {
      //  std::cout<< "SearchPlanner::GetDirection case:DISCRETE" << std::endl;
        mDiscretePlanner.setPosition(bel_pos);
        mDiscretePlanner.getDirection(direction);

    }

    }*/

}

void SearchPlanner::setControlMode(CTRLMODES::controlModes mode){
    ctrl_mode = mode;
}

void SearchPlanner::setTarget(const arma::vec3& target){
    mTarget=target;
}

 void SearchPlanner::draw(const arma::vec3& beliefPosition,arma::mat33 * const pRot){

     // TODO: take advantage of polymorphism, have mPlanner->draw(direction,position);

 /*    switch(ctrl_mode){

     case CTRLMODES::NONE:
     {
         break;
     }
     case CTRLMODES::GMM:
     {
        // mGmmPlanner.drawDirection(direction,mBeliefPos);
         if(isInAir){
             mGmmAir.drawConditional(beliefPosition,pRot);
         }else{
             mGmmPlanner.drawConditional(beliefPosition,pRot);
         }
        break;
     }
     case CTRLMODES::SIMPLE:
     {
         //mSimplePlanner.drawDirection(direction,mBeliefPos);
         break;
     }
     case CTRLMODES::HYBRID:
     {
     //   mHybridPlanner.drawDirection(direction,mBeliefPos);
        break;
     }

     }*/
 }
