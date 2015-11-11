#include "searchcontroller.h"



SearchController::SearchController(){

    init();
}

void SearchController::init(){
    bFirst=true;
    isInAir = true;
    T = -0.5;
    obs = arma::vec(4);
    ctrl_mode =CTRLMODES::NONE;

    mGmmAir.initConditional();
    mGmmPlanner.initConditional();

    mHybridPlanner.initialise();
}

void SearchController::GetDirection(arma::vec3& direction,const arma::vec3& bel_pos,double uncertainty){

    switch(ctrl_mode){

    case CTRLMODES::NONE:
    {
        break;
    }
    case CTRLMODES::GMM:
    {
        std::cout<< "CTRLMODES::GMM" << std::endl;
        obs(0) = bel_pos(0) + T;
        obs(1) = bel_pos(1);
        obs(2) = bel_pos(2);
        obs(3) = uncertainty;
        obs.print("obs");
        if(isInAir){
            std::cout<< "gmmAir" << std::endl;
            mGmmAir.condition(obs);
            mGmmAir.getDirection(direction);
        }else{
           // std::cout<< "gmmController" << std::endl;
            mGmmPlanner.condition(obs);
            mGmmPlanner.getDirection(direction);
        }
        break;
    }
    case CTRLMODES::SIMPLE:
    {
        std::cout<< "CTROLMODES::SIMPLE" << std::endl;
        mSimplePlanner.setCurrentPos(bel_pos);
        mSimplePlanner.setTarget(mTarget);
        mSimplePlanner.getDirection(direction);

        break;
    }
    case CTRLMODES::HYBRID:
    {
        obs(0) = bel_pos(0) + T;
        obs(1) = bel_pos(1);
        obs(2) = bel_pos(2);
        obs(3) = uncertainty;
        if(isInAir){
            std::cout<< "gmmAir" << std::endl;
            mGmmAir.condition(obs);
            mGmmAir.getDirection(direction);
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

    }


}

void SearchController::setControlMode(CTRLMODES::controlModes mode){
    ctrl_mode = mode;
}

void SearchController::setTarget(const arma::vec3& target){
    mTarget=target;
}
