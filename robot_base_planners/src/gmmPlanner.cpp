#include "robot_planners/gmmPlanner.h"
#include <limits>

using namespace planners;


/**

  BaseGMM Class Controller implementation

  **/


BaseGMM_EE_Planner::BaseGMM_EE_Planner(){
    bFirst = true;
}
BaseGMM_EE_Planner::BaseGMM_EE_Planner(const std::vector<std::size_t>& in,const std::vector<std::size_t>& out)
    : in(in),out(out)
{
    bFirst = true;
}


void BaseGMM_EE_Planner::load(std::string path_parameters){
    gmm.load(path_parameters);
}

void BaseGMM_EE_Planner::condition(arma::colvec& x){
    if(bFirst){
        gmm_c.condition(gmm,in,out);
        bFirst=false;
    }
    gmm_c.condition(x,gmm);
}

void BaseGMM_EE_Planner::drawConditional(const arma::vec3 &position, arma::mat33 * const pRot){

    /*if(max_pi > 0){
        for(unsigned int k = 0 ; k < gmm_cond.K;k++){
            mu_tmp = gmm_cond.getMu(k);
            mu_tmp = mu_tmp/arma::norm(mu_tmp,2);
            if(pRot != NULL){
                mu_tmp = (*pRot) * mu_tmp;
            }*/

            /*      pi_tmp = gmm_cond.Priors[k];
            scale = rescale(pi_tmp,0,max_pi,0.05,0.2);
            glPushMatrix();
            glTranslatef(position(0),position(1),position(2));
            GLTools::DrawOutline(true);
            glScaled(scale,scale,scale);
            colormap.jetColorMap(rgb,pi_tmp,0,max_pi);
            GLTools::SetColor(rgb[0],rgb[1],rgb[2],1);
            GLTools::DrawVector(Vector3(mu_tmp(0),mu_tmp(1),mu_tmp(2)));
            glPopMatrix();*/
    //    }
  //  }
}




/**

  GMR Planner implementation

  **/

GMR_EE_Planner::GMR_EE_Planner()
    :BaseGMM_EE_Planner()
{

}

GMR_EE_Planner::GMR_EE_Planner(const std::vector<std::size_t>& in,const std::vector<std::size_t>& out):
    BaseGMM_EE_Planner(in,out)
{

}

void GMR_EE_Planner::gmr(arma::colvec& x_in){
    condition(x_in);
    gmm_c.gmm_c.expection(velocity_ee);
}

void GMR_EE_Planner::get_ee_linear_velocity(arma::colvec3 &direction){
    direction = velocity_ee;
}

void GMR_EE_Planner::get_ee_angular_velocity(arma::colvec3 &ang_velocity){

}



/**

    GMA Controller implementation

**/


GMAPlanner::GMAPlanner(){
    bFirst=true;
}

GMAPlanner::GMAPlanner(string path_parameters){

    load(path_parameters);

}

void GMAPlanner::initConditional(){
    in.resize(4);
    out.resize(3);

    for(int i = 0; i < 4; i++){
        in[i] =i+3;
    }
    for(int i = 0; i < 3;i++){
        out[i] = i;
    }

}

void GMAPlanner::get_ee_linear_velocity(arma::vec3& direction){

    getIndexs();
    if(bFirst){
        pick_random_direction();
        bFirst = false;
    }else{
        if(!arma::is_finite(direction)){
            assert(arma::norm(mDirection_tmp,2) != 0);
            direction = mDirection_tmp;
        }
        if(arma::norm(direction,2) == 0){
            assert(arma::norm(mDirection_tmp,2) != 0);
            direction = mDirection_tmp;
        }
        assert(arma::norm(direction,2) != 0);

        direction = direction/arma::norm(direction,2);

        alpha(direction);
        // if no suitable direction, pic on at random
        // std::cout<< "max_alpha: " << max_alpha << " exp(-acos(0.5)): " << exp(-acos(0.5)) << std::endl;
        if(max_alpha < exp(-acos(0.5))){
            std::cout<< " (GMAPlanner)::pick at random <---------------- " << std::endl;
            pick_random_direction();
            direction = mDirection_tmp;
        }else{
            //            std::cout<< " (GMAPlanner)::compute conditionals " << std::endl;
            mDirection.zeros();
            for(unsigned int i = 0; i < index.size();i++){
                //mu = gmm_cond.getMu(index[i]);
                mu = mu/arma::norm(mu,2);
                assert(mu.is_finite());
                mDirection = mDirection + alphas[i] * mu;
            }


        }
    }

    assert(mDirection.is_finite());
    if(arma::norm(mDirection,2) != 0 ){
        mDirection_tmp = mDirection;
        mDirection_tmp = mDirection_tmp/arma::norm(mDirection_tmp,2);
    }

    direction = mDirection;
}

void GMAPlanner::get_ee_angular_velocity(arma::colvec3& ang_velocity){

}


void GMAPlanner::pick_random_direction(){
/*     dist_mus = std::discrete_distribution<int>(gmm_cond.Priors.begin(),gmm_cond.Priors.end());
    int k = dist_mus(generator);
    mDirection = gmm_cond.getMu(k);
    mDirection = mDirection/arma::norm(mDirection,2);*/
}

void GMAPlanner::alpha(const arma::vec3& direction){
    // std::cout<< "(GMAPlanner)::alpha" << std::endl;
    // std::cout<< " index.size(): " << index.size() << std::endl;
    alphas.resize(index.size());
    max_alpha = 0;
    sum_alpha = 0;
    unsigned int j = 0;

    for(unsigned int i = 0; i < index.size();i++){
        j = index[i];
    //    mu = gmm_cond.getMu(j);
        mu = mu/arma::norm(mu,2);
        assert(mu.is_finite());
        //alphas[i] = gmm_cond.Priors[j] * exp(-acos( arma::dot(mu,direction)));

        //    beta = -2/M_PI * log(0.001);
        //    alphas[i] = gmm_cond.Priors[j] * exp(-beta*std::fabs(arma::dot(mu,direction)));

        if(!arma::is_finite(alphas[i])){
            std::cout<< "GMAPlanner::alpha, is_finite: false" << std::endl;
            alphas[i] = 0.0;
        }
        assert(arma::is_finite(alphas[i]));


        if(alphas[i] > max_alpha){
            max_alpha = alphas[i];
        }
        sum_alpha += alphas[i];
    }

    for(unsigned int i = 0; i < alphas.size();i++){
        alphas[i] = alphas[i]/sum_alpha;
    }
    max_alpha = max_alpha/sum_alpha;

}

void GMAPlanner::getIndexs(){
    index.clear();
    //max_pi = 0;
   // pi_tmp = 0;
    max_index =0;
   /* for(unsigned int k = 0; k < gmm_cond.K;k++){
        //pi_tmp = gmm_cond.Priors[k];
        if(pi_tmp > 0.1){
            index.push_back(k);
            if(pi_tmp > max_pi){
                max_pi = pi_tmp;
                max_index = k;
            }
        }
    }*/
}



/**

  Simple Planner implementation

**/


SimplePlanner::SimplePlanner(){
    target.zeros();
}

void SimplePlanner::setTarget(const arma::vec3& mTarget){
    target = mTarget;
}

void SimplePlanner::setCurrentPos(const arma::vec& mCurrentPos){
    currPos = mCurrentPos;
}

void SimplePlanner::getDirection(arma::vec3 &direction){
    direction = target - currPos;
    direction = direction/arma::norm(direction,2);
}


/**

  Hyprid Planner implementation

  **/


void HybridPlanner::initialise(){
   // gmaPlanner.initConditional();
    bFirst=false;
}

void HybridPlanner::reset(){
    bFirst=false;
}

void HybridPlanner::setUncertainty(double uncertainty){
    this->uncertainty = uncertainty;
}

void HybridPlanner::get_ee_linear_velocity(arma::vec3& direction){
    if(bFirst || uncertainty < 0.062){
        simplePlanner.getDirection(direction);
        if(!bFirst){
            std::cout<< "--------> using simple controller" << std::endl;
            bFirst = true;
        }
    }else{
      //  gmaPlanner.get_ee_linear_velocity(direction);
    }
}

void HybridPlanner::get_ee_angular_velocity(arma::colvec3& ang_velocity){

}



