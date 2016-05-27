#include "robot_planners/gmmPlanner.h"
#include <limits>

using namespace planners;

#include <limits>

/**

  BaseGMM Class Controller implementation

  **/


BaseGMM_EE_Planner::BaseGMM_EE_Planner(){
    bFirst = true;
}

BaseGMM_EE_Planner::BaseGMM_EE_Planner(const std::string& path_parameters){
    gmm.load(path_parameters);
    in  = gmm.in;
    out = gmm.out;
    bFirst = true;
}

BaseGMM_EE_Planner::BaseGMM_EE_Planner(const std::string& path_parameters, const std::vector<std::size_t>& in,const std::vector<std::size_t>& out)
    : in(in),out(out)
{
    gmm.load(path_parameters);
    bFirst = true;
}

void BaseGMM_EE_Planner::condition(const arma::colvec& x){
    if(bFirst){
        std::cout<< "first Condition" << std::endl;
        std::cout<< "in.size(): " << in.size() << std::endl;
        std::cout<< "out.size(): " << out.size() << std::endl;
        gmm_c.condition(gmm,in,out);
        bFirst=false;
    }

    // std::cout<< "actual condition" << std::endl;
    gmm_c.condition(x,gmm);
}

const std::vector<std::size_t>& BaseGMM_EE_Planner::get_in() const{
    return in;
}

const std::vector<std::size_t>& BaseGMM_EE_Planner::get_out() const{
    return out;
}

/**

  GMR Planner implementation

  **/

GMR_EE_Planner::GMR_EE_Planner():
    BaseGMM_EE_Planner(){}

GMR_EE_Planner::GMR_EE_Planner(const std::string& path_parameters):
    BaseGMM_EE_Planner(path_parameters){}

GMR_EE_Planner::GMR_EE_Planner(const std::string& path_parameters, const std::vector<std::size_t>& in,const std::vector<std::size_t>& out):
    BaseGMM_EE_Planner(path_parameters,in,out){}

void GMR_EE_Planner::gmr(const arma::colvec& x_in){
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


GMAPlanner::GMAPlanner():
    BaseGMM_EE_Planner(){
    init();
}

GMAPlanner::GMAPlanner(const std::string& path_parameters):
    BaseGMM_EE_Planner(path_parameters){
    init();
}

GMAPlanner::GMAPlanner(const std::string& path_parameters, const std::vector<std::size_t>& in,const std::vector<std::size_t>& out):
    BaseGMM_EE_Planner(path_parameters,in,out){
    init();
}

void GMAPlanner::init(){
    alphas.resize(gmm.K);
    bFirst = true;
}

void GMAPlanner::get_alphas(const arma::colvec& w,const std::vector<arma::colvec>& V, const arma::colvec3& vtmp){
    //
    //  input -------------------------------------
    //      o   w : (K x 1)     :   set of weights
    //      o   V : (K x D)     :   set of K normalised velocities of dimension D
    //      o  vtmp : (1 x D)   :   previous applied velocity
    //  output ------------------------------------
    //      o  v : (1 x D)      : output control velocity normalised
    //

    assert(V.size() > 0);
    assert(V[0].n_elem == 3);

    //std::cout<< "get_alphas" << std::endl;
    //std::cout<< "is_finite(w):  " << w.is_finite() << std::endl;
    //std::cout<< "sum(w):        " << arma::sum(w) << std::endl;

    for(std::size_t k = 0; k < gmm.K;k++){
        if( std::fabs(arma::dot(V[k],vtmp)) > 1.0 ){
             alphas(k) = 0.0;
        }else{
            alphas(k)   = w(k) * exp(-1 * acos( arma::dot(V[k],vtmp)  ));
        }
    }

    alphas      = alphas / (arma::sum(alphas) + std::numeric_limits<double>::min());
    max_alpha   = arma::max(alphas);

}

void GMAPlanner::gmc(const arma::colvec& x_in,const arma::vec3& vtmp){
    condition(x_in);
    this->vtmp = vtmp;
    const std::vector<arma::colvec>& Means = gmm_c.gmm_c.get_means();
    for(std::size_t k = 0; k < gmm_c.gmm_c.K;k++){
        mu_k = Means[k];
        mu_k = arma::normalise(mu_k);
        gmm_c.gmm_c.set_mu(k,mu_k);
    }

    get_alphas(gmm_c.gmm_c.get_weigts(),gmm_c.gmm_c.get_means(),vtmp);
}


void GMAPlanner::get_ee_linear_velocity(arma::vec3& direction){
    if(bFirst){
        pick_random_direction(mDirection);
        bFirst = false;
    }else{
     //  std::cout<< "max_alpha: " << max_alpha << std::endl;
     //   if(max_alpha < exp(-acos(0.5))){
     //       pick_random_direction(mDirection);
     //   }else{
            mDirection.zeros();
            const std::vector<arma::colvec>& Means = gmm_c.gmm_c.get_means();
            for(std::size_t k = 0; k < gmm.K;k++){
                mDirection = mDirection + alphas(k) * Means[k];
            }
       // }
    }

    mDirection = arma::normalise(mDirection);


    if(!mDirection.is_finite()){
        std::cout<< "mDirection is not finite [GMAPlanner::get_ee_linear_velocity]!" << std::endl;
        mDirection.print("mDirection");
        mDirection.zeros();
    }

    direction  = mDirection;

}

void GMAPlanner::get_ee_angular_velocity(arma::colvec3& ang_velocity){

}


void GMAPlanner::pick_random_direction(arma::vec3& direction){
    std::cout<< "pick_random_direction" << std::endl;
    std::vector<double> w(gmm_c.gmm_c.get_weigts().n_elem);
    for(std::size_t i = 0; i < w.size();i++){
        w[i] = gmm_c.gmm_c.get_weigts()[i];
    }
    dist_mus = std::discrete_distribution<int>(w.begin(),w.end());
    int k = dist_mus(generator);
    direction = gmm_c.gmm_c.get_means()[k];
}

void GMAPlanner::print() const{
    std::cout<< "=== GMA Planner ===" << std::endl;
    if(alphas.size() > 0){
        std::cout<< "alpha = [";
        for(std::size_t i = 0; i < alphas.size()-1;i++){
            std::cout<< alphas[i] << " ";
        }
        std::cout<< alphas[alphas.size()-1] << "]";
    }else{
        std::cout<< "alpha = []" << std::endl;
    }
    std::cout<< "max_alpha: " << max_alpha << std::endl;
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



