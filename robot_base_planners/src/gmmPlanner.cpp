#include "robot_planners/gmmPlanner.h"
#include <limits>

using namespace planners;


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

GMR_EE_Planner::GMR_EE_Planner():
    BaseGMM_EE_Planner(){}

GMR_EE_Planner::GMR_EE_Planner(const std::string& path_parameters):
    BaseGMM_EE_Planner(path_parameters){}

GMR_EE_Planner::GMR_EE_Planner(const std::string& path_parameters, const std::vector<std::size_t>& in,const std::vector<std::size_t>& out):
    BaseGMM_EE_Planner(path_parameters,in,out){}

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


GMAPlanner::GMAPlanner():
    BaseGMM_EE_Planner(){}

GMAPlanner::GMAPlanner(const std::string& path_parameters):
    BaseGMM_EE_Planner(path_parameters){}

GMAPlanner::GMAPlanner(const std::string& path_parameters, const std::vector<std::size_t>& in,const std::vector<std::size_t>& out):
    BaseGMM_EE_Planner(path_parameters,in,out){}

void GMAPlanner::gmc(const arma::colvec& x_in,const arma::vec3& vtmp){
    condition(x_in);
    get_indexs();
    get_alpha(vtmp);
}


void GMAPlanner::get_ee_linear_velocity(arma::vec3& direction){
    if(bFirst){
        pick_random_direction(mDirection);
        bFirst = false;
    }else{

        if(max_alpha < exp(-acos(0.5))){
            pick_random_direction(mDirection);
        }else{
            std::cout<< " (GMAPlanner)::compute conditionals " << std::endl;
            mDirection.zeros();
            for(std::size_t i = 0; i < index.size();i++){
                mu = gmm_c.gmm_c.get_means()[index[i]];
                //mu = arma::normalise(mu);
                //assert(mu.is_finite());
                mDirection = mDirection + alphas[i] * mu;
            }
        }
    }
   if(!mDirection.is_finite()){
        std::cout<< "mDirection is not finite [GMAPlanner::get_ee_linear_velocity]!" << std::endl;
        mDirection.print("mDirection");
        mDirection.zeros();
   }

   /* if(arma::norm(mDirection,2) != 0 ){
        mDirection_tmp = mDirection;
        mDirection_tmp = arma::normalise(mDirection_tmp);
    }*/

    mDirection = arma::normalise(mDirection);
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

void GMAPlanner::get_alpha(const arma::vec3& vtmp){
    // std::cout<< "(GMAPlanner)::alpha" << std::endl;
    // std::cout<< " index.size(): " << index.size() << std::endl;
    alphas.resize(index.size());
    max_alpha = 0;
    sum_alpha = 0;
    std::size_t j = 0;

    for(std::size_t i = 0; i < index.size();i++){
        j = index[i];
        mu = gmm_c.gmm_c.get_means()[j]; //Mu(j);
        mu = mu/arma::norm(mu,2);
        assert(mu.is_finite());

        alphas[i] = gmm_c.gmm_c.get_weigts()[j] * exp(-acos( arma::dot(mu,vtmp)));

        //    beta = -2/M_PI * log(0.001);
        //    alphas[i] = gmm_cond.Priors[j] * exp(-beta*std::fabs(arma::dot(mu,direction)));

        if(!arma::is_finite(alphas[i])){
            std::cout<< "GMAPlanner::alpha, is_finite: false" << std::endl;
            alphas[i] = 0.0;
        }

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

void GMAPlanner::get_indexs(double threashold){
    index.clear();
    max_pi      = 0;
    max_index   = 0;
    double w_j;
    for(std::size_t k = 0; k < gmm_c.gmm_c.K;k++){
        w_j = gmm_c.gmm_c.get_weigts()[k];
        if(w_j > threashold){
            index.push_back(k);
            if(w_j > max_pi){
                max_pi      = w_j;
                max_index   = k;
            }
        }
    }
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

    if(index.size() > 0){
        std::cout<< "index = [";
        for(std::size_t i = 0; i < index.size()-1;i++){
            std::cout<< index[i] << " ";
        }
        std::cout<< index[index.size()-1] << "]";
    }else{
        std::cout<< "index = []" << std::endl;
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



