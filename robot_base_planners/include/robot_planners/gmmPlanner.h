#ifndef GMMCONTROLLER_H_
#define GMMCONTROLLER_H_

#include "basePlanner.h"
#include "statistics/distributions/distributions.h"
#include "statistics/distributions/gmm.h"


#include <string>
#include <armadillo>
#include <random>

namespace planners{


class BaseGMM_EE_Planner: public Base_EE_Planner{

public:

    BaseGMM_EE_Planner();

    BaseGMM_EE_Planner(const std::string& path_parameters);

    BaseGMM_EE_Planner(const std::string& path_parameters, const std::vector<std::size_t>& in,const std::vector<std::size_t>& out);

    const std::vector<std::size_t>& get_in() const;

    const std::vector<std::size_t>& get_out() const;

protected:

    void condition(const arma::colvec &x);

public:

    stats::GMM  gmm;
    stats::cGMM gmm_c;

    // indexs for conditioning
    std::vector<std::size_t> in;
    std::vector<std::size_t> out;

    bool bFirst;
};

class GMR_EE_Planner: public BaseGMM_EE_Planner {

public:

    GMR_EE_Planner();

    GMR_EE_Planner(const std::string& path_parameters);

    GMR_EE_Planner(const std::string& path_parameters,const std::vector<std::size_t>& in,const std::vector<std::size_t>& out);

    void gmr(const arma::colvec& x_in);

    void get_ee_linear_velocity(arma::colvec3 &direction);

    void get_ee_angular_velocity(arma::colvec3& ang_velocity);

private:

    arma::colvec3 velocity_ee; /// linear velocity of the end-effector

};

class GMAPlanner:     public BaseGMM_EE_Planner{


public:

    GMAPlanner();

    GMAPlanner(const std::string& path_parameters);

    GMAPlanner(const std::string& path_parameters, const std::vector<std::size_t>& in,const std::vector<std::size_t>& out);

    void gmc(const arma::colvec &x_in, const arma::vec3 &vtmp);

    void get_ee_linear_velocity(arma::vec3 &direction);

    void get_ee_angular_velocity(arma::colvec3& ang_velocity);

    void print() const;


private:

    void init();

    void get_alphas(const arma::colvec& w,const std::vector<arma::colvec>& V, const arma::colvec3& vtmp);

    void pick_random_direction(arma::vec3 &direction);

private:

    arma::colvec3 mu_k;

    bool bFirst;
    arma::vec3 mDirection;
    arma::vec3 vtmp;


    arma::colvec alphas;

    double      max_alpha;
    std::discrete_distribution<int> dist_mus;
    std::default_random_engine generator;

};

class SimplePlanner  {

public:

    SimplePlanner();

    void setTarget(const arma::vec3& mTarget);

    void setCurrentPos(const arma::vec& mCurrentPos);

    void getDirection(arma::vec3 &direction);



private:


    arma::vec3 target,currPos;


};

class HybridPlanner: public Base_EE_Planner{

public:

    void initialise();

    virtual void get_ee_linear_velocity(arma::vec3& direction);

    virtual void get_ee_angular_velocity(arma::colvec3& ang_velocity);

    void setUncertainty(double uncertainty);

    void reset();

private:

    double uncertainty;
    bool bFirst;

public:
    //GMAPlanner gmaPlanner;
    SimplePlanner simplePlanner;



};

}

#endif
