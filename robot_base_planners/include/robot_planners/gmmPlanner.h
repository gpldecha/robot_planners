#ifndef GMMCONTROLLER_H_
#define GMMCONTROLLER_H_

#include <kuka_action_server/action_server.h>
#include <kuka_action_server/base_ee_action.h>

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

    BaseGMM_EE_Planner(const std::vector<std::size_t>& in,const std::vector<std::size_t>& out);

    void load(std::string path_parameters);

    virtual void drawConditional(const arma::vec3 &position,arma::mat33 *const pRot = NULL );

protected:

    void condition(arma::colvec& x);

protected:

    stats::GMM  gmm;
    stats::cGMM gmm_c;

    // indexs for conditioning
    std::vector<std::size_t> in;
    std::vector<std::size_t> out;

    bool bFirst;
};

class GMR_EE_Planner:     public BaseGMM_EE_Planner {

public:

    GMR_EE_Planner();

    GMR_EE_Planner(const std::vector<std::size_t>& in,const std::vector<std::size_t>& out);

    void gmr(arma::colvec& x_in);

    void get_ee_linear_velocity(arma::colvec3 &direction);

    void get_ee_angular_velocity(arma::colvec3& ang_velocity);

private:

    arma::colvec3 velocity_ee; /// linear velocity of the end-effector

};

class GMAPlanner:     public BaseGMM_EE_Planner{


public:

    GMAPlanner();

    GMAPlanner(std::string path_parameters);

    virtual void initConditional();

    void get_ee_linear_velocity(arma::vec3 &direction);

    void get_ee_angular_velocity(arma::colvec3& ang_velocity);


private:

    void pick_random_direction();

    void alpha(const arma::vec3& direction);

    void getIndexs();

    void isOverTable();

private:


    arma::vec3 mu;


    bool bFirst;
    arma::vec3 mDirection;
    arma::vec3 mDirection_tmp;

    std::vector<double> alphas;
    std::vector<unsigned int> index;

    double max_alpha;
    double beta;
    double sum_alpha;
    unsigned int max_index;

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
