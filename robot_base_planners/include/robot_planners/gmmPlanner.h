#ifndef GMMCONTROLLER_H_
#define GMMCONTROLLER_H_

#include <kuka_action_server/action_server.h>
#include <kuka_action_server/base_ee_action.h>

#include "basePlanner.h"
#include "statistics/distributions/distributions.h"
#include "statistics/distributions/gmm.h"
//#include "colormap.h"
//#include "utilities.h"

#include <string>
#include <armadillo>
#include <random>

namespace planners{


class BaseGMMPlanner: public BasePlanner{

public:

    BaseGMMPlanner();

    virtual void initConditional() = 0;

    void load(std::string path_parameters);

    void condition(arma::vec& x);

    virtual void drawConditional(const arma::vec3 &position,arma::mat33 *const pRot = NULL );



protected:

    GMM gmm, gmm_cond;
    // indexs for conditioning
    std::vector<unsigned int> in;
    std::vector<unsigned int> out;

    // variables for drawing
  //  ColorMap colormap;
    double max_pi;
    arma::vec3 mu_tmp;
    double pi_tmp;
    double scale;
    unsigned char rgb[3];

    //MathLib::Mat4 HTable;
   //MathLib::TVector<4> Ttmp;

};

class GMRPlanner:     public BaseGMMPlanner {

public:

    virtual void initConditional();

    void getDirection(arma::vec3 &direction);



};

class GMAPlanner:     public BaseGMMPlanner{


public:

    GMAPlanner();

    GMAPlanner(std::string path_parameters);

    virtual void initConditional();

    void getDirection(arma::vec3 &direction);


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

class HybridPlanner: public BasePlanner{

public:

    void initialise();

    virtual void getDirection(arma::vec3& direction);

    void setUncertainty(double uncertainty);

    void reset();

private:

    double uncertainty;
    bool bFirst;

public:
    GMAPlanner gmaPlanner;
    SimplePlanner simplePlanner;



};

}

#endif
