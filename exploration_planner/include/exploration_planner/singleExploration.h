#ifndef SINGLEEXPLORATION_H_
#define SINGLEEXPLORATION_H_

// Armadillo (math library)

#include <armadillo>

// mycode

#include "baseExploration.h"
#include "singlebelief.h"
#include "distributions.h"
#include "searchPlanner.h"
#include "wrapobject.h"
#include "utilities.h"
#include "PointSensor.h"

#include "draw.h"

namespace EXPLOR{
    enum Modes {IDL,SEARCH,GRAVCOMP};
}


class SingleExploration : public BaseExploration{

    typedef boost::function < double (const MathLib::Vector3&)> Likelihood;
    typedef boost::function < std::pair<bool,ptsensor::feature> ()> PriorContact;


    enum SEARCH_TYPE {RobotSearch,PointSearch};

public:

    SingleExploration();

    SingleExploration(WorldInterface *const pWorldInterface,BaseBelief*  const pBelief,WrapObject *pWrapObject);

    void initBelief();

    void setLikelihoodFunction(Likelihood *Likelihoodf);

    void setPriorContact(PriorContact *fPriorContact);

    void setInitPositions(arma::vec3& truePosition,arma::vec3& beliefPosition);

    void ComputeDirection();

    void GetNextPosition(arma::vec3& nextPosition,double speed,double wait_threashold);

    void UpdateBeliefState(const arma::vec3& velocity);

    int RespondToConsoleCommand(const std::string cmd, const std::vector<std::string> &args);

    void setTarget(const arma::vec3& target);

    void getMoseLikelyState(arma::vec3& state);

    void setBeliefOrigin(const arma::vec& origin);

    void setBeliefWidth(const arma::vec3& scale);

    void draw();

    void hasReachedGoal(const arma::vec3& true_position,double threashold);

    EXPLOR::Modes getMode();

    void setMode(EXPLOR::Modes mode);

private:


private:

    void setDefaultValues();

    void setUpTransfomrations();

    void set_search_type(const string &search_type);

    void toMatlabFrameOfReference(const arma::vec3& stateOriginal,arma::vec3& stateRF);

    void directionWorldFrameOfReference(arma::vec3& direction);


    bool isOnTable(const arma::vec3& position,const arma::vec3& table_origin,double width,double length,double height);

    bool isOverTable(const arma::vec3& position,const arma::vec3& table_origin,double width,double length,double height);

    void collision_detection(arma::vec3& direction,arma::vec3& true_position,WrapObject* pWrapObject);

    bool isInOrUnderTable(const arma::vec3& position,const arma::vec3& table_origin,double width,double length,double height);

    void prettyPrint();

    std::string convertvec3ToStr(const arma::vec3& v);

    std::string ctrl_mode_2str(CTRLMODES::controlModes mode);

private:

    SingleBelief* pSingleBelief;
    Distribution* mDistribution;
    SearchPlanner mSearchPlanner;
    Likelihood *pLikelihoodf;
    PriorContact *pfPriorContact;
    WrapObject *pWrapObject;
    Utilities mUtilities;

    Smooth smooth;
    ExponentialMovingAverage exponential_mavrg;

    /// Parameters needed for search planner
    arma::vec mostLikelyState;
    arma::vec3 mGoalPosition;

    double uncertainty;

    /// Target Position which the robot has to follow
    arma::vec3 mTargetEndEffector;

    EXPLOR::Modes mMode;

    arma::vec mOrigin;
    arma::vec3 mWidth;
    arma::vec mObservableVariables;

    rtku::Draw mDraw;
    Vector3 tmp;

    MathLib::Mat4 HTable,invHTable;
    MathLib::TVector<4> Ttmp;
    MathLib::Mat3 Rot;
    arma::mat33 rot;
    arma::vec3 mBeliefStateTableRF;
    arma::vec3 mTruePositionTableRF;
    arma::vec3 mTablePosRF;
    arma::vec3 mTablePos;
    double table_sizes[3];

    bool bIsOvertable;
    bool bIsOnTable;
    bool bIsUnderTable;
    bool bSetStartPoint;

    bool bFirst,bFirst2;
public:
     arma::vec3 mDirection;
     arma::vec3 mVelocity;
     arma::vec3 mBeliefState;
     arma::vec3 mTruePosition;
     bool bFinished;

     SEARCH_TYPE mSearchType;


};





#endif
