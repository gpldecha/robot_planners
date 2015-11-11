#include "singleExploration.h"
#include <boost/lexical_cast.hpp>

double str2num(const std::string& num){
    return boost::lexical_cast<double>(num);
}

void SingleExploration::setDefaultValues(){
    pSingleBelief=NULL;
    mDistribution=NULL;
    pWrapObject=NULL;
    mMode = EXPLOR::IDL;
    bFirst = true;
    bFinished = false;
    bIsOnTable=true;
    bIsOvertable=true;
    bSetStartPoint=false;
    mSearchType = RobotSearch;
   // mSearchPlanner.setControlMode(CTRLMODES::SIMPLE);

}

SingleExploration::SingleExploration():BaseExploration(){
    setDefaultValues();

}

SingleExploration::SingleExploration(WorldInterface* const pWorldInterface,BaseBelief* const pBelief, WrapObject *pWrapObject):
    BaseExploration(pWorldInterface)
{
    setDefaultValues();



   try{
        pSingleBelief = dynamic_cast<SingleBelief*>(pBelief);
    }
    catch(std::bad_cast& bc)
    {
       std::cerr << "(singleExploration.cpp) bad_cast caught: " << bc.what() << '\n';
    }

    this->pWrapObject = pWrapObject;

    mostLikelyState = arma::zeros<arma::vec>(3);
    mObservableVariables = arma::zeros<arma::vec>(4);

    mOrigin = arma::zeros<arma::vec>(3);
//    mOrigin(0) = -1.0;
//    mOrigin(1) = 0.0;
//    mOrigin(2) = 1.0;


   mWidth(0) = 0.6;
   mWidth(1) = 1.1;
   mWidth(2) = 0.2;

 //  Use to get data for Costal Navigation
  /*  mWidth(0) = 0.6;
    mWidth(1) = 1.0;
    mWidth(2) = 0.2;
*/



    mOrigin(0) = (0.6 + 0.5/2) - mWidth(0)/2;
    mOrigin(1) = 0.0;
    mOrigin(2) = 0.4;



    mTruePosition.zeros();

    // Add Console Command
    pWorldInterface->AddConsoleCommand("search");
    pWorldInterface->AddConsoleCommand("setTruePosition");
    pWorldInterface->AddConsoleCommand("setBelSize");


}


void SingleExploration::setUpTransfomrations(){
    pWorldObject table = pWorldInterface->GetWorld()->Find("TableSearch");


    HTable.SetTranslation(table->GetReferenceFrame().GetOrigin());

    double angle = DEG2RAD(0);
    Rot.Zero();
    Rot(0,0) = cos(angle); Rot(0,1) = -sin(angle);
    Rot(1,0) = sin(angle); Rot(1,1) = cos(angle);
    Rot(2,2) = 1;
    for(unsigned int i = 0; i < 3; i++)
        for(unsigned int j = 0; j < 3;j++)
            rot(i,j) = Rot(i,j);

    HTable(3,3) = 1;
    HTable.SetOrientation(Rot);
    HTable.InverseTransformation(invHTable);
    mTablePosRF.zeros();

    Vector3 tmp;
    tmp = table->GetReferenceFrame().GetOrigin();
    mTablePos(0) = tmp(0); mTablePos(1) = tmp(1);mTablePos(2) = tmp(2);
    Vector3 scale =mUtilities.getScale(table);
    table_sizes[0] = scale.x();
    table_sizes[1] = scale.y();
    table_sizes[2] = scale.z();
    std::cout<< " === Table Scale === " << std::endl;
    std::cout<< " w: " << table_sizes[0] << std::endl;
    std::cout<< " l: " << table_sizes[1] << std::endl;
    std::cout<< " h: " << table_sizes[2] << std::endl;

}

void SingleExploration::initBelief(){

    setUpTransfomrations();

    arma::mat orient;
    orient.eye(3,3);
    std::cout<< "just before mDistribution" << std::endl;
    if(mDistribution != NULL)
        delete mDistribution;
    mDistribution = new Uniform(mOrigin,orient,mWidth(0),mWidth(1),mWidth(2));
    assert(mDistribution != NULL);
    pSingleBelief->init(mDistribution);

    // At the first time step, our belief state is the mostLikelyState
    pSingleBelief->getMostLikelyState(mostLikelyState);
    mBeliefState = mostLikelyState;

    smooth.reset();
    exponential_mavrg.reset();
    uncertainty = 1;
    bFinished=false;
    bSetStartPoint=false;

}

void SingleExploration::setInitPositions(arma::vec3& truePosition,arma::vec3& beliefPosition){
        mBeliefState = beliefPosition;
        mTruePosition = truePosition;
        bSetStartPoint=true;
}


void SingleExploration:: setBeliefOrigin(const arma::vec& origin){
    mOrigin = origin;
}


void SingleExploration::setBeliefWidth(const arma::vec3& scale){
    mWidth = scale;
}

void SingleExploration::setLikelihoodFunction(Likelihood *Likelihoodf){
    pSingleBelief->setLikelihoodFunction(Likelihoodf);
    pLikelihoodf = Likelihoodf;
}

void SingleExploration::setPriorContact(PriorContact *fPriorContact) {
    pfPriorContact = fPriorContact;
}

void SingleExploration::prettyPrint(){
    std::cout<< "=== Bool ===" << std::endl;
    if(bIsOvertable){
        std::cout<<" bIsOvertable: true" << std::endl;
    }else{
        std::cout<< " bIsOvertable: false" << std::endl;
    }
    if(bIsOnTable){
        std::cout<<" bIsOnTable: true" << std::endl;
    }else{
        std::cout<< " bIsOnTable: false" << std::endl;
    }


}

void SingleExploration::ComputeDirection(){


    switch(mMode){
    case EXPLOR::IDL:
    {
        break;
    }
     case EXPLOR::SEARCH:
    {

        mBeliefStateTableRF.zeros();
        mTruePositionTableRF.zeros();
        toMatlabFrameOfReference(mBeliefState,mBeliefStateTableRF);
        toMatlabFrameOfReference(mTruePosition,mTruePositionTableRF);

       // std::cout<< "=== check On Table Conditions ===" << std::endl;
       // mTablePos.print("mTablePos");
       // mTruePosition.print("mTruePosition");
        bIsOvertable = isOverTable(mTruePosition,mTablePos,table_sizes[0],table_sizes[1],table_sizes[2]);
        bIsOnTable   = isOnTable(mTruePosition,mTablePos,table_sizes[0],table_sizes[1],table_sizes[2]);

        uncertainty = pSingleBelief->getUncertainty();

        if(uncertainty < 0.001){
            uncertainty = 0.001;
        }


        //std::cout<< "mSearchPlanner.ctrl_mode: " << mSearchPlanner.ctrl_mode << std::endl;

        if(mSearchPlanner.ctrl_mode == (CTRLMODES::GMM)  ||  mSearchPlanner.ctrl_mode == (CTRLMODES::HYBRID)){

            ///////////////////////////////////
            //  Check if belief is off table //
            //      And Get Direction        //
            ///////////////////////////////////

           if(bIsOvertable){
                mSearchPlanner.isInAir = true;
                mSearchPlanner.GetDirection(mDirection,mBeliefStateTableRF,1);
                mDirection = mDirection/arma::norm(mDirection,2);

            }else if(bIsOnTable){
                mSearchPlanner.isInAir = false;
                mSearchPlanner.GetDirection(mDirection,mBeliefStateTableRF,uncertainty);
                mDirection(2) = 0;
                uncertainty = smooth.exponential(uncertainty,0.1);
                mDirection = mDirection/arma::norm(mDirection,2);
                exponential_mavrg.get(mDirection,0.009);
                mDirection = mDirection/arma::norm(mDirection,2);
            }else{
                mSearchPlanner.isInAir = true;
                mSearchPlanner.GetDirection(mDirection,mBeliefStateTableRF,1);
                mDirection = mDirection/arma::norm(mDirection,2);
                exponential_mavrg.get(mDirection,0.02);
                mDirection = mDirection/arma::norm(mDirection,2);
            }

            directionWorldFrameOfReference(mDirection);
        }else if (mSearchPlanner.ctrl_mode == CTRLMODES::SIMPLE){
            assert(mDirection.is_finite());
            assert(mBeliefState.is_finite());
            mSearchPlanner.GetDirection(mDirection,mBeliefState);
            assert(arma::sum(mDirection) != 0);
            mDirection = mDirection/arma::norm(mDirection,2);
        }else if (mSearchPlanner.ctrl_mode == CTRLMODES::DISCRETE){
            assert(mDirection.is_finite());
            assert(mBeliefState.is_finite());
            mBeliefState.print("mBeliefState");
            arma::vec3 tmp = mBeliefState;
                    tmp(2) = tmp(2) - 0.0025;
            mSearchPlanner.GetDirection(mDirection,tmp);
            exponential_mavrg.get(mDirection,0.1);
            /*if(bIsOnTable){
                mDirection(2) = 0;
            }*/
            if(arma::sum(mDirection) != 0){
               mDirection = mDirection/arma::norm(mDirection,2);
            }else{
                mDirection(0)=1;
            }


        }else{
            std::cout<< "(SingleExploration)::ComputeDirection, no such ctrl mode " <<  std::endl;
        }

        if(bFinished){
            mMode = EXPLOR::IDL;
        }

        break;
    }

    }


}

void SingleExploration::GetNextPosition(arma::vec3& nextPosition, double speed, double wait_threashold){

    if(bFirst){
        nextPosition = mTruePosition;
        bFirst = false;
    }

    if( arma::norm(nextPosition - mTruePosition,2) < wait_threashold){
        double DT = 0.01;
        mVelocity = mDirection * DT * speed;
        nextPosition = nextPosition + mVelocity;
    }
}

void SingleExploration::UpdateBeliefState(const arma::vec3& velocity){
    tmp(0) = mBeliefState(0);tmp(1) = mBeliefState(1); tmp(2) = mBeliefState(2);
    double ll = (*pLikelihoodf)(tmp);

    if(ll < 0.9){
        pSingleBelief->getMostLikelyState(mBeliefState);
    }else{
        mBeliefState = mBeliefState + velocity;
    }
}

void SingleExploration::hasReachedGoal(const arma::vec3& true_position, double threashold){
    double distance =arma::norm(true_position - mGoalPosition,2);
    if(distance < threashold ){
        bFinished = true;
        std::cout<< "==== Search Finished === " << std::endl;
    }

}

std::string SingleExploration::convertvec3ToStr(const arma::vec3& v){
        std::ostringstream ss1,ss2,ss3;
       ss1 << std::fixed << std::setprecision(1);
       ss2 << std::fixed << std::setprecision(1);
       ss3 << std::fixed << std::setprecision(1);
       ss1 << v(0);
       ss2 << v(1);
       ss3 << v(2);

       return "(" + ss1.str() + "," + ss2.str() + "," + ss3.str() + ")";
}

std::string SingleExploration::ctrl_mode_2str(CTRLMODES::controlModes mode){
    switch (mode){
    case CTRLMODES::GMM:
    {
        return "GMM";
    }
    case CTRLMODES::HYBRID:
    {
        return "HYBRID";
    }
    case CTRLMODES::SIMPLE:
    {
        return "SIMPLE";
    }
    case CTRLMODES::NONE:
    {
        return "NONE";
    }
    case CTRLMODES::DISCRETE:
    {
    return "DISCRETE";
    }

    }
}

int SingleExploration::RespondToConsoleCommand(const std::string cmd, const std::vector<std::string> &args){
    int nbArgs = args.size();

    if(cmd == "search"){
        if(nbArgs > 0){
            string arg0 = args[0];
            if(arg0 == "init"){
                initBelief();
                if(pSingleBelief->isInit()){
                    pWorldInterface->GetConsole()->Print("Belief init: true");
                }else{
                    pWorldInterface->GetConsole()->Print("Belief init: false");
                }
            }else if(arg0 == "type"){
                if(nbArgs == 2){
                    set_search_type(args[1]);
                }else{
                    pWorldInterface->GetConsole()->Print("availabe: [gmm, simple, hybrid, discrete]");
                }
            }else if(arg0 == "start"){
                if(pSingleBelief->isInit()){
                    mMode = EXPLOR::SEARCH;
                    if(!bSetStartPoint){
                        if(pSingleBelief != NULL){
                            pSingleBelief->sample(mTruePosition);
                            bSetStartPoint=true;
                        }else{
                            pWorldInterface->GetConsole()->Print("pSingleBlief is either NULL or not initialised");
                        }
                    }
                    mSearchPlanner.init();
                    bFirst=true;
                    pWorldInterface->GetConsole()->Print("=== Search has now started! ===");
                    pWorldInterface->GetConsole()->Print(" search type: " + ctrl_mode_2str(mSearchPlanner.ctrl_mode));
                    pWorldInterface->GetConsole()->Print(" mTruePosition: " + convertvec3ToStr(mTruePosition));
                    pWorldInterface->GetConsole()->Print(" mBeliefState: " + convertvec3ToStr(mBeliefState));

                }else{
                    pWorldInterface->GetConsole()->Print("You have to type 'search init' first!");
                }
            }else if(arg0 == "stop"){
                mMode = EXPLOR::IDL;
            }else if(arg0 == "reset"){
                mMode = EXPLOR::IDL;
                initBelief();

            }else if(arg0 == "setStartPoint"){
                std::cout<< "   choose the starting point   " << std::endl;
                if(pSingleBelief != NULL){
                    pSingleBelief->sample(mTruePosition);
                    bSetStartPoint=true;
                }else{
                    pWorldInterface->GetConsole()->Print("pSingleBelief is NULL");
                }

            }
        }else{
             pWorldInterface->GetConsole()->Print("Need to specify a command for search! {init,type,start}");
        }
    }else if(cmd == "setTruePosition"){
        if(nbArgs == 3){
            mTruePosition(0)    = boost::lexical_cast<double>(args[0]);
            mTruePosition(1)    = boost::lexical_cast<double>(args[1]);
            mTruePosition(2)    = boost::lexical_cast<double>(args[2]);
            pWorldInterface->GetConsole()->Print("True Position set");
        }else{
            pWorldInterface->GetConsole()->Print("(x,y,z) coordinates need to be specified !");
        }
     }else if(cmd == "setBelSize"){
        if (nbArgs == 3){

            mWidth(0) = boost::lexical_cast<double>(args[0]);
            mWidth(1) = boost::lexical_cast<double>(args[1]);
            mWidth(2) = boost::lexical_cast<double>(args[2]);
            mMode = EXPLOR::IDL;
            mWidth.print("  mWidth  ");
            initBelief();
        }else{

            pWorldInterface->GetConsole()->Print("setBelSize [size_x] [size_y] [size_z] ");
        }
    }


}

void SingleExploration::toMatlabFrameOfReference(const arma::vec3& stateOriginal,arma::vec3& stateRF){
   // stateOriginal.print("stateOriginal");
   // HTable.Print();

    Ttmp(0) = stateOriginal(0);
    Ttmp(1) = stateOriginal(1);
    Ttmp(2) = stateOriginal(2);
    Ttmp(3) = 1;
    Ttmp = invHTable * Ttmp;
    // z o--- y
    //   |
    //   x
    stateRF(0) = Ttmp(0);
    stateRF(1) = Ttmp(1);
    stateRF(2) = Ttmp(2);
    //   x
    //   |
    // z o--- y
   // stateRF(0) = -stateRF(0);
  //  stateRF.print("stateRF");

}

void SingleExploration::directionWorldFrameOfReference(arma::vec3& direction){
    tmp(0) = direction(0);
    tmp(1) = direction(1);
    tmp(2) = direction(2);
    tmp = HTable.GetOrientation() * tmp;
    direction(0) = tmp(0);
    direction(1) = tmp(1);
    direction(2) = tmp(2);
}

bool SingleExploration::isOnTable(const arma::vec3& true_position,const arma::vec3& table_origin,double width,double length,double height){

    double b = 0.01;
    double z_height = table_origin(2) + height/2;
    bool in_x = true_position(0) > table_origin(0) - width/2 - b && true_position(0) < table_origin(0) + width/2 + b;
    bool in_y = true_position(1) > table_origin(1) - length/2 - b && true_position(1) < table_origin(1) + length/2 + b;
    bool in_z = true_position(2) <= (z_height + 0.04);

 //   std::cout<< "  (isOnTable)    in_x: " << in_x << " in_y: " << in_y << " in_z: " << in_z << std::endl;

    if(in_x && in_y && in_z){
        return true;
        std::cout<< "isOnTable" << std::endl;
    }else{
        return false;
    }
}

bool SingleExploration::isOverTable(const arma::vec3& true_position,const arma::vec3& table_origin,double width,double length,double height){
    double b = 0.1;
    double z_height = table_origin(2) + height/2;

    bool in_x = true_position(0) > table_origin(0) - width/2 - b && true_position(0) < table_origin(0) + width/2 + b;
    bool in_y = true_position(1) > table_origin(1) - length/2 - b && true_position(1) < table_origin(1) + length/2 + b;
    bool in_z = true_position(2) > (z_height+ 0.04);

   // std::cout<< "  (isOverTables) in_x: " << in_x << " in_y: " << in_y << " in_z: " << in_z << std::endl;

    if(in_x && in_y && in_z){
        return true;
    }else{
        return false;
    }
}

bool SingleExploration::isInOrUnderTable(const arma::vec3& position,const arma::vec3& table_origin,double width,double length,double height){


    bool in_x = position(0) > table_origin(0) - width/2  && position(0) < table_origin(0) + width/2;
    bool in_y = position(1) > table_origin(1) - length/2 && position(1) < table_origin(1) + length/2;
    bool in_z = position(2) < table_origin(2) + height/2;

    if(in_x && in_y && in_z){
        return true;
    }else{
        return false;
    }


}

void SingleExploration::set_search_type(const std::string& search_type){
    if(search_type == "gmm"){
        mSearchPlanner.setControlMode(CTRLMODES::GMM);
         pWorldInterface->GetConsole()->Print("search type set to: gmm");
    }else if(search_type == "simple"){
        mSearchPlanner.setControlMode(CTRLMODES::SIMPLE);
        pWorldInterface->GetConsole()->Print("search type set to: simple");
    }else if (search_type == "hybrid"){
        mSearchPlanner.setControlMode(CTRLMODES::HYBRID);
        pWorldInterface->GetConsole()->Print("search type set to: hybrid");
    }else if (search_type == "discrete"){
        mSearchPlanner.setControlMode(CTRLMODES::DISCRETE);
        pWorldInterface->GetConsole()->Print("search type set to: discrete");
    }else{
        pWorldInterface->GetConsole()->Print("no such search type: " +search_type);
    }
}

void SingleExploration::getMoseLikelyState(arma::vec3& state){
    pSingleBelief->getMostLikelyState(mostLikelyState);
    state = mostLikelyState;
}

EXPLOR::Modes SingleExploration::getMode(){
    return mMode;
}

void SingleExploration::setMode(EXPLOR::Modes mode){
    mMode = mode;
}


void SingleExploration::setTarget(const arma::vec3& target){
    mSearchPlanner.setTarget(target);
    mGoalPosition = target;
}

void SingleExploration::draw(){
    mDraw.draw_point(mTruePosition,0.025,rtku::rgb(1,1,1));
    mDraw.draw_point(mBeliefState,0.025,rtku::rgb(0,0,1));

    if(mMode == EXPLOR::SEARCH){
        mSearchPlanner.draw(mBeliefState,&rot);

        if(mSearchPlanner.ctrl_mode == CTRLMODES::SIMPLE){
            mDraw.draw_vector(mBeliefState,mDirection,rtku::rgb(1,1,0));
            mDraw.draw_point(mBeliefState,0.025,rtku::rgb(0,0,1));
        }

        mDraw.draw_vector(mTruePosition,mDirection,rtku::rgb(1,1,0));

    }

  /*  if (mSearchPlanner.ctrl_mode == CTRLMODES::DISCRETE){

        mSearchPlanner.mDiscretePlanner.drawGrid();

    }*/
}
