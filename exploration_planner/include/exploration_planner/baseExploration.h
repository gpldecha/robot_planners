/**
    Abstract Base class of Exploration objects. Exploration objects has three main pointers,
    respectively a pointer to the Environment (pWorldInterface), to a Controller (pController)
    and to a Belief (pBelief). These are the main aspects needed for an exploration algorithm.
  **/


#ifndef BASEEXPLORATION_H_
#define BASEEXPLORATION_H_



// STD

#include <string>
#include <vector>

// mycode

#include "robot_planners/basePlanner.h"
//#include "basebelief.h"


class BaseExploration{


public:

    BaseExploration();

    //BaseExploration(WorldInterface* const pWorldInterface);

    virtual ~BaseExploration();

    virtual int RespondToConsoleCommand(const std::string cmd, const std::vector<std::string> &args) = 0;

protected:

    //WorldInterface*  pWorldInterface;


};

#endif
