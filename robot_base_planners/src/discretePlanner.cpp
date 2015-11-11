#include "robot_planners/discretePlanner.h"

using namespace planners;


DiscretePlanner::DiscretePlanner(){
    state_index=0;
    bandwidth=0.4;
    tmp.resize(1,6);
    //color = rtku::rgb(1,1,1);
    index=NULL;
    std::cout<< "DiscretePlanner::Constructor" << std::endl;
}

DiscretePlanner::~DiscretePlanner(){

    std::cout<< "DiscretePlanner::Distructor" << std::endl;
    if(index != NULL){
        delete index;
        index = NULL;
    }

}


void DiscretePlanner::load(const std::string& filename){
    std::cout<< "DiscretePlanner::load" << std::endl;
    if(policy.load(filename)){
        std::cout << "=== Loading discrete Policy === " << std::endl;
        std::cout<< "filename: " + filename << std::endl;
        std::cout << "  number of states: " << policy.n_rows << std::endl;

        S = policy.cols(0,2);
        A = policy.cols(3,5);
        for(unsigned int r=0; r < A.n_rows;r++){

            if(arma::sum(A.row(r)) == 0){
                std::cout<< "r: " << r << std::endl;

            }


        }

        std::cout<< "S: (" << S.n_rows << " x " << S.n_cols << ")" << std::endl;
        std::cout<< "A: (" << A.n_rows << " x " << A.n_cols << ")" << std::endl;


        cloud.pts.resize(S.n_rows,3);
        cloud.setPoints(S);


        index = new my_kd_tree_t(3, cloud, KDTreeSingleIndexAdaptorParams(20) );
        index->buildIndex();

        A.row(30912).print("A(30912)");

        query_pt[0] = 0.592046;  query_pt[1] = 0.143781; query_pt[2] =0.373109;
        ret_matches.clear();
        index->radiusSearch(&query_pt[0],bandwidth, ret_matches, params);


    }else{

        std::cout<< "error loading: " + filename << "  (DiscretePlanner::load)" << std::endl;

    }

}

void DiscretePlanner::setPosition(const arma::vec3& position){

    //std::cout<< "DiscretePlanner::setPosition" << std::endl;

    query_pt[0] = position(0) - 0.6;  query_pt[1] = position(1); query_pt[2] = position(2) + 0.025;

    //std::cout<< "query_pt( " << query_pt[0] << "," << query_pt[1] << "," << query_pt[2] << ")" << std::endl;


    ret_matches.clear();

   // std::cout<< "before radiusSeach 1" << std::endl;

    index->radiusSearch(&query_pt[0],bandwidth, ret_matches, params);
   // std::cout<< "index: " << index << std::endl;

   // std::cout<< "after radiusSeach" << std::endl;

    assert(ret_matches.size() != 0);
    state_index = ret_matches[0].first;
    std::cout<< "state_index: " << state_index << std::endl;
   // cloud.getPoint(state_index).print("closest point");
}

void DiscretePlanner::getDirection(arma::vec3& direction){
    // DiscretePlanner::setPosition should be called before this method
  //  std::cout<< "DiscretePlanner::getDirection" << std::endl;
   // std::cout<< "state_index: " << state_index << std::endl;
    tmp = policy.row(state_index);
  //  tmp.print("tmp");
    direction(0) = tmp(3);
    direction(1) = tmp(4);
    direction(2) = tmp(5);
   // direction.print("direction");

}


void DiscretePlanner::drawGrid(){
    //mDraw.draw_points(S.st(),0.005,color,5);
}
