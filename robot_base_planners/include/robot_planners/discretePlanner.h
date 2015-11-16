#ifndef DISCRETEPLANNER_H_
#define DISCRETEPLANNER_H_

#include "basePlanner.h"

// KDTree

#include <nanoflann.hpp>

// Armadillo

#include <armadillo>

// STL

#include <string>

// Mycode

//#include "draw.h"


using namespace nanoflann;

namespace planners{


class DiscretePlanner : public Base_EE_Planner{


public:

    struct PointCloud
    {

        //std::vector<Vector3>  pts;
        arma::mat pts;

        //void setPoints(std::vector<Vector3> points){
        //	pts = points;
        //}
        void setPoints(arma::mat points){
            //std::cout<< "set" << std::endl;
            pts.resize(points.n_rows,points.n_cols);
            for(unsigned int i=0;i<points.n_rows;i++){
                pts.row(i) = points.row(i);
            }
        }


        arma::vec3 getPoint(size_t index){
         //   std::cout<< "getPoint index: " << index << std::endl;

            return pts.row(index).st();
        }

        // Must return the number of data points
        inline size_t kdtree_get_point_count() const { return pts.n_rows; }

        // Returns the distance between the vector "p1[0:size-1]" and the data point with index "idx_p2" stored in the class:
        inline double kdtree_distance(const double *p1, const size_t idx_p2,size_t size) const
        {
            //std::cout<< "kdtree_distance" << std::endl;
            //arma::vec3 pt1; pt1(0) = p1[0]; pt1(1) = p1[1]; pt1(2) = p1[2];
            //arma::vec3 pt2 = pts.row(idx_p2).st();

           // std::cout<< "idx_p2: " << idx_p2 << std::endl;
           // std::cout<< "pts: (" << pts.n_rows << " x " << pts.n_cols << ")" << std::endl;
            pts.row(idx_p2);

            return sqrt((p1[0] - pts(idx_p2,0))*(p1[0] - pts(idx_p2,0)) + (p1[1] - pts(idx_p2,1))*(p1[1] - pts(idx_p2,1)) + (p1[2] - pts(idx_p2,2))*(p1[2] - pts(idx_p2,2)));

            //return	arma::norm(pt1 - pt2,2);
            //Vector3 pt1(p1[0],p1[1],p1[2]);
            //Vector3 pt2 = pts[idx_p2];
            //return (pt1 - pt2).Norm();
        }

        // Returns the dim'th component of the idx'th point in the class:
        // Since this is inlined and the "dim" argument is typically an immediate value, the
        //  "if/else's" are actually solved at compile time.
        inline double kdtree_get_pt(const size_t idx, int dim) const
        {
           // std::cout<< "kdtree_get_pt" << std::endl;
           // std::cout<< " dim: " << dim << std::endl;
           // std::cout<< " idx: " << idx << std::endl;
            if (dim==0){
                return pts(idx,0);
            }else if (dim==1){
                return pts(idx,1);
            }else{
                return pts(idx,2);
            }
        }

        template <class BBOX>
        bool kdtree_get_bbox(BBOX &bb) const { return false; }


    };

    // construct a kd-tree index:
    typedef KDTreeSingleIndexAdaptor<L2_Simple_Adaptor<double, PointCloud > , 	PointCloud, 3 /* dim */ > my_kd_tree_t;



    DiscretePlanner();

    ~DiscretePlanner();

    void load(const std::string& filename);

    void setPosition(const arma::vec3& position);

    virtual void get_ee_linear_velocity(arma::vec3& direction);

    void drawGrid();

private:

    arma::mat policy;
    arma::mat tmp;
    arma::mat S;
    arma::mat A;

    unsigned int state_index;


    PointCloud cloud;
    my_kd_tree_t *index;
    nanoflann::SearchParams params;
    double query_pt[3];
    std::vector<std::pair<size_t,double> > ret_matches;
    double bandwidth;
    //rtku::Draw mDraw;
    //rtku::rgb color;



};

}


#endif
