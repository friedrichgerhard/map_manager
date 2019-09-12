/* *****************************************************************
 *
 * map_manager
 *
 * Copyright (c) 2019
 * Institute of Mechatronic Systems,
 * Leibniz Universitaet Hannover.
 * (BSD License)
 * All rights reserved.
 *
 * http://www.imes.uni-hannover.de
 *
 * This software is distributed WITHOUT ANY WARRANTY; without
 * even the implied warranty of MERCHANTABILITY or FITNESS FOR
 * A PARTICULAR PURPOSE.
 *
 * For further information see http://www.linfo.org/bsdlicense.html
 *
 ******************************************************************/

/**
 * @file   graphoptimizer.h
 * @author Simon Ehlers (simon.ehlers@stud.uni-hannover.de)
 * @date   26.07.2019
 *
 * @brief  Optimization of the topological graph after loop closure using g2o. Based on https://github.com/RainerKuemmerle/g2o/blob/master/g2o/examples/tutorial_slam2d/tutorial_slam2d.cpp
 */

#ifndef GRAPHOPTIMIZER_H
#define GRAPHOPTIMIZER_H


#include <iostream>
#include <cmath>
#include <string>

#include "simulator.h"

#include "vertex_se2.h"
#include "vertex_point_xy.h"
#include "edge_se2.h"
#include "edge_se2_pointxy.h"
#include "types_tutorial_slam2d.h"


#include "g2o/core/sparse_optimizer.h"
#include "g2o/core/block_solver.h"
#include "g2o/core/factory.h"
#include "g2o/core/optimization_algorithm_factory.h"
#include "g2o/core/optimization_algorithm_gauss_newton.h"
#include "g2o/solvers/csparse/linear_solver_csparse.h"

#include <dbdriver/dbdriver.h>
#include <linkpoint/linkpoint.h>

#include "g2o/core/optimization_algorithm_levenberg.h"
#include "g2o/core/robust_kernel_impl.h"

#include "g2o/types/sba/types_sba.h"
#include "g2o/solvers/eigen/linear_solver_eigen.h"
#include "g2o/config.h"
#include "g2o/types/slam2d/types_slam2d.h"
#include "g2o/types/slam3d/types_slam3d.h"
#include "g2o/solvers/csparse/linear_solver_csparse.h"
#include "g2o/solvers/pcg/linear_solver_pcg.h"
#include "g2o/solvers/cholmod/linear_solver_cholmod.h"

#include <boost/config.hpp>
#include <boost/graph/graph_traits.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/dijkstra_shortest_paths.hpp>
#include <boost/property_map/property_map.hpp>
#include <boost/graph/graph_utility.hpp>
#include <linkpoint/linkpoint.h>
#include <dbdriver/dbdriver.h>
#include <lpgraph/lpgraph.h>


using namespace std;
using namespace g2o;
using namespace g2o::tutorial;

/**
 * @brief The GraphOptimizer class: Optimization of the topological graph after loop closure using g2o. Based on https://github.com/RainerKuemmerle/g2o/blob/master/g2o/examples/tutorial_slam2d/tutorial_slam2d.cpp
 */
class GraphOptimizer
{
public:

    // ######### METHODS #################
    /**
     * Method for optimizing the topological graph after a new loop closure was found.
     * Optimized by g2o the position of the link points is corrected based on ther covariance and the error which occured.
     * The Optimization uses the old linkpoint candidate as landmark and the other linkpoints as robot positions.
     * The error in global position between the new linkpoint and the old linkpoint candidate (which are supposed to have the same global position) is used for optimization.
     * We use the boost graph framework to build the g2o graph for convenience.
     * Based on https://github.com/RainerKuemmerle/g2o/blob/master/g2o/examples/tutorial_slam2d/tutorial_slam2d.cpp
     *
     * @brief GraphOptimizer::g2o_optimize_graph
     * @param url (std:string) linkpoint.db path
     * @param lp_candidate (LinkPoint) The linkpoint candidate the loop closure was with.
     * @param new_lp (LinkPoint) the new Linkpoint created after the loop closure was found.
     * @param tf_to_lp_candidate (tf::Transform) the ground truth Transform between both positions based on the pointcloud from ICP.
     * @param var_x (double) initial variance in x
     * @param var_y (double) initial varaince in y
     * @param var_theta (double) initial variance in theta
     */
    void  g2o_optimize_graph(std::string url, LinkPoint lp_candidate, LinkPoint new_lp, tf::Transform tf_to_lp_candidate, double var_x, double var_y, double var_theta);

    /**
     * Finds out which map positions (0 or 1) should be compared and computes the translation between two linkpoints.
     * @brief GraphOptimizer::get_edge_translation
     * @param lp (LinkPoint)
     * @param lp_edge_candidate (LinkPoint)
     * @return
     */
    Eigen::Vector2d get_edge_translation(LinkPoint lp, LinkPoint lp_edge_candidate);

    /**
     * Computes the rotation between two linkpoints.
     * @brief GraphOptimizer::get_edge_rotation
     * @param lp (LinkPoint)
     * @param lp_edge_candidate (LinkPoint)
     * @return
     */
    Eigen::Rotation2D<double> get_edge_rotation(LinkPoint lp, LinkPoint lp_edge_candidate);


    //########## TYPEDEF #########
    typedef BlockSolver< BlockSolverTraits<-1, -1> >  SlamBlockSolver;
    typedef LinearSolverCSparse<SlamBlockSolver::PoseMatrixType> SlamLinearSolver;

    //#### for boost graph ######
    // Create a struct to hold properties for each vertex (vertex = linkpoint)  
    typedef struct vertex_properties
    {
      int id;
      std::string label;

    } vertex_properties_t;

    // Create a struct to hold properties for each edge
    typedef struct edge_properties
    {
      std::string label;
      int   weight;
    } edge_properties_t;

    // Define the type of the graph
    typedef boost::adjacency_list<boost::setS, boost::vecS, boost::undirectedS, vertex_properties_t, edge_properties_t> graph_t;
    typedef graph_t::vertex_descriptor vertex_descriptor_t;
    typedef graph_t::edge_descriptor   edge_descriptor_t;
    typedef boost::property_map<graph_t, boost::vertex_index_t>::type index_map_t;
    typedef boost::iterator_property_map<vertex_descriptor_t*, index_map_t*, vertex_descriptor_t, vertex_descriptor_t&> predecessor_map_t;

    //objects
    DBDriver linkpoint_db_;
};

#endif // GRAPHOPTIMIZER_H
