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
 * @file   lpgraph.h
 * @author Simon Ehlers (simon.ehlers@stud.uni-hannover.de)
 * @date   26.07.2019
 *
 * @brief  Class to create a graph based on the boost graph library and plan the shortest path by the dijkstra algorithm.
 */

#ifndef LPGRAPH_H
#define LPGRAPH_H

#include <stdio.h>
#include <string>
#include <iostream>
#include <vector>
#include <math.h>

#include <ros/ros.h>
#include <tf/transform_listener.h>

#include <boost/config.hpp>
#include <boost/graph/graph_traits.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/dijkstra_shortest_paths.hpp>
#include <boost/property_map/property_map.hpp>
#include <boost/graph/graph_utility.hpp>
#include <linkpoint/linkpoint.h>
#include <dbdriver/dbdriver.h>

/**
 * @brief The LPGraph class: Class to create a graph based on the boost graph library and plan the shortest path by the dijkstra algorithm.
 */
class LPGraph
{
public:


    // ######## TYPEDEF ####################
    /**
     * @brief Create a struct to hold properties for each vertex (vertex = linkpoint)
     */
    typedef struct vertex_properties
    {
      int id;
      std::string label;

      double pos_x_0;
      double pos_y_0;
      double theta_0;
      int map_id_0;

      double pos_x_1;
      double pos_y_1;
      double theta_1;
      int map_id_1;

    } vertex_properties_t;

    /**
     * @brief Create a struct to hold properties for each edge
     */
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

    //parameters
    std::string lp_db_url_;
    std::map <std::string, vertex_descriptor_t> vertices_;
    std::map <std::string,  std::pair<edge_descriptor_t, bool>> edges_;

    // ######### METHODS #################

    /**
     * Builds a graph based on the existing linkpoints and maps in the database.
     * Linkpoints are the vertices which are connected by edges (transforms between the linkpoints).
     * Vertices are connected if the share the same map.
     *
     * @brief LPGraph::build_graph_from_db
     * @param url (std::string)
     * @param ignore_map_id_start (int) in this map is the starting point of planning, so the vertices should not be connected to each other on this particular map
     * @param ignore_map_id_target (int) in this map is the end point of planning, so the vertices should not be connected to each other on this particular map
     * @return bool
     */
    bool build_graph_from_db(std::string url, int ignore_map_id_start, int ignore_map_id_target);

    /**
     * Takes the linkpoints from the database and creates vertices.
     * @brief LPGraph::create_vertices_of_all_linkpoints
     * @return bool
     */
    bool create_vertices_of_all_linkpoints();

    /**
     * Creates the edges for each map by connecting all vertices that share the same map.
     *
     * @brief LPGraph::create_edges_for_each_map
     * @param ignore_map_id_start (int) in this map is the starting point of planning, so the vertices should not be connected to each other on this particular map
     * @param ignore_map_id_target (int) in this map is the end point of planning, so the vertices should not be connected to each other on this particular map
     * @return
     */
    bool create_edges_for_each_map(int ignore_map_id_start, int ignore_map_id_target);

    /**
     * computes the distance between two given Linkpoints on the map they share.
     *
     * @brief LPGraph::get_distance_int
     * @param lp (LinkPoint)
     * @param lp_edge_candidate (LinkPoint)
     * @return distance (int) distance in int as weight for the dijkstra algorithm
     */
    int get_distance_int(LinkPoint lp, LinkPoint lp_edge_candidate);

    /**
     * Dijkstra shortest path algorithm for planning the shortest path over linkpoints over multiple maps for navigation or global robot position tracking purposes.
     *
     * @brief LPGraph::dijkstra_shortest_path
     * @param start_lp_id (int) starting Linkpoint
     * @param target_lp_id (int) ending link point
     * @return shortest_path_ids (std::vector<int>) Linkpoint (vertices) ids which to use to get on the shortest path to the target linkpoint.
     */
    std::vector<int> dijkstra_shortest_path(int start_lp_id, int target_lp_id);

    /**
     * Method for adding start and target vertex to graph as well as edges for easy navigation.
     * @brief LPGraph::add_vertex_to_graph
     * @param lp (LinkPoint)
     * @param map_id (int)
     * @param label (std::string)
     * @return bool
     */
    bool add_vertex_to_graph(LinkPoint lp, int map_id);

    /**
     * Method that uses dijkstra shortest path algorithm to compute the linkpoints to the target.
     * @brief LPGraph::get_topological_path_to_target
     * @param url (std::string)
     * @param map_id_start (int)
     * @param target_pose (geometry_msgs::Pose) Pose on the target map
     * @param map_id_target (int)
     * @return shortest_path_ids (std::vector<int>)
     */
    std::vector<int> get_topological_path_to_target(std::string url, int map_id_start, geometry_msgs::Pose target_pose,int map_id_target);

    /**
     * Computes the topological path from world origin (origin map 0) to the closest linkpoint to the current robot position.
     * @brief LPGraph::get_topological_path_from_world_orig_to_robot
     * @param url (std::string)
     * @param map_id_robot (int)
     * @return
     */
    std::vector<int> get_topological_path_from_world_orig_to_robot(int map_id_robot);

    /**
     * Receives the current robot pose on the local map from the tf tree by using a transform listener.
     * @brief LPGraph::get_current_pose
     * @return tf (tf::Transform)
     */
    tf::Transform get_current_pose();

    //Create a graph object
    graph_t graph_;
    DBDriver linkpoint_db_;
};

#endif // LPGRAPH_H
