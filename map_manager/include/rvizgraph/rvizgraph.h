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
 * @file   rvizgraph.h
 * @author Simon Ehlers (simon.ehlers@stud.uni-hannover.de)
 * @date   09.08.2019
 *
 * @brief  ROS-Node to visualize the graph structure in Rviz
 */

#ifndef RVIZGRAPH_H
#define RVIZGRAPH_H

#include "ros/ros.h"
#include "ros/param.h"
#include "std_msgs/Bool.h"
#include "std_msgs/String.h"
#include "text_input/text_input.h"
#include "dbdriver/dbdriver.h"
#include "linkpoint/linkpoint.h"
#include "visualization_msgs/Marker.h"
#include "std_msgs/Int16.h"
#include "lpgraph/lpgraph.h"
#include "tf/transform_listener.h"

class RvizGraph
{
public:
    RvizGraph(ros::NodeHandle &node_handle);
private:
    // node handle
    ros::NodeHandle *node_;

    ros::Timer my_timer_;

    ros::Subscriber map_id_subscriber_;

    ros::Publisher local_marker_pub_;
    ros::Publisher global_marker_pub_;
    ros::Publisher edge_marker_pub_;


    int cur_map_index_;
    std::string lp_db_url_;
    std::string working_dir_ = "/home/ehlers/map_manager"; //default working directory for saving rtabmap databases
    visualization_msgs::Marker local_marker_;
    visualization_msgs::Marker global_marker_;
    visualization_msgs::Marker edge_marker_;


    //Objects
    DBDriver linkpoint_db_;
    TextInput text_io_;

    //######## TYPEDEF ################################################################################################
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

    //###### METHODS ##################################################################################################

    /**
     * Method to fill the link-point messages in the callback
     * @brief RvizGraph::fill_linkpoint_msgs
     * @param cur_map_id (int)
     */
    void fill_linkpoint_msgs();

    /**
     * Method to fill the edge messages in the callback
     * @brief RvizGraph::fill_edge_msgs
     * @param cur_map_id (int)
     */
    void fill_edge_msgs();


    //###### CALLBACKS #################################################################################################
    /**
     Subscriber callback to fill marker message
     * @brief RvizGraph::MapIDSubscriberCallback
     * @param msg
     */
    void MapIDSubscriberCallback(const std_msgs::Int16 &msg);

    /**
     * Timer callback for publishing visualisation to rviz
     * @brief RvizGraph::timerCallback
     * @param evt (const ros::TimerEvent)
     */
    void timerCallback(const ros::TimerEvent &evt);
};

#endif // RVIZGRAPH_H
