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
 * @file   rvizgraph.cpp
 * @author Simon Ehlers (simon.ehlers@stud.uni-hannover.de)
 * @date   09.08.2019
 *
 * @brief  ROS-Node to visualize the graph structure in Rviz
 */


#include "rvizgraph/rvizgraph.h"

//########## CONSTRUCTOR ###############################################################################################
RvizGraph::RvizGraph(ros::NodeHandle &node_handle):
    node_(&node_handle)
{
    // === PARAMETERS ===


    // === SUBSCRIBERS ===
    map_id_subscriber_ = node_->subscribe("/map_manager/cur_map_id", 1, &RvizGraph::MapIDSubscriberCallback, this);

    // === PUBLISHERS ===
    local_marker_pub_ = node_->advertise<visualization_msgs::Marker>("map_manager/local_linkpoints_rviz", 1);
    global_marker_pub_ = node_->advertise<visualization_msgs::Marker>("map_manager/global_linkpoints_rviz", 1);
    edge_marker_pub_ = node_->advertise<visualization_msgs::Marker>("map_manager/edges_rviz", 1);

    // === SERVICE CLIENTS ===

    // === TIMER ===
    my_timer_ = node_->createTimer(ros::Duration(0.2), &RvizGraph::timerCallback, this);

    cur_map_index_ = 0;
    
    working_dir_ = text_io_.get_working_dir(working_dir_);
    lp_db_url_ = working_dir_ + "/linkpoints.db";
}


//######## CALLBACK SUBSCRIBER #########################################################################################
void RvizGraph::MapIDSubscriberCallback(const std_msgs::Int16 &msg){

    ROS_INFO("In subscriber callback");

    cur_map_index_ = msg.data;
    RvizGraph::fill_linkpoint_msgs();
    RvizGraph::fill_edge_msgs();
}

//######## CALLBACK TIMER ##############################################################################################
void RvizGraph::timerCallback(const ros::TimerEvent &evt){

    //set timestamp
    local_marker_.header.stamp = ros::Time::now();
    global_marker_.header.stamp = ros::Time::now();

    //publish markers
    local_marker_pub_.publish(local_marker_);
    global_marker_pub_.publish(global_marker_);
    edge_marker_pub_.publish(edge_marker_);
}

//############# METHOD: FILL LINKPOINT MESSAGES #######################################################################
void RvizGraph::fill_linkpoint_msgs(){

    visualization_msgs::Marker local_marker;
    local_marker.header.frame_id = "map";
    local_marker.header.stamp = ros::Time();
    local_marker.type = visualization_msgs::Marker::SPHERE_LIST;
    local_marker.action = visualization_msgs::Marker::ADD;
    local_marker.scale.x = 0.5;
    local_marker.scale.y = 0.5;
    local_marker.scale.z = 0.1;
    local_marker.color.a = 1.0;
    local_marker.color.r = 0.0;
    local_marker.color.g = 1.0;
    local_marker.color.b = 0.0;
    //local_marker.lifetime = ros::Duration(0.25);

    visualization_msgs::Marker global_marker;
    global_marker.header.frame_id = "world";
    global_marker.header.stamp = ros::Time();
    global_marker.type = visualization_msgs::Marker::SPHERE_LIST;
    global_marker.action = visualization_msgs::Marker::ADD;
    global_marker.scale.x = 0.5;
    global_marker.scale.y = 0.5;
    global_marker.scale.z = 0.1;
    global_marker.color.a = 1.0;
    global_marker.color.r = 0.0;
    global_marker.color.g = 1.0;
    global_marker.color.b = 0.0;
    //global_marker.lifetime = ros::Duration(0.25);

    //iterate through all linkpoints on map and push it back to marker_
    std::vector<int> lp_ids = linkpoint_db_.get_linkpoint_ids(lp_db_url_);
    for (int i = 0; i < lp_ids.size(); i++) {
        LinkPoint lp = linkpoint_db_.get_linkpoint(lp_db_url_, lp_ids.at(i));

        int index;
        if(lp.get_map_id(0) == cur_map_index_){
            index = 0;
        }
        else if (lp.get_map_id(1) == cur_map_index_) {
            index = 1;
        }
        else {
            index = 2; //for world pose
        }

        if(index == 0 || index == 1){ //Linkpoint is on current local map. Use local map position.
            geometry_msgs::Point point;

            point.x = lp.get_pos_x(index);
            point.y = lp.get_pos_y(index);
            point.z = 0;

            local_marker.points.push_back(point);

            //set color to blue
            std_msgs::ColorRGBA c;
            c.r = 0.0;
            c.g = 0.0;
            c.b = 1.0;
            c.a = 1.0; //alpha

            local_marker.colors.push_back(c);
        }
        else if (index == 2) { //Linkpoint is not on current local map. Use global position and store in different marker message
            geometry_msgs::Point point;

            point.x = lp.get_pos_x(index);
            point.y = lp.get_pos_y(index);
            point.z = 0;

            global_marker.points.push_back(point);

            //set color to red
            std_msgs::ColorRGBA c;
            c.r = 1.0;
            c.g = 0.2;
            c.b = 0.0;
            c.a = 1.0; //alpha

            global_marker.colors.push_back(c);
        }
    }

    local_marker_ = local_marker; //save to member variable for timer callback
    global_marker_ = global_marker;
    ROS_INFO("Added all points to markers Message");
}

//############# METHOD: FILL EDGE MESSAGES #######################################################################
void RvizGraph::fill_edge_msgs(){

    //add edge with rviz marker
    visualization_msgs::Marker edges_marker;
    edges_marker.header.frame_id = "world";
    edges_marker.header.stamp = ros::Time();
    edges_marker.type = visualization_msgs::Marker::LINE_LIST;
    edges_marker.action = visualization_msgs::Marker::ADD;
    edges_marker.scale.x = 0.2;
    edges_marker.color.a = 1.0;
    edges_marker.color.r = 0.2;
    edges_marker.color.g = 1.0;
    edges_marker.color.b = 0.0;
    //edges_marker.lifetime = ros::Duration(0.25);

    LPGraph lp_graph_object;
    //Create a graph object
    graph_t graph;
    std::map <std::string, vertex_descriptor_t> vertices;
    std::map <std::string,  std::pair<edge_descriptor_t, bool>> edges;

    //create for every linkpoint a vertex
    std::vector<int> lp_ids = linkpoint_db_.get_linkpoint_ids(lp_db_url_);
    for(int i = 0; i < lp_ids.size(); i++){
        int id = lp_ids.at(i);
        LinkPoint lp = linkpoint_db_.get_linkpoint(lp_db_url_, id);

        //boost
        std::string vertex_id = "v_" + std::to_string(id);
        vertices[vertex_id] = boost::add_vertex(graph);
        //set properties for vertex
        graph[vertices[vertex_id]].id = id;
        graph[vertices[vertex_id]].label = vertex_id;

        ROS_INFO("Created vertex with id = %i\n", id);
    }


    //first loop to extract each linkpoint
    for(int i = 0; i < lp_ids.size(); i++){
        int id = lp_ids.at(i);
        LinkPoint lp = linkpoint_db_.get_linkpoint(lp_db_url_, id);
        int map_id_0 = lp.get_map_id(0);
        int map_id_1 = lp.get_map_id(1);

        //second loop to add each possible connection to linkpoint in first loop
        for(int k = 0; k < lp_ids.size(); k++){
            if(i != k){
                int id_2 = lp_ids.at(k);
                LinkPoint lp_edge_candidate = linkpoint_db_.get_linkpoint(lp_db_url_, id_2);

                //check if linkpoints are connected by sharing the same map
                //check if edge has to be ignored because start or target are on it
                if((map_id_0 == lp_edge_candidate.get_map_id(0))
                || (map_id_0 == lp_edge_candidate.get_map_id(1))
                || (map_id_1 == lp_edge_candidate.get_map_id(1))
                || (map_id_1 == lp_edge_candidate.get_map_id(0)))
                {
                    std::string edge_name = "e_" + std::to_string(id) + "_" + std::to_string(id_2);

                    std::string vertex_name = "v_" + std::to_string(id);
                    std::string vertex_name_1 = "v_" + std::to_string(id_2);

                    //try to add edge, if it does not exist yet it will return true, otherwise false
                    std::pair<edge_descriptor_t, bool> new_edge;
                    new_edge = boost::add_edge(vertices[vertex_name], vertices[vertex_name_1], graph);

                    //if bool is true and edge is new and added to graph
                    if(new_edge.second){
                        edges[edge_name] = new_edge;

                        //set properties for new edge
                        graph[edges[edge_name].first].weight = 1; //lp_graph_object.get_distance_int(lp, lp_edge_candidate);
                        graph[edges[edge_name].first].label = edge_name;
                        std::cout << "Added new edge with label " << edge_name << "\n";

                        for (int j=0; j <= 1; j++) {

                            LinkPoint cur_lp;
                            if(j==0){
                                cur_lp = lp;
                            }
                            else if (j == 1) {
                                cur_lp = lp_edge_candidate;
                            }

                            int index;
                            if(cur_lp.get_map_id(0) == cur_map_index_){
                                index = 0;
                            }
                            else if (cur_lp.get_map_id(1) == cur_map_index_) {
                                index = 1;
                            }
                            else {
                                index = 2; //for world pose
                            }


                            if(index == 0 || index == 1){ //Linkpoint is on current local map. Use local map position.
                                geometry_msgs::PointStamped point;
                                point.header.frame_id = "map";
                                point.header.stamp = ros::Time();

                                geometry_msgs::PointStamped world_point;

                                point.point.x = cur_lp.get_pos_x(index);
                                point.point.y = cur_lp.get_pos_y(index);
                                point.point.z = 0;

                                tf::TransformListener listener;
                                listener.waitForTransform("/map", "/world", ros::Time(0), ros::Duration(4.0));

                                try{
                                    listener.transformPoint("/world", point, world_point);
                                }
                                catch(tf::TransformException& ex){
                                    ROS_ERROR("Received an exception trying to transform a point : %s", ex.what());
                                }

                                edges_marker.points.push_back(world_point.point);

                            }
                            else if (index == 2) { //Linkpoint is not on current local map. Use global position and store in different marker message
                                geometry_msgs::Point point;

                                point.x = cur_lp.get_pos_x(index);
                                point.y = cur_lp.get_pos_y(index);
                                point.z = 0;

                                edges_marker.points.push_back(point);
                            }
                        }
                    }
                }
            }
        }
    }

    edge_marker_ = edges_marker;
}

//########## MAIN ######################################################################################################
int main(int argc, char** argv)
{
    ros::init(argc, argv, "rviz_graph");

    ros::NodeHandle node_handle;
    RvizGraph rviz_graph(node_handle);

    ROS_INFO("Node is spinning...");
    ros::spin();

    return 0;
}
