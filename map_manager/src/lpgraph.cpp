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
 * @file   lpgraph.cpp
 * @author Simon Ehlers (simon.ehlers@stud.uni-hannover.de)
 * @date   26.07.2019
 *
 * @brief  Class to create a graph based on the boost graph library and plan the shortest path by the dijkstra algorithm.
 */

#include "lpgraph/lpgraph.h"


bool LPGraph::build_graph_from_db(std::string url, int ignore_map_id_start, int ignore_map_id_target){

    lp_db_url_ = url;

    //create vertices
    LPGraph::create_vertices_of_all_linkpoints();

    //create edges
    LPGraph::create_edges_for_each_map(ignore_map_id_start, ignore_map_id_target);

    // Print out some useful information
    std::cout << "Graph:" << std::endl;
    boost::print_graph(graph_, boost::get(&vertex_properties_t::label,graph_));
    std::cout << "num_verts: " << boost::num_vertices(graph_) << std::endl;
    std::cout << "num_edges: " << boost::num_edges(graph_) << std::endl;

    return true;
}


bool LPGraph::create_vertices_of_all_linkpoints(){

    //create for every linkpoint a vertex
    int num_lp = linkpoint_db_.get_number_of_rows(lp_db_url_, "LINKPOINTS");
    for(int id = 0; id < num_lp; id++){
        LinkPoint lp = linkpoint_db_.get_linkpoint(lp_db_url_, id);
        std::string vertex_id = "v_" + std::to_string(id);

        vertices_[vertex_id] = boost::add_vertex(graph_);

        //set properties for vertex
        graph_[vertices_[vertex_id]].id = id;
        graph_[vertices_[vertex_id]].label = vertex_id;

        graph_[vertices_[vertex_id]].pos_x_0 = lp.get_pos_x(0);
        graph_[vertices_[vertex_id]].pos_y_0 = lp.get_pos_y(0);
        graph_[vertices_[vertex_id]].theta_0 = lp.get_theta(0);
        graph_[vertices_[vertex_id]].map_id_0 = lp.get_map_id(0);

        graph_[vertices_[vertex_id]].pos_x_1 = lp.get_pos_x(1);
        graph_[vertices_[vertex_id]].pos_y_1 = lp.get_pos_y(1);
        graph_[vertices_[vertex_id]].theta_1 = lp.get_theta(1);
        graph_[vertices_[vertex_id]].map_id_1 = lp.get_map_id(1);

        ROS_INFO("Created vertex with id = %i\n", id);
    }
    return true;
}


bool LPGraph::add_vertex_to_graph(LinkPoint lp, int map_id){

    //add vertex
    int id = lp.get_id(); //id = 100 for start and id = 101 for target
    std::string vertex_id = "v_" + std::to_string(id); //label is either start or target

    vertices_[vertex_id] = boost::add_vertex(graph_);

    //set properties for vertex
    graph_[vertices_[vertex_id]].id = id;
    graph_[vertices_[vertex_id]].label = vertex_id;

    graph_[vertices_[vertex_id]].pos_x_0 = lp.get_pos_x(0);
    graph_[vertices_[vertex_id]].pos_y_0 = lp.get_pos_y(0);
    graph_[vertices_[vertex_id]].theta_0 = lp.get_theta(0);
    graph_[vertices_[vertex_id]].map_id_0 = lp.get_map_id(0);

    ROS_INFO("Created vertex with id = %i\n", id);

    //add edges to adjacent vertices
    int num_lp = linkpoint_db_.get_number_of_rows(lp_db_url_, "LINKPOINTS");
    ROS_INFO("Num linkpoints = %i", num_lp);

    for(int id_edge_candidate = 0; id_edge_candidate < num_lp; id_edge_candidate++){
        LinkPoint lp_edge_candidate = linkpoint_db_.get_linkpoint(lp_db_url_, id_edge_candidate);
        if(lp_edge_candidate.get_map_id(0) == map_id
           || lp_edge_candidate.get_map_id(1) == map_id)
        {
            std::string edge_name = "e_" + std::to_string(id) + "_" + std::to_string(id_edge_candidate);

            std::string vertex_name = "v_" + std::to_string(id);
            std::string vertex_name_1 = "v_" + std::to_string(id_edge_candidate);

            //try to add edge, if it does not exist yet it will return true, otherwise false
            std::pair<edge_descriptor_t, bool> new_edge;
            new_edge = boost::add_edge(vertices_[vertex_name], vertices_[vertex_name_1], graph_);

            //if bool is true and edge is new and added to graph
            if(new_edge.second){
                edges_[edge_name] = new_edge;

                //set properties for new edge
                graph_[edges_[edge_name].first].weight = LPGraph::get_distance_int(lp, lp_edge_candidate);
                graph_[edges_[edge_name].first].label = edge_name;
                std::cout << "Added new edge with label " << edge_name << "\n";
            }
            else{
                std::cout << "Edge with label " << edge_name << " already exists... not added!\n";
            }
        }
    }

    // Print out some useful information
    std::cout << "Graph:" << std::endl;
    boost::print_graph(graph_, boost::get(&vertex_properties_t::label,graph_));
    std::cout << "num_verts: " << boost::num_vertices(graph_) << std::endl;
    std::cout << "num_edges: " << boost::num_edges(graph_) << std::endl;

    return true;
}


bool LPGraph::create_edges_for_each_map(int ignore_map_id_start, int ignore_map_id_target){
    int num_lp = linkpoint_db_.get_number_of_rows(lp_db_url_, "LINKPOINTS");
    //first loop to extract each linkpoint
    for(int id = 0; id < num_lp; id++){
        LinkPoint lp = linkpoint_db_.get_linkpoint(lp_db_url_, id);
        int map_id_0 = lp.get_map_id(0);
        int map_id_1 = lp.get_map_id(1);

        //second loop to add each possible connection to linkpoint in first loop
        for(int k = 0; k < num_lp; k++){
            if(id != k){
                LinkPoint lp_edge_candidate = linkpoint_db_.get_linkpoint(lp_db_url_, k);

                //check if linkpoints are connected by sharing the same map
                //check if edge has to be ignored because start or target are on it
                if((map_id_0 == lp_edge_candidate.get_map_id(0) && map_id_0 != ignore_map_id_start && map_id_0 != ignore_map_id_target)
                || (map_id_0 == lp_edge_candidate.get_map_id(1) && map_id_0 != ignore_map_id_start && map_id_0 != ignore_map_id_target)
                || (map_id_1 == lp_edge_candidate.get_map_id(1) && map_id_1 != ignore_map_id_start && map_id_1 != ignore_map_id_target)
                || (map_id_1 == lp_edge_candidate.get_map_id(0) && map_id_1 != ignore_map_id_start && map_id_1 != ignore_map_id_target))
                {
                    std::string edge_name = "e_" + std::to_string(id) + "_" + std::to_string(k);

                    std::string vertex_name = "v_" + std::to_string(id);
                    std::string vertex_name_1 = "v_" + std::to_string(k);

                    //try to add edge, if it does not exist yet it will return true, otherwise false
                    std::pair<edge_descriptor_t, bool> new_edge;
                    new_edge = boost::add_edge(vertices_[vertex_name], vertices_[vertex_name_1], graph_);

                    //if bool is true and edge is new and added to graph
                    if(new_edge.second){
                        edges_[edge_name] = new_edge;

                        //set properties for new edge
                        graph_[edges_[edge_name].first].weight = LPGraph::get_distance_int(lp, lp_edge_candidate);
                        graph_[edges_[edge_name].first].label = edge_name;
                        std::cout << "Added new edge with label " << edge_name << "\n";
                    }
                    else{
                        std::cout << "Edge with label " << edge_name << " already exists... not added!\n";
                    }
                }
            }
        }
    }
    return true;
}


int LPGraph::get_distance_int(LinkPoint lp, LinkPoint lp_edge_candidate){

    //determine which map positions (0 or 1) should be compared
    int index_0; //index for lp
    int index_1; //index for lp_edge_candidate
    if(lp.get_map_id(0) == lp_edge_candidate.get_map_id(0)){
        index_0 = 0;
        index_1 = 0;
    }
    else if (lp.get_map_id(0) == lp_edge_candidate.get_map_id(1)) {
        index_0 = 0;
        index_1 = 1;
    }
    else if (lp.get_map_id(1) == lp_edge_candidate.get_map_id(1)) {
        index_0 = 1;
        index_1 = 1;
    }
    else if (lp.get_map_id(1) == lp_edge_candidate.get_map_id(0)) {
        index_0 = 1;
        index_1 = 0;
    }
    else{
        ROS_ERROR("[LP Graph]: Error computing linkpoint distance!\n");
    }

    double delta_x = lp.get_pos_x(index_0) - lp_edge_candidate.get_pos_x(index_1);
    double delta_y = lp.get_pos_y(index_0) - lp_edge_candidate.get_pos_y(index_1);
    double distance = sqrt(pow(delta_x, 2) + pow(delta_y, 2));

    std::cout << "Computed distance (int) between lp " << lp.get_id() << " and lp " << lp_edge_candidate.get_id() << " is " << int(distance) << ".\n";

    return int(distance); //weights for dijkstra are integer
}


std::vector<int> LPGraph::dijkstra_shortest_path(int start_lp_id, int target_lp_id){

    std::string start_vertex_name = "v_" + std::to_string(start_lp_id);
    std::string target_vertex_name = "v_" + std::to_string(target_lp_id);

    // BGL Dijkstra's Shortest Path
     std::vector<int> distances( boost::num_vertices(graph_));
     std::vector<vertex_descriptor_t> predecessors(boost::num_vertices(graph_));

     boost::dijkstra_shortest_paths(graph_, vertices_[start_vertex_name],
                                    boost::weight_map(boost::get(&edge_properties_t::weight,graph_))
                                    .distance_map(boost::make_iterator_property_map(distances.begin(), boost::get(boost::vertex_index,graph_)))
                                    .predecessor_map(boost::make_iterator_property_map(predecessors.begin(), boost::get(boost::vertex_index,graph_)))
                                    );

     // Extract the shortest path from lp_start to lp_target
     typedef std::vector<edge_descriptor_t> path_t;
     path_t path;

     vertex_descriptor_t v = vertices_[target_vertex_name];
     for(vertex_descriptor_t u = predecessors[v]; u != v; v=u, u=predecessors[v])
     {
       std::pair<edge_descriptor_t,bool> edge_pair = boost::edge(u,v,graph_);
       path.push_back( edge_pair.first );
     }

     std::vector<int> shortest_path_ids; //vector containing the linkpoint ids of the shortest path

     std::cout << std::endl;
     std::cout << "Shortest Path from " << start_vertex_name << " to " << target_vertex_name << ":\n" << std::endl;
     for(path_t::reverse_iterator riter = path.rbegin(); riter != path.rend(); ++riter)
     {
       vertex_descriptor_t u_tmp = boost::source(*riter, graph_);
       vertex_descriptor_t v_tmp = boost::target(*riter, graph_);
       edge_descriptor_t   e_tmp = boost::edge(u_tmp, v_tmp, graph_).first;

       std::cout << "  " << graph_[u_tmp].label << " -> " << graph_[v_tmp].label << "    (weight: " << graph_[e_tmp].weight << ")" << std::endl;

       //add lp id to vector
       shortest_path_ids.push_back(graph_[u_tmp].id);

       if(riter == path.rend() -1 ){
           shortest_path_ids.push_back(graph_[v_tmp].id);
       }
     }

     //print vector
     std::cout << "\nshortest_path_ids:\n";
     for(int i = 0; i < shortest_path_ids.size(); i++){
         std::cout << shortest_path_ids[i] << "\n";
     }

     return shortest_path_ids;
}


std::vector<int> LPGraph::get_topological_path_to_target(std::string url, int map_id_start, geometry_msgs::Pose target_pose,int map_id_target){

    //add start vertex to the graph
    LinkPoint lp_start;
    tf::Transform cur_pose = LPGraph::get_current_pose();

    lp_start.set_id(100); //100 is the id for the start vertex
    lp_start.set_tag("start");
    lp_start.set_pos_x(cur_pose.getOrigin().x(), 0); //only filled with position at index 0
    lp_start.set_pos_y(cur_pose.getOrigin().y(), 0);
    lp_start.set_theta_from_z_w(cur_pose.getRotation().z(), cur_pose.getRotation().w(), 0);
    lp_start.set_map_id(map_id_start, 0);

    LPGraph::add_vertex_to_graph(lp_start, map_id_start);

    //add target vertex to graph
    LinkPoint lp_target;

    lp_target.set_id(101); //101 is the id for the target vertex
    lp_target.set_tag("target");
    lp_target.set_pos_x(target_pose.position.x, 0);
    lp_target.set_pos_y(target_pose.position.y, 0);
    lp_target.set_theta_from_z_w(target_pose.orientation.z, target_pose.orientation.w, 0);
    lp_target.set_map_id(map_id_target, 0);

    LPGraph::add_vertex_to_graph(lp_target, map_id_target);

    //compute vector of linkpoint ids containing the shortest path between start and target with dijkstra
    std::vector<int> shortest_path_ids = LPGraph::dijkstra_shortest_path(lp_start.get_id(), lp_target.get_id());

    return shortest_path_ids;
}


std::vector<int> LPGraph::get_topological_path_from_world_orig_to_robot(int map_id_robot){
    //add start vertex to the graph
    LinkPoint lp_world_orig;
    int map_id_world_orig = 0;

    lp_world_orig.set_id(100); //100 is the id for the start vertex
    lp_world_orig.set_tag("start");
    lp_world_orig.set_pos_x(0, 0); //only filled with position at index 0
    lp_world_orig.set_pos_y(0, 0);
    lp_world_orig.set_theta_from_z_w(0, 1, 0);
    lp_world_orig.set_map_id(map_id_world_orig, 0);

    LPGraph::add_vertex_to_graph(lp_world_orig, map_id_world_orig);

    //add robot (target) vertex to the graph
    LinkPoint lp_robot;
    tf::Transform cur_pose = LPGraph::get_current_pose();

    lp_robot.set_id(101); //100 is the id for the start vertex
    lp_robot.set_tag("target");
    lp_robot.set_pos_x(cur_pose.getOrigin().x(), 0); //only filled with position at index 0
    lp_robot.set_pos_y(cur_pose.getOrigin().y(), 0);
    lp_robot.set_theta_from_z_w(cur_pose.getRotation().z(), cur_pose.getRotation().w(), 0);
    lp_robot.set_map_id(map_id_robot, 0);

    LPGraph::add_vertex_to_graph(lp_robot, map_id_robot);

    //compute vector of linkpoint ids containing the shortest path between start and target with dijkstra
    std::vector<int> shortest_path_ids = LPGraph::dijkstra_shortest_path(lp_world_orig.get_id(), lp_robot.get_id());

    return shortest_path_ids;
}


tf::Transform LPGraph::get_current_pose(){
    //transform listener
    tf::TransformListener listener;

    std::string targetFrameID = "base_link";
    std::string sourceFrameID = "map";
    double waitTime = 0.5;

    bool bTransformAvailable = true;
    if(waitTime > 0)
        bTransformAvailable = listener.waitForTransform(targetFrameID,sourceFrameID,ros::Time(0),ros::Duration(waitTime));

    tf::StampedTransform transformStamped;
    if(bTransformAvailable){
        try{
            listener.lookupTransform(targetFrameID,sourceFrameID,ros::Time(0),transformStamped);
            ROS_INFO("Got Transform!");
        }
        catch (tf::TransformException ex){
            ROS_ERROR("%s",ex.what());
        }
    }
    else{
        ROS_WARN("Transform from '%s' to '%s' not available!",sourceFrameID.c_str(),targetFrameID.c_str());
    }
    tf::Transform tf;
    tf.setRotation(transformStamped.getRotation());
    tf.setOrigin(transformStamped.getOrigin());

    return tf;
}
