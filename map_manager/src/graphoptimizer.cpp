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
 * @file   graphoptimizer.cpp
 * @author Simon Ehlers (simon.ehlers@stud.uni-hannover.de)
 * @date   26.07.2019
 *
 * @brief  Optimization of the topological graph after loop closure using g2o. Based on https://github.com/RainerKuemmerle/g2o/blob/master/g2o/examples/tutorial_slam2d/tutorial_slam2d.cpp
 */

#include "graphoptimizer/graphoptimizer.h"

using namespace std;
using namespace g2o;
using namespace g2o::tutorial;


void GraphOptimizer::g2o_optimize_graph(std::string url, LinkPoint lp_candidate, LinkPoint new_lp, tf::Transform tf_to_lp_candidate, double var_x, double var_y, double var_theta)
{

    /*********************************************************************************
       * creating the optimization problem
       ********************************************************************************/
    //here we use a boost graph as a framework for building the g2o graph

    std::cout << "Creating the optimizer";
    // allocating the optimizer
    SparseOptimizer optimizer;
    auto linearSolver = new SlamLinearSolver();
    linearSolver->setBlockOrdering(false);
    SlamBlockSolver* blockSolver = new SlamBlockSolver(linearSolver);
    OptimizationAlgorithmGaussNewton* solver = new OptimizationAlgorithmGaussNewton(blockSolver);

    optimizer.setAlgorithm(solver);

    // ######## first adding all the vertices ########################################################
    //Create a graph object
    graph_t graph;
    std::map <std::string, vertex_descriptor_t> vertices;
    std::map <std::string,  std::pair<edge_descriptor_t, bool>> edges;

    std::cout << "Optimization: Adding robot poses ... ";
    //create for every linkpoint a vertex
    int num_lp = linkpoint_db_.get_number_of_rows(url, "LINKPOINTS");
    for(int id = 0; id < num_lp; id++){
        LinkPoint lp = linkpoint_db_.get_linkpoint(url, id);

        //boost
        std::string vertex_id = "v_" + std::to_string(id);
        vertices[vertex_id] = boost::add_vertex(graph);
        //set properties for vertex
        graph[vertices[vertex_id]].id = id;
        graph[vertices[vertex_id]].label = vertex_id;

        //g2o
        const g2o::tutorial::SE2 t(lp.get_pos_x(2), lp.get_pos_y(2), lp.get_theta(2)); //adding global pose of linkpoint
        g2o::tutorial::VertexSE2* linkpoint =  new g2o::tutorial::VertexSE2;
        linkpoint->setId(lp.get_id());
        linkpoint->setEstimate(t);
        optimizer.addVertex(linkpoint);
        ROS_INFO("Created vertex with id = %i\n", id);
    }
    std::cout << " Adding robot poses done." << endl;

    // ######## second add the odometry constraints #####################################################
    std::cout << "Optimization: Adding odometry measurements ... ";

    LPGraph lp_graph_object;
    //first loop to extract each linkpoint
    for(int id = 0; id < num_lp; id++){
        LinkPoint lp = linkpoint_db_.get_linkpoint(url, id);
        int map_id_0 = lp.get_map_id(0);
        int map_id_1 = lp.get_map_id(1);

        //second loop to add each possible connection to linkpoint in first loop
        for(int k = 0; k < num_lp; k++){
            if(id != k){
                LinkPoint lp_edge_candidate = linkpoint_db_.get_linkpoint(url, k);

                //check if linkpoints are connected by sharing the same map
                //check if edge has to be ignored because start or target are on it
                if((map_id_0 == lp_edge_candidate.get_map_id(0))
                || (map_id_0 == lp_edge_candidate.get_map_id(1))
                || (map_id_1 == lp_edge_candidate.get_map_id(1))
                || (map_id_1 == lp_edge_candidate.get_map_id(0)))
                {
                    std::string edge_name = "e_" + std::to_string(id) + "_" + std::to_string(k);

                    std::string vertex_name = "v_" + std::to_string(id);
                    std::string vertex_name_1 = "v_" + std::to_string(k);

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

                        //add edge also in g2o graph
                        g2o::tutorial::EdgeSE2* odometry = new g2o::tutorial::EdgeSE2;
                        g2o::tutorial::SE2 measurement;

                        g2o::tutorial::Simulator::GridPose prev_pose;
                        prev_pose.truePose = g2o::tutorial::SE2(lp.get_pos_x(2), lp.get_pos_y(2), lp.get_theta(2));
                        g2o::tutorial::Simulator::GridPose  cur_pose;
                        cur_pose.truePose = g2o::tutorial::SE2(lp_edge_candidate.get_pos_x(2), lp_edge_candidate.get_pos_y(2), lp_edge_candidate.get_theta(2));
                        measurement = prev_pose.truePose.inverse() * cur_pose.truePose;

                        Eigen::Matrix3d covariance;
                        covariance.fill(0.);
                        covariance(0, 0) = var_x;
                        covariance(1, 1) = var_y;
                        covariance(2, 2) = var_theta;
                        Eigen::Matrix3d information = covariance.inverse();

                        odometry->vertices()[0] = optimizer.vertex(id); //from id
                        odometry->vertices()[1] = optimizer.vertex(k); //to id
                        odometry->setMeasurement(measurement);
                        odometry->setInformation(information);
                        optimizer.addEdge(odometry);
                    }
                    else{
                        std::cout << "Edge with label " << edge_name << " already exists... not added!\n";
                    }
                }
            }
        }
    }
    cerr << "done." << endl;

    //################ Adding Landmarks ######################################################
    // add the landmark vertices.
    cerr << "Optimization: add landmark vertices ... ";


    Eigen::Vector2d landmark_pose;
    landmark_pose.x() = lp_candidate.get_pos_x(2);
    landmark_pose.y() = lp_candidate.get_pos_y(2);

    std::cout << "set landmark vertices is done!\n";

    g2o::tutorial::VertexPointXY* landmark = new g2o::tutorial::VertexPointXY; // only one landmark because we only have one loop closure
    landmark->setId(num_lp); //Landmark has always id = number of linkpoints (highest lp id is num_lp-1)
    std::cout << "set id of landmark to " << num_lp << " \n";
    landmark->setEstimate(landmark_pose);
    optimizer.addVertex(landmark);

    std::cout << "done.\n";

    //############## Adding Landmark observation ################################
    cerr << "Optimization: add landmark observations ... ";

    Eigen::Matrix2d covariance;
    covariance.fill(0.0);
    std::cout << "AFTER covariance.fill(0.0)\n";
    covariance(0, 0) = var_x;
    covariance(1, 1) = var_y;
    Eigen::Matrix2d information = covariance.inverse();
    std::cout << "after covariance matrix created\n";

    Eigen::Vector2d tf_to_landmark;
    tf_to_landmark.x() = tf_to_lp_candidate.getOrigin().getX();
    tf_to_landmark.y() = tf_to_lp_candidate.getOrigin().getY();
    std::cout << "received tf_to_lp_candidate and inserted!\n";

    g2o::tutorial::EdgeSE2PointXY* landmarkObservation =  new g2o::tutorial::EdgeSE2PointXY;
    std::cout << "Insert edge for landmark observation from " << new_lp.get_id() << " to " << num_lp << "\n";
    landmarkObservation->vertices()[0] = optimizer.vertex(new_lp.get_id());
    landmarkObservation->vertices()[1] = optimizer.vertex(num_lp);
    std::cout << "set from to\n";
    landmarkObservation->setMeasurement(tf_to_landmark);
    std::cout << "after setMeasurement\n";
    landmarkObservation->setInformation(information);
    std::cout << "After set Information\n";

    //adding parameter sensor offset
    g2o::tutorial::SE2 sensorOffsetTransf(0.0, 0.0, 0.0); //no sensor offset
    g2o::tutorial::ParameterSE2Offset* sensorOffset = new g2o::tutorial::ParameterSE2Offset; 
    sensorOffset->setOffset(sensorOffsetTransf);
    sensorOffset->setId(0);
    optimizer.addParameter(sensorOffset);
    landmarkObservation->setParameterId(0, sensorOffset->id());
    std::cout << "Added sensor offset\n";

    optimizer.addEdge(landmarkObservation);

    cerr << "done." << endl;

    /*********************************************************************************
    * optimization
    ********************************************************************************/

    // dump initial state to the disk
    optimizer.save("tutorial_before.g2o");
    std::cout << "saved g2o graph";

    // prepare and run the optimization
    // fix the first robot pose to account for gauge freedom
    g2o::tutorial::VertexSE2* firstRobotPose = dynamic_cast<g2o::tutorial::VertexSE2*>(optimizer.vertex(0));
    firstRobotPose->setFixed(true);

    //fix the landmark pose
    g2o::tutorial::VertexPointXY* fixedLandmark = dynamic_cast<g2o::tutorial::VertexPointXY*>(optimizer.vertex(num_lp));
    fixedLandmark->setFixed(true);

    optimizer.setVerbose(true);

    cerr << "Optimizing" << endl;
    optimizer.initializeOptimization();
    optimizer.optimize(30);
    cerr << "done." << endl;

    optimizer.save("tutorial_after.g2o");

    Eigen::Vector2d translation = firstRobotPose->estimate().translation();
    Eigen::Rotation2Dd rotation = firstRobotPose->estimate().rotation();

    for(int k = 0; k < num_lp; k++){
        g2o::tutorial::VertexSE2* vertex_pose = dynamic_cast<g2o::tutorial::VertexSE2*>(optimizer.vertex(k));
        Eigen::Vector2d translation = vertex_pose->estimate().translation();
        Eigen::Rotation2Dd rotation = vertex_pose->estimate().rotation();

        //retrieve linkpoint from database
        LinkPoint corrected_lp = linkpoint_db_.get_linkpoint(url, k);

        //print correction of linkpoints
        std::cout << "\nOld global pose of linkpoint with id " << k << "\n";
        std::cout << "x = " << corrected_lp.get_pos_x(2) << "\n"
                  << "y = " << corrected_lp.get_pos_y(2) << "\n"
                  << "theta = " << corrected_lp.get_theta(2) << "\n";

        std::cout << "\nNew corrected global pose of linkpoint with id " << k << "\n";
        std::cout << "x = " << translation.x() << "\n"
                  << "y = " << translation.y() << "\n"
                  << "theta = " << rotation.angle() << "\n";

        corrected_lp.set_tag("-");
        corrected_lp.set_pos_x(translation.x(), 2);
        corrected_lp.set_pos_y(translation.y(), 2);
        corrected_lp.set_theta(rotation.angle(), 2);

        cv::Mat image_front = linkpoint_db_.get_linkpoint_image(url, k, "IMAGE_FRONT");
        cv::Mat image_rear = linkpoint_db_.get_linkpoint_image(url, k, "IMAGE_REAR");

        //delete linkpoint in database
        linkpoint_db_.delete_linkpoint(url, k);

        //insert linkpoint with new optimized poses
        linkpoint_db_.insert_linkpoint_with_images(url, corrected_lp, image_front, image_rear);

        std::cout << "Updated Linkpoint " << k << " with corrected pose in db";
    }

    // freeing the graph memory
    optimizer.clear();
}


Eigen::Vector2d GraphOptimizer::get_edge_translation(LinkPoint lp, LinkPoint lp_edge_candidate){
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
        ROS_ERROR("[LP Graph]: Error computing g2o edge translation!\n");
    }

    double delta_x = lp.get_pos_x(index_0) - lp_edge_candidate.get_pos_x(index_1);
    double delta_y = lp.get_pos_y(index_0) - lp_edge_candidate.get_pos_y(index_1);

    Eigen::Vector2d transform;
    transform.x() = delta_x;
    transform.y() = delta_y;
    transform.z() = 0;

    return transform;
}


Eigen::Rotation2D<double> GraphOptimizer::get_edge_rotation(LinkPoint lp, LinkPoint lp_edge_candidate){
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
        ROS_ERROR("[LP Graph]: Error computing g2o edge translation!\n");
    }

    double delta_theta = lp.get_theta(index_0) - lp_edge_candidate.get_theta(index_1);

    Eigen::Rotation2D<double> rotation(delta_theta);

    return rotation;
}


