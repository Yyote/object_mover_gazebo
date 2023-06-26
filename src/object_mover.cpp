// 
// 1 - obj name
// 2 - obj x
// 3 - obj y
// 4 - obj z
// 5 - maximum displacement
// 6 - speed_x
// 7 - speed_y
// 8 - speed_z
// 9 - node loop rate
// 

#include "ros/ros.h"
#include <gazebo_msgs/SetModelState.h>
#include <gazebo_msgs/GetModelState.h>
#include <gazebo_msgs/ModelState.h>
#include <geometry_msgs/Point.h>
#include <ctime>
#include <stdlib.h>
#include <string>
#include <ctime>
#include <stdlib.h>

geometry_msgs::Point obstacle_coords; // GLOBAL_VAR
geometry_msgs::Point origin; // GLOBAL_VAR
std::string model_name; // GLOBAL_VAR
geometry_msgs::Point frame_displacement; // GLOBAL_VAR
float maximum_displacement = 0; // GLOBAL_VAR
double speed_x = 0.0; // GLOBAL_VAR
double speed_y = 0.0; // GLOBAL_VAR
double speed_z = 0.0; // GLOBAL_VAR
ros::ServiceClient client; // GLOBAL_VAR
std::vector<double_t> time_buffer; // GLOBAL_VAR


/**
* @brief Initializes time buffer
*/
void _init_time_buffer(std::vector<double> &time_buffer_in)
{
    std::vector<double> time_buffer;
    time_buffer.resize(2);
    if (time_buffer_in.size() != 2)
    {
        time_buffer_in.resize(2);
    }

    time_buffer.at(0) = time_buffer_in.at(0);
    time_buffer.at(1) = time_buffer_in.at(1);

    if(time_buffer.at(0) == 0 && time_buffer.at(1) == 0)
    {
        time_buffer.at(0) = ros::Time::now().toSec();
        time_buffer.at(1) = ros::Time::now().toSec();
    ROS_INFO_STREAM("Debug flag n9");
    }

    time_buffer_in = time_buffer;
}


/**
* @brief Used to oscilate an object in the defined limits
*/
void oscilate_by_x(std::vector<double_t> time_buffer_in, double &speed_along_x, double origin_x, double origin_y, double origin_z, double maximum_displacement)
{
    double delta = speed_along_x * (time_buffer.at(0) - time_buffer.at(1));
    
    if(speed_along_x > 0)
    {
        if(frame_displacement.x < maximum_displacement)
        {
            frame_displacement.x = frame_displacement.x + delta;
            obstacle_coords.x = origin_x + frame_displacement.x;
        }
        else
        {
            speed_along_x = -speed_along_x;
            frame_displacement.x = frame_displacement.x + delta;
            obstacle_coords.x = origin_x + frame_displacement.x;
        }
    }
    else if (speed_along_x < 0)
    {
        if(frame_displacement.x > -maximum_displacement)
        {
            frame_displacement.x = frame_displacement.x + delta;
            obstacle_coords.x = origin_x + frame_displacement.x;
        }
        else
        {
            speed_along_x = -speed_along_x;
            frame_displacement.x = frame_displacement.x + delta;
            obstacle_coords.x = origin_x + frame_displacement.x;
        }
    }
}


/**
* @brief Used to oscilate an object in the defined limits
*/
void oscilate_by_y(std::vector<double_t> time_buffer_in, double &speed_along_y, double origin_x, double origin_y, double origin_z, double maximum_displacement)
{
    double delta = speed_along_y * (time_buffer.at(0) - time_buffer.at(1));
    if(speed_along_y > 0)
    {
        if(frame_displacement.y < maximum_displacement)
        {
            frame_displacement.y = frame_displacement.y + delta;
            obstacle_coords.y = origin_y + frame_displacement.y;
        }
        else
        {
            speed_along_y = -speed_along_y;
            frame_displacement.y = frame_displacement.y + delta;
            obstacle_coords.y = origin_y + frame_displacement.y;
        }
    }
    else if (speed_along_y < 0)
    {
        if(frame_displacement.y > -maximum_displacement)
        {
            frame_displacement.y = frame_displacement.y + delta;
            obstacle_coords.y = origin_y + frame_displacement.y;
        }
        else
        {
            speed_along_y = -speed_along_y;
            frame_displacement.y = frame_displacement.y + delta;
            obstacle_coords.y = origin_y + frame_displacement.y;
        }
    }
}


/**
* @brief Used to oscilate an object in the defined limits
*/
void oscilate_by_z(std::vector<double_t> time_buffer_in, double &speed_along_z, double origin_x, double origin_y, double origin_z, double maximum_displacement)
{
    double delta = speed_along_z * (time_buffer.at(0) - time_buffer.at(1));
    if(speed_along_z > 0)
    {
        if(frame_displacement.z < maximum_displacement)
        {
            frame_displacement.z = frame_displacement.z + delta;
            obstacle_coords.z = origin_z + frame_displacement.z;
        }
        else
        {
            speed_along_z = -speed_along_z;
            frame_displacement.z = frame_displacement.z + delta;
            obstacle_coords.z = origin_z + frame_displacement.z;
        }
    }
    else if (speed_along_z < 0)
    {
        if(frame_displacement.z > -maximum_displacement)
        {
            frame_displacement.z = frame_displacement.z + delta;
            obstacle_coords.z = origin_z + frame_displacement.z;
        }
        else
        {
            speed_along_z = -speed_along_z;
            frame_displacement.z = frame_displacement.z + delta;
            obstacle_coords.z = origin_z + frame_displacement.z;
        }
    }
} 


/**
* @brief Loop to calculate the object movement
*/ 
void logic()
{

    _init_time_buffer(time_buffer);
    time_buffer.at(1) = time_buffer.at(0);
    time_buffer.at(0) = ros::Time::now().toSec();

    // Movement calculation
    oscilate_by_x(time_buffer, speed_x, origin.x, origin.y, origin.z, maximum_displacement);
    oscilate_by_y(time_buffer, speed_y, origin.x, origin.y, origin.z, maximum_displacement);
    oscilate_by_z(time_buffer, speed_z, origin.x, origin.y, origin.z, maximum_displacement);

    // Put calculated position into gazebo model state setter service
    gazebo_msgs::SetModelState model_state_srv;

    model_state_srv.request.model_state.model_name = model_name;
    model_state_srv.request.model_state.pose.position = obstacle_coords;
    model_state_srv.request.model_state.reference_frame = "world";

    bool success_of_the_call = 0;

    
    if (client.call(model_state_srv))
    {
        success_of_the_call = model_state_srv.response.success;
        if (success_of_the_call)
        {
            ROS_INFO_STREAM("Successfully called service" << model_state_srv.response.status_message); 
        }
        else
        {
            ROS_ERROR_STREAM("the call was successful but the model state was not set ---> " << model_state_srv.response.status_message);
        }
    }
    else
    {
        ROS_ERROR_STREAM("Failed to call service");
    }
}


int main(int argc, char **argv)
{
    srand(time(0));
    ros::init(argc, argv, "gazebo_model_state_node_" + std::to_string(rand()));

    int node_loop_rate = 0; // ???

    // Initialize nodehandle
    ros::NodeHandle n;

    // Gazebo set model state service
    client = n.serviceClient<gazebo_msgs::SetModelState>("/gazebo/set_model_state");
    gazebo_msgs::SetModelState model_state_srv;

    // Getting parameters from console args
    model_name = argv[1]; 

    // Initial coords of an object
    obstacle_coords.x = atof(argv[2]);
    obstacle_coords.y = atof(argv[3]);
    obstacle_coords.z = atof(argv[4]);
    origin = obstacle_coords;

    // Maximum displacement from initial coords
    maximum_displacement = atof(argv[5]);

    // By-axis speed setter
    speed_x = atof(argv[6]);
    speed_y = atof(argv[7]);
    speed_z = atof(argv[8]);

    // Node loop rate setter. Influences the smoothness of object movement
    node_loop_rate = atof(argv[9]);

    ros::Rate loop_rate(node_loop_rate);

    while(ros::ok())
    {
        logic();
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}