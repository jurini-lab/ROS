#include <iostream>

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"
#include <turtlesim/Spawn.h>

class TurtleController
{
private:
    ros::Publisher cmd_vel_pub;
    ros::Publisher cmd_vel_pub2;

    int linear_ , angular_;
    int linear2_ , angular2_;
    bool spawn_turtle_;

    geometry_msgs::Twist calculateCommand()
    {
        auto msg = geometry_msgs::Twist();
        
        // TODO: Control code goes here
        msg.linear.x = linear_;
        msg.angular.z = angular_;
        // std::cout << linear_ << std::endl;

        return msg;
    }

    geometry_msgs::Twist calculateCommand2()
    {
        auto msg = geometry_msgs::Twist();
        
        // TODO: Control code goes here
        msg.linear.x = linear2_;
        msg.angular.z = angular2_;

        return msg;
    }

public:
    TurtleController(){
        // Initialize ROS
        ros::NodeHandle n;
        n.param("/controller_1/linear_speed",linear_,linear_);
        n.param("/controller_1/angular_speed",angular_,angular_);

        n.param("/controller_2/linear_speed",linear2_,linear2_);
        n.param("/controller_2/angular_speed",angular2_,angular2_);

        // Create a publisher object, able to push messages
        cmd_vel_pub = n.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 10);
        cmd_vel_pub2 = n.advertise<geometry_msgs::Twist>("/turtle2/cmd_vel", 10);

        ros::service::waitForService("spawn");
        ros::ServiceClient spawnClient = n.serviceClient<turtlesim::Spawn>("spawn");

        turtlesim::Spawn srv;

        srv.request.x = 2.0;
        srv.request.y = 2.0;
        srv.request.theta = 0.0;
        srv.request.name = "turtle2";

        bool success = spawnClient.call(srv);
        ros::Duration(2.0).sleep();

        if(success){
            ROS_INFO_STREAM("Spawned a turtle named " << srv.request.name);
        }else{
            ROS_ERROR_STREAM("Failed to spawn.");
        }
    }

    void run(){
        // Send messages in a loop
        ros::Rate loop_rate(10);
        while (ros::ok())
        {
            // Calculate the command to apply
            auto msg = calculateCommand();
            auto msg2 = calculateCommand2();

            // Publish the new command
            cmd_vel_pub.publish(msg);
            cmd_vel_pub2.publish(msg2);

            // And throttle the loop
            loop_rate.sleep();
        }
    }

};


int main(int argc, char **argv){
    // Initialize ROS
    ros::init(argc, argv, "talker");

    // Create our controller object and run it
    auto controller = TurtleController();
    controller.run();

    // And make good on our promise
    return 0;
}
