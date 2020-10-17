#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

bool at_pickup_location = false;
bool at_dropoff_location = false;
bool picked_up = false;
bool dropped_off =false;

int main( int argc, char** argv )
{
  ros::init(argc, argv, "add_markers");
  ros::NodeHandle n;
  ros::Rate r(1);
  ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);

  uint32_t shape = visualization_msgs::Marker::CUBE;
  visualization_msgs::Marker marker;

  ros::param::set("/picked_up",picked_up);
  ros::param::set("/dropped_off",dropped_off);

  marker.header.frame_id = "map";
  marker.header.stamp = ros::Time::now();

  marker.ns = "basic_shapes";
  marker.id = 0;
  marker.type = shape;
  marker.action = visualization_msgs::Marker::ADD;

  marker.pose.position.x = 1;
  marker.pose.position.y = -2;
  marker.pose.position.z = 0;
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 1.0;

  marker.scale.x = 0.2;
  marker.scale.y = 0.2;
  marker.scale.z = 0.2;

  marker.color.r = 0.0f;
  marker.color.g = 0.0f;
  marker.color.b = 1.0f;
  marker.color.a = 1.0;

  marker.lifetime = ros::Duration();

  while (marker_pub.getNumSubscribers() < 1)
  {
    if (!ros::ok())
    {
      return 0;
    }
    ROS_WARN_ONCE("Please create a subscriber to the marker");
    sleep(1);
  }

  marker_pub.publish(marker);
  ROS_INFO("Marker published ");

  if(ros::param::has("/at_pickup_location") && !picked_up)
  {
    ros::param::get("/at_pickup_location",at_pickup_location);
    while(!at_pickup_location)
    {
      ROS_INFO("waiting to for Robot");
      ros::param::get("/at_pickup_location",at_pickup_location);
      r.sleep();
    }
    ROS_INFO("Robot is here, picking up...");
  }
  else
  { 
    ROS_INFO("wait for 5 sec");
    ros::Duration(5).sleep();
  }
    
  ROS_INFO("picked up.!");
  picked_up = true;
  ros::param::set("/picked_up",picked_up);
  marker.action = visualization_msgs::Marker::DELETE;
  marker_pub.publish(marker);
  
  if(ros::param::has("/at_dropoff_location") && !dropped_off)
  {
    ros::param::get("/at_dropoff_location",at_dropoff_location); 
    while(!at_dropoff_location)
    {
      ROS_INFO("On board");
      ros::param::get("/at_dropoff_location",at_dropoff_location);
      r.sleep();
    }
    ROS_INFO("Robot is at drop off location, dropping off...");
  }
  else
  {
    ros::Duration(5).sleep();
  }

  dropped_off = true;
  ros::param::set("/dropped_off",dropped_off);  
  marker.action = visualization_msgs::Marker::ADD;
  marker.pose.position.x = -3.0;
  marker.pose.position.y = -4.0;
  marker_pub.publish(marker);
  ROS_INFO("dropped off!");

}