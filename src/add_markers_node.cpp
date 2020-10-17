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

  // Set the frame ID and timestamp.  See the TF tutorials for information on these.
  marker.header.frame_id = "map";
  marker.header.stamp = ros::Time::now();

  // Set the namespace and id for this marker.  This serves to create a unique ID
  // Any marker sent with the same namespace and id will overwrite the old one
  marker.ns = "basic_shapes";
  marker.id = 0;

  // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
  marker.type = shape;

  // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
  marker.action = visualization_msgs::Marker::ADD;

  // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
  marker.pose.position.x = 1;
  marker.pose.position.y = -2;
  marker.pose.position.z = 0;
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 1.0;

  // Set the scale of the marker -- 1x1x1 here means 1m on a side
  marker.scale.x = 0.2;
  marker.scale.y = 0.2;
  marker.scale.z = 0.2;

  // Set the color -- be sure to set alpha to something non-zero!
  marker.color.r = 0.0f;
  marker.color.g = 0.0f;
  marker.color.b = 1.0f;
  marker.color.a = 1.0;

  marker.lifetime = ros::Duration();

  // Publish the marker
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
  ROS_INFO("published, now wait for 5 sec");

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
    picked_up = true;
    ros::param::set("/picked_up",picked_up);
    marker.action = visualization_msgs::Marker::DELETE;
    marker_pub.publish(marker);
  }
  
  if(ros::param::has("/at_dropoff_location") && !dropped_off)
  {
    ros::param::get("/at_dropoff_location",at_dropoff_location); 
    while(!at_dropoff_location)
    {
      ROS_INFO("On board");
      ros::param::get("/at_dropoff_location",at_dropoff_location);
      r.sleep();
    }
    ROS_INFO("Robot is at drop off location, dropped off");
    dropped_off = true;
    ros::param::set("/dropped_off",dropped_off);
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = -3.0;
    marker.pose.position.y = -4.0;
    marker_pub.publish(marker);
  }
  else
  {
    ros::Duration(5).sleep();
  }
  
  marker.pose.position.x = -3.0;
  marker.pose.position.y = -4.0;
  marker_pub.publish(marker);
  ROS_INFO("new pos published");
  ros::Duration(5).sleep();

}