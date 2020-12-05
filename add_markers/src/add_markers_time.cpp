#include <ros/ros.h>
#include <std_msgs/String.h>
#include <visualization_msgs/Marker.h>

class SubscribeAndPublish {
private:
    ros::NodeHandle n;
    ros::Publisher marker_pub;
    ros::Subscriber action_sub;
    visualization_msgs::Marker marker;

public:
    SubscribeAndPublish() {
        marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);
    }

    void initializePickUp() {
        // Set our shape type to be a cube
        uint32_t shape = visualization_msgs::Marker::CUBE;

        // Set the frame ID and timestamp.  See the TF tutorials for information on these.
        marker.header.frame_id = "map";
        marker.header.stamp = ros::Time::now();

        // Set the namespace and id for this marker.  This serves to create a unique ID
        // Any marker sent with the same namespace and id will overwrite the old one
        marker.ns = "basic_shapes";
        marker.id = 0;

        // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
        marker.type = shape;

        // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
        marker.pose.position.z = 0;
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;

        // Set the scale of the marker -- 1x1x1 here means 1m on a side
        marker.scale.x = 0.3;
        marker.scale.y = 0.3;
        marker.scale.z = 0.3;

        // Set the color -- be sure to set alpha to something non-zero!
        marker.color.r = 0.0f;
        marker.color.g = 1.0f;
        marker.color.b = 0.0f;
        marker.color.a = 1.0;

        marker.lifetime = ros::Duration();
        ros::Duration(1.0).sleep();

    }

    void addMarker(float x, float y) {
            marker.pose.position.x = x;
            marker.pose.position.y = y;
            marker.action = visualization_msgs::Marker::ADD;
            marker_pub.publish(marker);
    }

    void deleteMarker() {
            marker.action = visualization_msgs::Marker::DELETE;
            marker_pub.publish(marker);
    }

};

int main( int argc, char** argv )
{
    ros::init(argc, argv, "add_markers_time");

    // initialize pubsub object
    SubscribeAndPublish SAPObject;
    SAPObject.initializePickUp();

    SAPObject.addMarker(1.0, 1.0);
    ros::Duration(5.0).sleep();

    SAPObject.deleteMarker();
    ros::Duration(5.0).sleep();

    SAPObject.addMarker(3.0, 3.0);

    ros::spin();
    return 0;
}