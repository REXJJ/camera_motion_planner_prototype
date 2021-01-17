#include <ros/ros.h>
#include <std_srvs/Trigger.h>
#include <visualization_msgs/Marker.h>

#include <cmath>
#include <iostream>
#include <vector>
#include <fstream>
#include <Eigen/Dense>
#include <Eigen/Core>
#include <boost/algorithm/string.hpp>

using namespace std;

vector<vector<double>> readCameraLocations(string filename)
{
    vector<vector<double>> locations;
    ifstream file(filename);
    string line;
    while(getline(file,line))
    {
        vector<string> numbers;
        boost::split(numbers,line,boost::is_any_of(","));
        vector<double> loc;
        for(auto x:numbers)
        {
            loc.push_back(double(stof(x)));
        }
        locations.push_back(loc);
    }
    return locations;
}

class VizController
{
    public:
        VizController(ros::NodeHandle& n);
        ros::Publisher marker_pub_;
        // = n.advertise<visualization_msgs::Marker>("visualization_marker", 10)
        ros::ServiceServer publish_path_service_;
        bool trigger(std_srvs::TriggerRequest& req, std_srvs::TriggerResponse& res);

        float f;
};

VizController::VizController(ros::NodeHandle& nh)
{  
    marker_pub_ = nh.advertise<visualization_msgs::Marker>("visualization_marker", 10);
    publish_path_service_ = nh.advertiseService("publish_path",&VizController::trigger, this);
    std::cout<<"Visualization Controller Initialized.."<<std::endl;
    f = 0.0;
}

bool VizController::trigger(std_srvs::TriggerRequest& req, std_srvs::TriggerResponse& res)
{
    std::cout<<"Inside the service function."<<std::endl;
// %Tag(MARKER_INIT)%
    visualization_msgs::Marker points, line_strip, line_list;
    points.header.frame_id = line_strip.header.frame_id = line_list.header.frame_id = "/base_link";
    points.header.stamp = line_strip.header.stamp = line_list.header.stamp = ros::Time::now();
    points.ns = line_strip.ns = line_list.ns = "points_and_lines";
    points.action = line_strip.action = line_list.action = visualization_msgs::Marker::ADD;
    points.pose.orientation.w = line_strip.pose.orientation.w = line_list.pose.orientation.w = 1.0;
// %EndTag(MARKER_INIT)%

// %Tag(ID)%
    points.id = 0;
    line_strip.id = 1;
    line_list.id = 2;
// %EndTag(ID)%

// %Tag(TYPE)%
    points.type = visualization_msgs::Marker::POINTS;
    line_strip.type = visualization_msgs::Marker::LINE_STRIP;
    line_list.type = visualization_msgs::Marker::LINE_LIST;
// %EndTag(TYPE)%

// %Tag(SCALE)%
    // POINTS markers use x and y scale for width/height respectively
    points.scale.x = 0.02;
    points.scale.y = 0.02;

    // LINE_STRIP/LINE_LIST markers use only the x component of scale, for the line width
    line_strip.scale.x = 0.01;
    line_list.scale.x = 0.01;
// %EndTag(SCALE)%

// %Tag(COLOR)%
    // Points are green
    points.color.g = 1.0f;
    points.color.a = 1.0;

    // Line strip is blue
    line_strip.color.b = 1.0;
    line_strip.color.a = 1.0;

    // Line list is red
    line_list.color.r = 1.0;
    line_list.color.a = 1.0;
// %EndTag(COLOR)%
    string filename = "/home/rex/REX_WS/Catkin_WS/src/camera_motion_planner_prototype/visualization_helper/data/path.csv";
    auto locations = readCameraLocations(filename);

    for(int i=0;i<locations.size();i++)
    {
        geometry_msgs::Point p;
        auto loc = locations[i];
        p.x = loc[0];
        p.y = loc[1];
        p.z = loc[2];
        points.points.push_back(p);
        line_strip.points.push_back(p);
    }

    marker_pub_.publish(points);
    marker_pub_.publish(line_strip);
    // marker_pub_.publish(line_list);

    res.success=true;
    std::cout<<"Publishing path.."<<std::endl;
    return true;
}

int main( int argc, char** argv )
{
  ros::init(argc, argv, "points_and_lines");
  ros::NodeHandle n;

  ros::Rate r(30);
  VizController viz(n);

  while (ros::ok())
  {
    r.sleep();
    ros::spinOnce();
  }
}
// %EndTag(FULLTEXT)%


