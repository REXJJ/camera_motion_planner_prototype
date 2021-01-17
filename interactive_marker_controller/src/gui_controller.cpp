#include <ros/ros.h>

#include <interactive_markers/interactive_marker_server.h>
#include <interactive_markers/menu_handler.h>

#include <tf/transform_broadcaster.h>
#include <tf/tf.h>

#include <math.h>
#include <iostream>
/*********************************************/
//PCL HEADERS
/**********************************************/
#include <pcl/io/ply_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/io/io.h>
#include <pcl/common/transforms.h>
#include <pcl/common/common.h>

#include <Eigen/Dense>
#include <Eigen/Core>

#include "pcl_ros_utilities/pcl_ros_utilities.hpp"

using namespace visualization_msgs;
using namespace pcl;
using namespace std;
using namespace Eigen;


Eigen::Affine3f transformation;
ros::Publisher pub;

// %Tag(vars)%
boost::shared_ptr<interactive_markers::InteractiveMarkerServer> server;
interactive_markers::MenuHandler menu_handler;
pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
// %EndTag(vars)%

// %Tag(Box)%
Marker makeBox( InteractiveMarker &msg )
{
    Marker marker;

    marker.type = Marker::CUBE;
    marker.scale.x = msg.scale * 0.45;
    marker.scale.y = msg.scale * 0.45;
    marker.scale.z = msg.scale * 0.45;
    marker.color.r = 0.5;
    marker.color.g = 0.5;
    marker.color.b = 0.5;
    marker.color.a = 0.0;

    return marker;
}

InteractiveMarkerControl& makeBoxControl( InteractiveMarker &msg )
{
    InteractiveMarkerControl control;
    control.always_visible = true;
    control.markers.push_back( makeBox(msg) );
    msg.controls.push_back( control );

    return msg.controls.back();
}


std::vector<float> centroid;

std::string validate_seq(std::string seq)
{
    if(seq =="")
        seq = "ZYX";	
    bool invalid_flag = false;
    if(seq.size()!=3)
    {
        invalid_flag = true;
    }
    for (int i =0;i<3;++i)
        if(seq[i]!='X' && seq[i]!='Y' && seq[i]!='Z' && seq[i]!='x' && seq[i]!='y' && seq[i]!='z')
        {
            invalid_flag = true; 
            break;
        }
    if(invalid_flag)
    {
        std::cerr << "ERROR: Invalid Rotations Sequence: " << seq << std::endl;
        std::terminate();		
    }
    return seq;
}

//Default : ZYX
Eigen::Matrix3d eul2rot(Eigen::MatrixXd eul_angles, std::string seq="")
{
    seq = validate_seq(seq);
    Eigen::Matrix3d rot_mat = Eigen::Matrix3d::Identity();
    for (int i=0; i<3; ++i)
    {
        if(seq[i]=='X' || seq[i]=='x')
            rot_mat = rot_mat * Eigen::AngleAxisd(eul_angles(0,i), Eigen::Vector3d::UnitX());
        else if(seq[i]=='Y' || seq[i]=='y')
            rot_mat = rot_mat * Eigen::AngleAxisd(eul_angles(0,i), Eigen::Vector3d::UnitY());			
        else if(seq[i]=='Z' || seq[i]=='z')
            rot_mat = rot_mat * Eigen::AngleAxisd(eul_angles(0,i), Eigen::Vector3d::UnitZ());					
    }
    return rot_mat; 
}

Eigen::Matrix3d qt2rot(Eigen::MatrixXd quat)
{
	Eigen::Quaterniond q;
	q.x() = quat(0,0);
	q.y() = quat(0,1);
	q.z() = quat(0,2);
	q.w() = quat(0,3);	
	return q.normalized().toRotationMatrix();
}

Eigen::MatrixXd qt2eul(Eigen::MatrixXd quat, std::string seq="ZYX")
{
	seq = validate_seq(seq);
	Eigen::Matrix3d rot_mat = qt2rot(quat);
	int rot_idx[3];
	for (int i=0; i<3; ++i)
	{
		if(seq[i]=='X' || seq[i]=='x')
			rot_idx[i] = 0;
		else if(seq[i]=='Y' || seq[i]=='y')
			rot_idx[i] = 1;
		else if(seq[i]=='Z' || seq[i]=='z')
			rot_idx[i] = 2;
	}	
	Eigen::MatrixXd eul_angles(1,3);
	Eigen::Vector3d eul_angles_vec;
	eul_angles_vec = rot_mat.eulerAngles(rot_idx[0], rot_idx[1], rot_idx[2]);
	eul_angles(0,0) = eul_angles_vec[0];
	eul_angles(0,1) = eul_angles_vec[1];
	eul_angles(0,2) = eul_angles_vec[2];	
	return eul_angles;
}

Eigen::MatrixXd vectorToTransformationMatrix(vector<double>& a_T_b_static)
{
    Eigen::MatrixXd a_T_b = Eigen::MatrixXd::Identity(4,4);
    a_T_b(0,3) = a_T_b_static[0];        
    a_T_b(1,3) = a_T_b_static[1];
    a_T_b(2,3) = a_T_b_static[2];
    Eigen::MatrixXd eul_ang(1,3);
    eul_ang<<a_T_b_static[3],a_T_b_static[4],a_T_b_static[5];
    a_T_b.block(0,0,3,3) = eul2rot(eul_ang);
    return a_T_b;
}

Eigen::Affine3f vectorToAffineMatrix(vector<double> location)
{
    Eigen::MatrixXd two_T_one = vectorToTransformationMatrix(location);
    Eigen::Affine3f transformation = Eigen::Affine3f::Identity();
    for(int i=0;i<3;i++)
        for(int j=0;j<4;j++)
            transformation(i,j)=two_T_one(i,j);
    return transformation;
}


void update(vector<double> location_b)
{
    auto loc_b = vectorToAffineMatrix(location_b);
    Affine3f C = Affine3f::Identity();
    C(0,3) = centroid[0];
    C(1,3) = centroid[1];
    C(2,3) = centroid[2];
    auto transformation = C*loc_b*C.inverse();
    std::cout<<"Updation matrix: "<<std::endl;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_transformed(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::transformPointCloud (*cloud, *cloud_transformed,transformation);
    PCLUtilities::publishPointCloud<pcl::PointXYZRGB>(*cloud_transformed,pub);
}

void processFeedback( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback )
{
    std::ostringstream s;
    s << "Feedback from marker '" << feedback->marker_name << "' "
        << " / control '" << feedback->control_name << "'";

    std::ostringstream mouse_point_ss;
    if( feedback->mouse_point_valid )
    {
        mouse_point_ss << " at " << feedback->mouse_point.x
            << ", " << feedback->mouse_point.y
            << ", " << feedback->mouse_point.z
            << " in frame " << feedback->header.frame_id;
    }

    switch ( feedback->event_type )
    {
        case visualization_msgs::InteractiveMarkerFeedback::BUTTON_CLICK:
            ROS_INFO_STREAM( s.str() << ": button click" << mouse_point_ss.str() << "." );
            break;

        case visualization_msgs::InteractiveMarkerFeedback::MENU_SELECT:
            ROS_INFO_STREAM( s.str() << ": menu item " << feedback->menu_entry_id << " clicked" << mouse_point_ss.str() << "." );
            break;

        case visualization_msgs::InteractiveMarkerFeedback::POSE_UPDATE:
            {
                ROS_INFO_STREAM( s.str() << ": pose changed"
                        << "\nposition = "
                        << feedback->pose.position.x
                        << ", " << feedback->pose.position.y
                        << ", " << feedback->pose.position.z
                        << "\norientation = "
                        << feedback->pose.orientation.w
                        << ", " << feedback->pose.orientation.x
                        << ", " << feedback->pose.orientation.y
                        << ", " << feedback->pose.orientation.z
                        << "\nframe: " << feedback->header.frame_id
                        << " time: " << feedback->header.stamp.sec << "sec, "
                        << feedback->header.stamp.nsec << " nsec" );
                auto pose = feedback->pose;
                Eigen::MatrixXd quat(1,4);
                quat<<pose.orientation.x,pose.orientation.y,pose.orientation.z,pose.orientation.w;
                Eigen::MatrixXd eul = qt2eul(quat);
                vector<double> location = {pose.position.x-centroid[0],pose.position.y-centroid[1],pose.position.z-centroid[2],eul(0,0),eul(0,1),eul(0,2)};
                std::cout<<"Rotation : "<<std::endl;
                std::cout<<location[3]<<" "<<location[4]<<" "<<location[5]<<std::endl;
                update(location);
                break;
            }

        case visualization_msgs::InteractiveMarkerFeedback::MOUSE_DOWN:
            ROS_INFO_STREAM( s.str() << ": mouse down" << mouse_point_ss.str() << "." );
            break;

        case visualization_msgs::InteractiveMarkerFeedback::MOUSE_UP:
            ROS_INFO_STREAM( s.str() << ": mouse up" << mouse_point_ss.str() << "." );
            break;
    }

    server->applyChanges();
}
// %EndTag(processFeedback)%

// %Tag(6DOF)%
void make6DofMarker( bool fixed, unsigned int interaction_mode, const tf::Vector3& position, bool show_6dof )
{
    InteractiveMarker int_marker;
    int_marker.header.frame_id = "base_link";
    tf::pointTFToMsg(position, int_marker.pose.position);
    int_marker.scale = 1;

    int_marker.name = "simple_6dof";
    int_marker.description = "Simple 6-DOF Control";

    // insert a box
    makeBoxControl(int_marker);
    int_marker.controls[0].interaction_mode = interaction_mode;

    InteractiveMarkerControl control;

    if ( fixed )
    {
        int_marker.name += "_fixed";
        int_marker.description += "\n(fixed orientation)";
        control.orientation_mode = InteractiveMarkerControl::FIXED;
    }

    if (interaction_mode != visualization_msgs::InteractiveMarkerControl::NONE)
    {
        std::string mode_text;
        if( interaction_mode == visualization_msgs::InteractiveMarkerControl::MOVE_3D )         mode_text = "MOVE_3D";
        if( interaction_mode == visualization_msgs::InteractiveMarkerControl::ROTATE_3D )       mode_text = "ROTATE_3D";
        if( interaction_mode == visualization_msgs::InteractiveMarkerControl::MOVE_ROTATE_3D )  mode_text = "MOVE_ROTATE_3D";
        int_marker.name += "_" + mode_text;
        int_marker.description = std::string("3D Control") + (show_6dof ? " + 6-DOF controls" : "") + "\n" + mode_text;
    }

    if(show_6dof)
    {
        tf::Quaternion orien(1.0, 0.0, 0.0, 1.0);
        orien.normalize();
        tf::quaternionTFToMsg(orien, control.orientation);
        control.name = "rotate_x";
        control.interaction_mode = InteractiveMarkerControl::ROTATE_AXIS;
        int_marker.controls.push_back(control);
        control.name = "move_x";
        control.interaction_mode = InteractiveMarkerControl::MOVE_AXIS;
        int_marker.controls.push_back(control);

        orien = tf::Quaternion(0.0, 1.0, 0.0, 1.0);
        orien.normalize();
        tf::quaternionTFToMsg(orien, control.orientation);
        control.name = "rotate_z";
        control.interaction_mode = InteractiveMarkerControl::ROTATE_AXIS;
        int_marker.controls.push_back(control);
        control.name = "move_z";
        control.interaction_mode = InteractiveMarkerControl::MOVE_AXIS;
        int_marker.controls.push_back(control);

        orien = tf::Quaternion(0.0, 0.0, 1.0, 1.0);
        orien.normalize();
        tf::quaternionTFToMsg(orien, control.orientation);
        control.name = "rotate_y";
        control.interaction_mode = InteractiveMarkerControl::ROTATE_AXIS;
        int_marker.controls.push_back(control);
        control.name = "move_y";
        control.interaction_mode = InteractiveMarkerControl::MOVE_AXIS;
        int_marker.controls.push_back(control);
    }
    server->insert(int_marker);
    server->setCallback(int_marker.name, &processFeedback);
    if (interaction_mode != visualization_msgs::InteractiveMarkerControl::NONE)
        menu_handler.apply( *server, int_marker.name );
}
// %EndTag(6DOF)%

// %Tag(main)%
int main(int argc, char** argv)
{

    string filename = "/home/rex/Desktop/reference.pcd";
#if 1
    if (pcl::io::loadPCDFile<pcl::PointXYZRGB> (filename, *cloud) == -1) //* load the file
    {
        PCL_ERROR ("Couldn't read file for base. \n");
        return 0;
    }
#else
    pcl::PLYReader Reader;
    Reader.read(filename, *cloud);
#endif
    cloud->header.frame_id = "base_link";

    pcl::PointXYZRGB min_pt;
    pcl::PointXYZRGB max_pt;
    pcl::getMinMax3D<pcl::PointXYZRGB>(*cloud, min_pt, max_pt);

    float centroid_x = min_pt.x + (max_pt.x-min_pt.x)/2.0;
    float centroid_y = min_pt.y + (max_pt.y-min_pt.y)/2.0;
    float centroid_z = min_pt.z + (max_pt.z-min_pt.z)/2.0;

    centroid = {centroid_x,centroid_y,centroid_z};

    transformation = Affine3f::Identity();
    ros::init(argc, argv, "gui_controller");
    ros::NodeHandle n;
    pub = n.advertise<sensor_msgs::PointCloud2>("reference_object", 1);
    server.reset( new interactive_markers::InteractiveMarkerServer("basic_controls","",false) );
    ros::Duration(0.1).sleep();
    tf::Vector3 position;
    position = tf::Vector3(centroid_x,centroid_y,centroid_z);
    make6DofMarker( false, visualization_msgs::InteractiveMarkerControl::NONE, position, true );
    // position = tf::Vector3( 0, 3, 0);
    // make6DofMarker( true, visualization_msgs::InteractiveMarkerControl::NONE, position, true );
    server->applyChanges();
    ros::spin();
    server.reset();
}
// %EndTag(main)%
