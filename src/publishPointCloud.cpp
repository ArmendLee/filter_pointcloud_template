#include <ros/ros.h>
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <iostream>
#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <tf/transform_broadcaster.h>



ros::Publisher pub;

sensor_msgs::PointCloud2 sumCloud;

float tfParameters[2][6]={{0,0,0,0,0,0},
                          {0,0,0,0,0,0}};//旋转矩阵参数，前三个分别为xyz平移，后三个为旋转参数

void transform_coor(const sensor_msgs::PointCloud2 &originData,pcl::PointCloud<pcl::PointXYZRGB> &after_transform,int cam_index)
{
    pcl::PointCloud<pcl::PointXYZRGB> tmp;
    pcl::fromROSMsg(originData,tmp);
    Eigen::Affine3f transform_matrix = Eigen::Affine3f::Identity();
    // 定义沿着x轴平移2.5m
    transform_matrix.translation() << tfParameters[cam_index][0],tfParameters[cam_index][1],tfParameters[cam_index][2];

    // 与之前的旋转定义一样，绕z轴旋转一个theta角
    transform_matrix.rotate(Eigen::AngleAxisf(tfParameters[cam_index][3], Eigen::Vector3f::UnitX())*
                            Eigen::AngleAxisf(tfParameters[cam_index][4], Eigen::Vector3f::UnitY())*
                            Eigen::AngleAxisf(tfParameters[cam_index][5], Eigen::Vector3f::UnitZ()));//旋转矩阵
    pcl::transformPointCloud(tmp,after_transform,transform_matrix);

}


//void filt_pointcloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
//{
//    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
//    pcl::PointCloud<pcl::PointXYZ>Ptr tmp=*cloud;
//    sor.setInputCloud(cloud);
//    sor.setMeanK(50);//50个临近点 sor.setStddevMulThresh(1.0);
//    sor.setStddevMulThresh(1.0);
//    sor.setNegative(true);
//    sor.filter()
//
//}




void sum_and_calibrate_pc2_with_pcl (const sensor_msgs::PointCloud2ConstPtr& cam1,
                                     const sensor_msgs::PointCloud2ConstPtr& cam2,
                                     tf::Transform* transform_para,
                                    tf::TransformBroadcaster* br)
{
    pcl::PointCloud<pcl::PointXYZRGB> sum_pc;
    pcl::PointCloud<pcl::PointXYZRGB> pc1;
    pcl::PointCloud<pcl::PointXYZRGB> pc2;
//    pcl::PointCloud<pcl::PointXYZRGB> pc3;
//    pcl::PointCloud<pcl::PointXYZRGB> pc4;
    // Create a container for the data.
    transform_coor(*cam1,pc1,0);
    transform_coor(*cam2,pc2,1);
    sum_pc=pc1+pc2;
    sensor_msgs::PointCloud2 output;
    pcl::toROSMsg(sum_pc,output);
    // Do data processing here...
    // Publish the data.
    pub.publish (output);
    ROS_INFO("processing...");
    br->sendTransform(tf::StampedTransform(*transform_para, ros::Time::now(), "cam_1_link", "carrot1"));
}


int main (int argc, char** argv)
{
    // Initialize ROS
    ros::init (argc, argv, "publishPointCloud");
    tf::Transform transform_para;
    tf::TransformBroadcaster br;
    transform_para.setOrigin( tf::Vector3(0.0, 2.0, 0.0) );
    transform_para.setRotation( tf::Quaternion(3.14, 3.14, 3.14, 1) );
    ros::NodeHandle nh;
    while(nh.ok())
    {
        ROS_INFO("Ready to process pointcloud.");
        br.sendTransform(tf::StampedTransform(transform_para, ros::Time::now(), "camera_link", "carrot1"));
    }

    // Create a ROS subscriber for the input point cloud
    //ros::Subscriber sub = nh.subscribe ("input", 1, cloud_cb);
//    message_filters::Subscriber<sensor_msgs::PointCloud2> pc_cam1(nh,"/cam_1/depth_registered/points", 1);
//    message_filters::Subscriber<sensor_msgs::PointCloud2> pc_cam2(nh,"/cam_2/depth_registered/points", 1);
//    message_filters::Subscriber<sensor_msgs::PointCloud2> pc_cam1(nh,"/cam_1/depth_registered/points", 1);
//    message_filters::Subscriber<sensor_msgs::PointCloud2> pc_cam1(nh,"/cam_1/depth_registered/points", 1);

    // Create a ROS publisher for the output point cloud
//    pub = nh.advertise<sensor_msgs::PointCloud2> ("realsense_pointcloud", 1);
//    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2,sensor_msgs::PointCloud2,
//            tf::Transform*,tf::TransformBroadcaster*> MySyncPolicy;
//    message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), pc_cam1,pc_cam2);
//    sync.registerCallback(boost::bind(&sum_and_calibrate_pc2_with_pcl, _1, _2));

    // Spin
    ros::spin ();
}


//
//typedef pcl::PointXYZRGB PointT;typedef pcl::PointCloud<PointT> PointCloudT;
//
//PointCloudT::Ptr cloud(new PointCloudT);
//PointCloudT cloud_out;
//tf::TransformListener* tf_listener;
//std::string world_frame_id;
//std::string frame_id;
//std::string path;
//
//
//void cloud_cb (const PointCloudT::ConstPtr& callback_cloud)
//{
//    *cloud = *callback_cloud;
//    frame_id = cloud->header.frame_id;
//    std::stringstream ss;
//    std::string s1;
//    ss<<ros::Time::now();
//    s1=ss.str();
//
//    //获取转换信息
//    tf::StampedTransform transform;
//    try
//    {
//        tf_listener->lookupTransform(world_frame_id, frame_id, ros::Time(0), transform);
//    }
//    catch (tf::TransformException ex)
//    {
//        ROS_ERROR("transform exception: %s", ex.what());
//    }
//
//
//    //4*4 transform matrix:
//
//    Eigen::Affine3d pcl_transform;
//    tf::transformTFToEigen(transform, pcl_transform);
//
//    //transform:
//    pcl::transformPointCloud(*cloud,cloud_out,pcl_transform);
//
//    // cloud saving:
//    std::string file_name=path+"cloud_"+frame_id.substr(1, frame_id.length()-1)+s1+".pcd";
//    pcl::io::savePCDFileBinary(file_name,cloud_out);
//}
//
//
//
//int main (int argc, char** argv)
//{
//    ros::init(argc, argv, "cloudtf");
//    ros::NodeHandle nh("~");
//
//    std::string pointcloud_topic;
//    nh.param("pointcloud_topic", pointcloud_topic, std::string("/camera/depth_registered/points"));
//    nh.param("world_frame_id", world_frame_id, std::string("/odom"));
//    nh.param("path", path, std::string("/home/ubuntu1/pointtest/"));
//    std::cout << "Read some parameters from launch file." << std::endl;
//
//    tf_listener = new tf::TransformListener();
//
//    // Subscribers:
//    ros::Subscriber sub = nh.subscribe(pointcloud_topic, 1, cloud_cb);
//
//    ros::spin();
//
//    return 0;
//}
