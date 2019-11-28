#include <ros/ros.h>
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
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
//// Create the filtering object 直通滤波器，直接对点的坐标进行切割。如果使用线结构光扫描的方式采集点云，
/// 必然物体沿z向分布较广，但x,y向的分布处于有限范围内。
/// 此时可使用直通滤波器，确定点云在x或y方向上的范围，
/// 可较快剪除离群点，达到第一步粗处理的目的。
//    pcl::PassThrough<pcl::PointXYZ> pass;
//    pass.setInputCloud (cloud);
//    pass.setFilterFieldName ("z");
//    pass.setFilterLimits (0.0, 1.0);
////pass.setFilterLimitsNegative (true);
//    pass.filter (*cloud_filtered);
//

//    // Create the filtering object  如果使用高分辨率相机等设备对点云进行采集，往往点云会较为密集。
// 过多的点云数量会对后续分割工作带来困难。体素格滤波器可以达到向下采样同时不破坏点云本身几何结构的功能。
// 点云几何结构不仅是宏观的几何外形，也包括其微观的排列方式，比如横向相似的尺寸，纵向相同的距离。
// 随机下采样虽然效率比体素滤波器高，但会破坏点云微观结构。
//    pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
//    sor.setInputCloud (cloud);
//    sor.setLeafSize (0.01f, 0.01f, 0.01f);
//    sor.filter (*cloud_filtered);
//
//    // Create the filtering object  统计滤波器用于去除明显离群点（离群点往往由测量噪声引入）。
// 其特征是在空间中分布稀疏，可以理解为：每个点都表达一定信息量，某个区域点越密集则可能信息量越大。
// 噪声信息属于无用信息，信息量较小。所以离群点表达的信息可以忽略不计。考虑到离群点的特征，则可以定义某处点云小于某个密度，既点云无效。
// 计算每个点到其最近的k个点平均距离。则点云中所有点的距离应构成高斯分布。给定均值与方差，可剔除3∑之外的点。
//    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
//    sor.setInputCloud (cloud);
//    sor.setMeanK (50);
//    sor.setStddevMulThresh (1.0);
//    sor.filter (*cloud_filtered);
//
//    // build the filter半径滤波器与统计滤波器相比更加简单粗暴。
// 以某点为中心画一个圆计算落在该圆中点的数量，当数量大于给定值时，则保留该点，数量小于给定值则剔除该点。
// 此算法运行速度快，依序迭代留下的点一定是最密集的，但是圆的半径和圆内点的数目都需要人工指定。
//    pcl::RadiusOutlierRemoval<pcl::PointXYZ> outrem;
//    outrem.setInputCloud(cloud);
//    outrem.setRadiusSearch(0.8);
//    outrem.setMinNeighborsInRadius (2);
//    // apply filter
//    outrem.filter (*cloud_filtered);


using namespace std;




class cut_pointcloud
{
public:
    
    cut_pointcloud()
    {
        ros::param::get("~sub_topic", sub_topic);
        ros::param::get("~ad_topic", ad_topic);
        pub = nh.advertise<sensor_msgs::PointCloud2>(ad_topic, 1);
        sub = nh.subscribe(sub_topic,3,&cut_pointcloud::call_back,this);

  // if (n.hasParam("model"))
  //   n.getParam("model", model);
  // else
  //   {
  //     ROS_ERROR("%s: must provide a model name", name.c_str());
  //     return -1;
  //   }
    }

    void xpassthroughfilter(pcl::PointCloud<pcl::PointXYZ>::Ptr input,const pcl::PointCloud<pcl::PointXYZ>::Ptr output)
    {
        double x_max_range,x_min_range;
        ros::param::get("~x_min_range", x_min_range);
        ros::param::get("~x_max_range", x_max_range);
        pcl::PassThrough<pcl::PointXYZ> passx;
        passx.setInputCloud (input);
//pass.setFilterLimitsNegative (true);
        passx.setFilterFieldName("x");
        passx.setFilterLimits(x_min_range,x_max_range);
        passx.filter(*output);
    }

    void zpassthroughfilter(pcl::PointCloud<pcl::PointXYZ>::Ptr input,const pcl::PointCloud<pcl::PointXYZ>::Ptr output)
    {
        double z_max_range,z_min_range;
        ros::param::get("~z_min_range", z_min_range);
        ros::param::get("~z_max_range", z_max_range);
        pcl::PassThrough<pcl::PointXYZ> passz;
        passz.setInputCloud (input);
        passz.setFilterFieldName ("z");
        passz.setFilterLimits (z_min_range, z_max_range);
//pass.setFilterLimitsNegative (true);
        passz.filter (*output);
    }

    void voxelGrid_filter(pcl::PointCloud<pcl::PointXYZ>::Ptr input,const pcl::PointCloud<pcl::PointXYZ>::Ptr output)
    {
        double resolution;
        ros::param::get("~voxel_resolution", resolution);
        pcl::VoxelGrid<pcl::PointXYZ> voxelGrid_filter;
        voxelGrid_filter.setInputCloud (input);
        voxelGrid_filter.setLeafSize (resolution, resolution, resolution);
        voxelGrid_filter.filter (*output);
    }

    void RadiusOutlierRemoval(pcl::PointCloud<pcl::PointXYZ>::Ptr input,const pcl::PointCloud<pcl::PointXYZ>::Ptr output)
    {
        double search_radius;
        int pointcount;
        ros::param::get("~mincount", pointcount);
        ros::param::get("~setRadius", search_radius);
        pcl::RadiusOutlierRemoval<pcl::PointXYZ> sor;
        sor.setInputCloud(input);
        sor.setRadiusSearch(search_radius);//搜索半径
        sor.setMinNeighborsInRadius(pointcount);//半径内需要有三个点
        sor.setNegative(false);
        sor.filter(*output);
    }


    void process_pointcloud(const  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in,const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_final)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr tmp(new pcl::PointCloud<pcl::PointXYZ>);
//        pcl::copyPointCloud(*cloud_in,*tmp);
        bool xfilter,zfilter;
        ros::param::get("~xfilter", xfilter);
        ros::param::get("~zfilter", zfilter);
        if(xfilter)
        {
            xpassthroughfilter(cloud_in,cloud_final);
            pcl::copyPointCloud(*cloud_final,*cloud_in);
        }
        if(zfilter)
        {
            zpassthroughfilter(cloud_in,cloud_final);
            pcl::copyPointCloud(*cloud_final,*cloud_in);
        }
        voxelGrid_filter(cloud_in,cloud_final);
        pcl::copyPointCloud(*cloud_final,*cloud_in);
        RadiusOutlierRemoval(cloud_in,cloud_final);
//        pcl::copyPointCloud(*cloud_in,*tmp);
    }
    void call_back(const sensor_msgs::PointCloud2ConstPtr input)
    {
        cout<<"I RECEIVED INPUT !"<<endl;
        ros::Time begin = ros::Time::now();
        pcl::fromROSMsg(*input,*cloud_in);
        cut_pointcloud::process_pointcloud(cloud_in,cloud_final);//the function to process the pointcloud
        pcl::toROSMsg(*cloud_final,cloud_out);
        cloud_out.header.stamp = ros::Time::now();
        cloud_out.header.frame_id = "/camera_link";
        cout<< ros::Time::now().toSec()-begin.toSec()<<endl;//处理一帧耗时
        pub.publish(cloud_out);
        cout<<"I PROCESSED IT !"<<endl;
    }


private:
    ros::NodeHandle nh;
    ros::Publisher pub;
    ros::Subscriber sub;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in {new pcl::PointCloud<pcl::PointXYZ>};//转换为pcl格式
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_final {new pcl::PointCloud<pcl::PointXYZ>};//加工后的pcl格式
    sensor_msgs::PointCloud2 cloud_out;//转换为ros格式
    string sub_topic;
    string ad_topic;

};
int
main(int argc,char **argv)
{
    ros::init(argc,argv,"cut_pointcloud");
    cut_pointcloud cut_pointcloud1;
//    std::string port="";
//    ros::param::get("~port", port);
//    std::cout<<port<<std::endl;


    ros::spin();
}
