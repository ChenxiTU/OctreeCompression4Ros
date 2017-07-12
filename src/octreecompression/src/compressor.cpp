#include <iostream>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/String.h>
#include <sstream>
#include <stdio.h>
#include <stdlib.h>
//PCL specific includes
#include <pcl/compression/octree_pointcloud_compression.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/conversions.h>
#include <pcl/common/common.h>
#include <pcl/common/common_headers.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl_ros/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include "compressedpointcloud.h"
#include <ros/console.h>
//#include <inttypes.h>
// **** 10 ***** 20 ****** 30 ****** 40 ****** 50 ****** 60 ****** 70 ****** 80 ****** 90 ***** 100 ***** 110 ***** 120

using namespace std;
typedef pcl::PointXYZRGB PointT;

ros::Publisher pub;
pcl::io::OctreePointCloudCompression<PointT>* PointCloudEncoder;
pcl::PointCloud<PointT>::Ptr pclCloud(new pcl::PointCloud<PointT>);
struct configurationProfile_t
    {
      double pointResolution;
      const double octreeResolution;
      bool doVoxelGridDownSampling;
      unsigned int iFrameRate;
      const unsigned char colorBitResolution;
      bool doColorEncoding;
    };

// TODO: Could unsubscribe when no one is subscribed to the compressed output.
// Right now, though, we are assuming that local-loopback is low enough load that we don't care.
//const struct configurationProfile_t compressionProfiles_[COMPRESSION_PROFILE_COUNT] = {
//{
//    // PROFILE: LOW_RES_ONLINE_COMPRESSION_WITHOUT_COLOR
//       0.01, /* pointResolution = */
//       0.01, /* octreeResolution = */
//       true, /* doVoxelGridDownDownSampling = */
//       50, /* iFrameRate = */
//       4, /* colorBitResolution = */
//       false /* doColorEncoding = */
//    }, {
//    // PROFILE: LOW_RES_ONLINE_COMPRESSION_WITH_COLOR
//        0.01, /* pointResolution = */
//        0.01, /* octreeResolution = */
//        true, /* doVoxelGridDownDownSampling = */
//        50, /* iFrameRate = */
//        4, /* colorBitResolution = */
//        true /* doColorEncoding = */
//    }, {
//    // PROFILE: MED_RES_ONLINE_COMPRESSION_WITHOUT_COLOR
//        0.005, /* pointResolution = */
//        0.01, /* octreeResolution = */
//        false, /* doVoxelGridDownDownSampling = */
//        40, /* iFrameRate = */
//        5, /* colorBitResolution = */
//        false /* doColorEncoding = */
//    }, {
//    // PROFILE: MED_RES_ONLINE_COMPRESSION_WITH_COLOR
//        0.005, /* pointResolution = */
//        0.01, /* octreeResolution = */
//        false, /* doVoxelGridDownDownSampling = */
//        40, /* iFrameRate = */
//        5, /* colorBitResolution = */
//        true /* doColorEncoding = */
//    }, {
//   // PROFILE: HIGH_RES_ONLINE_COMPRESSION_WITHOUT_COLOR
//        0.0001, /* pointResolution = */
//        0.01, /* octreeResolution = */
//        false, /* doVoxelGridDownDownSampling = */
//        30, /* iFrameRate = */
//        7, /* colorBitResolution = */
//        false /* doColorEncoding = */
//    }, {
//    // PROFILE: HIGH_RES_ONLINE_COMPRESSION_WITH_COLOR
//        0.0001, /* pointResolution = */
//        0.01, /* octreeResolution = */
//        false, /* doVoxelGridDownDownSampling = */
//        30, /* iFrameRate = */
//        7, /* colorBitResolution = */
//        true /* doColorEncoding = */
//    }, {
//    // PROFILE: LOW_RES_OFFLINE_COMPRESSION_WITHOUT_COLOR
//        0.01, /* pointResolution = */
//        0.01, /* octreeResolution = */
//        true, /* doVoxelGridDownDownSampling = */
//        100, /* iFrameRate = */
//        4, /* colorBitResolution = */
//        false /* doColorEncoding = */
//    }, {
//    // PROFILE: LOW_RES_OFFLINE_COMPRESSION_WITH_COLOR
//        0.01, /* pointResolution = */
//        0.01, /* octreeResolution = */
//        true, /* doVoxelGridDownDownSampling = */
//        100, /* iFrameRate = */
//        4, /* colorBitResolution = */
//        true /* doColorEncoding = */
//    }, {
//    // PROFILE: MED_RES_OFFLINE_COMPRESSION_WITHOUT_COLOR
//        0.005, /* pointResolution = */
//        0.005, /* octreeResolution = */
//        true, /* doVoxelGridDownDownSampling = */
//        100, /* iFrameRate = */
//        5, /* colorBitResolution = */
//        false /* doColorEncoding = */
//    }, {
//    // PROFILE: MED_RES_OFFLINE_COMPRESSION_WITH_COLOR
//        0.005, /* pointResolution = */
//        0.01, /* octreeResolution = */
//        false, /* doVoxelGridDownDownSampling = */
//        100, /* iFrameRate = */
//        5, /* colorBitResolution = */
//        true /* doColorEncoding = */
//    }, {
//    // PROFILE: HIGH_RES_OFFLINE_COMPRESSION_WITHOUT_COLOR
//        0.0001, /* pointResolution = */
//        0.0001, /* octreeResolution = */
//        true, /* doVoxelGridDownDownSampling = */
//        100, /* iFrameRate = */
//        8, /* colorBitResolution = */
//        false /* doColorEncoding = */
//    }, {
//    // PROFILE: HIGH_RES_OFFLINE_COMPRESSION_WITH_COLOR
//        0.0001, /* pointResolution = */
//        0.01, /* octreeResolution = */
//        false, /* doVoxelGridDownDownSampling = */
//        100, /* iFrameRate = */
//        8, /* colorBitResolution = */
//        true /* doColorEncoding = */
//    }};




void cloud_cb (const sensor_msgs::PointCloud2::ConstPtr& rosCloud)
{
cout<<pub.getNumSubscribers()<<endl;
  if(pub.getNumSubscribers())
  {
    // Stringstream to store compressed point cloud
    std::stringstream compressedData;

    //////////////////////////////////////Not sure
    pcl::PCLPointCloud2 pcl_pc2;
    pcl_conversions::toPCL(*rosCloud,pcl_pc2);
    /////////////////////////////////////////////
    // Must convert from sensor_msg::PointCloud2 to pcl::PointCloud<PointT> for the encoder
   pcl::fromPCLPointCloud2 (pcl_pc2, *pclCloud);
   //cout<<rosCloud->header<<endl;
   cout<<sizeof(rosCloud->data)/sizeof(rosCloud->data[0])<<endl;
   cout<<rosCloud->height<<endl;
   cout<<rosCloud->point_step<<endl;

   //cout<<sizeof(rosCloud)<<endl;
   //cout<<sizeof(*rosCloud)<<endl;
  // ROS_INFO_NAMED("compressor" ,"Compressing cloud with frame [%s] and stamp [%l64d]", pclCloud->header.frame_id.c_str(), pclCloud->header.stamp);
    // Compress the pointcloud
    PointCloudEncoder->encodePointCloud (pclCloud, compressedData);

    //////////
//    pcl::io::OctreePointCloudCompression<PointT>* decoder_;
//    decoder_=new pcl::io::OctreePointCloudCompression<PointT> () ;
//    pcl::PointCloud<PointT>::Ptr pcl_cc(new pcl::PointCloud<PointT>);
//    decoder_->decodePointCloud(compressedData, pcl_cc);
//    cout<<"laaaal"<<endl;
//    delete(decoder_);

    // Pack into a compressed message
    compressed_pointcloud_transport::CompressedPointCloud output;
    output.header = rosCloud->header;
    output.data = compressedData.str();
    pub.publish(output);

    long original_size = sizeof(rosCloud->data);
    int compressed_size = sizeof(output);
    ROS_INFO_NAMED("compressor", "Published cloud, original size %ld bytes, compressed size %d bytes, %.3f of original.",
             original_size, compressed_size, (float)compressed_size/(float)original_size);
  }
  else

    ROS_INFO_NAMED("compressor" ,"Received input cloud but there are no subscribers; not publishing.");

}


pcl::io::compression_Profiles_e getCompressionProfile(ros::NodeHandle nh)
{
  cout<<"lalal"<<endl;
  std::string compression_level;
//  if(nh.getParam("compression_profile", compression_level))
//  {
//    if(compression_level == "LOW_RES_ONLINE_COMPRESSION_WITHOUT_COLOR")        return pcl::io::LOW_RES_ONLINE_COMPRESSION_WITHOUT_COLOR;
//    else if(compression_level == "LOW_RES_ONLINE_COMPRESSION_WITH_COLOR")      return pcl::io::LOW_RES_ONLINE_COMPRESSION_WITH_COLOR;
//    else if(compression_level == "MED_RES_ONLINE_COMPRESSION_WITHOUT_COLOR")   return pcl::io::MED_RES_ONLINE_COMPRESSION_WITHOUT_COLOR;
//    else if(compression_level == "MED_RES_ONLINE_COMPRESSION_WITH_COLOR")      return pcl::io::MED_RES_ONLINE_COMPRESSION_WITH_COLOR;
//    else if(compression_level == "HIGH_RES_ONLINE_COMPRESSION_WITHOUT_COLOR")  return pcl::io::HIGH_RES_ONLINE_COMPRESSION_WITHOUT_COLOR;
//    else if(compression_level == "HIGH_RES_ONLINE_COMPRESSION_WITH_COLOR")     return pcl::io::HIGH_RES_ONLINE_COMPRESSION_WITH_COLOR;
//    else if(compression_level == "LOW_RES_OFFLINE_COMPRESSION_WITHOUT_COLOR")  return pcl::io::LOW_RES_OFFLINE_COMPRESSION_WITHOUT_COLOR;
//    else if(compression_level == "LOW_RES_OFFLINE_COMPRESSION_WITH_COLOR")     return pcl::io::LOW_RES_OFFLINE_COMPRESSION_WITH_COLOR;
//    else if(compression_level == "MED_RES_OFFLINE_COMPRESSION_WITHOUT_COLOR")  return pcl::io::MED_RES_OFFLINE_COMPRESSION_WITHOUT_COLOR;
//    else if(compression_level == "MED_RES_OFFLINE_COMPRESSION_WITH_COLOR")     return pcl::io::MED_RES_OFFLINE_COMPRESSION_WITH_COLOR;
//    else if(compression_level == "HIGH_RES_OFFLINE_COMPRESSION_WITHOUT_COLOR") return pcl::io::HIGH_RES_OFFLINE_COMPRESSION_WITHOUT_COLOR;
//    else if(compression_level == "HIGH_RES_OFFLINE_COMPRESSION_WITH_COLOR")    return pcl::io::HIGH_RES_OFFLINE_COMPRESSION_WITH_COLOR;
//    else return pcl::io::MED_RES_ONLINE_COMPRESSION_WITH_COLOR;
//  }
//   else
//  {
    return pcl::io::MANUAL_CONFIGURATION;
//  }
}

int main(int argc, char** argv) {
  // Initialize ROS
  ros::init(argc, argv, "cloud_compressor");
  ros::NodeHandle nh, pnh("~");
  //ROS_DEBUG_NAMED("compressor" ,"Received input cloud but there are no subscribers; not publishing.");
  cout<<"lalal1"<<endl;
  // Initialize encoder
//  configurationProfile_t compressionProfile =
//  {      0.01, /* pointResolution = */
//         0.01, /* octreeResolution = */
//         true, /* doVoxelGridDownDownSampling = */
//         50, /* iFrameRate = */
//         4, /* colorBitResolution = */
//         false /* doColorEncoding = */
//  };

  bool showStatistics;
  double pointResolution;
  float octreeResolution;
  bool doVoxelGridDownDownSampling;
  unsigned int iFrameRate;
  bool doColorEncoding;
 unsigned int colorBitResolution;

   showStatistics = false;
   pointResolution = 0.03;
   octreeResolution = 0.03f;
   doVoxelGridDownDownSampling = true;
   iFrameRate = 100;
   doColorEncoding = false;
   colorBitResolution = 4;

  pcl::io::compression_Profiles_e compressionProfile = getCompressionProfile(pnh);





  //PointCloudEncoder = new pcl::io::OctreePointCloudCompression<PointT>(compressionProfile, false);
  PointCloudEncoder = new pcl::io::OctreePointCloudCompression<PointT>(compressionProfile, showStatistics, pointResolution,
                                                                       octreeResolution, doVoxelGridDownDownSampling, iFrameRate,
        doColorEncoding, static_cast<unsigned char> (colorBitResolution));

  cout<<"lalal2"<<endl;
  //Create a ROS subscriber for the input point cloud
  pub = nh.advertise<compressed_pointcloud_transport::CompressedPointCloud>("/octree_compressed", 10);
  //pub = nh.advertise<std_msgs::String>("output", 10);
  ros::Subscriber sub = nh.subscribe("/velodyne_points", 100, cloud_cb);
  cout<<"lalal3"<<endl;
  //Create a ROS publisher for the output point cloud


  //Spin
  ros::spin();
}


