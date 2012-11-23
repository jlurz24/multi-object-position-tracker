#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <tf/transform_listener.h>
#include <cmvision/Blobs.h>
#include <position_tracker/DetectedObjects.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <message_filters/time_synchronizer.h>
#include <pcl/io/pcd_io.h>
#include <pcl/features/feature.h>
#include <cmath>
#include <boost/math/constants/constants.hpp>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/voxel_grid.h>

using namespace std;

#define MOD_VOXEL 0

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
typedef PointCloud::ConstPtr PointCloudConstPtr;
typedef PointCloud::Ptr PointCloudPtr;

static const double CLUSTER_DISTANCE_TOLERANCE_M = 0.02;
static const int MIN_CLUSTER_SIZE = 50;
static const int MAX_CLUSTER_SIZE = 25000;
static const double VOXEL_LEAF_SIZE_M = 0.01;

class MultiObjectDetector {
  private:
    ros::NodeHandle nh;
    ros::NodeHandle privateHandle;
    tf::TransformListener tf;
    std::string objectName;
   
    auto_ptr<message_filters::Subscriber<cmvision::Blobs> > blobsSub;
    auto_ptr<message_filters::Subscriber<sensor_msgs::PointCloud2> > depthPointsSub;
    
    double lastUpdate;
 
    // Publisher for the resulting position event.
    ros::Publisher pub; 
    
    auto_ptr<message_filters::TimeSynchronizer<cmvision::Blobs, sensor_msgs::PointCloud2> > sync;

 public:
    MultiObjectDetector() : privateHandle("~"){
      
      privateHandle.getParam("object_name", objectName);
      ROS_INFO("Detecting blobs with object name %s", objectName.c_str());

      // Publish the object location
      ros::SubscriberStatusCallback cb = boost::bind(&MultiObjectDetector::connectCB, this);
      pub = nh.advertise<position_tracker::DetectedObjects>("object_locations/" + objectName, 1, cb, cb);
      ROS_INFO("Initialization of object detector complete");
    }
    
    ~MultiObjectDetector(){
    }

 private:
    void connectCB(){
      if(pub.getNumSubscribers() == 1){
        startListening();
      }
      else if(pub.getNumSubscribers() == 0) {
        stopListening();
      }
    }

    void stopListening(){
      ROS_INFO("Stopping listeners for multi object detector");
      blobsSub->unsubscribe(); 
      depthPointsSub->unsubscribe();
    }

    void startListening(){
      ROS_INFO("Starting to listen for blob messages");

      if(blobsSub.get() == NULL){
        // Listen for message from cm vision when it sees an object.
        blobsSub.reset(new message_filters::Subscriber<cmvision::Blobs>(nh, "/blobs", 1));
      }
      else {
        blobsSub->subscribe();
      }
      
      // List for the depth messages
      if(depthPointsSub.get() == NULL){
        depthPointsSub.reset(new message_filters::Subscriber<sensor_msgs::PointCloud2>(nh, "/wide_stereo/left/points", 3));
      }
      else {
        depthPointsSub->subscribe();
      }
  
      if(sync.get() == NULL){
        // Sync the two messages
        sync.reset(new message_filters::TimeSynchronizer<cmvision::Blobs, sensor_msgs::PointCloud2>(*blobsSub, *depthPointsSub, 3));
      
        sync->registerCallback(boost::bind(&MultiObjectDetector::finalBlobCallback, this, _1, _2));
      }
      ROS_INFO("Registration for blob events complete.");
    }

    void finalBlobCallback(const cmvision::BlobsConstPtr& blobsMsg, const sensor_msgs::PointCloud2ConstPtr& depthPointsMsg){

      // Check if there are detected blobs.
      if(blobsMsg->blobs.size() == 0){
        ROS_INFO("No blobs detected");
        return;
      }
      
      pcl::PointCloud<pcl::PointXYZ> depthCloud;
      pcl::fromROSMsg(*depthPointsMsg, depthCloud);

      PointCloudPtr allBlobs;
      const vector<pcl::PointIndices> blobClouds = splitBlobs(depthCloud, blobsMsg, allBlobs);

      if(blobClouds.size() == 0){
        ROS_INFO("No blobs to use for centroid detection");
        return;
      }

      // Iterate over each detected blob and determine it's centroid.
      position_tracker::DetectedObjects objects;
      for(unsigned int i = 0; i < blobClouds.size(); ++i){
        Eigen::Vector4f centroid;
        pcl::compute3DCentroid(*allBlobs, blobClouds[i].indices, centroid);

        // Convert the centroid to a point stamped
        geometry_msgs::PointStamped resultPoint;
        resultPoint.header.frame_id = "wide_stereo_optical_frame";
        resultPoint.header.stamp = depthPointsMsg->header.stamp;

        // Convert the centroid to a geometry msg point
        resultPoint.point.x = centroid[0];
        resultPoint.point.y = centroid[1];
        resultPoint.point.z = centroid[2];

        geometry_msgs::PointStamped resultPointMap;
        resultPointMap.header.frame_id = "/base_footprint";
        resultPointMap.header.stamp = depthPointsMsg->header.stamp;
        tf.waitForTransform(resultPointMap.header.frame_id, resultPoint.header.frame_id, resultPoint.header.stamp, ros::Duration(5.0));
        tf.transformPoint(resultPointMap.header.frame_id, resultPoint, resultPointMap);
        objects.positions.push_back(resultPointMap);
 
        ROS_INFO("Detected object with center %f %f %f", centroid[0], centroid[1], centroid[2]);
     }
     
     ROS_INFO("Detected %lu objects", objects.positions.size());     
     // Broadcast the result
     pub.publish(objects);
   }

    const vector<pcl::PointIndices> splitBlobs(const pcl::PointCloud<pcl::PointXYZ>& depthCloud, const cmvision::BlobsConstPtr& blobsMsg, PointCloudPtr& allBlobsOut){
       // Iterate over all the blobs and create a single cloud of all points.
       // We will subdivide this blob again later.
       PointCloudPtr allBlobs(new PointCloud);
       allBlobs->header = depthCloud.header;
       allBlobs->is_dense = false;
       allBlobs->height = 1;
       
       for(unsigned int k = 0; k < blobsMsg->blobs.size(); ++k){
          const cmvision::Blob blob = blobsMsg->blobs[k];
          if(objectName.size() > 0 && objectName != blob.colorName){
            continue;
          }

          for(unsigned int i = blob.left; i <= blob.right; ++i){
            for(unsigned int j = blob.top; j <= blob.bottom; ++j){
              pcl::PointXYZ point = depthCloud.points.at(j * blobsMsg->image_width + i);
              allBlobs->points.push_back(point);
            }
          }
      }
   
#if MOD_VOXEL
      // Use a voxel grid to downsample the input to a 1cm grid.
      ROS_INFO("PointCloud before filtering has %lu datapoints.", allBlobs->points.size());
      pcl::VoxelGrid<pcl::PointXYZ> vg;
      pcl::PointCloud<pcl::PointXYZ>::Ptr allBlobsFiltered(new pcl::PointCloud<pcl::PointXYZ>);
      vg.setInputCloud(allBlobs);
      vg.setLeafSize(VOXEL_LEAF_SIZE_M, VOXEL_LEAF_SIZE_M, VOXEL_LEAF_SIZE_M);
      vg.filter(*allBlobsFiltered);
      ROS_INFO("PointCloud after filtering has %lu datapoints.", allBlobsFiltered->points.size());
      allBlobs = allBlobsFiltered;
 #endif

      ROS_INFO("Starting cluster extraction on cloud of size %lu @ %f", allBlobs->points.size(), ros::Time::now().toSec());
      std::vector<pcl::PointIndices> clusterIndices;
      pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
      ec.setClusterTolerance(CLUSTER_DISTANCE_TOLERANCE_M);
      ec.setMinClusterSize(MIN_CLUSTER_SIZE);
      ec.setMaxClusterSize(MAX_CLUSTER_SIZE);
      ec.setInputCloud(allBlobs);
      ec.extract(clusterIndices);
      ROS_INFO("Ending cluster extraction @ %f", ros::Time::now().toSec());
      
      allBlobsOut = allBlobs;
      return clusterIndices;
    }
};

int main(int argc, char **argv){
  ros::init(argc, argv, "multi_object_detector");
  MultiObjectDetector mobd;
  ros::spin();
  return 0;
}

