#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <tf/transform_listener.h>
#include <cmvision/Blobs.h>
#include <position_tracker/DetectedObjects.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <pcl/io/pcd_io.h>
#include <pcl/features/feature.h>
#include <cmath>
#include <boost/math/constants/constants.hpp>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/voxel_grid.h>
#include <visualization_msgs/MarkerArray.h>

using namespace std;

#define MOD_VOXEL 0

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
typedef PointCloud::ConstPtr PointCloudConstPtr;
typedef PointCloud::Ptr PointCloudPtr;
typedef message_filters::sync_policies::ApproximateTime<cmvision::Blobs, sensor_msgs::PointCloud2> BlobCloudSyncPolicy;
typedef message_filters::Synchronizer<BlobCloudSyncPolicy> BlobCloudSync;

static const double CLUSTER_DISTANCE_TOLERANCE_M = 0.04;
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
    
    double clusterDistanceTolerance;
    
    // Publisher for the resulting position event.
    ros::Publisher pub; 

    // Publish for visualization
    ros::Publisher markerPub;
    
    auto_ptr<BlobCloudSync> sync;

 public:
    MultiObjectDetector() : privateHandle("~"){
      privateHandle.param<string>("object_name", objectName, "balls");
      ROS_DEBUG("Detecting blobs with object name %s", objectName.c_str());

      // Publish the object location
      ros::SubscriberStatusCallback connectCB = boost::bind(&MultiObjectDetector::startListening, this);
      ros::SubscriberStatusCallback disconnectCB = boost::bind(&MultiObjectDetector::stopListening, this);

      pub = nh.advertise<position_tracker::DetectedObjects>("object_locations/" + objectName, 1, connectCB, disconnectCB);
      markerPub = nh.advertise<visualization_msgs::MarkerArray>("object_locations/markers", 1, connectCB, disconnectCB);
      ROS_DEBUG("Initialization of object detector complete");
    }
    
 private:
    void stopListening(){
      if(pub.getNumSubscribers() == 0 && markerPub.getNumSubscribers() == 0){
        ROS_DEBUG("Stopping listeners for multi object detector");
        if(blobsSub.get()){
          blobsSub->unsubscribe();
        }
        if(depthPointsSub.get()){
          depthPointsSub->unsubscribe();
        }
      }
    }

    void startListening(){
      if(pub.getNumSubscribers() + markerPub.getNumSubscribers() != 1){
        return;
      }

      ROS_DEBUG("Starting to listen for blob messages");
 
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
        sync.reset(new BlobCloudSync(BlobCloudSyncPolicy(10), *blobsSub, *depthPointsSub));
      
        sync->registerCallback(boost::bind(&MultiObjectDetector::finalBlobCallback, this, _1, _2));
      }
      ROS_DEBUG("Registration for blob events complete.");
    }

    void finalBlobCallback(const cmvision::BlobsConstPtr& blobsMsg, const sensor_msgs::PointCloud2ConstPtr& depthPointsMsg){
      ROS_DEBUG("Received a blobs message @ %f", ros::Time::now().toSec());

      // Check if there are detected blobs.
      if(blobsMsg->blobs.size() == 0){
        ROS_DEBUG("No blobs detected");
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
      position_tracker::DetectedObjectsPtr objects(new position_tracker::DetectedObjects);;
      visualization_msgs::MarkerArrayPtr markers(new visualization_msgs::MarkerArray);

      if(!tf.waitForTransform("/base_footprint", "/wide_stereo_optical_frame", depthPointsMsg->header.stamp, ros::Duration(5.0))){
        ROS_INFO("Transform from wide_stereo_optical_frame to base_footprint is not yet available");
        return;
      }

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
        tf.transformPoint(resultPointMap.header.frame_id, resultPoint, resultPointMap);
        objects->positions.push_back(resultPointMap);

        // TODO: Move to separate function 
        if(markerPub.getNumSubscribers() > 0){
          visualization_msgs::Marker marker;
          marker.id = i;
          marker.ns = "position_tracker";
          marker.action = visualization_msgs::Marker::ADD;
          marker.type = visualization_msgs::Marker::SPHERE;
          marker.header = resultPointMap.header;
          marker.action = visualization_msgs::Marker::ADD;
          marker.pose.position = resultPointMap.point;
          marker.pose.orientation.x = 0;
          marker.pose.orientation.y = 0;
          marker.pose.orientation.z = 0;
          marker.pose.orientation.w = 1;
          marker.color.a = 1;
          marker.color.r = 0;
          marker.color.g = 1;
          marker.color.b = 0;
          marker.scale.x = marker.scale.y = marker.scale.z = 0.04;
          markers->markers.push_back(marker);
        }
     }
   
     // Publish the markers message  
     if(markerPub.getNumSubscribers() > 0){
       markerPub.publish(markers);
     }

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
      pcl::VoxelGrid<pcl::PointXYZ> vg;
      pcl::PointCloud<pcl::PointXYZ>::Ptr allBlobsFiltered(new pcl::PointCloud<pcl::PointXYZ>);
      vg.setInputCloud(allBlobs);
      vg.setLeafSize(VOXEL_LEAF_SIZE_M, VOXEL_LEAF_SIZE_M, VOXEL_LEAF_SIZE_M);
      vg.filter(*allBlobsFiltered);
      allBlobs = allBlobsFiltered;
 #endif

      std::vector<pcl::PointIndices> clusterIndices;
      pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
      ec.setClusterTolerance(CLUSTER_DISTANCE_TOLERANCE_M);
      ec.setMinClusterSize(MIN_CLUSTER_SIZE);
      ec.setMaxClusterSize(MAX_CLUSTER_SIZE);
      ec.setInputCloud(allBlobs);
      ec.extract(clusterIndices);
      
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

