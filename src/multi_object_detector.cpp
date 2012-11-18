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

using namespace std;

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
typedef PointCloud::ConstPtr PointCloudConstPtr;
typedef PointCloud::Ptr PointCloudPtr;

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

      const vector<PointCloudConstPtr> blobClouds = splitBlobs(depthCloud, blobsMsg);
      if(blobClouds.size() == 0){
        ROS_INFO("No blobs to use for centroid detection");
        return;
      }

      // Iterate over each detected blob and determine it's centroid.
      position_tracker::DetectedObjects objects;
      for(unsigned int i = 0; i < blobClouds.size(); ++i){
        Eigen::Vector4f centroid;
        pcl::compute3DCentroid(*blobClouds[i], centroid);

        // Convert the centroid to a point stamped
        geometry_msgs::PointStamped resultPoint;
        resultPoint.header.frame_id = "wide_stereo_optical_frame";
        resultPoint.header.stamp = depthPointsMsg->header.stamp;

        // Convert the centroid to a geometry msg point
        resultPoint.point.x = centroid[0];
        resultPoint.point.y = centroid[1];
        resultPoint.point.z = centroid[2];

        geometry_msgs::PointStamped resultPointMap;
        resultPointMap.header.frame_id = "/map";
        resultPointMap.header.stamp = depthPointsMsg->header.stamp;
        tf.transformPoint("/map", resultPoint, resultPointMap);
        objects.positions.push_back(resultPointMap);
     }

      // Broadcast the result
      pub.publish(objects);
    }

    const vector<PointCloudConstPtr> splitBlobs(const pcl::PointCloud<pcl::PointXYZ>& depthCloud, const cmvision::BlobsConstPtr& blobsMsg){
         
       vector<PointCloudConstPtr> results;

       // Iterate over all the blobs and create depth point clouds for any matching objects.
       for(unsigned int k = 0; k < blobsMsg->blobs.size(); ++k){
          const cmvision::Blob blob = blobsMsg->blobs[k];
          if(objectName.size() > 0 && objectName != blob.colorName){
            continue;
          }

          PointCloudPtr splitBlob(new PointCloud);
          splitBlob->header = depthCloud.header;
          splitBlob->is_dense = false;
          splitBlob->height = 1;
          for(unsigned int i = blob.left; i <= blob.right; ++i){
            for(unsigned int j = blob.top; j <= blob.bottom; ++j){
              pcl::PointXYZ point = depthCloud.points.at(j * blobsMsg->image_width + i);
              splitBlob->push_back(point);
            }
          }
          results.push_back(splitBlob);
      }
    
      return results;
    }
};

int main(int argc, char **argv){
  ros::init(argc, argv, "multi_object_detector");
  MultiObjectDetector mobd;
  ros::spin();
  return 0;
}

