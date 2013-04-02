#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <tf/transform_listener.h>
#include <cmvision/Blobs.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <message_filters/time_synchronizer.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/features/feature.h>
#include <cmath>
#include <boost/math/constants/constants.hpp>

using namespace std;

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
typedef PointCloud::ConstPtr PointCloudConstPtr;
typedef PointCloud::Ptr PointCloudPtr;

class ObjectDetector {
  private:
    ros::NodeHandle nh;
    ros::NodeHandle privateHandle;
    tf::TransformListener tf;
    std::string objectName;
    
    auto_ptr<message_filters::Subscriber<cmvision::Blobs> > blobsSub;
    auto_ptr<message_filters::Subscriber<sensor_msgs::PointCloud2> > depthPointsSub;
    
    // Publisher for the resulting position event.
    ros::Publisher pub; 
    
    auto_ptr<message_filters::TimeSynchronizer<cmvision::Blobs, sensor_msgs::PointCloud2> > sync;
 public:
    ObjectDetector() : privateHandle("~"){
      
      privateHandle.getParam("object_name", objectName);
      ROS_INFO("Detecting blob with object name %s", objectName.c_str());

      // Publish the object location
      ros::SubscriberStatusCallback cb = boost::bind(&ObjectDetector::connectCB, this);
      pub = nh.advertise<geometry_msgs::PoseStamped>("object_location/" + objectName, 1, cb, cb);
      ROS_INFO("Initialization of object detector complete");
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
      ROS_INFO("Stopping listeners for object detector");
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
      
        sync->registerCallback(boost::bind(&ObjectDetector::finalBlobCallback, this, _1, _2));
      }
      ROS_INFO("Registration for blob events complete.");
    }

    void finalBlobCallback(const cmvision::BlobsConstPtr& blobsMsg, const sensor_msgs::PointCloud2ConstPtr& depthPointsMsg){

      // Check if there is a detected blob.
      if(blobsMsg->blobs.size() == 0){
        ROS_INFO("No blobs detected");
        return;
      }
      
      pcl::PointCloud<pcl::PointXYZ> depthCloud;
      pcl::fromROSMsg(*depthPointsMsg, depthCloud);

      pcl::ModelCoefficients coefficients;
      pcl::PointIndices inliers;

      const PointCloudConstPtr blobsCloud = detectPlane(depthCloud, blobsMsg, inliers, coefficients);
      if(blobsCloud.get() == NULL || inliers.indices.size() == 0){
        ROS_INFO("No inliers to use for centroid detection");
        return;
      }

      Eigen::Vector4f centroid;
      pcl::compute3DCentroid(*blobsCloud, inliers, centroid);

      geometry_msgs::Vector3Stamped normalInImageFrame;
      normalInImageFrame.vector.x = coefficients.values[0];
      normalInImageFrame.vector.y = coefficients.values[1];
      normalInImageFrame.vector.z = coefficients.values[2];
      normalInImageFrame.header.frame_id = "wide_stereo_optical_frame";
      normalInImageFrame.header.stamp = depthPointsMsg->header.stamp;

      // Calculate the yaw in the base footprint frame to ensure the z axis remains vertical in the yaw calculation step.
      tf.waitForTransform("wide_stereo_optical_frame", "/base_footprint", depthPointsMsg->header.stamp, ros::Duration(10.0));

      geometry_msgs::Vector3Stamped normalStamped;
      tf.transformVector("/base_footprint", normalInImageFrame, normalStamped);
      
      double yaw = atan(normalStamped.vector.x / -normalStamped.vector.y);
      yaw += boost::math::constants::pi<double>() / 2.0;
      
      // Ensure the yaw is the same direction as the robot.
      if(yaw < -boost::math::constants::pi<double>() / 2.0){
        yaw += boost::math::constants::pi<double>();
      } else if (yaw > boost::math::constants::pi<double>() / 2.0){
        yaw -= boost::math::constants::pi<double>();
      }

      geometry_msgs::QuaternionStamped qInBaseFootprint;
      qInBaseFootprint.header.frame_id = "base_footprint";
      qInBaseFootprint.header.stamp = depthPointsMsg->header.stamp;
      qInBaseFootprint.quaternion = tf::createQuaternionMsgFromRollPitchYaw(0, 0, yaw);

      // Convert orientation to map frame.
      tf.waitForTransform(qInBaseFootprint.header.frame_id, "map", qInBaseFootprint.header.stamp, ros::Duration(10));
      geometry_msgs::QuaternionStamped q;
      tf.transformQuaternion("map", qInBaseFootprint, q);

      // Convert the centroid and quaternion to a PoseStamped
      geometry_msgs::PoseStamped resultPose;
      resultPose.header.frame_id = "wide_stereo_optical_frame";
      resultPose.header.stamp = depthPointsMsg->header.stamp;

      // Convert the centroid to a geometry msg point
      resultPose.pose.position.x = centroid[0];
      resultPose.pose.position.y = centroid[1];
      resultPose.pose.position.z = centroid[2];
      tf::quaternionTFToMsg(tf::Quaternion::getIdentity(), resultPose.pose.orientation);

      geometry_msgs::PoseStamped resultPoseMap;
      resultPoseMap.header.frame_id = "/map";
      resultPoseMap.header.stamp = depthPointsMsg->header.stamp;
      tf.transformPose("/map", resultPose, resultPoseMap);
      resultPoseMap.pose.orientation = q.quaternion;

      // Broadcast the result
      pub.publish(resultPoseMap);
    }

    const PointCloudPtr detectPlane(const pcl::PointCloud<pcl::PointXYZ>& depthCloud, const cmvision::BlobsConstPtr& blobsMsg, pcl::PointIndices& inliers, pcl::ModelCoefficients& coefficients){
         
       PointCloudPtr depthCloudFiltered(new PointCloud);

       depthCloudFiltered->header = depthCloud.header;
       depthCloudFiltered->is_dense = false;
       depthCloudFiltered->height = 1;

       for(unsigned int k = 0; k < blobsMsg->blobs.size(); ++k){
          const cmvision::Blob blob = blobsMsg->blobs[k];
          if(objectName.size() > 0 && objectName != blob.colorName){
            continue;
          }
          for(unsigned int i = blob.left; i <= blob.right; ++i){
            for(unsigned int j = blob.top; j <= blob.bottom; ++j){
              pcl::PointXYZ point = depthCloud.points.at(j * blobsMsg->image_width + i);
              depthCloudFiltered->push_back(point);
            }
          }
       }
        
       if(depthCloudFiltered->points.size() == 0){
          ROS_INFO("No blob points found.");
          return depthCloudFiltered;
       }

        // Create the segmentation object
        pcl::SACSegmentation<pcl::PointXYZ> seg;
  
        // Optional
        seg.setOptimizeCoefficients(true);
        
        // Mandatory
        seg.setModelType(pcl::SACMODEL_PLANE);
        seg.setMethodType(pcl::SAC_RANSAC);
        seg.setDistanceThreshold(0.01);

        seg.setInputCloud(depthCloudFiltered);
        seg.segment(inliers, coefficients);

        if(inliers.indices.size () == 0){
          ROS_INFO("Could not estimate a planar model for the given dataset.");
          return depthCloudFiltered;
        }

        return depthCloudFiltered;
    }
};

int main(int argc, char **argv){
  ros::init(argc, argv, "object_detector");
  ObjectDetector obd;
  ros::spin();
  return 0;
}

