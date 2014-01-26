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
#include <pcl/common/geometry.h>
#include <sensor_msgs/CameraInfo.h>
#include <image_geometry/pinhole_camera_model.h>
#include <tf2/LinearMath/btVector3.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <limits>
#include <google/profiler.h>

namespace {

#define ENABLE_PROFILING 0

using namespace std;
using namespace pcl;


typedef boost::function<PointXYZ(const unsigned int, const unsigned int, unsigned int, unsigned int)> ResolverFunction;

inline Eigen::Vector3f operator-(const PointXYZ& p1, const PointXYZ& p2) {
    return Eigen::Vector3f(p1.x - p2.x, p1.y - p2.y, p1.z - p2.z);
}

typedef PointCloud<PointXYZ> PointCloudXYZ;
typedef PointCloudXYZ::ConstPtr PointCloudXYZConstPtr;
typedef PointCloudXYZ::Ptr PointCloudXYZPtr;
typedef message_filters::sync_policies::ApproximateTime<cmvision::Blobs, sensor_msgs::PointCloud2> BlobCloudSyncPolicy;
typedef message_filters::Synchronizer<BlobCloudSyncPolicy> BlobCloudSync;

typedef message_filters::sync_policies::ApproximateTime<cmvision::Blobs, sensor_msgs::CameraInfo> BlobCameraSyncPolicy;
typedef message_filters::Synchronizer<BlobCameraSyncPolicy> BlobCameraSync;

static const double DOG_HEIGHT_DEFAULT = 0.1;

class MultiObjectDetector {
private:
    ros::NodeHandle nh;
    ros::NodeHandle privateHandle;
    tf::TransformListener tf;
    std::string objectName;
    std::string outputFrame;

    auto_ptr<message_filters::Subscriber<cmvision::Blobs> > blobsSub;
    auto_ptr<message_filters::Subscriber<sensor_msgs::PointCloud2> > depthPointsSub;
    auto_ptr<message_filters::Subscriber<sensor_msgs::CameraInfo> > cameraInfoSub;

    double clusterDistanceTolerance;
    double voxelLeafSize;
    double dogHeight;

    int minClusterSize;
    int maxClusterSize;
    bool stereoCameraMode;

    // Publisher for the resulting position event.
    ros::Publisher pub;

    // Publish for visualization
    ros::Publisher markerPub;

    auto_ptr<BlobCloudSync> sync;

    auto_ptr<BlobCameraSync> blobCameraSync;

public:
    MultiObjectDetector() :
        privateHandle("~") {
        privateHandle.param<string>("object_name", objectName, "dog");
        ROS_DEBUG("Detecting blobs with object name %s", objectName.c_str());

        privateHandle.param<double>("cluster_distance_tolerance", clusterDistanceTolerance, 0.1);
        privateHandle.param<double>("voxel_leaf_size", voxelLeafSize, 0.0);
        privateHandle.param<int>("min_cluster_size", minClusterSize, 75);
        privateHandle.param<int>("max_cluster_size", maxClusterSize, 25000);
        privateHandle.param<bool>("stereo_camera_mode", stereoCameraMode, true);
        privateHandle.param<string>("output_frame", outputFrame, "/base_footprint");
        nh.param("dog_height", dogHeight, DOG_HEIGHT_DEFAULT);

        // Publish the object location
        ros::SubscriberStatusCallback connectCB = boost::bind(&MultiObjectDetector::startListening,
                this);
        ros::SubscriberStatusCallback disconnectCB = boost::bind(
                &MultiObjectDetector::stopListening, this);

        pub = nh.advertise<position_tracker::DetectedObjects>("object_locations/" + objectName, 1,
                connectCB, disconnectCB);
        markerPub = nh.advertise<visualization_msgs::MarkerArray>("object_locations/markers", 1,
                connectCB, disconnectCB);
        ROS_DEBUG("Initialization of object detector complete");
    }

private:
    void stopListening() {
        if (pub.getNumSubscribers() == 0 && markerPub.getNumSubscribers() == 0) {
            ROS_DEBUG("Stopping listeners for multi object detector");
            if (blobsSub.get()) {
                blobsSub->unsubscribe();
            }
            if (depthPointsSub.get()) {
                depthPointsSub->unsubscribe();
            }
            if (cameraInfoSub.get()) {
                cameraInfoSub->unsubscribe();
            }
        }
    }

    void startListening() {
        if (pub.getNumSubscribers() + markerPub.getNumSubscribers() != 1) {
            return;
        }

        ROS_DEBUG("Starting to listen for blob messages");

        if (blobsSub.get() == NULL) {
            // Listen for message from cm vision when it sees an object.
            blobsSub.reset(new message_filters::Subscriber<cmvision::Blobs>(nh, "/blobs", 10));
        }
        else {
            blobsSub->subscribe();
        }

        if (stereoCameraMode) {
            // Listen for the depth messages
            if (depthPointsSub.get() == NULL) {
                depthPointsSub.reset(
                        new message_filters::Subscriber<sensor_msgs::PointCloud2>(nh, "/points",
                                3));
            }
            else {
                depthPointsSub->subscribe();
            }
            if (sync.get() == NULL) {
                // Sync the two messages
                sync.reset(new BlobCloudSync(BlobCloudSyncPolicy(10), *blobsSub, *depthPointsSub));

                sync->registerCallback(
                        boost::bind(&MultiObjectDetector::finalBlobCallback, this, _1, _2));
            }
        }
        else {
            // Listen for the camera info messages
            if (cameraInfoSub.get() == NULL) {
                cameraInfoSub.reset(
                        new message_filters::Subscriber<sensor_msgs::CameraInfo>(nh,
                                "/camera_info_in", 10));
            }
            else {
                cameraInfoSub->subscribe();
            }
            if (blobCameraSync.get() == NULL) {
                // Sync the two messages
                blobCameraSync.reset(
                        new BlobCameraSync(BlobCameraSyncPolicy(10), *blobsSub, *cameraInfoSub));

                blobCameraSync->registerCallback(
                        boost::bind(&MultiObjectDetector::finalBlobCameraInfoCallback, this, _1,
                                _2));
            }
        }
        ROS_DEBUG("Registration for blob events complete.");
    }

    btVector3 intersection(const btVector3& n, const btVector3& p0, const btVector3& l,
            const btVector3& l0) {
        // Make sure everything is normalized
        assert(n.length() - 1 < 1e-6);
        assert(l.length() - 1 < 1e-6);

        btScalar denom = n.dot(l);
        if (fabs(denom) < 1e-6) {
            return btVector3(numeric_limits<double>::quiet_NaN(),
                    numeric_limits<double>::quiet_NaN(), numeric_limits<double>::quiet_NaN());
        }

        btScalar t = n.dot(p0 - l0) / denom;
        if (t < 0) {
            // Intersection behind the camera
            return btVector3(numeric_limits<double>::quiet_NaN(),
                    numeric_limits<double>::quiet_NaN(), numeric_limits<double>::quiet_NaN());
        }
        // Now project the vector l to the plane.
        return l0 + (l * t);
    }

    void finalBlobCameraInfoCallback(const cmvision::BlobsConstPtr& blobsMsg,
            const sensor_msgs::CameraInfoConstPtr& cameraInfo) {
        ROS_DEBUG("Received a blobs containing %lu blobs and camerainfo messages @ %f", blobsMsg->blobs.size(), ros::Time::now().toSec());

        image_geometry::PinholeCameraModel cameraModel;
        cameraModel.fromCameraInfo(*cameraInfo);

        // Lookup the translation from the ground to the camera
        tf.waitForTransform(cameraInfo->header.frame_id, "/base_footprint", blobsMsg->header.stamp, ros::Duration(5.0));
        tf::StampedTransform cameraOrigin;
        try {
          tf.lookupTransform("/base_footprint", cameraInfo->header.frame_id, cameraInfo->header.stamp, cameraOrigin);
        }
        catch(tf::TransformException& e){
          ROS_WARN("Failed to lookup transform: %s", e.what());
          return;
        }

        ROS_DEBUG("Camera origin in base frame is: %f, %f, %f", cameraOrigin.getOrigin().x(), cameraOrigin.getOrigin().y(), cameraOrigin.getOrigin().z());
        
        ResolverFunction func = boost::bind(&MultiObjectDetector::resolveFromCameraInfo, this, _1,
                _2, _3, _4, cameraModel, cameraOrigin);

        // The result is already transformed
        std_msgs::Header resultHeader;
        resultHeader.frame_id = "/base_footprint";
        resultHeader.stamp = blobsMsg->header.stamp;
        finalBlobCallbackHelper(blobsMsg, resultHeader, func, 1 /* stride */);
    }

    void finalBlobCallback(const cmvision::BlobsConstPtr& blobsMsg,
            const sensor_msgs::PointCloud2ConstPtr& depthPointsMsg) {

        ROS_DEBUG("Received blobs and depth cloud messages @ %f", ros::Time::now().toSec());

        PointCloudXYZ depthCloud;
        fromROSMsg(*depthPointsMsg, depthCloud);
        assert(depthCloud.points.size() == blobsMsg->image_width * blobsMsg->image_height);
        assert(depthPointsMsg->header.frame_id == blobsMsg->header.frame_id);

        ResolverFunction func = boost::bind(&MultiObjectDetector::resolveFromDepthMessage, this, _1,
                _2, _3, _4, depthCloud);
        finalBlobCallbackHelper(blobsMsg, depthPointsMsg->header, func, 1 /* stride */);
    }

    PointXYZ resolveFromCameraInfo(const unsigned int u, const unsigned int v,
            const unsigned int imageWidth, const unsigned int imageHeight,
            const image_geometry::PinholeCameraModel& cameraModel,
            const tf::StampedTransform& cameraToBaseFootprint) {

        cv::Point3d p = cameraModel.projectPixelTo3dRay(cv::Point2d(u, v));
        tf::Vector3 pVector(p.x, p.y, p.z);

        tf::Vector3 cameraOrigin = cameraToBaseFootprint.getOrigin();

        // Transform to base_footprint. We already waited for this transform previously.
        tf::Vector3 pInBase = cameraToBaseFootprint * pVector;
        
        // Now push out the z to the dog height
        const tf::Vector3 originToP = (pInBase - cameraOrigin).normalized();
        const tfScalar angle = originToP.angle(tf::Vector3(0, 0, -1));
        const tfScalar length = tfScalar(1) / tfCos(angle) * (cameraOrigin.z() - dogHeight);

        const tf::Vector3 adjusted = cameraOrigin + length * originToP;
        return PointXYZ(adjusted.x(), adjusted.y(), adjusted.z());
    }

    PointXYZ resolveFromDepthMessage(const unsigned int u, const unsigned int v,
            const unsigned int imageWidth, const unsigned int imageHeight,
            const PointCloudXYZ& depthCloud) {
        unsigned int index = u * imageWidth + v;
        if (index >= depthCloud.points.size()) {
            ROS_WARN("Depth cloud does not contain point for index %u", index);
            return PointXYZ(numeric_limits<double>::quiet_NaN(),
                    numeric_limits<double>::quiet_NaN(), numeric_limits<double>::quiet_NaN());
        }
        return depthCloud.points.at(index);
    }

    void finalBlobCallbackHelper(const cmvision::BlobsConstPtr& blobsMsg,
            const std_msgs::Header& header, const ResolverFunction& resolver,
            const unsigned int stride) {

        // Initialize the result message
        position_tracker::DetectedObjectsPtr objects(new position_tracker::DetectedObjects);
        objects->header.frame_id = outputFrame;
        objects->header.stamp = header.stamp;
        // Check if there are detected blobs.
        if (blobsMsg->blobs.size() == 0) {
            ROS_DEBUG("No blobs detected");
            publish(objects);
            return;
        }

        ROS_DEBUG("Depth or camera points frame is %s and blobsMsg frame is %s",
                header.frame_id.c_str(), blobsMsg->header.frame_id.c_str());

        PointCloudXYZPtr allBlobs;
        const vector<PointIndices> blobClouds = splitBlobs(resolver, blobsMsg, allBlobs, stride);

        if (blobClouds.size() == 0) {
            ROS_DEBUG("No blobs to use for centroid detection");
            publish(objects);
            return;
        }

        // Iterate over each detected blob and determine its centroid.
        if (!tf.waitForTransform(outputFrame, header.frame_id, header.stamp,
                ros::Duration(5.0))) {
            ROS_WARN("Transform from %s to %s is not yet available",
                    header.frame_id.c_str(), outputFrame.c_str());
            return;
        }

        for (unsigned int i = 0; i < blobClouds.size(); ++i) {
            Eigen::Vector4f centroid;
            ROS_DEBUG("Computing centroid for blob with %lu indices", blobClouds[i].indices.size());
            compute3DCentroid(*allBlobs, blobClouds[i].indices, centroid);

            // Convert the centroid to a point stamped
            geometry_msgs::PointStamped resultPoint;
            resultPoint.header.frame_id = header.frame_id;
            resultPoint.header.stamp = header.stamp;

            // Convert the centroid to a geometry msg point
            ROS_DEBUG("Computed centroid in frame %s with coordinates %f, %f, %f",
                    header.frame_id.c_str(), centroid[0], centroid[1], centroid[2]);

            resultPoint.point.x = centroid[0];
            resultPoint.point.y = centroid[1];
            resultPoint.point.z = centroid[2];

            geometry_msgs::PointStamped resultPointMap;
            resultPointMap.header.frame_id = outputFrame;
            resultPointMap.header.stamp = header.stamp;
            tf.transformPoint(resultPointMap.header.frame_id, resultPoint, resultPointMap);
            ROS_DEBUG("Transformed centroid to frame %s with coordinates %f %f %f",
                    resultPointMap.header.frame_id.c_str(), resultPointMap.point.x,
                    resultPointMap.point.y, resultPointMap.point.z);
            objects->positions.push_back(resultPointMap);
        }

        ROS_DEBUG("Publishing %lu detected objects", objects->positions.size());
        publish(objects);
    }

    void publish(const position_tracker::DetectedObjects::ConstPtr objects) {
        // Publish the markers message
        if (markerPub.getNumSubscribers() > 0) {
            publishMarkers(objects);
        }

        // Broadcast the result
        pub.publish(objects);
    }

    void publishMarkers(position_tracker::DetectedObjectsConstPtr objects) {
        visualization_msgs::MarkerArrayPtr markers(new visualization_msgs::MarkerArray);
        for (unsigned int i = 0; i < objects->positions.size(); ++i) {
            visualization_msgs::Marker marker;
            marker.id = i;
            marker.ns = "multi_object_detector" + nh.resolveName("/blobs");
            marker.action = visualization_msgs::Marker::ADD;
            marker.type = visualization_msgs::Marker::SPHERE;
            marker.header = objects->positions[i].header;
            marker.lifetime = ros::Duration(1);
            marker.action = visualization_msgs::Marker::ADD;
            marker.pose.position = objects->positions[i].point;
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

        markerPub.publish(markers);
    }

    const vector<PointIndices> splitBlobs(const ResolverFunction& resolver,
            const cmvision::BlobsConstPtr blobsMsg, PointCloudXYZPtr& allBlobsOut,
            const unsigned int stride) {

        // Iterate over all the blobs and create a single cloud of all points.
        // We will subdivide this blob again later.
        PointCloudXYZPtr allBlobs(new PointCloudXYZ);
        ROS_DEBUG("Blobs message has %lu blobs", blobsMsg->blobs.size());
        for (unsigned int k = 0; k < blobsMsg->blobs.size(); ++k) {
            const cmvision::Blob blob = blobsMsg->blobs[k];
            if (objectName.size() > 0 && objectName != blob.colorName) {
                ROS_DEBUG("Skipping blob named %s as it does not match", objectName.c_str());
                continue;
            }

            ROS_DEBUG("Blob image dimensions. Left %u right %u top %u bottom %u width %u height %u",
                    blob.left, blob.right, blob.top, blob.bottom, blobsMsg->image_width,
                    blobsMsg->image_height);
            for (unsigned int j = blob.top; j <= blob.bottom; j += stride) {
                for (unsigned int i = blob.left; i <= blob.right; i += stride) {
                    PointXYZ point = resolver(j, i, blobsMsg->image_width, blobsMsg->image_height);
                    allBlobs->points.push_back(point);
                }
            }
        }

        allBlobs->is_dense = false;
        allBlobs->width = allBlobs->points.size();
        allBlobs->height = 1;

        if (voxelLeafSize > 0) {
            // Use a voxel grid to downsample the input to a 1cm grid.
            VoxelGrid<PointXYZ> vg;
            PointCloudXYZPtr allBlobsFiltered(new PointCloudXYZ);
            vg.setInputCloud(allBlobs);
            vg.setLeafSize(voxelLeafSize, voxelLeafSize, voxelLeafSize);
            vg.filter(*allBlobsFiltered);
            allBlobs = allBlobsFiltered;
        }

        vector<int> removedIndices;
        pcl::removeNaNFromPointCloud(*allBlobs, *allBlobs, removedIndices);
        if (allBlobs->size() == 0) {
            ROS_INFO("No remaining blob points after removing NaNs");
            return std::vector<PointIndices>();
        }
        ROS_DEBUG("Points available for blob position. Extracting clusters.");

        // Creating the KdTree object for the search method of the extraction
        pcl::search::KdTree<PointXYZ>::Ptr tree(new pcl::search::KdTree<PointXYZ>);
        tree->setInputCloud(allBlobs);

        std::vector<PointIndices> clusterIndices;
        EuclideanClusterExtraction<PointXYZ> ec;
        ec.setClusterTolerance(clusterDistanceTolerance);
        ec.setMinClusterSize(minClusterSize);
        ec.setMaxClusterSize(maxClusterSize);
        ec.setSearchMethod(tree);
        ec.setInputCloud(allBlobs);
        ec.extract(clusterIndices);
        ROS_DEBUG("Extracted %lu clusters from blobs", clusterIndices.size());
        allBlobsOut = allBlobs;
        return clusterIndices;
    }
};

}
int main(int argc, char **argv) {
    ros::init(argc, argv, "multi_object_detector");
#if ENABLE_PROFILING == 1
    ProfilerStart(("/tmp/" + ros::this_node::getName() +".prof").c_str());
#endif
    MultiObjectDetector mobd;
    ros::spin();
#if ENABLE_PROFILING == 1
    ProfilerStop();
    ProfilerFlush();
#endif
    return 0;
}

