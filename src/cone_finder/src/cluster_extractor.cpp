#include <iostream>
#include <fstream>
#include <memory>
#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

#include <Eigen/Dense>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/PointStamped.h>

#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/common/centroid.h>

#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>

namespace ClusterExtractorNS {
class ClusterExtractor : public nodelet::Nodelet
{

private:
    ros::NodeHandle n_;
    ros::Subscriber cloud_sub;
    ros::Publisher cloud_pub;
    unsigned int seq = 0;

public:
    virtual void onInit() 
    {
        NODELET_DEBUG("Creating subscribers and publishers");
        
        n_ = getPrivateNodeHandle();
        cloud_sub = n_.subscribe("input", 10, &ClusterExtractor::cloudcb, this);
        cloud_pub = n_.advertise<sensor_msgs::PointCloud2>("/euclidean_clustering/clusters", 1);
    }

    // this function gets called every time new pcl data comes in
    void cloudcb(const sensor_msgs::PointCloud2ConstPtr &scan)
    {
        
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
        // Convert to pcl
        pcl::fromROSMsg(*scan, *cloud);
       
        NODELET_DEBUG("Begin cluster extraction %d", (int) cloud->size());

        // create a vector for storing the indices of the clusters
        std::vector<pcl::PointIndices> cluster_indices;

        // setup extraction:
        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
        pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
        ec.setClusterTolerance(1); // cm
        ec.setMinClusterSize(3);
        ec.setMaxClusterSize(5000);
        ec.setSearchMethod(tree);
        ec.setInputCloud(cloud);
        // perform cluster extraction
        ec.extract(cluster_indices);


	    int number_clusters = (int) cluster_indices.size();
        NODELET_DEBUG("Number of clusters found: %d", number_clusters);
            
        // We will fill this cloud with centroids
        pcl::PointCloud<pcl::PointXYZ>::Ptr centroids (new pcl::PointCloud<pcl::PointXYZ>);

	    // Each index represents one cluster
	    // Iterate through indexes
        for(const auto & indexes : cluster_indices)
        {
            // Convert indexes to points and store in curr_cluster
            pcl::PointCloud<pcl::PointXYZ>::Ptr curr_cluster (new pcl::PointCloud<pcl::PointXYZ>);
            for (const auto & point : indexes.indices)
                curr_cluster->points.push_back(cloud->points[point]);
                
            curr_cluster->width = curr_cluster->points.size();
            curr_cluster->height = 1;
            curr_cluster->is_dense = true;

            // compute centroid
            Eigen::Vector4f centroid;
            pcl::compute3DCentroid(*curr_cluster, centroid);
            auto centroid_point = pcl::PointXYZ(centroid(0), centroid(1), centroid(2));
            centroids->points.push_back(centroid_point);
        }
        
        sensor_msgs::PointCloud2 publish_cloud;
        pcl::toROSMsg(*centroids.get(), publish_cloud);
        publish_cloud.header.frame_id = "velodyne";
        publish_cloud.header.seq = seq++;
        publish_cloud.header.stamp = scan->header.stamp;

        cloud_pub.publish(boost::make_shared<sensor_msgs::PointCloud2>(publish_cloud));
    }
};

}

PLUGINLIB_EXPORT_CLASS(ClusterExtractorNS::ClusterExtractor, nodelet::Nodelet);
