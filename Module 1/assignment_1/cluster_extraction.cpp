#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/crop_box.h>
#include "Renderer.hpp"
#include <pcl/point_types.h>
#include <pcl/common/common.h>
#include <chrono>
#include <unordered_set>
#include "tree_utilities.hpp"
#include <boost/filesystem.hpp>
#include <thread>

#define USE_PCL_LIBRARY
using namespace lidar_obstacle_detection;

typedef std::unordered_set<int> my_visited_set_t;

//This function sets up the custom kdtree using the point cloud
void setupKdtree(typename pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, my_pcl::KdTree* tree, int dimension)
{
    //insert point cloud points into tree
    for (int i = 0; i < cloud->size(); ++i)
    {
        tree->insert({cloud->at(i).x, cloud->at(i).y, cloud->at(i).z}, i);
    }
}

/*
OPTIONAL
This function computes the nearest neighbors and builds the clusters
    - Input:
        + cloud: Point cloud to be explored
        + target_ndx: i-th point to visit
        + tree: kd tree for searching neighbors
        + distanceTol: Distance tolerance to build the clusters 
        + visited: Visited points
        + cluster: Here we add points that will represent the cluster
        + max: Max cluster size
    - Output:
        + visited: already visited points
        + cluster: at the end of this function we will have one cluster
*/
void proximity(typename pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int target_ndx, my_pcl::KdTree* tree, float distanceTol, my_visited_set_t& visited, std::vector<int>& cluster, int max)
{
	if (cluster.size() < max)
    {
        cluster.push_back(target_ndx);
        visited.insert(target_ndx);

        std::vector<float> point {cloud->at(target_ndx).x, cloud->at(target_ndx).y, cloud->at(target_ndx).z};
    
        // get all neighboring indices of point
        std::vector<int> neighborNdxs = tree->search(point, distanceTol);

        for (int neighborNdx : neighborNdxs)
        {
            // if point was not visited
            if (visited.find(neighborNdx) == visited.end())
            {
                proximity(cloud, neighborNdx, tree, distanceTol, visited, cluster, max);
            }

            if (cluster.size() >= max)
            {
                return;
            }
        }
    }
}

/*
OPTIONAL
This function builds the clusters following a euclidean clustering approach
    - Input:
        + cloud: Point cloud to be explored
        + tree: kd tree for searching neighbors
        + distanceTol: Distance tolerance to build the clusters 
        + setMinClusterSize: Minimum cluster size
        + setMaxClusterSize: Max cluster size
    - Output:
        + cluster: at the end of this function we will have a set of clusters
TODO: Complete the function
*/
std::vector<pcl::PointIndices> euclideanCluster(typename pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, my_pcl::KdTree* tree, float distanceTol, int setMinClusterSize, int setMaxClusterSize)
{
    my_visited_set_t visited{}; //already visited points
    std::vector<pcl::PointIndices> clusters; // clusters to be built
    //for every point of the cloud
    for (int i = 0; i < cloud->points.size(); i++)
    {
        if (visited.find(i) == visited.end())
        {
            std::vector<int> cluster;
            proximity(cloud, i, tree, distanceTol, visited, cluster, setMaxClusterSize); 
            if (cluster.size() >= setMinClusterSize && cluster.size() <= setMaxClusterSize)
            {
                pcl::PointIndices point_indices;
                point_indices.indices = cluster;
                clusters.push_back(point_indices);
            }
        }
    }
    return clusters;
}



void 
ProcessAndRenderPointCloud (Renderer& renderer, pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud)
{
    // TODO: 1) Downsample the dataset 
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_f (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane (new pcl::PointCloud<pcl::PointXYZ> ());
    // added from here
    std::cerr << "PointCloud before filtering: " << cloud->width * cloud->height 
    << " data points (" << pcl::getFieldsList (*cloud) << ")." << std::endl;
    // Create the filtering object
    pcl::VoxelGrid<pcl::PointXYZ> sor;
    sor.setInputCloud (cloud); //loading the point cloud into sor object
    sor.setLeafSize (0.1f, 0.1f, 0.1f); //this value defines how much the PC is filtered   bigger the leaf size..more points filtered
    sor.filter (*cloud_filtered); //writting the output to cloud_filtered object
    // for printing the output
    std::cerr << "PointCloud after filtering: " << cloud_filtered->width * cloud_filtered->height 
    << " data points (" << pcl::getFieldsList (*cloud_filtered) << ")." << std::endl;
    // writting the output to the file.
    pcl::PCDWriter writer;
    std::stringstream ss;
    ss << "cloud_filtered.pcd";
    writer.write<pcl::PointXYZ> (ss.str (), *cloud_filtered, false); 

    // 2) here we crop the points that are far away from us, in which we are not interested
    pcl::CropBox<pcl::PointXYZ> cb(true);
    cb.setInputCloud(cloud_filtered);
    cb.setMin(Eigen::Vector4f (-20, -6, -2, 1));
    cb.setMax(Eigen::Vector4f ( 30, 7, 5, 1));
    cb.filter(*cloud_filtered); 

    // TODO: 3) Segmentation and apply RANSAC

    pcl::SACSegmentation<pcl::PointXYZ> seg;
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    seg.setOptimizeCoefficients (true);
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setMaxIterations (1000);
    seg.setDistanceThreshold (0.2); //0.2 for dataset1  0.6 for dataset2

    
    // TODO: 4) iterate over the filtered cloud, segment and remove the planar inliers 

    int i=0, nr_points = (int) cloud_filtered->size ();
    // While there are more than ground plane...
    while (cloud_filtered->size () > 0.3 * nr_points)
    {
        // Segment the largest planar component from the remaining cloud
        seg.setInputCloud (cloud_filtered);
        seg.segment (*inliers, *coefficients);
        if (inliers->indices.size () == 0)
        {
            std::cout << "Could not estimate a planar model for the given dataset." << std::endl;
            break;
        }

        // Extract the planar inliers from the input cloud
        pcl::ExtractIndices<pcl::PointXYZ> extract;
        extract.setInputCloud (cloud_filtered);
        extract.setIndices (inliers);
        extract.setNegative (false);

        // Get the points associated with the planar surface
        extract.filter (*cloud_plane);
        std::cout << "PointCloud representing the planar component: " << cloud_plane->size () << " data points." << std::endl;

        extract.setNegative (true); // Remove the planar inliers, extract the rest
        extract.filter (*cloud_f);
        *cloud_filtered = *cloud_f;
    }

    // TODO: 5) Create the KDTree and the vector of PointIndices

    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud (cloud_filtered); 
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;

    ec.setMinClusterSize (20); 
    ec.setMaxClusterSize (50000);
    ec.setSearchMethod (tree);
    ec.setInputCloud (cloud_filtered);

    // TODO: 6) Set the spatial tolerance for new cluster candidates (pay attention to the tolerance!!!)
    std::vector<pcl::PointIndices> cluster_indices;

    ec.setClusterTolerance (0.4); // 0.4 for dataset 1  0.6 for dataset2
    ec.extract (cluster_indices);

    #if False

    // PCL functions

    // HERE 6)

    #else
    // using optional Euclidean clustering function
    std::cout << "Using Euclidean clustering" << std::endl;
    my_pcl::KdTree treeM;
    treeM.set_dimension(3);
    setupKdtree(cloud_filtered, &treeM, 3);

    // Use your euclideanCluster function here
    float clusterTolerance = 0.4; // Your cluster tolerance value
    int setMinClusterSize = 20;   // Your minimum cluster size
    int setMaxClusterSize = 50000; // Your maximum cluster size

    cluster_indices = euclideanCluster(cloud_filtered, &treeM, clusterTolerance, setMinClusterSize, setMaxClusterSize);
    #endif

    std::vector<Color> colors = {Color(1,0,0), Color(1,1,0), Color(0,0,1), Color(1,0,1), Color(0,1,1)};


    /**Now we extracted the clusters out of our point cloud and saved the indices in cluster_indices. 

    To separate each cluster out of the vector<PointIndices> we have to iterate through cluster_indices, create a new PointCloud for each entry and write all points of the current cluster in the PointCloud.
    Compute euclidean distance
    **/
    int j = 0;
    int clusterId = 0;
    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
        for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
        cloud_cluster->push_back ((*cloud_filtered)[*pit]); 
        cloud_cluster->width = cloud_cluster->size ();
        cloud_cluster->height = 1;
        cloud_cluster->is_dense = true;

        // renderer.RenderPointCloud(cloud,"originalCloud"+std::to_string(clusterId),colors[2]);
        // TODO: 7) render the cluster and plane without rendering the original cloud 
        //<-- here
        //----------
        renderer.RenderPointCloud(cloud_cluster, "cloud_cluster" + std::to_string(clusterId), colors[clusterId % colors.size()]);
        

        //Here we create the bounding box on the detected clusters
        pcl::PointXYZ minPt, maxPt;
        pcl::getMinMax3D(*cloud_cluster, minPt, maxPt);

        Box box{minPt.x, minPt.y, minPt.z,
        maxPt.x, maxPt.y, maxPt.z};
        renderer.RenderBox(box, j);

        std::stringstream ss;
        ss << "cloud_cluster_" << j << ".pcd";
        writer.write<pcl::PointXYZ> (ss.str (), *cloud_cluster, false); //*
        
        ++clusterId;
        j++;
    }  
    renderer.RenderPointCloud(cloud_plane,"plane",colors[4]);

}


int main(int argc, char* argv[])
{
    Renderer renderer;
    renderer.InitCamera(CameraAngle::XY);
    // Clear viewer
    renderer.ClearViewer();

    pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud(new pcl::PointCloud<pcl::PointXYZ>);

    std::vector<boost::filesystem::path> stream(boost::filesystem::directory_iterator{"/home/asif/Documents/assignment_1/build/dataset_1"},
                                                boost::filesystem::directory_iterator{});

    // sort files in ascending (chronological) order
    std::sort(stream.begin(), stream.end());

    auto streamIterator = stream.begin();

    while (not renderer.WasViewerStopped())
    {
	
        renderer.ClearViewer();

        pcl::PCDReader reader;
        reader.read (streamIterator->string(), *input_cloud);
        auto startTime = std::chrono::steady_clock::now();

        ProcessAndRenderPointCloud(renderer,input_cloud);
        auto endTime = std::chrono::steady_clock::now();
        auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
        std::cout << "[PointCloudProcessor<PointT>::ReadPcdFile] Loaded "
        << input_cloud->points.size() << " data points from " << streamIterator->string() <<  "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;

        streamIterator++;
        if(streamIterator == stream.end())
            streamIterator = stream.begin();

        renderer.SpinViewerOnce();
    }
}
