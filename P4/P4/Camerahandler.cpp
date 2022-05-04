#include "Camerahandler.h"

#define _CRT_SECURE_NO_WARNINGS

using namespace royale;
using namespace sample_utils;
using namespace std;
    
// Pointcloud objects
pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);
pcl::PointCloud<PointT>::Ptr cloudFilteredX(new pcl::PointCloud<PointT>); // We need to make a new cloud for each filter because of
pcl::PointCloud<PointT>::Ptr cloudFilteredXY(new pcl::PointCloud<PointT>);// boost shared Ptr will go out of scope and free the pointer
pcl::PointCloud<PointT>::Ptr cloudFilteredXYZ(new pcl::PointCloud<PointT>);// if we do not, we will invalidate the heap.
pcl::PointCloud<PointT>::Ptr cloudDownsampled(new pcl::PointCloud<PointT>);  
pcl::PointCloud<PointT>::Ptr cloudPlaneRemoved(new pcl::PointCloud<PointT>); 
pcl::PointCloud<PointT>::Ptr cloudFiltered(new pcl::PointCloud<PointT>);
pcl::PointCloud<PointT>::Ptr cloud_cluster(new pcl::PointCloud<PointT>);
pcl::PointIndices::Ptr indices(new pcl::PointIndices);
pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer);
bool isViewer = false;
int vp = 0;



Camerahandler::Camerahandler()
    {
        hasRun = false;
        float bckgr_gray_level = 1.0;  // Black:=0.0
        float txt_gray_lvl = 1.0 - bckgr_gray_level;
        int vp = 0; // Default viewport
        bool isViewer = false;
    }

    /**
    * Creates a listener which will have callbacks from two sources - the Royale framework, when a
    * new frame is received, and the UI toolkit, when the graphics are ready to repaint.
    */
/*
    explicit Camerahandler::Camerahandler(const royale::Vector<royale::StreamId>& streamIds) :
        m_streamIds(streamIds)
    {
    }
    */

    void Camerahandler :: onNewData(const royale::DepthData* data)  {
        
        cloud = points2pcl(data, 100); 
        //std::cout << "\nRead pointcloud from " << cloud->size() << " data points.\n" << std::endl;
        if (!isViewer) {
            viewer = initViewer();
            isViewer = true;
        }
        if (cloud->size() == 0)
        {
            return;
        }
        viewer->removeAllShapes();
        viewer->removeAllPointClouds();
        else if (cloud->size() > 0) {
            
            //XYZfilter(cloud);
            float filt_leaf_size = 0.01;
            std::array<float, 6> filter_lims = { -0.15, 0.15, -0.15, 0.15, 0, 3 }; // x-min, x-max, y-min, y-max, z-min, z-max
            pcl::PassThrough<PointT> pass(true);
           
            pass.setInputCloud(cloud);
            pass.setFilterFieldName("x");
            pass.setFilterLimits(filter_lims[0], filter_lims[1]);
            pass.filter(*cloudFilteredX);

            pass.setInputCloud(cloudFilteredX);
            pass.setFilterFieldName("y");
            pass.setFilterLimits(filter_lims[2], filter_lims[3]);
            pass.filter(*cloudFilteredXY);

            pass.setInputCloud(cloudFilteredXY);
            pass.setFilterFieldName("z");
            pass.setFilterLimits(filter_lims[4], filter_lims[5]);
            pass.filter(*cloudFilteredXYZ);

            pcl::VoxelGrid<pcl::PointXYZ> dsfilt;
            dsfilt.setInputCloud(cloudFilteredXYZ);
            dsfilt.setLeafSize(filt_leaf_size, filt_leaf_size, filt_leaf_size);
            dsfilt.filter(*cloudDownsampled);

            buffer.push(cloudDownsampled);
            indx++;
        }
        pcl::visualization::PointCloudColorHandlerCustom<PointT> cloudDownsampled_color_h(cloudDownsampled, 255, 0, 0);
        viewer->addPointCloud(cloudDownsampled, cloudDownsampled_color_h, "Cloud Donwsampled", vp)
        if (cloudDownsampled->size() == 0)
        {
            return;
        }

        // perform ransac planar filtration to remove table top
        pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
        pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
        // Create the segmentation object
        pcl::SACSegmentation<pcl::PointXYZ> seg1;
        // Optional
        seg1.setOptimizeCoefficients(true);
        // Mandatory
        seg1.setModelType(pcl::SACMODEL_PLANE);
        seg1.setMethodType(pcl::SAC_RANSAC);
        seg1.setMaxIterations(100);
        seg1.setDistanceThreshold(0.015);
        seg1.setInputCloud(cloudDownsampled);
        seg1.segment(*inliers, *coefficients);

        // Create the filtering object
        pcl::ExtractIndices<pcl::PointXYZ> extract;
        extract.setInputCloud(cloudDownsampled);
        extract.setIndices(inliers);
        extract.setNegative(true);
        extract.filter(*cloudPlaneRemoved);

        // Create the KdTree object for the search method of the extraction
        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
        tree->setInputCloud(cloudPlaneRemoved);

        // create the extraction object for the clusters
        std::vector<pcl::PointIndices> cluster_indices;
        pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
        // specify euclidean cluster parameters
        ec.setClusterTolerance(0.01); // 2cm
        ec.setMinClusterSize(100);
        ec.setMaxClusterSize(25000);
        ec.setSearchMethod(tree);
        ec.setInputCloud(cloudPlaneRemoved);
        // exctract the indices pertaining to each cluster and store in a vector of pcl::PointIndices
        ec.extract(cluster_indices);

        if (cluster_indices.size() == 0) {
            return;
        }

        float distOrigin2center = 0;
        float smallestdist = 1000;
        std::vector<pcl::PointIndices>::const_iterator ClosestIndex;
        for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); ++it)
        {
            pcl::CentroidPoint<PointT> centroid;
            PointT center;

            for (const auto& idx : it->indices){
                centroid.add((*cloudPlaneRemoved)[idx]);
            }
            centroid.get(center);
            distOrigin2center = sqrt(pow(center.x, 2) + pow(center.y, 2));
            if(smallestdist > distOrigin2center)
            {
                smallestdist = distOrigin2center;
                ClosestIndex = it;
            }

        }
        cloud_cluster->clear();
        for (const auto& idx : ClosestIndex->indices) {
            cloud_cluster->push_back((*cloudPlaneRemoved)[idx]);
        }

        RANSACHandler Ransacer(cloud);
        auto [Cylinderratio, Radiuscyl] = Ransacer.check_cyl(cloud_cluster);
        std::cout << "Cylinder Ratio: " << Cylinderratio << " Radius: " << Radiuscyl *100 << " cm" << endl;
        auto [Sphratio, Radiussph] = Ransacer.check_sph(cloud_cluster);
        std::cout << "Sphere Ratio: " << Sphratio << " Radius: " << Radiussph * 100 << " cm" << endl;
        float Boxratio = Ransacer.check_box(cloud_cluster);
        std::cout << "Box Ratio: " << Boxratio <<  endl;
        /*
        std::vector<float> ratios;
        ratios.push_back(Cylinderratio); ratios.push_back(Sphratio); ratios.push_back(Boxratio);
        int shape = std::max_element(ratios.begin(), ratios.end()) - ratios.begin();
        switch (shape) {

            case 0:  //Cylinder 

            case 1: 

            case 2: 
               std::vector<float> boxdim = Ransacer.shape_box();
        }*/
        return;
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr Camerahandler::points2pcl(const royale::DepthData* data, uint8_t depthConfidence)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

        cloud->is_dense = true;
        for (size_t i = 0; i < data->points.size(); ++i) {
            if (data->points.at(i).depthConfidence >= depthConfidence) {
                cloud->push_back(pcl::PointXYZ(data->points.at(i).x, data->points.at(i).y, data->points.at(i).z));
            }
        }
        return cloud;
    }


pcl::visualization::PCLVisualizer::Ptr Camerahandler::initViewer()
{
    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
    viewer->createViewPort(0.0, 0.0, 1.0, 1.0, vp);
    viewer->setCameraPosition(0.0, 0.0, -0.5, 0.0, -1.0, 0.0, vp);
    viewer->setSize(800, 600);
    viewer->setBackgroundColor(bckgr_gray_level, bckgr_gray_level, bckgr_gray_level, vp);
    viewer->addCoordinateSystem(0.25); // Global reference frame (on-camera)

    return viewer;
}
