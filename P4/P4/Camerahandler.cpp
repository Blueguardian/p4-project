#include "Camerahandler.h"

#define _CRT_SECURE_NO_WARNINGS

using namespace royale;
using namespace sample_utils;
using namespace std;

    
// Pointcloud global objects and variables
pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);
pcl::PointCloud<PointT>::Ptr cloudFilteredX(new pcl::PointCloud<PointT>); // We need to make a new cloud for each filter because of
pcl::PointCloud<PointT>::Ptr cloudFilteredXY(new pcl::PointCloud<PointT>);// boost shared Ptr will go out of scope and free the pointer
pcl::PointCloud<PointT>::Ptr cloudFilteredXYZ(new pcl::PointCloud<PointT>);// if we do not, we will invalidate the heap.
pcl::PointCloud<PointT>::Ptr cloudDownsampled(new pcl::PointCloud<PointT>);  
pcl::PointCloud<PointT>::Ptr cloudPlaneRemoved(new pcl::PointCloud<PointT>); 
pcl::PointCloud<PointT>::Ptr cloudFiltered(new pcl::PointCloud<PointT>);
pcl::PointCloud<PointT>::Ptr cloud_cluster(new pcl::PointCloud<PointT>);
pcl::PointIndices::Ptr indices(new pcl::PointIndices);

// Viewer global variables
pcl::visualization::PCLVisualizer::Ptr viewerz;
bool isViewer = false;
int vp = 0;
int runCounter = 0;
std::string my_string = "";

PointT operator+ (const PointT Point1, const PointT Point2) {
    return PointT(Point1.x + Point2.x, Point1.y + Point2.y, Point1.z + Point2.z);
}

Camerahandler::Camerahandler()
    {
        hasRun = false;
        float bckgr_gray_level = 1.0;  // Black:=0.0
        float txt_gray_lvl = 1.0 - bckgr_gray_level;
        int vp = 0; // Default viewport
        bool isViewer = false;
    }

void Camerahandler::onNewData(const royale::DepthData* data)  {
        
        cloud = points2pcl(data, 100);
        //std::cout << "\nRead pointcloud from " << cloud->size() << " data points.\n" << std::endl;
        if (!isViewer) 
        {
            viewerz = initViewer();
            isViewer = true;
        }
        viewerz->removeAllShapes();
        viewerz->removeAllPointClouds();

        if (!cloud->empty())
        {
            //XYZfilter(cloud);
            float filt_leaf_size = 0.005;
            std::array<float, 6> filter_lims = { -0.15, 0.15, -0.15, 0.15, 0.1, 0.6 }; // x-min, x-max, y-min, y-max, z-min, z-max
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

            if (!cloudDownsampled->size() == 0)
            {
                // perform ransac planar filtration to remove table top
                pcl::ModelCoefficients::Ptr tablecoefficients(new pcl::ModelCoefficients);
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
                seg1.segment(*inliers, *tablecoefficients);

                // Create the filtering object
                pcl::ExtractIndices<pcl::PointXYZ> extract;
                extract.setInputCloud(cloudDownsampled);
                extract.setIndices(inliers);
                extract.setNegative(true); //change back to true
                extract.filter(*cloudPlaneRemoved);

                // Create the KdTree object for the search method of the extraction
                pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
                tree->setInputCloud(cloudPlaneRemoved);
                // create the extraction object for the clusters
                std::vector<pcl::PointIndices> cluster_indices;
                pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
                // specify euclidean cluster parameters
                ec.setClusterTolerance(0.01); // 1cm
                ec.setMinClusterSize(500);
                ec.setMaxClusterSize(25000);
                ec.setSearchMethod(tree);
                ec.setInputCloud(cloudPlaneRemoved);
                // exctract the indices pertaining to each cluster and store in a vector of pcl::PointIndices
                ec.extract(cluster_indices);

                if (!cluster_indices.size() == 0) {
                    float distOrigin2center = 0;
                    float smallestdist = 1000;
                    std::vector<pcl::PointIndices>::const_iterator ClosestIndex;
                    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); ++it)
                    {
                        pcl::CentroidPoint<PointT> centroid;
                        PointT center;

                        for (const auto& idx : it->indices) {
                            centroid.add((*cloudPlaneRemoved)[idx]);
                        }
                        centroid.get(center);
                        distOrigin2center = sqrt(pow(center.x, 2) + pow(center.y, 2));
                        if (smallestdist > distOrigin2center)
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
                    auto [Cylratio, cylcoeffs, cylpoints] = Ransacer.check_cyl(cloud_cluster);
                    //std::cout << "Cylinder Ratio: " << Cylratio << endl;
                    auto [Sphratio, sphcoeffs, sphpoints] = Ransacer.check_sph(cloud_cluster);
                    //std::cout << "Sphere Ratio: " << Sphratio << endl;
                    auto [Boxratio, boxcoeffs_vec, boxpoints_vec] = Ransacer.check_box(cloud_cluster);
                    //std::cout << "Box Ratio: " << Boxratio << endl;
                    
                    viewerz->removeAllPointClouds();
                    viewerz->removeAllShapes();
                    pcl::visualization::PointCloudColorHandlerCustom<PointT> cloud_cluster_color_h(cloud_cluster, 255, 255, 255);
                    viewerz->addPointCloud(cloud_cluster, cloud_cluster_color_h, "Cluster", vp);
                    
                    if (Cylratio < 50 && Sphratio < 50 && Boxratio < 50) {
                        viewerz->spinOnce(1, true);
                        return;
                    }


                    std::vector<float> ratios;
                    ratios.push_back(Cylratio); ratios.push_back(Sphratio); ratios.push_back(Boxratio);
                    int shape = std::max_element(ratios.begin(), ratios.end()) - ratios.begin();
                    float wristAngleDeg = 0;
                    float handAperture = 0;

                    
                    float length;
                    float width;
                    float depth;
                    float height;

                    // Debugging, using pcl::visualizer
                    switch (shape) {

                    case 0:  //Cylinder 
                        {
                        pcl::visualization::PointCloudColorHandlerCustom<PointT> cloudDownsampled_color_h(cylpoints, 255, 0, 0);
                        viewerz->addPointCloud(cylpoints, cloudDownsampled_color_h, "Inliers", vp);
                        //viewerz->addCylinder(cylcoeffs);
                        PointT cylvec = PointT(cylcoeffs.values[3], cylcoeffs.values[4], cylcoeffs.values[5]);
                        PointT normVecPlan = PointT(tablecoefficients->values[0], tablecoefficients->values[1], tablecoefficients->values[2]);
                        PointT PointOnCyl(cylcoeffs.values[0], cylcoeffs.values[1], cylcoeffs.values[2]);
                        std::array<float, 2> cylExtremes = Ransacer.getPointCloudExtremes(*cylpoints, PointOnCyl, cylvec);
                        height = abs(cylExtremes[0] - cylExtremes[1]);
                        
                        
                        wristAngleDeg = acos(Ransacer.dotProduct(cylvec, normVecPlan) / (Ransacer.normPointT(cylvec)*Ransacer.normPointT(normVecPlan)))*180/3.14;
                        if (wristAngleDeg > 90) {
                            wristAngleDeg = wristAngleDeg - 180;
                        }

                        handAperture = cylcoeffs.values[6]*2 +0.02;
                        cout << "angle: " << wristAngleDeg << endl;
                        break;
                        }

                    case 1: // Sphere
                        {
                        pcl::visualization::PointCloudColorHandlerCustom<PointT> cloudDownsampled_color_h(sphpoints, 255, 0, 0);
                        viewerz->addPointCloud(sphpoints, cloudDownsampled_color_h, "Inliers", vp);
                        //viewerz->addSphere(sphcoeffs);
                        wristAngleDeg = 45;
                        handAperture = sphcoeffs.values[3] * 2 + 0.02;
                        if (runCounter < 100) {
                            my_string += std::to_string(sphcoeffs.values[3]) + "\n";
                        }
                        break;
                        }

                    case 2: //Box 
                        {
                        auto [dims, eigvecs, centroids] = Ransacer.shape_box(boxpoints_vec);
                        auto[angle, aperture] = Ransacer.boxangle(boxcoeffs_vec, *tablecoefficients, centroids, eigvecs);
                        wristAngleDeg = angle;
                        handAperture = aperture;
                        //std::cout << dims[0] << " " << dims[1] << " " << dims[2] << endl;
                        pcl::visualization::PointCloudColorHandlerCustom<PointT> boxpoints1_color_h(boxpoints_vec[0], 255, 0, 0);
                        pcl::visualization::PointCloudColorHandlerCustom<PointT> boxpoints2_color_h(boxpoints_vec[1], 0, 255, 0);
                        pcl::visualization::PointCloudColorHandlerCustom<PointT> boxpoints3_color_h(boxpoints_vec[2], 0, 0, 255);
                        viewerz->addPointCloud(boxpoints_vec[0], boxpoints1_color_h, "Inliers1", vp);
                        viewerz->addPointCloud(boxpoints_vec[1], boxpoints2_color_h, "Inliers2", vp);
                        viewerz->addPointCloud(boxpoints_vec[2], boxpoints3_color_h, "Inliers3", vp);

                        viewerz->addArrow(centroids[0] + eigvecs[0][1], centroids[0], 255, 0, 0, true, "p1v1");
                        //viewerz->addArrow(centroids[0] + eigvecs[0][0], centroids[0], 255, 0, 0, true, "p1v2");
                        viewerz->addArrow(centroids[1] + eigvecs[1][0], centroids[1], 0, 255, 0, true, "p2v1");
                        //viewerz->addArrow(centroids[1] + eigvecs[1][1], centroids[1] , 0, 255, 0, true, "p2v2");
                        //viewerz->addArrow(centroids[2] + eigvecs[2][0], centroids[2] , 0, 0, 255, true, "p3v1");
                        viewerz->addArrow(centroids[2] + eigvecs[2][1], centroids[2] , 0, 0, 255, true, "p3v2");
                        /*
                        length = Ransacer.normPointT(eigvecs[1][0]);
                        width = Ransacer.normPointT(eigvecs[0][1]);
                        depth = Ransacer.normPointT(eigvecs[2][1]);
                        if (runCounter < 100) {
                            my_string += std::to_string(length) + " " + std::to_string(width) + " " + std::to_string(depth) + "\n";
                        }
                        */
                        
                        break;
                        }
                    }
                        
                   
                    viewerz->spinOnce(1, true);

                    if(runCounter == 100){
                    std::ofstream file("filename");
                    file << my_string;
                    }
                    runCounter++;
                    //cout << "Run : " << runCounter << "\n";

                    return;
                }
                else {
                    return;
                }

                
            }
            else {
                return;
            }
        }

        else {
            return;
        }


        

        

        
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
    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("3D Viewer", true));
    viewer->createViewPort(0.0, 0.0, 1.0, 1.0, vp);
    viewer->setCameraPosition(0.0, 0.0, -0.5, 0.0, -1.0, 0.0, vp);
    viewer->setSize(800, 600);
    viewer->setBackgroundColor(0, 0, 0, vp);
    viewer->addCoordinateSystem(0.25); // Global reference frame (on-camera)

    return viewer;
}
