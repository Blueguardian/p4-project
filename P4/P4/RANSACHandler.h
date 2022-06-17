//
// Created by mogens on 25/04/2022.
//

#ifndef P4_PROJECT_RANSACHANDLER_H
#define P4_PROJECT_RANSACHANDLER_H
#define _CRT_SECURE_NO_WARNINGS

#include <royale.hpp>
#include <tuple>
#include <iostream>
#include <mutex>
#include "UDPCom.h"

#include <pcl/common/impl/io.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/projection_matrix.h>
#include <pcl/ModelCoefficients.h>                                              //Planar segmentation,Extract Indices,Cylinder model segmentation
#include <pcl/point_types.h>                                                    //Planar segmentation,Extract Indices ,Cylinder model segmentation
#include <pcl/filters/voxel_grid.h>                                             //Extract Indices                        http://pointclouds.org/documentation/tutorials/extract_indices.php#id1
#include <pcl/filters/extract_indices.h>                                        //Extract Indices  Cylinder model segmentation
#include <pcl/filters/passthrough.h>                                            //Cylinder model segmentation
#include <pcl/features/normal_3d.h>                                             //Cylinder model segmentation
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/don.h>
#include <pcl/sample_consensus/method_types.h>                                  //Planar segmentation,Extract Indices ,Cylinder model segmentation
#include <pcl/sample_consensus/model_types.h>                                   //Planar segmentation  //Extract Indices  Cylinder model segmentation
#include <pcl/segmentation/sac_segmentation.h>                                  //Planar segmentation  //Extract Indices      Cylinder model segmentation + others?
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/visualization/pcl_visualizer.h>                                   //http://pointclouds.org/documentation/tutorials/pcl_visualizer.php#pcl-visualizer
#include <pcl/console/time.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/sample_consensus/sac_model_sphere.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/common/pca.h>
#include <pcl/surface/mls.h>

#include <pcl/filters/conditional_removal.h>


#include <chrono>
#include <thread>
#include <sample_utils/PlatformResources.hpp>

using namespace royale;
using namespace sample_utils;
using namespace std;

typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

class RANSACHandler {
    
public:
    std::array<float, 3> plane_centerPoint_x;
    std::array<float, 3> plane_centerPoint_y;
    std::array<float, 3> plane_centerPoint_z;
    float box_hight;
    float box_radius;
    RANSACHandler(pcl::PointCloud<PointT>::Ptr& cloud);
    
    std::array<float, 2> getPointCloudExtremes(const pcl::PointCloud<PointT>& cloud, pcl::PointXYZ center, pcl::PointXYZ direction);
    float dotProduct(pcl::PointXYZ a, pcl::PointXYZ b);
    pcl::PointXYZ crossProduct(pcl::PointXYZ a, pcl::PointXYZ b);
    float normPointT(pcl::PointXYZ c);
    bool Checkorthogonal(std::vector<pcl::ModelCoefficients>, int i);
    PointT flipVector(PointT vec1, PointT vec2);

    tuple <float, float> boxangle(std::vector<pcl::ModelCoefficients> boxcoeffs, pcl::ModelCoefficients planecoeffs, std::vector <PointT> centroids, std::vector<std::vector<PointT>> eigenvecs);
    tuple <float, pcl::ModelCoefficients, pcl::PointCloud<pcl::PointXYZ>::Ptr> check_cyl(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
    tuple <float, std::vector<pcl::ModelCoefficients>, std::vector< pcl::PointCloud<pcl::PointXYZ>::Ptr>> check_box(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
    tuple <float, pcl::ModelCoefficients, pcl::PointCloud<pcl::PointXYZ>::Ptr> check_sph(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud);

    tuple <std::vector <float>, std::vector<std::vector<PointT>>, std::vector<PointT>>  shape_box(std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> boxinliers_vec);
    void shape_cyl(pcl::ModelCoefficients& cyl, const pcl::ModelCoefficients& coefficients, const pcl::PointCloud<PointT>& cloud);
};



#endif //P4_PROJECT_RANSACHANDLER_H
