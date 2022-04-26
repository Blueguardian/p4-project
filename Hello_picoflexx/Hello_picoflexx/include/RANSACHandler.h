//
// Created by mogens on 25/04/2022.
//

#ifndef P4_PROJECT_RANSACHANDLER_H
#define P4_PROJECT_RANSACHANDLER_H

#include <royale.hpp>
#include <iostream>
#include <mutex>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/projection_matrix.h>
#include <pcl/ModelCoefficients.h>                                              //Planar segmentation,Extract Indices,Cylinder model segmentation
#include <pcl/io/ply_io.h>                                                      //?? pcl/io/io.h (http://pointclouds.org/documentation/tutorials/planar_segmentation.php#planar-segmentation)
#include <pcl/point_types.h>                                                    //Planar segmentation,Extract Indices ,Cylinder model segmentation
#include <pcl/filters/voxel_grid.h>                                             //Extract Indices                        http://pointclouds.org/documentation/tutorials/extract_indices.php#id1
#include <pcl/filters/extract_indices.h>                                        //Extract Indices  Cylinder model segmentation
#include <pcl/filters/passthrough.h>                                            //Cylinder model segmentation
#include <pcl/features/normal_3d.h>                                             //Cylinder model segmentation
#include <pcl/sample_consensus/method_types.h>                                  //Planar segmentation,Extract Indices ,Cylinder model segmentation
#include <pcl/sample_consensus/model_types.h>                                   //Planar segmentation  //Extract Indices  Cylinder model segmentation
#include <pcl/segmentation/sac_segmentation.h>                                  //Planar segmentation  //Extract Indices      Cylinder model segmentation + others?
#include <pcl/visualization/pcl_visualizer.h>                                   //http://pointclouds.org/documentation/tutorials/pcl_visualizer.php#pcl-visualizer
#include <pcl/console/time.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/sample_consensus/sac_model_sphere.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>

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

    pcl::PointCloud<PointT>::Ptr Ptcloud;

    RANSACHandler(pcl::PointCloud<PointT> &cloud){
        Ptcloud = cloud;
    }



};


#endif //P4_PROJECT_RANSACHANDLER_H
