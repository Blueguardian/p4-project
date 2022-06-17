//
// Created by mogens on 25/04/2022.
//

#ifndef P4_PROJECT_CAMERAHANDLER_H
#define P4_PROJECT_CAMERAHANDLER_H


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
#include <stack>
#include <sample_utils/PlatformResources.hpp>

typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

using namespace royale;
using namespace sample_utils;
using namespace std;

class Camerahandler : public royale::IDepthDataListener
{


public:

    //Public Method Prototypes
    Camerahandler(bool vis  = true);
    explicit Camerahandler(const royale::Vector<royale::StreamId>& streamIds);
    void onNewData(const royale::DepthData* data) override;

private:
    /**
    * The StreamIds for all streams that are expected to be received.  For this example, it's a
    * constant set, so doesn't need protecting with a mutex.
    */
    const royale::Vector<royale::StreamId> m_streamIds;

    /**
    * Updated in each call to onNewData, for each stream it contains the most recently received
    * frame.  Should only be accessed with m_lockForReceivedData held.
    */
    

    //Private Method prototypes
    pcl::PointCloud<pcl::PointXYZ>::Ptr points2pcl(const royale::DepthData* data, uint8_t depthConfidence);
    
    void filter(const pcl::PointCloud<PointT>::Ptr &ptcloud);
    void initViewer();
    void viewer(pcl::PointCloud<PointT>::Ptr cloud);

    
};


#endif //P4_PROJECT_CAMERAHANDLER_H
