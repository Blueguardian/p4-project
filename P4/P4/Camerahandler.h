
//
// Created by mogens on 25/04/2022.
//
#define _CRT_SECURE_NO_WARNINGS

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
#include <RANSACHandler.h>



#pragma once

typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

using namespace royale;
using namespace sample_utils;
using namespace std;

class Camerahandler : public royale::IDepthDataListener
{
public:

    //public fields
    std::mutex flagMutex;
    bool undistortImage;
    bool hasRun;
    int indx;
    std::stack<pcl::PointCloud<PointT>::Ptr> buffer;

    //Public Method Prototypes
    Camerahandler();
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

    struct MyFrameData
    {
        std::vector<uint32_t> exposureTimes;
        std::vector<std::string> asciiFrame;
    };

    std::map<royale::StreamId, MyFrameData> m_receivedData;
    std::mutex m_lockForReceivedData;
    bool isViewer;

    int vp; // Default viewport
    float bckgr_gray_level;  // Black:=0.02
    float txt_gray_lvl;
    //std::array<float, 6> filter_lims; // picoflexx depth z-axis (Min ? m)
    //float filt_leaf_size;
    //const royale::Vector<royale::StreamId> m_streamIds;

    //Private Method prototypes
    pcl::PointCloud<pcl::PointXYZ>::Ptr points2pcl(const royale::DepthData* data, uint8_t depthConfidence);

    void XYZfilter(pcl::PointCloud<PointT>::Ptr& ptcloud);
    void viewerUpdate(pcl::visualization::CloudViewer& viewer, pcl::PointCloud<PointT>::Ptr& cloud);
    void viewerOneOff(pcl::visualization::CloudViewer& viewer);
};


#endif //P4_PROJECT_CAMERAHANDLER_H


