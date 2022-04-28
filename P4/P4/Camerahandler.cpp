#include "Camerahandler.h"

#define _CRT_SECURE_NO_WARNINGS

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
#include "Camerahandler.h"

using namespace royale;
using namespace sample_utils;
using namespace std;

    

Camerahandler::Camerahandler()
    {
        hasRun = false;
        float filt_leaf_size = 0.005;
        std::array<float, 6> filter_lims = { -0.050, 0.050, -0.050, 0.050, 0.000, 0.150 };
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
        if (!isViewer) {  
            isViewer = true;
        }
        pcl::visualization::CloudViewer viewer("CloudViewer");
        // Pointcloud objects
        pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);
        cloud = points2pcl(data, 242); //127(50),204(80),229(90),242(95),252(99) //this->depth_confidence
        std::cout << "\nRead pointcloud from " << cloud->size() << " data points.\n" << std::endl;
        //viewer.showCloud(cloud, "cloud");

        if (cloud->size() == 0)
        {
            return;
        }

        else if (cloud->size() > 0 && indx < 10) {
            filter(cloud);
            //buffer.push(cloud);
            indx++;
        }
        else {

        }
    }
    /**
    * The StreamIds for all streams that are expected to be received.  For this example, it's a
    * constant set, so doesn't need protecting with a mutex.
    */
   

    void Camerahandler::initViewer()
    {  
    }

    void Camerahandler::viewer(pcl::PointCloud<PointT>::Ptr cloud)
    {     
    }

    /**
    * Updated in each call to onNewData, for each stream it contains the most recently received
    * frame.  Should only be accessed with m_lockForReceivedData held.
    */

    void Camerahandler::filter(const pcl::PointCloud<PointT>::Ptr& ptcloud)
    {
        pcl::PassThrough<PointT> pass(true);

        pass.setInputCloud(ptcloud);
        pass.setFilterFieldName("x");
        pass.setFilterLimits(filter_lims[0], filter_lims[1]);
        pass.filter(*ptcloud);

        pass.setInputCloud(ptcloud);
        pass.setFilterFieldName("y");
        pass.setFilterLimits(filter_lims[2], filter_lims[3]);
        pass.filter(*ptcloud);

        pass.setInputCloud(ptcloud);
        pass.setFilterFieldName("z");
        pass.setFilterLimits(filter_lims[4], filter_lims[5]);
        pass.filter(*ptcloud);

        pcl::VoxelGrid<pcl::PointXYZ> dsfilt;
        dsfilt.setInputCloud(ptcloud);
        dsfilt.setLeafSize(this->filt_leaf_size, this->filt_leaf_size, this->filt_leaf_size);
        dsfilt.filter(*ptcloud);
        std::cerr << "PointCloud after downsampling: " << ptcloud->width * ptcloud->height << " data points." << std::endl;
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr Camerahandler::points2pcl(const royale::DepthData* data, uint8_t depthConfidence)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

        cloud->is_dense = false;
        for (size_t i = 0; i < data->points.size(); ++i) {
            if (data->points.at(i).depthConfidence >= depthConfidence) {
                cloud->push_back(pcl::PointXYZ(data->points.at(i).x, data->points.at(i).y, data->points.at(i).z));
            }
        }
        return cloud;
    }


