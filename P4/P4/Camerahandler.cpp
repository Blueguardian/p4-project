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

#include <pcl/console/print.h>
#include <cstddef>
#include <pcl/impl/pcl_base.hpp>

#include <chrono>
#include <thread>
#include <sample_utils/PlatformResources.hpp>
#include "Camerahandler.h"

using namespace royale;
using namespace sample_utils;
using namespace std;

pcl::visualization::CloudViewer viewer("CloudViewer");
    

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
        
        // Pointcloud objects
        pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);
        cloud = points2pcl(data, 242); //127(50),204(80),229(90),242(95),252(99) //this->depth_confidence
        RANSACHandler Ransacer(cloud);
        std::cout << "\nRead pointcloud from " << cloud->size() << " data points.\n" << std::endl;

        if (cloud->size() == 0)
        {
            return;
        }

        else if (cloud->size() > 0) {
            XYZfilter(cloud);
            buffer.push(cloud);
            indx++;
        }

        if (cloud->size() == 0)
        {
            return;
        }

        if (!isViewer) {
            viewerOneOff(viewer);
            viewerUpdate(viewer, cloud);
            isViewer = true;
        }
        else {
            viewerUpdate(viewer, cloud);
        }
        float cylinder_ratio = Ransacer.check_cyl(cloud);
        std::cout << cylinder_ratio << endl;
        //Ransacer.shape_cyl();
        
    }
    /**
    * The StreamIds for all streams that are expected to be received.  For this example, it's a
    * constant set, so doesn't need protecting with a mutex.
    */
   

    void Camerahandler::viewerOneOff(pcl::visualization::CloudViewer& viewer)
    {
        pcl::PointXYZ o;
        o.x = 1.0;
        o.y = 0;
        o.z = 0;
        std::cout << "i only run once" << std::endl;
    }

    void Camerahandler::viewerUpdate(pcl::visualization::CloudViewer& viewer, pcl::PointCloud<PointT>::Ptr& cloud)
    {
    viewer.showCloud(cloud, "cloud");
    }

    void Camerahandler::XYZfilter(pcl::PointCloud<PointT>::Ptr& ptcloud)
    {

        float filt_leaf_size = 0.005;
        std::array<float, 6> filter_lims = { -0.50, 0.50, -0.50, 0.50, 0.000, 0.50 }; // x-min, x-max, y-min, y-max, z-min, z-max
        pcl::PassThrough<PointT> pass(true);
        pcl::PointCloud<PointT>::Ptr cloudFiltered(new pcl::PointCloud<PointT>);

        pass.setInputCloud(ptcloud);
        pass.setFilterFieldName("x");
        pass.setFilterLimits(filter_lims[0], filter_lims[1]);
        pass.filter(*cloudFiltered);

        pass.setInputCloud(cloudFiltered);
        pass.setFilterFieldName("y");
        pass.setFilterLimits(filter_lims[2], filter_lims[3]);
        pass.filter(*cloudFiltered);

        pass.setInputCloud(cloudFiltered);
        pass.setFilterFieldName("z");
        pass.setFilterLimits(filter_lims[4], filter_lims[5]);
        pass.filter(*cloudFiltered);

        pcl::VoxelGrid<pcl::PointXYZ> dsfilt;
        dsfilt.setInputCloud(cloudFiltered);
        dsfilt.setLeafSize(filt_leaf_size, filt_leaf_size, filt_leaf_size);
        dsfilt.filter(*cloudFiltered);
        std::cerr << "PointCloud after downsampling: " << cloudFiltered->width * cloudFiltered->height << " data points." << std::endl;
    }

    /*
    void filt(pcl::PointCloud<PointT>::Ptr& output) 
    {
        if (!initCompute())
            return;

        if (input_.get() == &output)  // cloud_in = cloud_out
        {
            PointCloud output_temp;
            applyFilter(output_temp);
            output_temp.header = input_->header;
            output_temp.sensor_origin_ = input_->sensor_origin_;
            output_temp.sensor_orientation_ = input_->sensor_orientation_;
            pcl::copyPointCloud(output_temp, output);
        }
        else
        {
            output.header = input_->header;
            output.sensor_origin_ = input_->sensor_origin_;
            output.sensor_orientation_ = input_->sensor_orientation_;
            applyFilter(output);
        }

        deinitCompute();
    }
    */

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


