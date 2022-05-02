#include "Camerahandler.h"

#define _CRT_SECURE_NO_WARNINGS

using namespace royale;
using namespace sample_utils;
using namespace std;

pcl::visualization::CloudViewer viewer("CloudViewer");

    
// Pointcloud objects
pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);
pcl::PointCloud<PointT>::Ptr cloudFiltered(new pcl::PointCloud<PointT>); // We need to make a new cloud for each filter because of
pcl::PointCloud<PointT>::Ptr cloudFiltered2(new pcl::PointCloud<PointT>);// boost shared Ptr will go out of scope and free the pointer
pcl::PointCloud<PointT>::Ptr cloudFiltered3(new pcl::PointCloud<PointT>);// if we do not, we will invalidate the heap.
pcl::PointCloud<PointT>::Ptr cloudFiltered4(new pcl::PointCloud<PointT>);  
pcl::PointCloud<PointT>::Ptr cloudFiltered5(new pcl::PointCloud<PointT>); 
pcl::PointIndices::Ptr indices(new pcl::PointIndices);


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
        
        cloud = points2pcl(data, 0); 
        RANSACHandler Ransacer(cloud);
        std::cout << "\nRead pointcloud from " << cloud->size() << " data points.\n" << std::endl;

        if (cloud->size() == 0)
        {
            return;
        }

        else if (cloud->size() > 0) {
            
            //XYZfilter(cloud);
            float filt_leaf_size = 0.005;
            std::array<float, 6> filter_lims = { -0.15, 0.15, -0.15, 0.15, 0, 3 }; // x-min, x-max, y-min, y-max, z-min, z-max
            pcl::PassThrough<PointT> pass(true);
           
            pass.setInputCloud(cloud);
            pass.setFilterFieldName("x");
            pass.setFilterLimits(filter_lims[0], filter_lims[1]);
            pass.filter(*cloudFiltered);

            pass.setInputCloud(cloudFiltered);
            pass.setFilterFieldName("y");
            pass.setFilterLimits(filter_lims[2], filter_lims[3]);
            pass.filter(*cloudFiltered2);

            pass.setInputCloud(cloudFiltered2);
            pass.setFilterFieldName("z");
            pass.setFilterLimits(filter_lims[4], filter_lims[5]);
            pass.filter(*cloudFiltered3);

            pcl::VoxelGrid<pcl::PointXYZ> dsfilt;
            dsfilt.setInputCloud(cloudFiltered3);
            dsfilt.setLeafSize(filt_leaf_size, filt_leaf_size, filt_leaf_size);
            dsfilt.filter(*cloudFiltered4);

            std::cerr << "PointCloud after downsampling: " << cloudFiltered4->width * cloudFiltered4->height << " data points." << std::endl;
            buffer.push(cloudFiltered4);
            indx++;
        }

        if (cloud->size() == 0)
        {
            return;
        }

        if (!isViewer) {
            viewerOneOff(viewer);
            viewer.showCloud(cloud, "OG");
            viewer.showCloud(cloudFiltered4, "Filtered");
            //viewerUpdate(viewer, cloudFiltered4);
            isViewer = true;
        }
        else {
            viewer.showCloud(cloud, "OG");
            viewer.showCloud(cloudFiltered4, "Filtered");
            //viewerUpdate(viewer, cloudFiltered4);

        }
        //float cylinder_ratio = Ransacer.check_cyl(cloudFiltered4);
        //std::cout << cylinder_ratio << endl;
        //Ransacer.shape_cyl();,


        return;
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

    /*
    void XYZfilter(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud){
    float filt_leaf_size = 0.005;
    std::array<float, 6> filter_lims = { -0.15, 0.15, -0.15, 0.15, 0, 3 }; // x-min, x-max, y-min, y-max, z-min, z-max
    pcl::PassThrough<PointT> pass(true);

    pass.setInputCloud(cloud);
    pass.setFilterFieldName("x");
    pass.setFilterLimits(filter_lims[0], filter_lims[1]);
    pass.filter(*cloud);

    pass.setInputCloud(cloud);
    pass.setFilterFieldName("y");
    pass.setFilterLimits(filter_lims[2], filter_lims[3]);
    pass.filter(*cloud);

    pass.setInputCloud(cloud);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(filter_lims[4], filter_lims[5]);
    pass.filter(*cloud);

    pcl::VoxelGrid<pcl::PointXYZ> dsfilt;
    dsfilt.setInputCloud(cloud);
    dsfilt.setLeafSize(filt_leaf_size, filt_leaf_size, filt_leaf_size);
    dsfilt.filter(*cloud);
    }*/
