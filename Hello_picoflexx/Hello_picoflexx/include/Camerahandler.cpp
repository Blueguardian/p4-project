#define _CRT_SECURE_NO_WARNINGS

#include <royale.hpp>
#include <iostream>
#include <mutex>
#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

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
#include <Camerahandler.h>

using namespace royale;
using namespace sample_utils;
using namespace std;
using namespace cv;

class Camerahandler : public royale::IDepthDataListener
{


public:

    //Public global fields
    MyListener() :
            undistortImage(true)
    {
        hasRun = false;
        this.filt_leaf_size = 0.005
        this.filter_lims = { -0.050, 0.050, -0.050, 0.050, 0.000, 0.150 };
        this.txt_gray_lvl = 1.0 - bckgr_gray_level;
        this.bckgr_gray_level = 1.0;  // Black:=0.0
        this.vp = 0; // Default viewport
        this.isViewer = false;
    }

    /**
    * Creates a listener which will have callbacks from two sources - the Royale framework, when a
    * new frame is received, and the UI toolkit, when the graphics are ready to repaint.
    */
    explicit MyListener(const royale::Vector<royale::StreamId>& streamIds) :
            m_streamIds(streamIds)
    {
    }

    void onNewData(const royale::DepthData* data) override {
        if (!isViewer) {

            viewer.runOnVisualizationThreadOnce(viewerOneOff);
            isViewer = true;
        }
        // Pointcloud objects
        pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);
        pcl::PointCloud<PointT>::Ptr cloud_filtered(new pcl::PointCloud<PointT>);
        pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);
        // Transform rs2::pointcloud into pcl::PointCloud<PointT>::Ptr
        pcl::console::TicToc time;
        time.tic();
        cloud = points2pcl(data, 242); //127(50),204(80),229(90),242(95),252(99) //this->depth_confidence
        std::cout << "\nRead pointcloud from " << cloud->size() << " data points (in " << time.toc() << " ms).\n" << std::endl;
        if (cloud->size() == 0)
        {
            return;
        }
        else if(cloud->size() > 0 && indx < 10){
            this.filter(cloud)
            this.buffer.push_back(cloud)
            indx++;
        }
        else {

        }



    }
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

    void filter(const pcl::PointCloud<PointT>::Ptr &ptcloud)
    {
        pcl::PassThrough<PointT> pass(true);
        pcl::NormalEstimation<PointT, pcl::Normal> ne;
        pcl::SACSegmentationFromNormals<PointT, pcl::Normal> seg;
        pcl::ExtractIndices<PointT> extract;
        pcl::ExtractIndices<pcl::Normal> extract_normals;
        pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>());

        pcl::ModelCoefficients::Ptr coefficients_plane(new pcl::ModelCoefficients), coefficients_cylinder(new pcl::ModelCoefficients);
        pcl::PointIndices::Ptr inliers_plane(new pcl::PointIndices), inliers_cylinder(new pcl::PointIndices);
        // Do some filtering
    }





};

