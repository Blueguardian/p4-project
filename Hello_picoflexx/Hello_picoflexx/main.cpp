//
// Created by mogens on 25/04/2022.
//

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
#include <include/RANSACHandler.cpp>
#include <include/EMGHandler.cpp>
#include <include/Camerahandler.h>

#include <chrono>
#include <thread>
#include <sample_utils/PlatformResources.hpp>

using namespace royale;
using namespace sample_utils;
using namespace std;
using namespace cv;

typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

int main (int argc, char *argv[])
{
    // Windows requires that the application allocate these, not the DLL.
    PlatformResources resources;

    // This is the data listener which will receive callbacks.  It's declared
    // before the cameraDevice so that, if this function exits with a 'return'
    // statement while the camera is still capturing, it will still be in scope
    // until the cameraDevice's destructor implicitly de-registers the listener.
    CameraHandler camhandle;

    // this represents the main camera device object
    std::unique_ptr<ICameraDevice> cameraDevice;

    CameraManager manager;

    royale::Vector<royale::String> camlist(manager.getConnectedCameraList());
    cout << "Detected " << camlist.size() << " camera(s)." << endl;

    if (!camlist.empty())
    {
        cameraDevice = manager.createCamera(camlist[0]);
    }
    // the camera device is now available and CameraManager can be deallocated here

    // IMPORTANT: call the initialize method before working with the camera device
    auto status = cameraDevice->initialize();

    // retrieve the lens parameters from Royale
    // LensParameters lensParameters;
    // status = cameraDevice->getLensParameters(lensParameters);
    // camhandle.setLensParameters(lensParameters);

    cameraDevice->registerDataListener(&camhandle);
    cameraDevice->startCapture();
    // register a data listener

    int currentKey = 0;
    while (currentKey != 27)
    {
        // wait until a key is pressed
        // currentKey = waitKey(1);

        currentKey = getchar();
        if (currentKey == 'd')
        {

            // toggle the undistortion of the image
            camhandle.toggleUndistort();
        }
    }
    cameraDevice->stopCapture();
    return 0;
}