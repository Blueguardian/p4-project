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

typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

void correctCylShape(pcl::ModelCoefficients& cyl, const pcl::ModelCoefficients& coefficients, const pcl::PointCloud<PointT>& cloud);
std::array<float, 2> getPointCloudExtremes(const pcl::PointCloud<PointT>& cloud, pcl::PointXYZ center, pcl::PointXYZ direction);
float normPointT(pcl::PointXYZ c);
float dotProduct(pcl::PointXYZ a, pcl::PointXYZ b);
void viewerPsycho(pcl::visualization::PCLVisualizer& viewer);
void viewerOneOff(pcl::visualization::PCLVisualizer& viewer);

using namespace royale;
using namespace sample_utils;
using namespace std;
using namespace cv;

double dimensions;
int user_data = 0;
pcl::visualization::CloudViewer viewer("CloudViewer");



class MyListener : public royale::IDepthDataListener
{

    /**
    * Data that has been received in onNewData, and will be printed in the paint() method.
    */
    struct MyFrameData2
    {
        std::vector<uint32_t> exposureTimes;
        std::vector<std::string> asciiFrame;
    };

public:

    MyListener() :
        undistortImage(true)
    {
        hasRun = false;
    }

    /**
    * Creates a listener which will have callbacks from two sources - the Royale framework, when a
    * new frame is received, and the UI toolkit, when the graphics are ready to repaint.
    */
    explicit MyListener(const royale::Vector<royale::StreamId>& streamIds) :
        m_streamIds(streamIds)
    {
    }

    /**
    * This callback is called for each depth frame that is captured.  In a mixed-mode use case
    * (a use case with multiple streams), each callback refers to data from a single stream.
    */
    void onNewData(const royale::DepthData* data) override
    {
//        cout << "start" << endl;
        if (!isViewer) {
            
            viewer.runOnVisualizationThreadOnce(viewerOneOff);
            isViewer = true;
        }
        cout << "1" << endl;
        // Pointcloud objects
        pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);
        pcl::PointCloud<PointT>::Ptr cloud_filtered(new pcl::PointCloud<PointT>);
        pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);
        cout << "2" << endl;
        // Transform rs2::pointcloud into pcl::PointCloud<PointT>::Ptr
        pcl::console::TicToc time;
        time.tic();
        cloud = points2pcl(data, 242); //127(50),204(80),229(90),242(95),252(99) //this->depth_confidence
        std::cout << "\nRead pointcloud from " << cloud->size() << " data points (in " << time.toc() << " ms).\n" << std::endl;
        if (cloud->size() == 0)
        {
            return;
        }
        cout << "3" << endl;
        // http://pointclouds.org/documentation/tutorials/cylinder_segmentation.php

        // PCL objects
        pcl::PassThrough<PointT> pass(true);
        pcl::NormalEstimation<PointT, pcl::Normal> ne;
        pcl::SACSegmentationFromNormals<PointT, pcl::Normal> seg;
        pcl::ExtractIndices<PointT> extract;
        pcl::ExtractIndices<pcl::Normal> extract_normals;
        pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>());

        pcl::ModelCoefficients::Ptr coefficients_plane(new pcl::ModelCoefficients), coefficients_cylinder(new pcl::ModelCoefficients);
        pcl::PointIndices::Ptr inliers_plane(new pcl::PointIndices), inliers_cylinder(new pcl::PointIndices);
        
        cloud_filtered = cloud;

        // Build a passthrough filter to remove unwanted points
       /*
        time.tic();
        pass.setInputCloud(cloud);
        pass.setFilterFieldName("x");
        pass.setFilterLimits(filter_lims[0], filter_lims[1]);
        try {
            pass.filter(*cloud_filtered);
        }

        catch (const std::exception& ex) {
            cout << "1" << endl;
            cerr << ex.what() << endl;
            
        }

        pass.setInputCloud(cloud_filtered);
        pass.setFilterFieldName("y");
        pass.setFilterLimits(filter_lims[2], filter_lims[3]);
        try {
            pass.filter(*cloud_filtered);
        }

        catch (const std::exception& ex) {
            cout << "2" << endl;
            cerr << ex.what() << endl;
        }
        
        pass.setInputCloud(cloud_filtered);
        pass.setFilterFieldName("z");
        pass.setFilterLimits(filter_lims[4], filter_lims[5]);
        try {
            pass.filter(*cloud_filtered);
        }
        
        catch (const std::exception& ex) {
            cout << "3" << endl;
            cerr << ex.what() << endl;
        }
        
        */
       
        std::cerr << "PointCloud after filtering: " << cloud_filtered->points.size() << " data points." << std::endl;
        
        // Downsampling the filtered point cloud
        pcl::VoxelGrid<pcl::PointXYZ> dsfilt;
        dsfilt.setInputCloud(cloud_filtered);
        dsfilt.setLeafSize(this->filt_leaf_size, this->filt_leaf_size, this->filt_leaf_size);
        dsfilt.filter(*cloud_filtered);
        std::cerr << "PointCloud after downsampling: " << cloud_filtered->width * cloud_filtered->height << " data points." << std::endl;
        /*
        // Draw filtered PointCloud
        pcl::visualization::PointCloudColorHandlerCustom<PointT> cloud_filtered_in_color_h(cloud, (int)255 * txt_gray_lvl, (int)255 * txt_gray_lvl,
            (int)255 * txt_gray_lvl);
        
        cout << "5" << endl;
        // Estimate point normals
        ne.setSearchMethod(tree);
        ne.setInputCloud(cloud_filtered);
        ne.setKSearch(50);
        ne.compute(*cloud_normals);
        cout << "5.1" << endl;
        time.tic();
        // Create the segmentation object for cylinder segmentation and set all the parameters
        seg.setOptimizeCoefficients(true);
        seg.setModelType(pcl::SACMODEL_CYLINDER);
        seg.setMethodType(pcl::SAC_RANSAC);
        seg.setNormalDistanceWeight(0.1);
        seg.setMaxIterations(10000);
        seg.setDistanceThreshold(0.05);
        seg.setRadiusLimits(0.005, 0.040);
        seg.setInputCloud(cloud_filtered);
        seg.setInputNormals(cloud_normals);
        cout << "5.2" << endl;
        // Obtain the cylinder inliers and coefficients
        seg.segment(*inliers_cylinder, *coefficients_cylinder);
        //std::cerr << "Cylinder inliers: " << *inliers_cylinder << std::endl;
        //std::cerr << "Cylinder coefficients: " << *coefficients_cylinder << std::endl;
        cout << "6" << endl;
        // Save the cylinder inliers
        extract.setInputCloud(cloud_filtered);
        extract.setIndices(inliers_cylinder);
        extract.setNegative(false);
        
        extract.filter(*cloud_cylinder);
        cout << "7" << endl;
        */
        pcl::PointCloud<PointT>::Ptr cloud_cylinder(new pcl::PointCloud<PointT>());
        if ( !cloud_cylinder->points.empty()) {
            std::cerr << "\nCan't find the cylindrical component";
        }
        else {
            //std::cerr << "PointCloud CYLINDER: " << cloud_cylinder->points.size() << " data points (in " << time.toc() << " ms)." << std::endl;

            // ICP aligned point cloud is red
            pcl::visualization::PointCloudColorHandlerCustom<PointT> cloud_cylinder_color_h(cloud_cylinder, 180, 20, 20);
            viewer.showCloud(cloud_filtered, "cloud_cylinder");

           viewer.runOnVisualizationThread(viewerPsycho);
            
            // Plot cylinder shape
            //pcl::ModelCoefficients::Ptr corrected_coefs_cylinder(new pcl::ModelCoefficients);
            //correctCylShape(*corrected_coefs_cylinder, *coefficients_cylinder, *cloud_cylinder);
            //viewer.addCylinder(*corrected_coefs_cylinder, "cylinder");

            //viewer->spinOnce(1, true);

            cout << "end" << endl;
        }

        // Or here use a_callback_to_other_ui_of_your_choice();
        // You can also choose to plot everything down here in one go.
    }
    
    void setLensParameters(const LensParameters& lensParameters)
    {
        // Construct the camera matrix
        // (fx   0    cx)
        // (0    fy   cy)
        // (0    0    1 )
        Mat cameraMatrix = (Mat1d(3, 3) << lensParameters.focalLength.first, 0, lensParameters.principalPoint.first,
            0, lensParameters.focalLength.second, lensParameters.principalPoint.second,
            0, 0, 1);

        // Construct the distortion coefficients
        // k1 k2 p1 p2 k3
        distortionCoefficients = (Mat1d(1, 5) << lensParameters.distortionRadial[0],
            lensParameters.distortionRadial[1],
            lensParameters.distortionTangential.first,
            lensParameters.distortionTangential.second,
            lensParameters.distortionRadial[2]);
    }

    void toggleUndistort()
    {
        std::lock_guard<std::mutex> lock(flagMutex);
        undistortImage = !undistortImage;
    }

    Mat distortionCoefficients;

    std::mutex flagMutex;
    bool undistortImage;
    bool hasRun;

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
    std::map<royale::StreamId, MyFrameData> m_receivedData;
    std::mutex m_lockForReceivedData;


    bool isViewer = false;

    int vp = 0; // Default viewport
    float bckgr_gray_level = 1.0;  // Black:=0.0
    float txt_gray_lvl = 1.0 - bckgr_gray_level;
    std::array<float, 6> filter_lims = { -0.050, 0.050, -0.050, 0.050, 0.000, 0.150 }; // picoflexx depth z-axis (Min ? m)
    float filt_leaf_size = 0.005;

  /*  pcl::visualization::PCLVisualizer::Ptr initViewer()
    {
        pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
        viewer->createViewPort(0.0, 0.0, 1.0, 1.0, vp);
        viewer->setCameraPosition(0.0, 0.0, -0.5, 0.0, -1.0, 0.0, vp);
        viewer->setSize(800, 600);
        viewer->setBackgroundColor(bckgr_gray_level, bckgr_gray_level, bckgr_gray_level, vp);
        viewer->addCoordinateSystem(0.25); // Global reference frame (on-camera)

        return viewer;
    }*/

    pcl::PointCloud<pcl::PointXYZ>::Ptr points2pcl(const royale::DepthData* data, uint8_t depthConfidence)
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
};

void correctCylShape(pcl::ModelCoefficients& cyl, const pcl::ModelCoefficients& coefficients, const pcl::PointCloud<PointT>& cloud)     //from the supervisor... 
{
    pcl::PointXYZ point_on_axis(coefficients.values[0], coefficients.values[1], coefficients.values[2]);
    pcl::PointXYZ axis_direction(coefficients.values[3], coefficients.values[4], coefficients.values[5]);
    std::array<float, 2> arr(getPointCloudExtremes(cloud, point_on_axis, axis_direction));

    pcl::PointXYZ point_bottom;
    point_bottom.x = point_on_axis.x + arr[0] * axis_direction.x / normPointT(axis_direction);
    point_bottom.y = point_on_axis.y + arr[0] * axis_direction.y / normPointT(axis_direction);
    point_bottom.z = point_on_axis.z + arr[0] * axis_direction.z / normPointT(axis_direction);
    pcl::PointXYZ bottom_top_direction;
    bottom_top_direction.x = (-arr[0] + arr[1]) * axis_direction.x / normPointT(axis_direction);
    bottom_top_direction.y = (-arr[0] + arr[1]) * axis_direction.y / normPointT(axis_direction);
    bottom_top_direction.z = (-arr[0] + arr[1]) * axis_direction.z / normPointT(axis_direction);

    //hight         we *1000 to make should we make sure we dont run out if the double range. eventho it would probably not happen
    int ratioCoef = 1000;
    int x_1000 = bottom_top_direction.x * ratioCoef;
    int y_1000 = bottom_top_direction.y * ratioCoef;
    int z_1000 = bottom_top_direction.z * ratioCoef;

    int powX = x_1000 * x_1000;
    int powY = y_1000 * y_1000;
    int powZ = z_1000 * z_1000;
    long powXYZ = powX + powY + powZ;
    long squarXYZ2 = pow(powXYZ, 0.5);
    //long squarXYZ2 = squarXYZ * 2;
    //cylinder_hight = (double)squarXYZ2 / (ratioCoef);
    //std::cout << "powX: " << powX << " | powY: " << powY << " | powZ: " << powZ << " || x+y+z= " << powXYZ <<std::endl;
    //std::cout << "result quard: " << squarXYZ << " | result * 2 = " << squarXYZ2 << std::endl;
    //std::cout << "Cylinder Hight: " << cylinder_hight << " mm" << std::endl;

    cyl.values.push_back(point_bottom.x);                                   //point_on_axis.x : the X coordinate of a point located on the cylinder axis
    cyl.values.push_back(point_bottom.y);                                   //point_on_axis.y : the Y coordinate of a point located on the cylinder axis
    cyl.values.push_back(point_bottom.z);                                   //point_on_axis.z : the Z coordinate of a point located on the cylinder axis
    cyl.values.push_back(bottom_top_direction.x);                           //axis_direction.x : the X coordinate of the cylinder's axis direction
    cyl.values.push_back(bottom_top_direction.y);                           //axis_direction.y : the Y coordinate of the cylinder's axis direction
    cyl.values.push_back(bottom_top_direction.z);                           //axis_direction.z : the Z coordinate of the cylinder's axis direction
    cyl.values.push_back(coefficients.values[6]);                           //radius : the cylinder's radius in meter
    //cyl.values.push_back(cyl_hight);

    dimensions = coefficients.values[6];
}

std::array<float, 2> getPointCloudExtremes(const pcl::PointCloud<PointT>& cloud, pcl::PointXYZ center, pcl::PointXYZ direction)     //from the supervisor... 
{
    std::array<float, 2> arr = { 1000.0, -1000.0 };
    pcl::PointXYZ vec;
    float scalar_proj;
    for (size_t i = 0; i < cloud.points.size(); ++i) {
        vec.x = cloud.points[i].x - center.x;
        vec.y = cloud.points[i].y - center.y;
        vec.z = cloud.points[i].z - center.z;
        scalar_proj = dotProduct(direction, vec) / normPointT(direction);
        if (scalar_proj < arr[0])
            arr[0] = scalar_proj;
        if (scalar_proj > arr[1])
            arr[1] = scalar_proj;
    }
    return arr;
}

float dotProduct(pcl::PointXYZ a, pcl::PointXYZ b)
{
    return a.x * b.x + a.y * b.y + a.z * b.z;
}

float normPointT(pcl::PointXYZ c)
{
    return std::sqrt(c.x * c.x + c.y * c.y + c.z * c.z);
}

void viewerOneOff(pcl::visualization::PCLVisualizer& viewer)
{
    viewer.setBackgroundColor(0.5, 0.5, 0.5);
    pcl::PointXYZ o;
    o.x = 1.0;
    o.y = 0;
    o.z = 0;
    std::cout << "i only run once" << std::endl;

}

void viewerPsycho(pcl::visualization::PCLVisualizer& viewer)
{
    static unsigned count = 0;
    std::stringstream ss;
    ss << "Once per viewer loop: " << count++;
    viewer.removeShape("text", 0);
    viewer.addText(ss.str(), 200, 300, "text", 0);

    //FIXME: possible race condition here:
    user_data++;
}

int main(int argc, char* argv[])
{
    // Windows requires that the application allocate these, not the DLL.
    PlatformResources resources;

    // This is the data listener which will receive callbacks.  It's declared
    // before the cameraDevice so that, if this function exits with a 'return'
    // statement while the camera is still capturing, it will still be in scope
    // until the cameraDevice's destructor implicitly de-registers the listener.
    MyListener listener;

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
    LensParameters lensParameters;
    status = cameraDevice->getLensParameters(lensParameters);
    listener.setLensParameters(lensParameters);

    cameraDevice->registerDataListener(&listener);
    cameraDevice->startCapture();
    // register a data listener

    int currentKey = 0;
    while (currentKey != 27)
    {
        // wait until a key is pressed
        currentKey = waitKey(1);
        

        if (currentKey == 'd')
        {
            
            // toggle the undistortion of the image
            listener.toggleUndistort();
        }
    }
    cameraDevice->stopCapture();
    return 0;
}

/*
// WriteCloud.cpp
#define _CRT_SECURE_NO_WARNINGS
#include <pcl/visualization/cloud_viewer.h>
#include <iostream>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/common_headers.h>


int user_data;

void
viewerOneOff(pcl::visualization::PCLVisualizer& viewer)
{
    viewer.setBackgroundColor(1.0, 0.5, 1.0);
    pcl::PointXYZ o;
    o.x = 1.0;
    o.y = 0;
    o.z = 0;
    viewer.addSphere(o, 0.25, "sphere", 0);
    std::cout << "i only run once" << std::endl;

}

void
viewerPsycho(pcl::visualization::PCLVisualizer& viewer)
{
    static unsigned count = 0;
    std::stringstream ss;
    ss << "Once per viewer loop: " << count++;
    viewer.removeShape("text", 0);
    viewer.addText(ss.str(), 200, 300, "text", 0);

    //FIXME: possible race condition here:
    user_data++;
}

int main()
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    //pcl::io::loadPCDFile("my_point_cloud.pcd", *cloud);


    int numPoints = 20;

    for (int i = 0; i < numPoints; i++)
    {
        pcl::PointXYZ point;

        point.x = 1024 * rand() / (RAND_MAX + 1.0f);
        point.y = 1024 * rand() / (RAND_MAX + 1.0f);
        point.z = 1024 * rand() / (RAND_MAX + 1.0f);

        cloud->points.push_back(point);
    }

    // for simplicity, use an "unorganized" cloud, "width" = num points, "height" = 1
    cloud->width = (int)cloud->points.size();
    cloud->height = 1;

    pcl::visualization::CloudViewer viewer("CloudViewer");

    //blocks until the cloud is actually rendered
    viewer.showCloud(cloud);

    //use the following functions to get access to the underlying more advanced/powerful
    //PCLVisualizer

    //This will only get called once
    viewer.runOnVisualizationThreadOnce(viewerOneOff);

    //This will get called once per visualization iteration
    viewer.runOnVisualizationThread(viewerPsycho);
    while (!viewer.wasStopped())
    {
        //you can also do cool processing here
        //FIXME: Note that this is running in a separate thread from viewerPsycho
        //and you should guard against race conditions yourself...
        user_data++;
    }
    return 0;
}*/