#include "Camerahandler.h"

#define _CRT_SECURE_NO_WARNINGS

using namespace royale;
using namespace sample_utils;
using namespace std;

extern UDPCom udp;
    
// Pointcloud global objects and variables
pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);
pcl::PointCloud<PointT>::Ptr fullcloud(new pcl::PointCloud<PointT>);
pcl::PointCloud<PointT>::Ptr cloudFilteredX(new pcl::PointCloud<PointT>); // We need to make a new cloud for each filter because of
pcl::PointCloud<PointT>::Ptr cloudFilteredXY(new pcl::PointCloud<PointT>);// boost shared Ptr will go out of scope and free the pointer
pcl::PointCloud<PointT>::Ptr cloudFilteredXYZ(new pcl::PointCloud<PointT>);// if we do not, we will invalidate the heap.
pcl::PointCloud<PointT>::Ptr cloudDownsampled(new pcl::PointCloud<PointT>);  
pcl::PointCloud<PointT>::Ptr cloudPlaneRemoved(new pcl::PointCloud<PointT>); 
pcl::PointCloud<PointT>::Ptr cloudFiltered(new pcl::PointCloud<PointT>);
pcl::PointCloud<PointT>::Ptr cloud_cluster(new pcl::PointCloud<PointT>);
pcl::PointIndices::Ptr indices(new pcl::PointIndices);

// Viewer global variables
pcl::visualization::PCLVisualizer::Ptr viewerz;
bool isViewer = false;
bool isBox = false;
bool isCyl = false;
bool isSph = false;
pcl::ModelCoefficients LastObjectCoeffs;

float maxGrasDist = 0.1;

int vp = 0;
int runCounter = 0;

vector<int> shapeCounters = {0,0,0};
vector<float> apertureAves = {0,0,0};
vector<float> wristAngleAves = {0,45,0};
float angleSmoothing = 0.90;
float apertureSmoothing = 0.90;

bool AutograspOn = false;

PointT operator+ (const PointT Point1, const PointT Point2) {
    return PointT(Point1.x + Point2.x, Point1.y + Point2.y, Point1.z + Point2.z);
}

PointT operator* (const PointT Point1, const float multiplier) {
    return PointT(Point1.x * multiplier, Point1.y * multiplier, Point1.z * multiplier);
}

PointT operator* (const PointT Point1, const double multiplier) {
    return PointT(Point1.x * multiplier, Point1.y * multiplier, Point1.z * multiplier);
}

PointT operator* (const PointT Point1, const int multiplier) {
    return PointT(Point1.x * multiplier, Point1.y * multiplier, Point1.z * multiplier);
}

template <typename T> int sgn(T val) {
    return (T(0) < val) - (val < T(0));
}

Camerahandler::Camerahandler()
    {
        hasRun = false;
        float bckgr_gray_level = 1.0;  // Black:=0.0
        float txt_gray_lvl = 1.0 - bckgr_gray_level;
        int vp = 0; // Default viewport
        bool isViewer = false;
    }

void Camerahandler::onNewData(const royale::DepthData* data)  {

       if(AutograspOn){
        fullcloud = points2pcl(data, 0);
        if(fullcloud->size() > 50){ 
        bool shouldCloseHandSimple = autoGraspSimple(fullcloud); // bad but simple autograsping algorithm
        }

        // Primitive-specific autograsping algorithms
        bool shouldCloseHandPriSpec = false;

        
        if (isCyl) {
            shouldCloseHandPriSpec = autoGraspCyl(fullcloud);
        }

        else if (isSph) {
             shouldCloseHandPriSpec = autoGraspSph(fullcloud);
         }
       
        else if (isBox) {
            shouldCloseHandPriSpec = autoGraspBox(fullcloud);
        }
       }

        

        cloud = points2pcl(data, 100);

        //std::cout << "\nRead pointcloud from " << cloud->size() << " data points.\n" << std::endl;

        if (!isViewer) 
        {
            viewerz = initViewer();
            isViewer = true;




           


        }
        viewerz->removeAllShapes();
        viewerz->removeAllPointClouds();

        if (!cloud->empty())
        {

            float filt_leaf_size = 0.005;
            float Croppinglimit = 0.15; // defines x-min, x-max, y-min, y-max 0.15

            pcl::PassThrough<PointT> pass(true);
            
            pass.setInputCloud(cloud);
            pass.setFilterFieldName("x");
            pass.setFilterLimits(-Croppinglimit, Croppinglimit);
            pass.filter(*cloudFilteredX);

            pass.setInputCloud(cloudFilteredX);
            pass.setFilterFieldName("y");
            pass.setFilterLimits(-Croppinglimit, Croppinglimit);
            pass.filter(*cloudFilteredXY);

            pass.setInputCloud(cloudFilteredXY);
            pass.setFilterFieldName("z");
            pass.setFilterLimits(0.1, 0.60); // 0.1 -> 0.6
            pass.filter(*cloudFilteredXYZ);
            
            pcl::VoxelGrid<pcl::PointXYZ> dsfilt;
            dsfilt.setInputCloud(cloudFilteredXYZ);
            dsfilt.setLeafSize(filt_leaf_size, filt_leaf_size, filt_leaf_size);
            dsfilt.filter(*cloudDownsampled);

            if (!cloudDownsampled->size() == 0)
            {
                
                // perform ransac planar filtration to remove table top
                pcl::ModelCoefficients::Ptr tablecoefficients(new pcl::ModelCoefficients);
                pcl::PointIndices::Ptr inliers(new pcl::PointIndices);

                pcl::SACSegmentation<pcl::PointXYZ> seg1;

                seg1.setOptimizeCoefficients(true);
                // Mandatory
                seg1.setModelType(pcl::SACMODEL_PLANE);
                seg1.setMethodType(pcl::SAC_RANSAC);
                seg1.setMaxIterations(100);
                seg1.setDistanceThreshold(0.007);
                seg1.setInputCloud(cloudDownsampled);
                seg1.segment(*inliers, *tablecoefficients);

                // Create the filtering object
                pcl::ExtractIndices<pcl::PointXYZ> extract;
                extract.setInputCloud(cloudDownsampled);
                extract.setIndices(inliers);
                extract.setNegative(true); //change back to true
                extract.filter(*cloudPlaneRemoved);

                // Create the KdTree object for the search method of the extraction
                pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
                tree->setInputCloud(cloudPlaneRemoved);
                // create the extraction object for the clusters
                std::vector<pcl::PointIndices> cluster_indices;
                pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
                // specify euclidean cluster parameters
                ec.setClusterTolerance(0.007); // 1cm
                ec.setMinClusterSize(200);
                ec.setMaxClusterSize(25000);
                ec.setSearchMethod(tree);
                ec.setInputCloud(cloudPlaneRemoved);
                // exctract the indices pertaining to each cluster and store in a vector of pcl::PointIndices
                ec.extract(cluster_indices);

                if (!cluster_indices.size() == 0) {
                    float distOrigin2center = 0;
                    float smallestdist = 1000;
                    std::vector<pcl::PointIndices>::const_iterator ClosestIndex;
                    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); ++it)
                    {
                        pcl::CentroidPoint<PointT> centroid;
                        PointT center;

                        for (const auto& idx : it->indices) {
                            centroid.add((*cloudPlaneRemoved)[idx]);
                        }
                        centroid.get(center);
                        distOrigin2center = sqrt(pow(center.x, 2) + pow(center.y, 2));
                        if (smallestdist > distOrigin2center)
                        {
                            smallestdist = distOrigin2center;
                            ClosestIndex = it;
                        }
                    }

                    cloud_cluster->clear();
                    for (const auto& idx : ClosestIndex->indices) {
                        cloud_cluster->push_back((*cloudPlaneRemoved)[idx]);
                    }

                    RANSACHandler Ransacer(cloud);
                    auto [Cylratio, cylcoeffs, cylpoints] = Ransacer.check_cyl(cloud_cluster);
                    //std::cout << "Cylinder Ratio: " << Cylratio << endl;
                    auto [Sphratio, sphcoeffs, sphpoints] = Ransacer.check_sph(cloud_cluster);
                    //std::cout << "Sphere Ratio: " << Sphratio << endl;
                    auto [Boxratio, boxcoeffs_vec, boxpoints_vec] = Ransacer.check_box(cloud_cluster);
                    //std::cout << "Box Ratio: " << Boxratio << endl;

                    viewerz->removeAllPointClouds();
                    viewerz->removeAllShapes();

                    pcl::visualization::PointCloudColorHandlerCustom<PointT> cloud_color_h(cloudDownsampled, 0, 0, 255);
                    viewerz->addPointCloud(cloudDownsampled, cloud_color_h, "Downsampled", vp);

                    pcl::visualization::PointCloudColorHandlerCustom<PointT> cloud_cluster_color_h(cloud_cluster, 255, 255, 255);
                    viewerz->addPointCloud(cloud_cluster, cloud_cluster_color_h, "Cluster", vp);

                    int minratio = 60;
                    if (Cylratio < minratio && Sphratio < minratio && Boxratio < minratio) {
                        viewerz->spinOnce(1, true);
                        return;
                    }

                    std::vector<float> ratios;
                    ratios.push_back(Cylratio); ratios.push_back(Sphratio); ratios.push_back(Boxratio);
                    int shape = std::max_element(ratios.begin(), ratios.end()) - ratios.begin();
                    float wristAngleDeg = 0;
                    float handAperture = 0;
                    
                    float length = -1;
                    float width = -1;
                    float depth = -1;
                    float height = 0;
                    float radius = 0;
                    float gripDist = 0.13;
                    
                    // Debugging, using pcl::visualizer
                    switch (shape) {

                    case 0:  //Cylinder 
                        {
                       
                        
                        PointT cylvec = PointT(cylcoeffs.values[3], cylcoeffs.values[4], cylcoeffs.values[5]);
                        PointT normVecPlan = PointT(tablecoefficients->values[0], tablecoefficients->values[1], tablecoefficients->values[2]);
                        PointT PointOnCyl(cylcoeffs.values[0], cylcoeffs.values[1], cylcoeffs.values[2]);

                        //viewerz->addArrow(PointOnCyl + cylvec, PointOnCyl, 255, 0, 0);
                        std::array<float, 2> cylExtremes = Ransacer.getPointCloudExtremes(*cylpoints, PointOnCyl, cylvec);
                        width = abs(cylExtremes[0] - cylExtremes[1]); //height = abs(cylExtremes[0] - cylExtremes[1]);
                        length = cylcoeffs.values[6];//radius = cylcoeffs.values[6];
                        
                        cout << "Width: " << width << "Radius" << length << "\n";
                        pcl::visualization::PointCloudColorHandlerCustom<PointT> cloudDownsampled_color_h(cylpoints, 255, 0, 0);
                        viewerz->addPointCloud(cylpoints, cloudDownsampled_color_h, "Inliers", vp);
      
                        cylcoeffs.values[3] = cylcoeffs.values[3]* width; cylcoeffs.values[4] = cylcoeffs.values[4] * width; cylcoeffs.values[5] = cylcoeffs.values[5] * width;
                        viewerz->addCylinder(cylcoeffs);

                        //cout << cylvec << "\n";
                        if (cylvec.y > 0) {
                            cylvec = cylvec * -1;        
                        }

                        wristAngleDeg = acos(Ransacer.dotProduct(cylvec, normVecPlan) / (Ransacer.normPointT(cylvec) * Ransacer.normPointT(normVecPlan))) * 180 / 3.14;

                        if(cylvec.x > 0 ){
                            wristAngleDeg = -1 * wristAngleDeg;
                        }
                       
                        /*if (cylvec.x < 0 || cylvec.y < 0) {
                            wristAngleDeg = wristAngleDeg * sgn(wristAngleAve);
                        }*/
                        viewerz->addArrow(PointOnCyl + cylvec, PointOnCyl, 255, 0, 0);

                        if (runCounter == 0) {
                            wristAngleAves[0] = wristAngleDeg;
                            apertureAves[0] = handAperture;
                        }
                        handAperture = cylcoeffs.values[6] * 2 + 0.02;
                        apertureAves[0] = apertureSmoothing * apertureAves[0] + (1 - apertureSmoothing) * handAperture;
                        wristAngleAves[0] = angleSmoothing * wristAngleAves[0] + (1 - angleSmoothing) * wristAngleDeg;

                        //cout << "angle: " << wristAngleAve << endl;

                        shapeCounters[0]++;
                        isCyl = true;
                        LastObjectCoeffs = cylcoeffs;
                        break;
                        }

                    case 1: // Sphere
                        {
                        pcl::visualization::PointCloudColorHandlerCustom<PointT> cloudDownsampled_color_h(sphpoints, 255, 0, 0);
                        viewerz->addPointCloud(sphpoints, cloudDownsampled_color_h, "Inliers", vp);
                        viewerz->addSphere(sphcoeffs);
                        wristAngleDeg = 45;
                        handAperture = sphcoeffs.values[3] * 2 + 0.02;
                        length = sphcoeffs.values[3];

                        if (runCounter == 0) {
                            apertureAves[1] = handAperture;
                        }

                        apertureAves[1] = apertureSmoothing * apertureAves[1] + (1 - apertureSmoothing) * handAperture;

                        shapeCounters[1]++;
                        isSph = true;
                        LastObjectCoeffs = sphcoeffs;
                        break;
                        }
                        

                    case 2: //Box 
                        {
                        auto [dims, eigvecs, centroids] = Ransacer.shape_box(boxpoints_vec);
                        auto[wristAngleDeg, handAperture] = Ransacer.boxangle(boxcoeffs_vec, *tablecoefficients, centroids, eigvecs);
                        //wristAngleDeg = angle;
                        //handAperture = aperture;
                        //std::cout << dims[0] << " " << dims[1] << " " << dims[2] << endl;
                        pcl::visualization::PointCloudColorHandlerCustom<PointT> boxpoints1_color_h(boxpoints_vec[0], 255, 0, 0);
                        pcl::visualization::PointCloudColorHandlerCustom<PointT> boxpoints2_color_h(boxpoints_vec[1], 0, 255, 0);
                        pcl::visualization::PointCloudColorHandlerCustom<PointT> boxpoints3_color_h(boxpoints_vec[2], 0, 0, 255);
                        viewerz->addPointCloud(boxpoints_vec[0], boxpoints1_color_h, "Inliers1", vp);
                        viewerz->addPointCloud(boxpoints_vec[1], boxpoints2_color_h, "Inliers2", vp);
                        viewerz->addPointCloud(boxpoints_vec[2], boxpoints3_color_h, "Inliers3", vp);
                        
                        viewerz->addArrow(centroids[0] + eigvecs[0][1], centroids[0], 255, 0, 0, true, "p1v1");
                        viewerz->addArrow(centroids[0] + eigvecs[0][0], centroids[0], 255, 0, 0, true, "p1v2");
                        viewerz->addArrow(centroids[1] + eigvecs[1][0], centroids[1], 0, 255, 0, true, "p2v1");
                        //viewerz->addArrow(centroids[1] + eigvecs[1][1], centroids[1] , 0, 255, 0, true, "p2v2");
                        //viewerz->addArrow(centroids[2] + eigvecs[2][0], centroids[2] , 0, 0, 255, true, "p3v1");
                        viewerz->addArrow(centroids[2] + eigvecs[2][1], centroids[2] , 0, 0, 255, true, "p3v2");

                        length = Ransacer.normPointT(eigvecs[1][0]);
                        width = Ransacer.normPointT(eigvecs[0][1]);
                        depth = Ransacer.normPointT(eigvecs[2][1]);
                        if (length > 1)
                            length = 0;

                        if (depth > 1)
                            depth = 0;

                        if (width > 1)
                            width = 0;


                        if (runCounter == 0) {
                            wristAngleAves[2] = wristAngleDeg;
                            apertureAves[2] = handAperture;
                        }

                        wristAngleAves[2] = angleSmoothing * wristAngleAves[2] + (1 - angleSmoothing) * wristAngleDeg;
                        apertureAves[2] = apertureSmoothing * apertureAves[2] + (1 - apertureSmoothing) * handAperture;
                        //cout << wristAngleAve << endl;
                        //cout << 'BOX' << endl;

                        shapeCounters[2]++;
                        isBox = true;
                        LastObjectCoeffs = boxcoeffs_vec[0]; // this will need fixing, makes no sense to only take the first entry.
                        break;
                        }
                    }
                
                    runCounter = shapeCounters[0] + shapeCounters[1] + shapeCounters[2];

                    if(runCounter == 20){

                    int shapeChosenAve = std::max_element(shapeCounters.begin(), shapeCounters.end()) - shapeCounters.begin(); //choose shape
                    handAperture = apertureAves[shapeChosenAve];
                    wristAngleDeg = wristAngleAves[shapeChosenAve];

                    float maxhandaperture = 0.11; //max hand opening distance of 11 cm
                    int gripclosure;
                    if(handAperture < 0.11){
                        gripclosure = (handAperture / maxhandaperture) * 100;

                        float rotationstep = 1.6; // 160 deg/100 steps 
                        int rotation = wristAngleDeg / rotationstep;
                        if (gripclosure < 100 && rotation < 100) {
                            std::string my_string = "2,0," + std::to_string(gripclosure) + "," + std::to_string(rotation) + ",0,50,50,50";
                            // Position mode, palmar grip, grip closure, wrist rotation, wrist flexion, max grip speed, max rotation speed, max flexion speed.

                            //int sendOk = sendto(out, my_string.c_str(), my_string.size() + 1, 0, (sockaddr*)&server, sizeof(server));

                            cout << my_string << " \n";
                        }
                    }
                    runCounter = 0;
                    apertureAves = { 0,0,0 };
                    wristAngleAves = { 0,45,0 };
                    shapeCounters = { 0,0,0 };
                 }

                    
                    
                    viewerz->spinOnce(1, true);

                    return;
                }
                else {
                    return;
                }

            }
            else {
            
                return;
            }
        }

        else {
            return;
        }
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

bool Camerahandler::autoGraspSimple(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {
    bool objInRange = false;
    float Croppinglimit = 0.05; // defines x-min, x-max, y-min, y-max,

    pcl::PassThrough<PointT> pass(true);

    pass.setInputCloud(cloud);
    pass.setFilterFieldName("x");
    pass.setFilterLimits(-Croppinglimit, Croppinglimit);
    pass.filter(*cloudFilteredX);

    pass.setInputCloud(cloudFilteredX);
    pass.setFilterFieldName("y");
    pass.setFilterLimits(-Croppinglimit, Croppinglimit);
    pass.filter(*cloudFilteredXY);

    pass.setInputCloud(cloudFilteredXY);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(0.0, 0.15);
    pass.filter(*cloudFilteredXYZ);

    Eigen::Vector4f centroid;
    pcl::compute3DCentroid(*cloudFilteredXYZ, centroid);
    float distToCent = sqrt(pow(centroid.x(),2) + pow(centroid.y(),2) + pow(centroid.z(),2));

    if (distToCent < 0.012 && distToCent > 0) {

        objInRange = true;
    }

    return objInRange;
}

bool Camerahandler::autoGraspCyl(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) { //Finds smallest distance to cylinders line axis 
    bool shouldClose = false;
    PointT CylPoint(LastObjectCoeffs.values[0], LastObjectCoeffs.values[1], LastObjectCoeffs.values[2]); // The Point on the axis
    PointT CylVec(LastObjectCoeffs.values[3], LastObjectCoeffs.values[4], LastObjectCoeffs.values[5]); // The Point on the axis

    double t = -1 * (CylPoint.x * CylVec.x + CylPoint.y * CylVec.y + CylPoint.z * CylVec.z) / (pow(CylVec.x, 2) + pow(CylVec.y, 2) + pow(CylVec.z, 2));

    float dist = sqrt(pow(CylPoint.x + CylVec.x*t, 2) + pow(CylPoint.y + CylVec.y * t, 2) + pow(CylPoint.z + CylVec.z * t, 2));

    if (dist < maxGrasDist) {
        shouldClose = true;
    }
    return shouldClose;
}

bool Camerahandler::autoGraspSph(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {
    bool ShouldClose = false;

    PointT Center(LastObjectCoeffs.values[0], LastObjectCoeffs.values[1], LastObjectCoeffs.values[2]);

    if (std::sqrt(Center.x * Center.x + Center.y * Center.y + Center.z * Center.z) < maxGrasDist) //If center is within grasping distance hand closes.
    {
        ShouldClose = true;
    }

    return ShouldClose;
}

bool Camerahandler::autoGraspBox(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {

    return false;
}

pcl::visualization::PCLVisualizer::Ptr Camerahandler::initViewer()
{
    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("3D Viewer", true));
    viewer->createViewPort(0.0, 0.0, 1.0, 1.0, vp);
    viewer->setCameraPosition(0.0, 0.0, -0.5, 0.0, -1.0, 0.0, vp);
    viewer->setSize(800, 600);
    viewer->setBackgroundColor(0, 0, 0, vp);
    viewer->addCoordinateSystem(0.25); // Global reference frame (on-camera)

    return viewer;
}
