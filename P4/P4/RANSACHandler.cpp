//
// Created by mogens on 25/04/2022.
//



#include "RANSACHandler.h"

pcl::ModelCoefficients::Ptr coefficients_cylinder(new pcl::ModelCoefficients);
pcl::PointIndices :: Ptr inliers_cylinder(new pcl::PointIndices);
pcl::PointCloud<PointT>::Ptr cylPoints(new pcl::PointCloud<PointT>);
//pcl::PointIndices inliers_plane1(new pcl::PointIndices), inliers_plane2(new pcl::PointIndices), inliers_plane3(new pcl::PointIndices);

//box variables
pcl::ModelCoefficients coefficients_planes1 /*(new pcl::ModelCoefficients)*/, coefficients_planes2 /*(new pcl::ModelCoefficients)*/, coefficients_planes3/*(new pcl::ModelCoefficients)*/;

pcl::PointIndices::Ptr inliers_plane1(new pcl::PointIndices), inliers_plane2(new pcl::PointIndices), inliers_plane3(new pcl::PointIndices);
pcl::ModelCoefficients::Ptr coefficients_box(new pcl::ModelCoefficients);
pcl::PointCloud<PointT>::Ptr cloud_plane1(new pcl::PointCloud<PointT>()), cloud_plane2(new pcl::PointCloud<PointT>()), cloud_plane3(new pcl::PointCloud<PointT>());
pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>());
pcl::SACSegmentation<pcl::PointXYZ> seg_box; //pcl::SACSegmentationFromNormals<PointT, pcl::Normal> seg_box;
pcl::ExtractIndices<PointT> extract_box;
pcl::NormalEstimation <PointT, pcl::Normal> ne;
std::vector <pcl::PointIndices::Ptr> inliers_vec{ inliers_plane1, inliers_plane2, inliers_plane3 };
std::vector <pcl::ModelCoefficients> plane_coe_vec{ coefficients_planes1, coefficients_planes2, coefficients_planes3 };
std::vector <pcl::PointCloud<pcl::PointXYZ>::Ptr> plane_vec{ cloud_plane1, cloud_plane2, cloud_plane3 };
std::vector <pcl::Indices> inlier_indicies;


pcl::PointCloud<pcl::Normal>::Ptr first(new pcl::PointCloud<pcl::Normal>), second(new pcl::PointCloud<pcl::Normal>), third(new pcl::PointCloud<pcl::Normal>), fourth(new pcl::PointCloud<pcl::Normal>);
std::vector <pcl::PointCloud<pcl::Normal>::Ptr> cloud_normals{ first, second, third, fourth };
pcl::PointCloud<pcl::PointXYZ>::Ptr first_cloud(new pcl::PointCloud<pcl::PointXYZ>), second_cloud(new pcl::PointCloud<pcl::PointXYZ>), third_cloud(new pcl::PointCloud<pcl::PointXYZ>), fourth_cloud(new pcl::PointCloud<pcl::PointXYZ>);
std::vector <pcl::PointCloud<pcl::PointXYZ>::Ptr> plane_clouds{ first_cloud, second_cloud, third_cloud, fourth_cloud };
pcl::SampleConsensusModelPlane<pcl::PointXYZ>::Ptr model(new pcl::SampleConsensusModelPlane<pcl::PointXYZ>(plane_clouds[0]));
pcl::PointCloud<PointT>::Ptr inlierPoints(new pcl::PointCloud<PointT>);

pcl::RandomSampleConsensus<pcl::PointXYZ> sac(model, 1);

//shape Box variables
pcl::PointCloud<PointT>::Ptr demeanedCloud(new pcl::PointCloud<PointT>);
pcl::PointCloud<PointT>::Ptr TransformedCloud(new pcl::PointCloud<PointT>);

//Sphere variables
pcl::ModelCoefficients::Ptr coefficients_sphere(new pcl::ModelCoefficients);

pcl::PointIndices::Ptr inliers_sphere(new pcl::PointIndices);
pcl::PointCloud<PointT>::Ptr shpPoints(new pcl::PointCloud<PointT>);



RANSACHandler::RANSACHandler(pcl::PointCloud<PointT>::Ptr& cloud) {
    Ptcloud = cloud;   
}


PointT operator* (const PointT Point1, const float multiplier) {
    return PointT(Point1.x * multiplier, Point1.y * multiplier, Point1.z * multiplier);
}


std::array<float, 2> RANSACHandler::getPointCloudExtremes(const pcl::PointCloud<PointT>& cloud, pcl::PointXYZ center, pcl::PointXYZ direction)
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

float RANSACHandler::dotProduct(pcl::PointXYZ a, pcl::PointXYZ b)
{
    return a.x * b.x + a.y * b.y + a.z * b.z;
}

inline pcl::PointXYZ RANSACHandler::crossProduct(pcl::PointXYZ a, pcl::PointXYZ b) {
    pcl::PointXYZ result;
    result.x = a.y * b.z - a.z * b.y;
    result.y = a.z * b.x - a.x * b.z;
    result.z = a.x * b.y - a.y * b.x;

    return result;
}

float RANSACHandler::normPointT(pcl::PointXYZ c)
{
    return std::sqrt(c.x * c.x + c.y * c.y + c.z * c.z);
}

bool RANSACHandler::Checkorthogonal(std::vector<pcl::ModelCoefficients> coeefs, int i) {
    cout << "coeffs size: " << coeefs.size() << endl;
    bool isOrthogonal = false;
    float threshold = 0.1;
    std::vector<float> dotProducts = {0,0,0};

    if (i < 3) {
        isOrthogonal = true;
    return isOrthogonal;
    }

    PointT normal1(coeefs[0].values[0], coeefs[0].values[1], coeefs[0].values[2]);

    PointT normal2(coeefs[1].values[0], coeefs[1].values[1] , coeefs[1].values[2] );
    dotProducts.push_back(dotProduct(normal1, normal2));

    if (i == 3) {
    PointT normal3(coeefs[2].values[0], coeefs[2].values[1], coeefs[2].values[2]);
    dotProducts.push_back(dotProduct(normal1, normal3));
    dotProducts.push_back(dotProduct(normal2, normal3));
    }
    float biggestdot = *std::max_element(dotProducts.begin(), dotProducts.end());
    //cout << " Dot " << biggestdot << endl;
    if (biggestdot < threshold) {
        isOrthogonal = true;
    }     
        return isOrthogonal;
}

void RANSACHandler::shape_cyl(pcl::ModelCoefficients& cyl, const pcl::ModelCoefficients& coefficients, const pcl::PointCloud<PointT>& cloud)
{
    pcl::PointXYZ cnt(coefficients.values[0], coefficients.values[1], coefficients.values[2]);
    pcl::PointXYZ cnt_direc(coefficients.values[3], coefficients.values[4], coefficients.values[5]);
    std::array<float, 2> point_ext(getPointCloudExtremes(cloud, cnt, cnt_direc));

    //Calculate height based on extremes of cylinder
    float cylinder_height = point_ext[0]-point_ext[1];

    cyl.values.push_back(cnt.x);                          //point_on_axis.x : the X coordinate of a point located on the cylinder axis
    cyl.values.push_back(cnt.y);                          //point_on_axis.y : the Y coordinate of a point located on the cylinder axis
    cyl.values.push_back(cnt.z);                          //point_on_axis.z : the Z coordinate of a point located on the cylinder axis
    cyl.values.push_back(cnt_direc.x);                    //axis_direction.x : the X coordinate of the cylinder's axis direction
    cyl.values.push_back(cnt_direc.y);                    //axis_direction.y : the Y coordinate of the cylinder's axis direction
    cyl.values.push_back(cnt_direc.z);                    //axis_direction.z : the Z coordinate of the cylinder's axis direction
    cyl.values.push_back(coefficients.values[6]);           //radius : the cylinder's radius in meter
    cyl.values.push_back(cylinder_height);
}

tuple <std::vector <float>,std::vector<std::vector<PointT>>, std::vector<PointT>>  RANSACHandler::shape_box(std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> boxinliers_vec) {
//std::vector <float>  RANSACHandler::shape_box(std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> boxinliers_vec) {
    //for loop with runs the amount of planes
    
    std::vector <float> boxDim = {0,0,0};                  //boxDim[0] = width; BoxDim[1] = hight
    std::vector <pcl::PointIndices::Ptr> model_indices = {inliers_plane1, inliers_plane2, inliers_plane3};
    std::vector <std::vector<float>> dimension_vector = {{0,0},{0,0},{0,0}};
    std::vector <float> lengths = { 0,0,0 };
    std::vector <PointT> eigVec = { pcl::PointXYZ(0,0,0), pcl::PointXYZ(0,0,0) ,pcl::PointXYZ(0,0,0) };
    std::vector <std::vector< PointT>> eigVectors;
    std::vector <PointT> centroids;
    Eigen::Matrix3f eigen_vec;
    Eigen::Vector3f eigen_val;
    float planes[3][4];
    float box_length, box_width, box_height;
    float plane_length, plane_width;

    for(int i = 0; i < boxinliers_vec.size(); i++) {

        demeanedCloud->clear();
        TransformedCloud->clear();

        Eigen::Vector4f centroid;
        pcl::compute3DCentroid(*boxinliers_vec[i], centroid);
        pcl::PointXYZ cnt_coord(centroid[0], centroid[1], centroid[2]);
        centroids.push_back(cnt_coord);
        pcl::PointXYZ demeanedpoint;

        for (int idx = 0; idx < boxinliers_vec[i]->points.size(); idx++) {
            demeanedpoint.x = boxinliers_vec[i]->points[idx].x - cnt_coord.x;
            demeanedpoint.y = boxinliers_vec[i]->points[idx].y - cnt_coord.y;
            demeanedpoint.z = boxinliers_vec[i]->points[idx].z - cnt_coord.z;
            demeanedCloud->push_back(demeanedpoint);
        }
        
        Eigen::Vector4f pca_cnt;
        pcl::compute3DCentroid(*demeanedCloud, pca_cnt);
        Eigen::Matrix3f cov_mat;
        pcl::computeCovarianceMatrixNormalized(*demeanedCloud, pca_cnt, cov_mat);
        Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigen_solver(cov_mat, Eigen::ComputeEigenvectors);
        Eigen::Matrix3f eig_vec = eigen_solver.eigenvectors();
        Eigen::Vector3f eig_val = eigen_solver.eigenvalues();
        eig_vec.col(2) = eig_vec.col(0).cross(eig_vec.col(1));
        //std::cout << "Eigen vectors: " << eig_vec << " Eigen values: " << eig_val << std::endl;

        std::vector<float> eig_val_conv = {eig_val[0, 0], eig_val[0, 1], eig_val[0, 2]};
        auto maxElementIndex = std::max_element(eig_val_conv.begin(), eig_val_conv.end()) - eig_val_conv.begin();
        eig_val_conv.erase(eig_val_conv.begin() + maxElementIndex);
        auto secondBiggestElementsIndex = std::max_element(eig_val_conv.begin(), eig_val_conv.end()) - eig_val_conv.begin();

        pcl::PointXYZ PC1(eig_vec(0, maxElementIndex), eig_vec(1, maxElementIndex), eig_vec(2, maxElementIndex)); //principal component 1
        pcl::PointXYZ PC2(eig_vec(0, secondBiggestElementsIndex), eig_vec(1, secondBiggestElementsIndex), eig_vec(2, secondBiggestElementsIndex)); //principal component 2

        pcl::PointXYZ transformedPoint;
        for (int idx = 0; idx < demeanedCloud->points.size(); idx++) {
            transformedPoint.x = dotProduct(demeanedCloud->points[idx], PC1); //demeanedCloud->points[idx].x * PC1.x + demeanedCloud->points[idx].y * PC1.y + demeanedCloud->points[idx].z * PC1.z; //these should be done using dot product
            transformedPoint.y = dotProduct(demeanedCloud->points[idx], PC2); //demeanedCloud->points[idx].x * PC2.x + demeanedCloud->points[idx].y * PC2.y + demeanedCloud->points[idx].z * PC2.z;
            transformedPoint.z = 0; //The depth of the planes should be zero.
            TransformedCloud->push_back(transformedPoint);
        }
        
        std::array<float, 2> planedim1 = getPointCloudExtremes(*TransformedCloud, cnt_coord, PC1); //max and min along PC1
        std::array<float, 2> planedim2 = getPointCloudExtremes(*TransformedCloud, cnt_coord, PC2);

        
        dimension_vector[i][0] = abs(planedim1[0] - planedim1[1]); //save x 
        dimension_vector[i][1] = abs(planedim2[0] - planedim2[1]); //save y

        eigVec[0] = (PC1 * (dimension_vector[i][0]/2)); //scale the eigenvectors
        eigVec[1] = (PC2 * (dimension_vector[i][1]/2));
        eigVectors.push_back(eigVec);

        cout << "Box dims: " << dimension_vector[i][0] << "  " << dimension_vector[i][1] << endl;

    }

    cout << "\n";
    
    for(int i = 0; i < 3; i++){
        lengths[i] = dimension_vector[i][0];
    }

    auto maxElementIndex = std::max_element(lengths.begin(), lengths.end()) - lengths.begin();
    box_length = lengths[maxElementIndex];
    lengths.erase(lengths.begin() + maxElementIndex);
    maxElementIndex = std::max_element(lengths.begin(), lengths.end()) - lengths.begin();
    box_width = lengths[maxElementIndex];
    lengths.erase(lengths.begin() + maxElementIndex);
    maxElementIndex = std::max_element(lengths.begin(), lengths.end()) - lengths.begin();
    box_height = lengths[maxElementIndex];
    
    //NaN error
    if (box_length != box_length) {
        box_length = 0;
        boxDim[0] = box_length;
    }
    else {
        boxDim[0] = box_length;
    }
    //NaN error
    if (box_width != box_width) {
        box_width = 0;
        boxDim[1] = box_width;
    }
    else {
        boxDim[1] = box_width;
    }
    if (box_height != box_height) {
        box_height = 0;
        boxDim[2] = box_height;
    }
    else {
        boxDim[2] = box_height;
    }

    return { boxDim, eigVectors,centroids};
   }

tuple <float, pcl::ModelCoefficients, pcl::PointCloud<pcl::PointXYZ>::Ptr> RANSACHandler::check_cyl(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {

    pcl::NormalEstimation<PointT, pcl::Normal> ne;
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);
    pcl::SACSegmentationFromNormals<PointT, pcl::Normal> seg_cylinder;
    pcl::ExtractIndices<PointT> extract_cylinder;
    pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>());

    ne.setSearchMethod(tree);
    ne.setInputCloud(cloud);
    ne.setKSearch(50);
    ne.compute(*cloud_normals);

    seg_cylinder.setOptimizeCoefficients(true);
    seg_cylinder.setMethodType(pcl::SAC_RANSAC);
    seg_cylinder.setModelType(pcl::SACMODEL_CYLINDER);
    seg_cylinder.setNormalDistanceWeight(0.01);
    seg_cylinder.setMaxIterations(500);
    seg_cylinder.setDistanceThreshold(0.005);
    seg_cylinder.setRadiusLimits(0.01, 0.120);
    seg_cylinder.setInputCloud(cloud);
    seg_cylinder.setInputNormals(cloud_normals);

    //Obtain the inliers of cylinder
    seg_cylinder.segment(*inliers_cylinder, *coefficients_cylinder);

    // Save the cylinder inliers                                       
    extract_cylinder.setIndices(inliers_cylinder);
    extract_cylinder.setInputCloud(cloud);
    extract_cylinder.setNegative(false);
    extract_cylinder.filter(*cylPoints);

    //cloudpoint ratio
    float cylinderRatio = (double)cylPoints->points.size() / (double)cloud->points.size() * 100;
    return { cylinderRatio, *coefficients_cylinder, cylPoints };
    }

tuple <float, std::vector<pcl::ModelCoefficients>, std::vector <pcl::PointCloud<pcl::PointXYZ>::Ptr>> RANSACHandler::check_box(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {
    
    plane_clouds[0] = cloud;
    
    seg_box.setOptimizeCoefficients(true);
    seg_box.setMethodType(pcl::SAC_LMEDS);
    seg_box.setModelType(pcl::SACMODEL_PLANE);
    seg_box.setMaxIterations(1000);
    seg_box.setDistanceThreshold(0.005);

    double ratio_planes[3];
    ratio_planes[0] = 0;
    ratio_planes[1] = 0;
    ratio_planes[2] = 0;

    int i = 0;
        int nPoints = cloud->points.size();
    while (plane_clouds[i]->size() > nPoints * 0.1 && i < 3)
    {
        seg_box.setInputCloud(plane_clouds[i]);
        seg_box.segment(*inliers_vec[i], plane_coe_vec[i]);

        //Save plane inliers 
        pcl::ExtractIndices<pcl::PointXYZ> extract_plane;
        extract_plane.setInputCloud(plane_clouds[i]);
        extract_plane.setIndices(inliers_vec[i]);
        extract_plane.setNegative(false);
        extract_plane.filter(*plane_vec[i]);
        //save the remaining cloud points
        extract_plane.setNegative(true);
        extract_plane.filter(*plane_clouds[i+1]);

        //Ratio at which points are considered belonging to the plane
        ratio_planes[i] = (double)plane_vec[i]->points.size() / (double)nPoints;

        i++;
    }
    
    float boxRatio = ((double)ratio_planes[0] + (double)ratio_planes[1] + (double)ratio_planes[2]) * 100;
   
    if (!Checkorthogonal(plane_coe_vec, i)) {
        boxRatio = 0;
    }

    *inlierPoints = *plane_vec[0] +  *plane_vec[1] + *plane_vec[2];
    
    return { boxRatio, plane_coe_vec, plane_vec };
}

tuple <float, pcl::ModelCoefficients, pcl::PointCloud<pcl::PointXYZ>::Ptr> RANSACHandler::check_sph(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud) {

    pcl::NormalEstimation<PointT, pcl::Normal> ne;
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);
    pcl::SACSegmentation<PointT> seg_sph;
    pcl::ExtractIndices<PointT> extract_sph;
    pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>());

    ne.setSearchMethod(tree);
    ne.setInputCloud(cloud);
    ne.setKSearch(50);
    ne.compute(*cloud_normals);

    seg_sph.setOptimizeCoefficients(true);
    seg_sph.setMethodType(pcl::SAC_RANSAC);
    seg_sph.setModelType(pcl::SACMODEL_SPHERE);
    //seg_sph.setNormalDistanceWeight(0.01);
    seg_sph.setMaxIterations(500);
    seg_sph.setDistanceThreshold(0.0012);
    seg_sph.setRadiusLimits(0.005, 0.150);
    seg_sph.setInputCloud(cloud);
    //seg_sph.setInputNormals(cloud_normals);

    //Obtain the inliers of sphere
    seg_sph.segment(*inliers_sphere, *coefficients_sphere);
    /*if (coefficients_sphere == nullptr)
    {   
        //coefficients_sphere = new pcl::ModelCoefficients();
        return { *coefficients_sphere, coefficients_sphere->values[3]};
    }*/
    // Save the sphere inliers                                       
    extract_sph.setIndices(inliers_sphere);
    extract_sph.setInputCloud(cloud);
    extract_sph.setNegative(false);
    extract_sph.filter(*shpPoints);

    //cloudpoint ratio
    
    float sphRatio = (double)shpPoints->points.size() / (double)cloud->points.size() * 100;
    return { sphRatio, *coefficients_sphere, shpPoints };
}
