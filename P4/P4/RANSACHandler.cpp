//
// Created by mogens on 25/04/2022.
//



#include "RANSACHandler.h"

pcl::ModelCoefficients::Ptr coefficients_cylinder(new pcl::ModelCoefficients);
pcl::PointIndices :: Ptr inliers_cylinder(new pcl::PointIndices);
pcl::PointCloud<PointT>::Ptr cylPoints(new pcl::PointCloud<PointT>);
//pcl::PointIndices inliers_plane1(new pcl::PointIndices), inliers_plane2(new pcl::PointIndices), inliers_plane3(new pcl::PointIndices);

//box variables
pcl::ModelCoefficients::Ptr coefficients_planes1(new pcl::ModelCoefficients), coefficients_planes2(new pcl::ModelCoefficients), coefficients_planes3(new pcl::ModelCoefficients);
pcl::PointIndices::Ptr inliers_plane1(new pcl::PointIndices), inliers_plane2(new pcl::PointIndices), inliers_plane3(new pcl::PointIndices);
pcl::ModelCoefficients::Ptr coefficients_box(new pcl::ModelCoefficients);
pcl::PointCloud<PointT>::Ptr cloud_plane1(new pcl::PointCloud<PointT>()), cloud_plane2(new pcl::PointCloud<PointT>()), cloud_plane3(new pcl::PointCloud<PointT>());
pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>());
pcl::SACSegmentation<pcl::PointXYZ> seg_box; //pcl::SACSegmentationFromNormals<PointT, pcl::Normal> seg_box;
pcl::ExtractIndices<PointT> extract_box;
pcl::NormalEstimation <PointT, pcl::Normal> ne;
std::array<pcl::PointIndices::Ptr, 3> inliers_array;
std::array<pcl::ModelCoefficients::Ptr, 3> plane_coe_array;
std::array<pcl::PointCloud<pcl::PointXYZ>::Ptr, 3> plane_array;
std::vector <pcl::Indices> inlier_indicies;


pcl::PointCloud<pcl::Normal>::Ptr first(new pcl::PointCloud<pcl::Normal>), second(new pcl::PointCloud<pcl::Normal>), third(new pcl::PointCloud<pcl::Normal>), fourth(new pcl::PointCloud<pcl::Normal>);
std::vector <pcl::PointCloud<pcl::Normal>::Ptr> cloud_normals{ first, second, third, fourth };
pcl::PointCloud<pcl::PointXYZ>::Ptr first_cloud(new pcl::PointCloud<pcl::PointXYZ>), second_cloud(new pcl::PointCloud<pcl::PointXYZ>), third_cloud(new pcl::PointCloud<pcl::PointXYZ>), fourth_cloud(new pcl::PointCloud<pcl::PointXYZ>);
std::vector <pcl::PointCloud<pcl::PointXYZ>::Ptr> plane_clouds{ first_cloud, second_cloud, third_cloud, fourth_cloud };
pcl::SampleConsensusModelPlane<pcl::PointXYZ>::Ptr model(new pcl::SampleConsensusModelPlane<pcl::PointXYZ>(plane_clouds[0]));
pcl::PointCloud<PointT>::Ptr inlierPoints(new pcl::PointCloud<PointT>);

pcl::RandomSampleConsensus<pcl::PointXYZ> sac(model, 1);

//Sphere variables
pcl::ModelCoefficients::Ptr coefficients_sphere(new pcl::ModelCoefficients);

pcl::PointIndices::Ptr inliers_sphere(new pcl::PointIndices);
pcl::PointCloud<PointT>::Ptr shpPoints(new pcl::PointCloud<PointT>);



RANSACHandler::RANSACHandler(pcl::PointCloud<PointT>::Ptr& cloud) {
    Ptcloud = cloud;   
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

pcl::PointXYZ RANSACHandler::crossProduct(pcl::PointXYZ a, pcl::PointXYZ b) {
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

std::vector <float> RANSACHandler::shape_box(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {

    //for loop with runs the amount of planes
    std::vector <float> boxDim = {0,0,0};                  //boxDim[0] = width; BoxDim[1] = hight
    std::vector<pcl::PointIndices::Ptr> model_indices = {inliers_plane1, inliers_plane2, inliers_plane3};
    std::vector<std::vector<float>> dimension_vector;
    Eigen::Matrix3f eigen_vec;
    Eigen::Vector3f eigen_val;
    float planes[3][4];
    float box_length, box_width, box_height;
    float plane_length, plane_width;

    for(int i = 0; i < 3; i++){
        for(int j = 0; j < 4; j++){
            planes[i][j] = coefficients_planes1->values[j];
        }
    }


    for(int i = 0; i < 3; i++){

        float p_cx = plane_centerPoint_x[i];
        float p_cy = plane_centerPoint_y[i];
        float p_cz = plane_centerPoint_z[i];


        Eigen::Vector4f centroid;
        pcl::compute3DCentroid(*cloud, centroid);
        pcl::PointXYZ cnt_coord(centroid[0], centroid[1], centroid[2]);
        /*
        * The eigenvector corresponding to the largest eigenvalue should be the direction in of the longest dimension of the plane
        * The eigenvector corresponding to the next-largest eigenvalue should be the direction of the second dimension of the plane
         * The test should return the correct eigenvectors, but will have to be tested, otherwise another method has to be used
         * This is sadly just a test due to not knowning what the output is nor what it is based upon
        */

        pcl::PointCloud<pcl::PointXYZ>::Ptr cloudPCAprojection (new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PCA<pcl::PointXYZ> pca;
        pca.setInputCloud(cloud);
        pca.setIndices(inliers_array[i]);
        eigen_vec = pca.getEigenVectors();
        eigen_val = pca.getEigenValues();
        std::cout << "Eigen vectors: " << eigen_vec << "Eigen values: " << eigen_val << std::endl;

        Eigen::Vector4f pca_cnt;
        pcl::compute3DCentroid(*cloud, pca_cnt);
        Eigen::Matrix3f cov_mat;
        computeCovarianceMatrixNormalized(*cloud, pca_cnt, cov_mat);
        Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigen_solver(cov_mat, Eigen::ComputeEigenvectors);
        Eigen::Matrix3f eig_vec = eigen_solver.eigenvectors();
        Eigen::Vector3f eig_val = eigen_solver.eigenvalues();
        eig_vec.col(2) = eig_vec.col(0).cross(eig_vec.col(1));
        std::cout << "Eigen vectors: " << eig_vec << " Eigen values: "  << eig_val << std::endl;

        std::vector<float> eig_val_conv = {eig_val[0, 0], eig_val[0, 1], eig_val[0, 2]};
        auto maxElementIndex = std::max_element(eig_val_conv.begin(), eig_val_conv.end()) - eig_val_conv.begin();
        pcl::PointXYZ eig_vec_direc(eig_vec(0, maxElementIndex), eig_vec(1, maxElementIndex), eig_vec(2, maxElementIndex));

        //Saving the length and width of each plane:
        pcl::PointCloud<pcl::PointXYZ>::Ptr Planecloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::ExtractIndices<pcl::PointXYZ> extracter;
        extracter.setIndices(model_indices[i]);
        extracter.setInputCloud(cloud);
        extracter.setNegative(false);
        extracter.filter(*Planecloud);

        std::array<float, 2> length = getPointCloudExtremes(*Planecloud, cnt_coord, eig_vec_direc);
        plane_length = length[0]-length[1];
        eig_val_conv.erase(eig_val_conv.begin() + maxElementIndex);
        /*
        maxElementIndex = std::max_element(eig_val_conv.begin(), eig_val_conv.end()) - eig_val_conv.begin();
        pcl::PointXYZ eig_vec_direc1(eig_vec[0, maxElementIndex], *eig_vec[1][maxElementIndex], eig_vec[2, maxElementIndex]);
        std::array<float, 2> width = getPointCloudExtremes(*Planecloud, cnt_coord, eig_vec_direc1);
        plane_width = width[0]- width[1];

        std::vector<float> dim = {plane_length, plane_width};
        dimension_vector.push_back(dim);
        */
    }

    
     //* This is gonna be wierd, but here we go
     //* Save the length values of each plane, compare the size and take the largest for "box length"
     //* Do the same for width and height after the element has been removed, NOTE: lengths will always be the largest!
     
    
    std::vector<float> lengths;
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

    return boxDim;

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
    seg_cylinder.setDistanceThreshold(0.003);
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

tuple <float, pcl::ModelCoefficients, pcl::PointCloud<pcl::PointXYZ>::Ptr> RANSACHandler::check_box(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {

    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne_BOX;
    plane_clouds[0] = cloud;

    int nPoints = cloud->points.size();
    int i = 0;
    double ratio_planes[3];

    plane_coe_array[0] = coefficients_planes1;
    plane_coe_array[1] = coefficients_planes2;
    plane_coe_array[2] = coefficients_planes3;
    plane_array[0] = cloud_plane1;
    plane_array[1] = cloud_plane2;
    plane_array[2] = cloud_plane3;
    
    ne_BOX.setSearchMethod(tree);
    ne_BOX.setInputCloud(cloud);
    ne_BOX.setKSearch(cloud->width/4);
    ne_BOX.compute(*cloud_normals[0]);

    seg_box.setOptimizeCoefficients(true);
    seg_box.setMethodType(pcl::SAC_LMEDS);
    seg_box.setModelType(pcl::SACMODEL_PLANE);
    seg_box.setMaxIterations(1000);
    seg_box.setDistanceThreshold(0.0005);
    
    inliers_array[0] = inliers_plane1;
    inliers_array[1] = inliers_plane2;
    inliers_array[2] = inliers_plane3;
    ratio_planes[0] = 0;
    ratio_planes[1] = 0;
    ratio_planes[2] = 0;

    while (plane_clouds[i]->size() > nPoints * 0.1 && i < 3)
    {
        seg_box.setInputCloud(plane_clouds[i]);
        //seg_box.setInputNormals(cloud_normals[i]);
        seg_box.segment(*inliers_array[i], *plane_coe_array[i]);

        //Save plane inliers 
        pcl::ExtractIndices<pcl::PointXYZ> extract_plane;
        extract_plane.setInputCloud(plane_clouds[i]);
        extract_plane.setIndices(inliers_array[i]);
        extract_plane.setNegative(false);
        extract_plane.filter(*plane_array[i]);
        //save the remaining cloud points
        extract_plane.setNegative(true);
        extract_plane.filter(*plane_clouds[i+1]);


        //Ratio at which points are considered belonging to the plane
        ratio_planes[i] = (double)plane_array[i]->points.size() / (double)nPoints;

        //Compute the center point and store it in the plane_centerPoint arrays
        Eigen::Vector4f centroid;
        pcl::compute3DCentroid(*plane_clouds[i], centroid);
        plane_centerPoint_x[i] = centroid[0];
        plane_centerPoint_y[i] = centroid[1];
        plane_centerPoint_z[i] = centroid[2];

        // Remove the planar inliers from normals
        pcl::ExtractIndices<pcl::Normal> extract_normals_box;
        extract_normals_box.setInputCloud(cloud_normals[i]);
        extract_normals_box.setIndices(inliers_array[i]);
        extract_normals_box.setNegative(true);
        extract_normals_box.filter(*cloud_normals[i+1]);

        i++;
    }
    //pcl::PointXYZ cnt_coord(centroid[0], centroid[1], centroid[2]);
    
    //pcl::PointXYZ plane2Normal(plane_coe_array[1]->values[0], plane_coe_array[1]->values[1], plane_coe_array[1]->values[2]); 
    //pcl::PointXYZ plane3Normal(plane_coe_array[2]->values[0], plane_coe_array[2]->values[1], plane_coe_array[2]->values[2]);
    //pcl::PointXYZ plane1Normal(plane_coe_array[0]->values[0], plane_coe_array[0]->values[1], plane_coe_array[0]->values[2]);

    //cout << dotProduct(plane1Normal, plane2Normal) << endl;
    //cout << dotProduct(plane1Normal, plane3Normal) << endl;
    //cout << dotProduct(plane2Normal, plane3Normal) << endl;
    /*
    cout << "plane coeffs " << endl;
       
    cout << " Inliers: " << plane_array[0]->width << " OG cloud size " << plane_clouds[0]->size() << endl;
    cout << plane_coe_array[0]->values[0] << " " << plane_coe_array[0]->values[1] << " " << plane_coe_array[0]->values[2] << endl;

    if(plane_coe_array[1]->values.size() > 0){
        cout << "plane 2 " << plane_coe_array[1]->values[0] << plane_coe_array[1]->values[1] << plane_coe_array[1]->values[2] << endl;
    }

    if (plane_coe_array[2]->values.size() > 0){
        cout << "plane 3 " << plane_coe_array[2]->values[0] << plane_coe_array[2]->values[1] << plane_coe_array[2]->values[2] << "\n";
    }
    */
    float boxRatio = ((double)ratio_planes[0] + (double)ratio_planes[1] + (double)ratio_planes[2])*100;
    
    *inlierPoints = *plane_array[0] + *plane_array[1] + *plane_array[2];
    
    return { boxRatio, *plane_coe_array[0], inlierPoints };
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
