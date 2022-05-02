//
// Created by mogens on 25/04/2022.
//



#include "RANSACHandler.h"

pcl::ModelCoefficients::Ptr coefficients_sphere(new pcl::ModelCoefficients);
pcl::ModelCoefficients::Ptr coefficients_box(new pcl::ModelCoefficients);
pcl::ModelCoefficients::Ptr coefficients_cylinder(new pcl::ModelCoefficients);
pcl::PointIndices :: Ptr inliers_cylinder(new pcl::PointIndices);
pcl::PointCloud<PointT>::Ptr cylPoints(new pcl::PointCloud<PointT>);

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

float RANSACHandler::normPointT(pcl::PointXYZ c)
{
    return std::sqrt(c.x * c.x + c.y * c.y + c.z * c.z);
}

float RANSACHandler::check_cyl(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud){
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
    seg_cylinder.setMaxIterations(10000);
    seg_cylinder.setDistanceThreshold(0.0012);
    seg_cylinder.setRadiusLimits(0.005, 0.150);
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
    return cylinderRatio;
}

void RANSACHandler::shape_cyl(pcl::ModelCoefficients& cyl, const pcl::ModelCoefficients& coefficients, const pcl::PointCloud<PointT>& cloud)
{
    pcl::PointXYZ p_axis(coefficients.values[0], coefficients.values[1], coefficients.values[2]);
    pcl::PointXYZ axis(coefficients.values[3], coefficients.values[4], coefficients.values[5]);
    std::array<float, 2> arr(getPointCloudExtremes(cloud, p_axis, axis));

    pcl::PointXYZ p_low;
    p_low.x = p_axis.x + arr[0] * axis.x / normPointT(axis);
    p_low.y = p_axis.y + arr[0] * axis.y / normPointT(axis);
    p_low.z = p_axis.z + arr[0] * axis.z / normPointT(axis);
    pcl::PointXYZ n_direction;
    n_direction.x = (-arr[0] + arr[1]) * axis.x / normPointT(axis);
    n_direction.y = (-arr[0] + arr[1]) * axis.y / normPointT(axis);
    n_direction.z = (-arr[0] + arr[1]) * axis.z / normPointT(axis);

    //height
    long cylinder_height = pow((n_direction.x * n_direction.x + n_direction.y * n_direction.y + n_direction.z * n_direction.z), 0.5);

    cyl.values.push_back(p_low.x);                          //point_on_axis.x : the X coordinate of a point located on the cylinder axis
    cyl.values.push_back(p_low.y);                          //point_on_axis.y : the Y coordinate of a point located on the cylinder axis
    cyl.values.push_back(p_low.z);                          //point_on_axis.z : the Z coordinate of a point located on the cylinder axis
    cyl.values.push_back(n_direction.x);                    //axis_direction.x : the X coordinate of the cylinder's axis direction
    cyl.values.push_back(n_direction.y);                    //axis_direction.y : the Y coordinate of the cylinder's axis direction
    cyl.values.push_back(n_direction.z);                    //axis_direction.z : the Z coordinate of the cylinder's axis direction
    cyl.values.push_back(coefficients.values[6]);           //radius : the cylinder's radius in meter
    cyl.values.push_back(cylinder_height);
}

double* RANSACHandler::shape_box(const int nPlanes, const pcl::ModelCoefficients& p1, const pcl::ModelCoefficients& p2, const pcl::ModelCoefficients& p3) {
    //for loop with runs the amount of planes
    double boxDim[2];                   //boxDim[0] = width; BoxDim[1] = hight
    double planes[3][4];

    if (nPlanes > 2) {                  //if the amount of planes is more then 2
        //first plane
        planes[0][0] = p1.values[0];
        planes[0][1] = p1.values[1];
        planes[0][2] = p1.values[2];
        planes[0][3] = p1.values[3];
        //second plane
        planes[1][0] = p2.values[0];
        planes[1][1] = p2.values[1];
        planes[1][2] = p2.values[2];
        planes[1][3] = p2.values[3];
        //third plane
        planes[2][0] = p3.values[0];
        planes[2][1] = p3.values[1];
        planes[2][2] = p3.values[2];
        planes[2][3] = p3.values[3];
    }
    else {                              //if there are 2 or less planes do this.
        //first plane
        planes[0][0] = p1.values[0];
        planes[0][1] = p1.values[1];
        planes[0][2] = p1.values[2];
        planes[0][3] = p1.values[3];
        //second plane
        planes[1][0] = p2.values[0];
        planes[1][1] = p2.values[1];
        planes[1][2] = p2.values[2];
        planes[1][3] = p2.values[3];
    }


    for (int i = 0, j = nPlanes - 1; i < j; ++i) {                    //3 planes = 2 runs

        //define normal vectors
        double p1_nx, p1_ny, p1_nz, p2_nx, p2_ny, p2_nz;

        //define length and angle variables
        double c_length, a_angle, a_length;

        //find normal vectors positions
        p1_nx = planes[i][0] / (sqrt(pow(planes[i][0], 2) + pow(planes[i][1], 2) + pow(planes[i][2], 2)));
        p1_ny = planes[i][1] / (sqrt(pow(planes[i][0], 2) + pow(planes[i][1], 2) + pow(planes[i][2], 2)));
        p1_nz = planes[i][2] / (sqrt(pow(planes[i][0], 2) + pow(planes[i][1], 2) + pow(planes[i][2], 2)));

        p2_nx = planes[i][0] / (sqrt(pow(planes[i + 1][0], 2) + pow(planes[i + 1][1], 2) + pow(planes[i + 1][2], 2)));
        p2_ny = planes[i][1] / (sqrt(pow(planes[i + 1][0], 2) + pow(planes[i + 1][1], 2) + pow(planes[i + 1][2], 2)));
        p2_nz = planes[i][2] / (sqrt(pow(planes[i + 1][0], 2) + pow(planes[i + 1][1], 2) + pow(planes[i + 1][2], 2)));

        //cout << myShape << "-" << i << "; 1nx" << p1_nx << "; 1ny: " << p1_ny << "; 1nz: " << p1_nz << "; 2nx: " << p2_nx << "; 2ny: " << p2_ny << "; 2nz: " << p2_nz << std::endl;

        //C_angle = invCos((n_A ° n_B )  /  (n_A| * | n_B| ))
        //note ° = dotproduct [ a.x* b.x + a.y * b.y + a.z * b.z ]

        long double plane_dotProduct = (p1_nx * p2_nx) + (p1_ny * p2_ny) + (p1_nz * p2_nz);
        long double p1_length = sqrt((pow(p1_nx, 2) + pow(p1_ny, 2) + pow(p1_nz, 2)));
        long double p2_length = sqrt((pow(p2_nx, 2) + pow(p2_ny, 2) + pow(p2_nz, 2)));
        long double c_angle = asin(plane_dotProduct / ((p1_length * p2_length)));

        //cout << myShape << "-" << i << "; dotProduct: " << plane_dotProduct << " ; plane_1 lenght: " << p1_length << " ; plane_2 length: " << p2_length << " ; angle C: " << c_angle << std::endl;


        //Plane_centerPoint[plane_counter];
        float p1_cx = plane_centerPoint_x[i];
        float p1_cy = plane_centerPoint_y[i];
        float p1_cz = plane_centerPoint_z[i];

        float p2_cx = plane_centerPoint_x[i + 1];
        float p2_cy = plane_centerPoint_y[i + 1];
        float p2_cz = plane_centerPoint_z[i + 1];

        c_length = sqrt((pow(p2_cx, 2) - pow(p1_cx, 2)) +
            (pow(p2_cy, 2) - pow(p1_cy, 2)) +
            (pow(p2_cz, 2) - pow(p1_cz, 2)));


        //find Another angle (assuming A=B)
        a_angle = (180 - c_angle) / 2;

        //cout << myShape << "-" << i << "; a_angle: " << a_angle << std::endl;

        //find lengh of a
        a_length = (c_length * sin(a_angle)) / sin(c_angle);


        //cout << myShape << "-" << i << "; a_length: " << a_length << std::endl;

        //to mm
        boxDim[i] = a_length * 1000 * 2;


    }
    //NaN error
    if (boxDim[0] != boxDim[0]) {
        box_radius = 0;
    }
    else {
        box_radius = boxDim[0];
    }
    //NaN error
    if (boxDim[1] != boxDim[1]) {
        box_hight = 0;
    }
    else {
        box_hight = boxDim[1];
    }


    return boxDim;
}


float RANSACHandler::check_box(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud) {

    pcl::ModelCoefficients::Ptr coefficients_planes1;
    pcl::ModelCoefficients::Ptr coefficients_planes2;
    pcl::ModelCoefficients::Ptr coefficients_planes3;

    std::array<pcl::ModelCoefficients::Ptr, 3> plane_coe_array;
    plane_coe_array[0] = coefficients_planes1;
    plane_coe_array[1] = coefficients_planes2;
    plane_coe_array[2] = coefficients_planes3;

    pcl::PointCloud<PointT>::Ptr cloud_fitted(new pcl::PointCloud<PointT>());
    pcl::PointCloud<PointT>::Ptr cloud_plane2(new pcl::PointCloud<PointT>());
    pcl::PointCloud<PointT>::Ptr cloud_plane3(new pcl::PointCloud<PointT>());

    std::array<pcl::PointCloud<pcl::PointXYZ>::Ptr, 3> plane_array;
    plane_array[0] = cloud_fitted;
    plane_array[1] = cloud_plane2;
    plane_array[2] = cloud_plane3;
    
    pcl::NormalEstimation<PointT, pcl::Normal> ne;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in;
    cloud_in = cloud;
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);
    pcl::SACSegmentationFromNormals<PointT, pcl::Normal> seg_box;
    pcl::ExtractIndices<PointT> extract_box;
    pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>());

    pcl::PointIndices::Ptr inliers_plane1(new pcl::PointIndices), inliers_plane2(new pcl::PointIndices), inliers_plane3(new pcl::PointIndices);

    ne.setSearchMethod(tree);
    ne.setInputCloud(cloud_in);
    ne.setKSearch(50);
    ne.compute(*cloud_normals);

    seg_box.setOptimizeCoefficients(true);
    seg_box.setMethodType(pcl::SAC_RANSAC);
    seg_box.setModelType(pcl::SACMODEL_PLANE);
    seg_box.setNormalDistanceWeight(0.01);
    seg_box.setMaxIterations(10000);
    seg_box.setDistanceThreshold(0.001);
    seg_box.setRadiusLimits(0.005, 0.150);

    //_______________first plane
    std::array<pcl::PointIndices::Ptr, 3 >inliers_array;
    inliers_array[0] = inliers_plane1;
    inliers_array[1] = inliers_plane2;
    inliers_array[2] = inliers_plane3;

    int nPoints = cloud_in->points.size();

    double ratio_planes[3];
    ratio_planes[0] = 0;
    ratio_planes[1] = 0;
    ratio_planes[2] = 0;

    int plane_counter = 0;

    while (cloud_in->points.size() > nPoints * 0.3 && plane_counter < 3)
    {
        seg_box.setInputCloud(cloud_in);
        seg_box.setInputNormals(cloud_normals);
        seg_box.segment(*inliers_array[plane_counter], *plane_coe_array[plane_counter]);
        //std::cerr << "Before extracting, inliers_plane.size: " << inliers_array[plane_counter]->indices.size() << std::endl;

        if (inliers_array[plane_counter]->indices.size() == 0) {
            //std::cout << "Error: cound not estimate a planer model. ||  Amount of Points: " << cloud_filtered->points.size() << std::endl;
            break;
        }

        // Save plane_1 inliers  
        pcl::ExtractIndices<pcl::PointXYZ> extract_plane;
        extract_plane.setInputCloud(cloud_in);
        extract_plane.setIndices(inliers_array[plane_counter]);
        extract_plane.setNegative(false);
        extract_plane.filter(*plane_array[plane_counter]);

        //couldpoint ratio
        ratio_planes[plane_counter] = (double)plane_array[plane_counter]->points.size() / (double)nPoints * 100;
        //std::cerr << "Extracted Points: " << plane_array[plane_counter]->points.size() << " | Total points: " << (double)cloud_filtered->points.size() << " Plane ratio: " << ratio_planes[plane_counter] << std::endl;


        //get center

        Eigen::Vector4f centroid;
        pcl::compute3DCentroid(*cloud_in, centroid);

        //float test = centroid[0];
        plane_centerPoint_x[plane_counter] = centroid[0];
        plane_centerPoint_y[plane_counter] = centroid[1];
        plane_centerPoint_z[plane_counter] = centroid[2];


        // Save plane_1 inliers  
        pcl::ExtractIndices<pcl::PointXYZ> extract_planes;
        extract_planes.setInputCloud(cloud_in);
        extract_planes.setIndices(inliers_array[plane_counter]);
        extract_planes.setNegative(true);
        extract_planes.filter(*cloud_in);

        //std::cerr << "Extracted negative points: " << cloud_filtered->points.size() << std::endl;
        //std::cerr << "inliers_plane.size for the negative plane: " << inliers_array[plane_counter]->indices.size() << std::endl;

        // Remove the planar inliers from normals
        pcl::ExtractIndices<pcl::Normal> extract_normals_box;
        extract_normals_box.setInputCloud(cloud_normals);
        extract_normals_box.setIndices(inliers_array[plane_counter]);
        extract_normals_box.setNegative(true);
        extract_normals_box.filter(*cloud_normals);

        ++plane_counter;
    }
    //cloud_fitted = plane_array[0];                                                  // to check the enpty statment
    float boxRatio = (double)ratio_planes[0] + (double)ratio_planes[1] + (double)ratio_planes[2];
    //std::cout << "Box ratio: "<< boxRatio << std::endl;
    return boxRatio;
}

float RANSACHandler::check_sph(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, pcl::ModelCoefficients::Ptr coefficients_sph) {
    pcl::NormalEstimation<PointT, pcl::Normal> ne;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in;
    cloud_in = cloud;
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);
    pcl::SACSegmentationFromNormals<PointT, pcl::Normal> seg_sph;
    pcl::ExtractIndices<PointT> extract_sph;
    pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>());
    pcl::PointIndices::Ptr inliers_sph(new pcl::PointIndices);

    ne.setSearchMethod(tree);
    ne.setInputCloud(cloud_in);
    ne.setKSearch(50);
    ne.compute(*cloud_normals);

    seg_sph.setOptimizeCoefficients(true);
    seg_sph.setMethodType(pcl::SAC_RANSAC);
    seg_sph.setModelType(pcl::SACMODEL_SPHERE);
    seg_sph.setNormalDistanceWeight(0.01);
    seg_sph.setMaxIterations(10000);
    seg_sph.setDistanceThreshold(0.005);
    seg_sph.setRadiusLimits(0.005, 0.15);
    seg_sph.setInputCloud(cloud_in);
    seg_sph.setInputNormals(cloud_normals);

    //Obtain the inliers of cylinder
    seg_sph.segment(*inliers_sph, *coefficients_sph);

    // Save the cylinder inliers                                       
    extract_sph.setInputCloud(cloud_in);
    extract_sph.setIndices(inliers_sph);
    extract_sph.setNegative(false);
    extract_sph.filter(*cloud_in);

    //cloudpoint ratio
    float sphRatio = (double)cloud_in->points.size() / (double)cloud_in->points.size() * 100;
    return sphRatio;
}
