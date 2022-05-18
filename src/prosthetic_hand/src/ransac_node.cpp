#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/registration/icp.h>
#include <pcl/console/time.h>
#include <pcl/console/parse.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/sample_consensus/sac_model_sphere.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/common/centroid.h>
#include <pcl/registration/icp.h>
#include <pcl/console/time.h>   // TicToc
#include <pcl/point_cloud.h>
#include <pcl/point_traits.h>
#include <pcl/PointIndices.h>
#include <pcl/cloud_iterator.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/features/normal_3d.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/time.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/common/common.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>


#include <iostream>
#include <thread>
#include <algorithm>
#include <iomanip>
#include <sstream>
#include <mutex>
#include <chrono>
#include <string>
#include <cmath>
#include <boost/foreach.hpp>
#include <pcl_ros/point_cloud.h>

#include <boost/foreach.hpp>

#define PCL_COMMON_CENTROID_H_

typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloudT;
typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

//using namespace sample_utils;
using namespace std;

//////////////////////Variables///////////////////////////////////////////////

bool next_iteration = false;
int objectShape;
double dimensions;
int graspTypeCase;
int setMinRatio = 20;       //30%
double cylinderRatio = 0;
double sphereRatio = 0;
double boxRatio = 0;
int shape_id = 0;

double objDiameter;
double cylinder_diameter;
double sphere_daimeter;
double box_radius;
double cylinder_hight;
double sphere_hight;
double box_hight;
double hexahedron_dimensions[3][3];
int myShape;
pcl::PointCloud<pcl::PointXYZ> cloud;
float dotProduct(pcl::PointXYZ, pcl::PointXYZ);
float normPointT(pcl::PointXYZ);
std::array<float, 6> getPointCloudBoundaries(const pcl::PointCloud<PointT>&);
int graspIdentifier(double size);
int bestGraspRatio(double shape_ratio[], int arraySize);

//void correctCylShape(pcl::ModelCoefficients&, const pcl::ModelCoefficients&, const pcl::PointCloud<PointT>&);
//void correctSphereShape(pcl::ModelCoefficients& sphere, const pcl::ModelCoefficients& coefficients, const pcl::PointCloud<PointT>& cloud);
//std::array<float, 2> getPointCloudExtremesSphere(const pcl::PointCloud<PointT>& cloud, pcl::PointXYZ center);
//double* boxDimentions(int amountOfPlanes, const pcl::ModelCoefficients& plane0, const pcl::ModelCoefficients& plane1, const pcl::ModelCoefficients& plane2);


//////////////////////////////////////////////////////////////////

std::array<float, 3> plane_centerPoint_x{
    plane_centerPoint_x[0],
    plane_centerPoint_x[1],
    plane_centerPoint_x[2]
};
std::array<float, 3> plane_centerPoint_y{
    plane_centerPoint_y[0],
    plane_centerPoint_y[1],
    plane_centerPoint_y[2]
};
std::array<float, 3> plane_centerPoint_z{
    plane_centerPoint_z[0],
    plane_centerPoint_z[1],
    plane_centerPoint_z[2]
};

enum Shape {
    SHAPE_CYLINDER,
    SHAPE_SPHERE,
    SHAPE_PLANE
};
//int myShape;

///////////////////////////////////////////////////////////////////////////

void print4x4Matrix(const Eigen::Matrix4d& matrix)
{
    printf("Rotation matrix :\n");
    printf("    | %6.3f %6.3f %6.3f | \n", matrix(0, 0), matrix(0, 1), matrix(0, 2));
    printf("R = | %6.3f %6.3f %6.3f | \n", matrix(1, 0), matrix(1, 1), matrix(1, 2));
    printf("    | %6.3f %6.3f %6.3f | \n", matrix(2, 0), matrix(2, 1), matrix(2, 2));
    printf("Translation vector :\n");
    printf("t = < %6.3f, %6.3f, %6.3f >\n\n", matrix(0, 3), matrix(1, 3), matrix(2, 3));
}

void keyboardEventOccurred(const pcl::visualization::KeyboardEvent& event, void* nothing)
{
    if (event.getKeySym() == "space" && event.keyDown())
        next_iteration = true;
}

////////////////////////////////////////////////////////////////////////////////////

void ransac_fitting() {
  /* code */
  for (int myShape = 0; myShape <=2;){
    std::stringstream msg_general0;
    msg_general0 << "_______________ " << myShape << " -  loop starting _______________" << std::endl;
    std::cout << msg_general0.str();


    pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);

        //Datasets
    //    pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);           //create the cloud
        pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);  //create a object could_narmals


    ///////////////////////////////////////////////////////////////////////////////////////

        // Transform rs2::pointcloud into pcl::PointCloud<PointT>::Ptr                      rs2?? whaaat
        pcl::console::TicToc time;                                                          //create a variable called time
        pcl::console::TicToc tloop;                                                         //
        pcl::console::TicToc Ttest;
        pcl::console::TicToc Tseg;
        time.tic();
        tloop.tic();

        // PCL objects
        pcl::PassThrough<PointT> pass(true);
        pcl::NormalEstimation<PointT, pcl::Normal> ne;
        pcl::SACSegmentationFromNormals<PointT, pcl::Normal> seg_cylinder;
        pcl::SACSegmentationFromNormals<PointT, pcl::Normal> seg_sphere;
        pcl::SACSegmentationFromNormals<PointT, pcl::Normal> seg_plane;
        pcl::ExtractIndices<PointT> extract_cylinder;
        pcl::ExtractIndices<PointT> extract_sphere;
        pcl::ExtractIndices<PointT> extract_plane;
        pcl::ExtractIndices<pcl::Normal> extract_normals;
        pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>());

        ///////////////////////////////////////////////////////////////////
            //Declare the inliers for the different planes and shapes
        /////////////////////////////////////////////////////////////////
          //Cylinder inliers
            pcl::ModelCoefficients::Ptr coefficients_plane_c(new pcl::ModelCoefficients), coefficients_cylinder(new pcl::ModelCoefficients);
            pcl::PointIndices::Ptr inliers_plane_c(new pcl::PointIndices), inliers_cylinder(new pcl::PointIndices);
          //Sphere inliers
            pcl::ModelCoefficients::Ptr coefficients_plane_S(new pcl::ModelCoefficients), coefficients_sphere(new pcl::ModelCoefficients);
            pcl::PointIndices::Ptr inliers_plane_S(new pcl::PointIndices), inliers_sphere(new pcl::PointIndices);
          //Plane inliers
            pcl::ModelCoefficients::Ptr coefficients_plane_p(new pcl::ModelCoefficients), coefficients_planes1(new pcl::ModelCoefficients);
            pcl::PointIndices::Ptr inliers_plane_p(new pcl::PointIndices), inliers_plane1(new pcl::PointIndices);
            pcl::ModelCoefficients::Ptr coefficients_plane_p2(new pcl::ModelCoefficients), coefficients_planes2(new pcl::ModelCoefficients);
            pcl::PointIndices::Ptr inliers_plane_p2(new pcl::PointIndices), inliers_plane2(new pcl::PointIndices);
            pcl::ModelCoefficients::Ptr coefficients_plane_p3(new pcl::ModelCoefficients), coefficients_planes3(new pcl::ModelCoefficients);
            pcl::PointIndices::Ptr inliers_plane_p3(new pcl::PointIndices), inliers_plane3(new pcl::PointIndices);

            // Estimate point normals
           time.tic();
            ne.setSearchMethod(tree);
          //  pcl::PointCloud<pcl::PointXYZ> point_cloud;
            //pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(*cloud);
            ne.setInputCloud(cloud);
            ne.setKSearch(50);
            ne.compute(*cloud_normals);
            std::stringstream msg_general4;
            msg_general4 << myShape << "; Estimating point normals (" << time.toc() << ")." << std::endl;
            std::cout << msg_general4.str();

            Tseg.tic();

            std::stringstream msg_cylinder1;
            std::stringstream msg_sphere1;
            std::stringstream msg_plane1;

            switch (myShape) {
            case Shape::SHAPE_CYLINDER:
                // Create the segmentation object for cylinder segmentation and set all the parameters
                seg_cylinder.setOptimizeCoefficients(true);
                seg_cylinder.setMethodType(pcl::SAC_RANSAC);
                seg_cylinder.setModelType(pcl::SACMODEL_CYLINDER);
                seg_cylinder.setNormalDistanceWeight(0.01);
                seg_cylinder.setMaxIterations(10000);
                seg_cylinder.setDistanceThreshold(0.0012);
                seg_cylinder.setRadiusLimits(0.005, 0.150);
                seg_cylinder.setInputCloud(cloud);
                seg_cylinder.setInputNormals(cloud_normals);
                msg_cylinder1 << myShape << "; SetModelType to: CYLINDER" << std::endl;
                std::cout << msg_cylinder1.str();
                break;
            case Shape::SHAPE_SPHERE:
                // Create the segmentation object for cylinder seg_spherementation and set all the parameters
                seg_sphere.setOptimizeCoefficients(true);
                seg_sphere.setMethodType(pcl::SAC_RANSAC);
                seg_sphere.setModelType(pcl::SACMODEL_SPHERE);
                seg_sphere.setNormalDistanceWeight(0.01);
                seg_sphere.setMaxIterations(10000);
                seg_sphere.setDistanceThreshold(0.005);
                seg_sphere.setRadiusLimits(0.005, 0.15);
                seg_sphere.setInputCloud(cloud);
                seg_sphere.setInputNormals(cloud_normals);
                msg_sphere1 << myShape << "; SetModelType to: SPHERE" << std::endl;
                std::cout << msg_sphere1.str();
                break;
            case Shape::SHAPE_PLANE:
                // Create the seg_spherementation object for cylinder segmentation and set all the parameters
                seg_plane.setOptimizeCoefficients(true);
                seg_plane.setMethodType(pcl::SAC_RANSAC);
                seg_plane.setModelType(pcl::SACMODEL_PLANE);
                seg_plane.setNormalDistanceWeight(0.01);
                seg_plane.setMaxIterations(10000);
                seg_plane.setDistanceThreshold(0.001);
                seg_plane.setRadiusLimits(0.005, 0.150);
                //seg_plane.setInputCloud(cloud);
                //seg_plane.setInputNormals(cloud_normals);
                msg_plane1 << myShape << "; SetModelType to: PLANE" << std::endl;
                std::cout << msg_plane1.str();
                break;
            }
            std::stringstream msg_general5;
            msg_general5 << myShape << "; segmentation time: " << Tseg.toc() << " ms" << std::endl;
            std::cout << msg_general5.str();

            ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
            //Variables to describe the ratio for each shape
            //////////////////////////////////////////////////////////////////
                  time.tic();
                pcl::PointCloud<PointT>::Ptr cloud_fitted(new pcl::PointCloud<PointT>());
                pcl::PointCloud<PointT>::Ptr cloud_plane2(new pcl::PointCloud<PointT>());
                pcl::PointCloud<PointT>::Ptr cloud_plane3(new pcl::PointCloud<PointT>());

                pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ff(new pcl::PointCloud<pcl::PointXYZ>), cloud_f(new pcl::PointCloud<pcl::PointXYZ>);


                std::array<pcl::PointCloud<pcl::PointXYZ>::Ptr, 3> plane_array;
                plane_array[0] = cloud_fitted;
                plane_array[1] = cloud_plane2;
                plane_array[2] = cloud_plane3;

                //pcl::ModelCoefficients::Ptr coefficients_plane_p(new pcl::ModelCoefficients), coefficients_planes1(new pcl::ModelCoefficients);
                std::array<pcl::ModelCoefficients::Ptr, 3> plane_coe_array;
                plane_coe_array[0] = coefficients_planes1;
                plane_coe_array[1] = coefficients_planes2;
                plane_coe_array[2] = coefficients_planes3;


                pcl::PointIndices::Ptr inliers_plane_face(new pcl::PointIndices), inliers_cube_face(new pcl::PointIndices);

                size_t plane_counter = 0;


                std::stringstream msg_cylinder2;
                std::stringstream msg_cylinder3;
                std::stringstream msg_cylinder4;
                std::stringstream msg_sphere2;
                std::stringstream msg_sphere3;
                std::stringstream msg_sphere4;
                std::array < std::stringstream, 3> msg_plane2;
                msg_plane2[0];
                msg_plane2[1];
                msg_plane2[2];
                std::array < std::stringstream, 3> msg_plane3;
                msg_plane3[0];
                msg_plane3[1];
                msg_plane3[2];
                std::array < std::stringstream, 3> msg_plane4;
                msg_plane4[0];
                msg_plane4[1];
                msg_plane4[2];
                std::array < std::stringstream, 3> msg_plane4_coe;
                msg_plane4_coe[0];
                msg_plane4_coe[1];
                msg_plane4_coe[2];
                std::array < std::stringstream, 3> msg_plane5;
                msg_plane5[0];
                msg_plane5[1];
                msg_plane5[2];
                std::array < std::stringstream, 3> msg_plane6;
                msg_plane6[0];
                msg_plane6[1];
                msg_plane6[2];
                std::array < std::stringstream, 3> msg_planeCentroid;
                msg_planeCentroid[0];
                msg_planeCentroid[1];
                msg_planeCentroid[2];
                std::array < std::stringstream, 3> msg_plane7;
                msg_plane7[0];
                msg_plane7[1];
                msg_plane7[2];
                std::stringstream msg_plane8;


                //////////////////////////////////////////////////////////////////////////////////////////////////////
                    //Switch case to obtain the inliers and coefficients of each mode, saving the inliers and the ratio
                ////////////////////////////////////////////////////////////////////////////
                    std::cerr << myShape << "; Get ratio of: ";
                    switch (myShape) {
                    case Shape::SHAPE_CYLINDER:
                        std::cerr << myShape << "; Get ratio of: CYLINDER" << std::endl;
                        //Obtain the inliers of cylinder
                        seg_cylinder.segment(*inliers_cylinder, *coefficients_cylinder);
                        //std::cerr << "Cylinder inliers: " << *inliers_cylinder << std::endl;

                        msg_cylinder2 << myShape << "; Cylinder coefficients: " << *coefficients_cylinder << std::endl;
                        std::cout << msg_cylinder2.str();

                        // Save the cylinder inliers
                        extract_cylinder.setInputCloud(cloud);
                        extract_cylinder.setIndices(inliers_cylinder);
                        extract_cylinder.setNegative(false);
                        extract_cylinder.filter(*cloud_fitted);

                        //point cloud ratio                                                                  nice
                        cylinderRatio = (double)cloud_fitted->points.size() / (double)cloud->points.size() * 100;
                        //std::cerr << "cylinderPoints_size: " << cloud_fitted->points.size() << "  ;  filteredPoints_size: " << cloud->points.size() << std::endl;
                        //std::cerr << " - Cylinder ratio: " << cylinderRatio << " CylinderRadius: " << coefficients_cylinder->values[6]*1000 << " mm" << std::endl;

                        msg_cylinder3 << myShape << "; cylinderPoints_size: " << cloud_fitted->points.size() << "  ;  filteredPoints_size: " << cloud->points.size() << std::endl;
                        std::cout << msg_cylinder3.str();
                        msg_cylinder4 << myShape << "; -----> Cylinder ratio: " << cylinderRatio << " CylinderRadius: " << coefficients_cylinder->values[6] * 1000 << " mm" << std::endl;
                        std::cout << msg_cylinder4.str();
                        break;
                    case Shape::SHAPE_SPHERE:
                        std::cerr << myShape << "; Get ratio of: SPHERE" << std::endl;
                        //obtian inliers of sphere
                        seg_sphere.segment(*inliers_sphere, *coefficients_sphere);
                        //std::cerr << "Sphere inliers: " << *inliers_sphere << std::endl;
                        msg_sphere2 << myShape << "; Sphere coefficients: " << *coefficients_sphere << std::endl;
                        std::cout << msg_sphere2.str();
                        //std::cerr << "Sphere coefficients: " << *coefficients_sphere << std::endl;

                        // Save the sphere inliers
                        extract_sphere.setInputCloud(cloud);
                        extract_sphere.setIndices(inliers_sphere);
                        extract_sphere.setNegative(false);
                        extract_sphere.filter(*cloud_fitted);

                        //couldpoint ratio                                                                  nice
                        sphereRatio = (double)cloud_fitted->points.size() / (double)cloud->points.size() * 100;
                        //std::cerr << "spherePoints_size: " << cloud_fitted->points.size() << "  ;  filteredPoints_size: " << cloud->points.size() << std::endl;
                        //std::cerr << "Sphere ratio: " << sphereRatio << "  |  SphereRadius: " << coefficients_sphere->values[3]*1000 << " mm" <<std::endl;
                        msg_sphere3 << myShape << "; -----> spherePoints_size: " << cloud_fitted->points.size() << "  ;  filteredPoints_size: " << cloud->points.size() << std::endl;
                        std::cout << msg_sphere3.str();
                        msg_sphere4 << myShape << "; Sphere ratio: " << sphereRatio << "  |  SphereRadius: " << coefficients_sphere->values[3] * 1000 << " mm" << std::endl;
                        std::cout << msg_sphere4.str();
                        break;
                    case Shape::SHAPE_PLANE:
                  //_______________first plane
                  std::cerr << myShape << "; PLANE" << std::endl;
                  //pcl::PointIndices::Ptr inliers_plane_p(new pcl::PointIndices), inliers_plane1(new pcl::PointIndices);
                  std::array<pcl::PointIndices::Ptr, 3 >inliers_array;
                  inliers_array[0] = inliers_plane1;
                  inliers_array[1] = inliers_plane2;
                  inliers_array[2] = inliers_plane3;

                  int nPoints = cloud->points.size();

                  double ratio_planes[3];
                  ratio_planes[0] = 0;
                  ratio_planes[1] = 0;
                  ratio_planes[2] = 0;

                  while (cloud->points.size() > nPoints * 0.3 && plane_counter < 3)
                  {
                      //std::cout << "___________________while loop starts______________nr. " << (plane_counter+1) << " _____________" << std::endl;
                      msg_plane2[plane_counter] << myShape << "; ______ Plane2Box while loop|| nr. " << (plane_counter) << " ______" << std::endl;
                      std::cout << msg_plane2[plane_counter].str();

                      //std::cout << "Original amount of points: " << cloud->points.size() << std::endl;
                      msg_plane3[plane_counter] << myShape << "; Original amount of points: " << cloud->points.size() << std::endl;
                      std::cout << msg_plane3[plane_counter].str();


                      seg_plane.setInputCloud(cloud);
                      seg_plane.setInputNormals(cloud_normals);
                      seg_plane.segment(*inliers_array[plane_counter], *plane_coe_array[plane_counter]);
                      //std::cerr << "Before extracting, inliers_plane.size: " << inliers_array[plane_counter]->indices.size() << std::endl;

                      msg_plane4_coe[plane_counter] << myShape << "; plane coefficients: " << *plane_coe_array[plane_counter] << std::endl;
                      std::cout << msg_plane4_coe[plane_counter].str();
                      msg_plane4[plane_counter] << myShape << "; Before extracting, inliers_plane.size: " << inliers_array[plane_counter]->indices.size() << std::endl;
                      std::cout << msg_plane4[plane_counter].str();

                      if (inliers_array[plane_counter]->indices.size() == 0) {                                                                                             //prev inliers
                          //std::cout << "Error: cound not estimate a planer model. ||  Amount of Points: " << cloud->points.size() << std::endl;
                          msg_plane5[plane_counter] << myShape << "; Error: cound not estimate a planer model. ||  Amount of Points: " << cloud->points.size() << std::endl;
                          std::cout << msg_plane5[plane_counter].str();
                          break;
                      }



                      ////////////////////////////PCA

                      Eigen::Matrix3f covariance_matrix;

                      Eigen::Vector4f xyz_centroid;

                      compute3DCentroid (*cloud, xyz_centroid);

                      computeCovarianceMatrix (*cloud, xyz_centroid, covariance_matrix);


                      Eigen::EigenSolver<Eigen::MatrixXf> eigensolver;
                          eigensolver.compute(covariance_matrix);
                          Eigen::VectorXf eigen_values = eigensolver.eigenvalues().real();
                          Eigen::MatrixXf eigen_vectors = eigensolver.eigenvectors().real();
                          std::vector<std::tuple<float, Eigen::VectorXf>> eigen_vectors_and_values;

                          for(int i=0; i<eigen_values.size(); i++){
                              std::tuple<float, Eigen::VectorXf> vec_and_val(eigen_values[i], eigen_vectors.row(i));
                              eigen_vectors_and_values.push_back(vec_and_val);
                          }
                          std::sort(eigen_vectors_and_values.begin(), eigen_vectors_and_values.end(),
                              [&](const std::tuple<float, Eigen::VectorXf>& a, const std::tuple<float, Eigen::VectorXf>& b) -> bool{
                                  return std::get<0>(a) <= std::get<0>(b);
                          });

                          int index = 0;
                          for(auto const vect : eigen_vectors_and_values){
                              eigen_values(index) = std::get<0>(vect);
                              eigen_vectors.row(index) = std::get<1>(vect);
                              index++;
                          }


                       hexahedron_dimensions[plane_counter][0] = eigen_values[0];
                       hexahedron_dimensions[plane_counter][1] = eigen_values[1];
                      hexahedron_dimensions[plane_counter][2] = eigen_values[2];








                      // Save plane_1 inliers
                      pcl::ExtractIndices<pcl::PointXYZ> extract_plane;
                      extract_plane.setInputCloud(cloud);
                      extract_plane.setIndices(inliers_array[plane_counter]);
                      extract_plane.setNegative(false);
                      extract_plane.filter(*plane_array[plane_counter]);

                      //couldpoint ratio                                                                  nice
                      ratio_planes[plane_counter] = (double)plane_array[plane_counter]->points.size() / (double)nPoints * 100;
                      //std::cerr << "Extracted Points: " << plane_array[plane_counter]->points.size() << " | Total points: " << (double)cloud->points.size() << " Plane ratio: " << ratio_planes[plane_counter] << std::endl;
                      msg_plane6[plane_counter] << myShape << "; ---> Extracted Points: " << plane_array[plane_counter]->points.size() << " | Total points: " << (double)cloud->points.size() << " Plane ratio: " << ratio_planes[plane_counter] << std::endl;
                      std::cout << msg_plane6[plane_counter].str();


/*
                            std::cout << "Loaded "
                                        << cloud->width * cloud->height
                                        << " data points from test_pcd.pcd with the following fields: "
                                        << std::endl;

                            for (const auto& point: *cloud)
                                std::cout << "    " << point.x
                                          << " "    << point.y
                                          << " "    << point.z << std::endl;
*/


                      //get
                      std::ostringstream oss;
                      oss << "merged_fitpc2.pcd" + plane_counter;
                      //reader.read(oss, *plane_array[plane_counter]);
                      Eigen::Vector4f centroid;
                      pcl::compute3DCentroid(*cloud, centroid);
                      //cout<<centroid[0] << endl << centroid[1] << endl << centroid[2] << endl;
                      msg_planeCentroid[plane_counter] << myShape << "-" << plane_counter << " | CENTROID x: " << centroid[0] << " | CENTROID y: " << centroid[1] << " | CENTROID z: " << centroid[2] << std::endl;
                      std::cout << msg_planeCentroid[plane_counter].str();

                      //float test = centroid[0];
                      plane_centerPoint_x[plane_counter] = centroid[0];
                      plane_centerPoint_y[plane_counter] = centroid[1];
                      plane_centerPoint_z[plane_counter] = centroid[2];


                      // Save plane_1 inliers
                      pcl::ExtractIndices<pcl::PointXYZ> extract_planes;
                      extract_planes.setInputCloud(cloud);
                      extract_planes.setIndices(inliers_array[plane_counter]);
                      extract_planes.setNegative(true);
                      extract_planes.filter(*cloud);

                      //std::cerr << "Extracted negative points: " << cloud->points.size() << std::endl;
                      //std::cerr << "inliers_plane.size for the negative plane: " << inliers_array[plane_counter]->indices.size() << std::endl;
                      msg_plane7[plane_counter] << myShape << "; Extracted negative points: " << cloud->points.size() << " || inliers_plane.size for the negative plane: " << inliers_array[plane_counter]->indices.size() << std::endl;
                      std::cout << msg_plane7[plane_counter].str();

                      // Remove the planar inliers from normals
                      pcl::ExtractIndices<pcl::Normal> extract_normals_box;
                      extract_normals_box.setInputCloud(cloud_normals);
                      extract_normals_box.setIndices(inliers_array[plane_counter]);
                      extract_normals_box.setNegative(true);
                      extract_normals_box.filter(*cloud_normals);

                      ++plane_counter;
                  }
                  //cloud_fitted = plane_array[0];                                                  // to check the enpty statment
                  boxRatio = (double)ratio_planes[0] + (double)ratio_planes[1] + (double)ratio_planes[2];
                  //std::cout << "Box ration: "<< boxRatio << std::endl;
                  msg_plane8 << myShape << "; -----> Box ration: " << boxRatio << std::endl;
                  std::cout << msg_plane8.str();

                  //cloud_fitted = plane_array[plane_counter];
                        break;
                    }
                    std::stringstream msg_general6;
                    msg_general6 << myShape << "; extracting of the object finish in " << time.toc() << " ms" << std::endl;
                    std::cout << msg_general6.str();


                        pcl::ModelCoefficients::Ptr corrected_coefs_cylinder(new pcl::ModelCoefficients);

                        std::stringstream msg_cylinder6;
                        std::stringstream msg_sphere6;
                        std::stringstream msg_plane10;
                        int cmRatio = 100;
                    //////////////////////////////////////////////////////////
                    //Switch cases to obtain the radius of the different shapes
                    ///////////////////////////////////////////////////////////
                       switch (myShape) {
                        case Shape::SHAPE_CYLINDER:
  //  correctCylShape(*corrected_coefs_cylinder, *coefficients_cylinder, *cloud_fitted); //Problem
                            cylinder_diameter = (coefficients_cylinder->values[6] * cmRatio * 2);
                            cylinder_hight = cylinder_hight / 10;                             //check out the correct cylinder shap function
                            msg_cylinder6 << myShape << "; Cylinder; radius: " << cylinder_diameter << " cm, hight: " << cylinder_hight << " cm" << std::endl;
                            std::cout << msg_cylinder6.str();
                            break;
                        case Shape::SHAPE_SPHERE:
                            sphere_daimeter = coefficients_sphere->values[3];
                            sphere_hight = sphere_daimeter;
                            msg_sphere6 << myShape << "; sphere; radius: " << sphere_daimeter << " cm,  hight: " << sphere_hight << " cm" << std::endl;
                            std::cout << msg_sphere6.str();
                            break;
                        case Shape::SHAPE_PLANE:
    //          boxDimentions(plane_counter, *plane_coe_array[0], *plane_coe_array[1], *plane_coe_array[2]); //Problem
                            box_radius;
                            box_hight;

                            msg_plane10 << myShape << "; box; radius: " << hexahedron_dimensions[0][0] << " cm,  hight: " << hexahedron_dimensions[0][1] << " cm" << std::endl;
                            std::cout << msg_plane10.str();
                            break;
                        }


myShape++;

}

}

////////////////////////////////Here we subscribe to the depth data
void depth_handler(const sensor_msgs::ImageConstPtr &msg){
  int div = 1;
  int stride = 4;
  float factor = 1;
  ROS_INFO("depth_handler------------------------------------------");

      cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_16UC1);
      //cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_8UC3);
      cv::Mat depth = cv_ptr->image;
      //generate point cloud
      cloud.width = depth.cols;
      cloud.height = depth.rows;
      float fx = 1.0;  //_intrinsics(0, 0);
      float fy = 1.0;  //_intrinsics(1, 1);
      float cx = 1.0;  //_intrinsics(0, 2);
      float cy = 1.0;  //_intrinsics(1, 2);
      for (int i = 0; i < depth.rows; i += stride){
        for (int j = 0; j < depth.cols; j += stride){
            float Z = depth.at<uint16_t>(i, j) / 1;
            if(Z){
            pcl::PointXYZ p;
            p.x = (i - cx) * Z / fx / div;
            p.y = (j - cy) * Z / fy / div;
            p.z = Z / div;
            cloud.points.push_back(p);
        }
      }
      /*pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
      viewer->setBackgroundColor (0, 0, 0);
      viewer->addPointCloud<pcl::PointXYZ> (cloud, "sample cloud");
      viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");
      viewer->addCoordinateSystem (1.0);
      viewer->initCameraParameters ();
      while (!viewer->wasStopped ()){
        viewer->spinOnce (100);
        boost::this_thread::sleep (boost::posix_time::microseconds (100000));
      }
*/
    }
    ransac_fitting();
}


///////////////////////////////////////////////////////////////////
      //Finds the best ratio, by comparring the different shape ratios
  ///////////////////////////////////////////////////////////////////
          int bestGraspRatio(double shape_ratio[], int arraySize) {
              std::cout << "______Best Grasp______" << std::endl;
              double max = shape_ratio[0];

              for (int i = 0; i < arraySize - 1; i++) {
                  if (shape_ratio[i + 1] > max) {
                      shape_id = i + 1;
                      max = shape_ratio[i + 1];
                      std::cout << "i: " << shape_id << ", value: " << max << std::endl;
                  }
              }
              std::cout << "best shape is shape number: " << shape_id << ", with a value of:" << max << " % \n";

              return shape_id;
          }

          ///////////////////////////////////////////////
          //Decision tree to determine the best grasp type
          //////////////////////////////////////////////
                  double graspIdentifier(double objDiameter, double objHight, int shape_id, double hexahedron_dimensions) {
                      double ObjSize;
                      double MinSize = 0;
                      double maxObjSize = 10;
                      std::cout << objDiameter << std::endl;


                      if (shape_id == 0) {
                                if (objDiameter > 3) {							//cylinder > radius 10mm:
                                    graspTypeCase = 1;
                                    std::cout << "\n" << "\n" << "cylinder with a diameter bigger 3cm, with a size of: " << objDiameter << std::endl;
                                    std::cout << "________ CYLINDER: Selected grasp type: Leteral Power Grip ________" << std::endl;

                                }
                                else {									    //cylinder <= radius 10mm:
                                    graspTypeCase = 2;
                                    std::cout << "\n" << "cylinder with a diameter smaller 3cm, with a size of: " << objDiameter << std::endl;
                                    std::cout << "________ CYLINDER: Selected grasp type: Lateral Pinch ________" << std::endl;
                                }
                            }

                            else if (shape_id == 1) {
                                ObjSize = objDiameter * 100 * 2;
                                if (ObjSize > 4.5) {							//ball  > diameter 15mm:
                                    graspTypeCase = 3;
                                    std::cout << "\n" << "\n" << "ball with a diameter bigger 4.5cm, with a size of: " << ObjSize << std::endl;
                                    std::cout << "________ SPHERE: Selected grasp type: Opposition Power Grip ________" << std::endl;
                                }
                                else {									    //ball  <= diameter 15mm:
                                    graspTypeCase = 4;
                                    std::cout << "\n" << "\n" << "ball with a diameter smaller 4.5cm, with a size of: " << ObjSize << std::endl;
                                    std::cout << "________ SPHERE: Selected grasp type: Tripod Pinch ________" << std::endl;
                                }
                            }

                            else if (shape_id == 2) {
                                /*
                                if ((objDiameter > maxObjSize) || (objDiameter = 0)) {
                                    std::cout << "Radius/width of the object is bigger then: " << maxObjSize << ", with a size of: " << objDiameter << std::endl;
                                    if ((objHight > maxObjSize) || (objDiameter = 0)) {
                                        std::cout << "Hight of the object is bigger then: " << maxObjSize << ", with a size of: " << objHight << std::endl;
                                        std::cout << "Error: object is to big. Both the radius and the hight of the object is bigger then: " << maxObjSize << " cm." << std::endl;
                                        return 0;
                                    }
                                    else {
                                        ObjSize = objHight;
                                        std::cout << "Radius/width is used to select the grasptype." << std::endl;
                                    }
                                }
                                else {
                                    ObjSize = objDiameter;
                                    std::cout << "Radius/width is used to select the grasptype." << std::endl;
                                }
                                */
                                if (objDiameter > 4) {							//box  > diameter 20mm:
                                    graspTypeCase = 5;
                                    std::cout << "\n" << "\n" << "box with a diameter bigger 2cm, with a size of: " << objDiameter << std::endl;
                                    std::cout << "________ BOX: Selected grasp type: Opposition Power Grip ________" << std::endl;
                                }
                                else if (objDiameter < 4) {									    //box <= diameter 20mm:
                                    graspTypeCase = 6;
                                    std::cout << "\n" << "\n" << "box with a diameter bigger 2cm, with a size of: " << objDiameter << std::endl;
                                    std::cout << "________ BOX: Selected grasp type: Leteral Power Grip ________" << std::endl;
                                }
                            }

                            else {                                          //error
                                std::cout << "________ Selected grasp type: palm grasp - Object is to big for one hand diameter" << std::endl;
                            }
                            std::cout << std::endl;                                                     // "The selected grasp type is number: " << graspTypeCase <<
                            return graspTypeCase;
                        }






/////////////////////////////////////////////////////////////////////////////////////////////////////////////

int main(int argc, char** argv){
  //unique_ptr<MyListener> listener;
  ros::init(argc, argv, "ransac_node");
  ros::NodeHandle nh;
  ROS_INFO("Ransac node started------------------------------------------");
  image_transport::ImageTransport it(nh);
  image_transport::Subscriber sub = it.subscribe("unknown_depth", 1, depth_handler);
  ros::spin();
}
