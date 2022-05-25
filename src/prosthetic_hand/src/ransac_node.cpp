#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/registration/icp.h>
#include <pcl/console/time.h>
#include <pcl/console/parse.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/io/pcd_io.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/sample_consensus/sac_model_sphere.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/common/centroid.h>
#include <pcl/point_cloud.h>
#include <pcl/point_traits.h>
#include <pcl/PointIndices.h>
#include <pcl/cloud_iterator.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/ply_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/features/normal_3d.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
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
#include <memory>


#include <boost/foreach.hpp>

#define PCL_COMMON_CENTROID_H_

typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloudT;
typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

using namespace std;

//////////////////////Variables//////////////////
bool next_iteration = false;
int shape_id = 0;
int myShape;
int graspTypeCase;
int setMinRatio = 20;
int objectShape;
double objDiameter;
double cylinder_diameter;
double sphere_daimeter;
double box_radius;
double cylinder_height;
double sphere_height;
double box_height;
double dimensions;
double cylinderRatio = 0;
double sphereRatio = 0;
double boxRatio = 0;
double hexahedron_dimensions[3][3];
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_pointer( new pcl::PointCloud<pcl::PointXYZ>);

float dotProduct(pcl::PointXYZ, pcl::PointXYZ);
float normPointT(pcl::PointXYZ);
std::array<float, 6> getPointCloudBoundaries(const pcl::PointCloud<PointT>&);

int graspIdentifier(double size);
int bestGraspRatio(double shape_ratio[], int arraySize);

void correctCylShape(pcl::ModelCoefficients&, const pcl::ModelCoefficients&, const pcl::PointCloud<PointT>&);
void correctSphereShape(pcl::ModelCoefficients& sphere, const pcl::ModelCoefficients& coefficients, const pcl::PointCloud<PointT>& cloud);
std::array<float, 2> getPointCloudExtremesSphere(const pcl::PointCloud<PointT>& cloud, pcl::PointXYZ center);
double* boxDimentions(int amountOfPlanes, const pcl::ModelCoefficients& plane0, const pcl::ModelCoefficients& plane1, const pcl::ModelCoefficients& plane2);

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

///////////////////////////////////////////////////////////////////////////

void print4x4Matrix(const Eigen::Matrix4d& matrix){
    printf("Rotation matrix :\n");
    printf("    | %6.3f %6.3f %6.3f | \n", matrix(0, 0), matrix(0, 1), matrix(0, 2));
    printf("R = | %6.3f %6.3f %6.3f | \n", matrix(1, 0), matrix(1, 1), matrix(1, 2));
    printf("    | %6.3f %6.3f %6.3f | \n", matrix(2, 0), matrix(2, 1), matrix(2, 2));
    printf("Translation vector :\n");
    printf("t = < %6.3f, %6.3f, %6.3f >\n\n", matrix(0, 3), matrix(1, 3), matrix(2, 3));
}

void keyboardEventOccurred(const pcl::visualization::KeyboardEvent& event, void* nothing){
    if (event.getKeySym() == "space" && event.keyDown())
        next_iteration = true;
}

double graspIdentifier(double objDiameter, double objHeight, int shape_id, double hexahedron_dimensions) {
    double ObjSize;
    double MinSize = 0;
    double maxObjSize = 10;
    std::cout << objDiameter << std::endl;

    if (shape_id == 0) {
              if (objDiameter > 5) {
                  graspTypeCase = 1;
//                  std::cout << "\n" << "\n" << "______Cylinder with a diameter larger than 5cm, with size of: " << objDiameter << std::endl;
                  std::cout << "________ CYLINDER: Selected grasp type: Medium wrap ________" << std::endl;

              }
              else if (objDiameter > 2 & objDiameter < 5){
                  graspTypeCase = 2;
//                  std::cout << "\n" << "__________Cylinder with a diameter smaller 5cm, with a size of: " << objDiameter << std::endl;
                  std::cout << "________ CYLINDER: Selected grasp type: Lateral tripod ________" << std::endl;
              }
              else{
                graspTypeCase = 3;
                //                  std::cout << "\n" << "__________Cylinder with a diameter smaller 5cm, with a size of: " << objDiameter << std::endl;
                std::cout << "________ CYLINDER: Selected grasp type: Palmar pinch ________" << std::endl;

              }

          }

          else if (shape_id == 1) {
              ObjSize = objDiameter * 100 * 2;
              if (ObjSize > 5) {
                  graspTypeCase = 4;
    //              std::cout << "\n" << "\n" << "Ball with a diameter bigger 5.5cm, with a size of: " << ObjSize << std::endl;
                  std::cout << "________ SPHERE: Selected grasp type: Power sphere ________" << std::endl;
              }
              else if (ObjSize > 2 & ObjSize < 5) {									    //ball  <= diameter 15mm:
                  graspTypeCase = 5;
    //              std::cout << "\n" << "\n" << "ball with a diameter smaller 4.5cm, with a size of: " << ObjSize << std::endl;
                  std::cout << "________ SPHERE: Selected grasp type: Tripod  ________" << std::endl;
              }
              else {
              graspTypeCase = 6;
    //              std::cout << "\n" << "\n" << "ball with a diameter smaller 5.5cm and bigger than 2cm, with a size of: " << ObjSize << std::endl;
                  std::cout << "________ SPHERE: Selected grasp type: Palmar pinch  ________" << std::endl;

          }
        }

          else if (shape_id == 2) {

              if (objDiameter > 2) {
                  graspTypeCase = 5;
    //              std::cout << "\n" << "\n" << "Box with a diameter bigger 4cm, with a size of: " << objDiameter << std::endl;
                  std::cout << "________ Cuboid: Selected grasp type: Medium wrap ________" << std::endl;
              }

              else {
               graspTypeCase = 7;
  //                std::cout << "\n" << "\n" << "Box with a diameter smaller than 2cm, with a size of: " << objDiameter << std::endl;
                  std::cout << "________Cuboid: Selected grasp type: Lateral  ________" << std::endl;


              }

          }


          return graspTypeCase;
}

void start_here(){

    for (int myShape = 0; myShape <=2;){




      std::stringstream msg_general0;
      std::cout << msg_general0.str();
      pcl::PointCloud<PointT>::Ptr cloud_filtered(new pcl::PointCloud<PointT>);
      pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);

      ///////////////////////////////////////////////////////////////////////////////////////

          // Transform rs2::pointcloud into pcl::PointCloud<PointT>::Ptr
          pcl::console::TicToc time;
          pcl::console::TicToc tloop;
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

              ne.setSearchMethod(tree);
              ne.setInputCloud(cloud_pointer);
              ne.setKSearch(50);
              ne.compute(*cloud_normals);
              std::stringstream msg_general4;

              std::stringstream msg_cylinder1;
              std::stringstream msg_sphere1;
              std::stringstream msg_plane1;

              switch (myShape) {
              case Shape::SHAPE_CYLINDER:
                  // Create the segmentation object for cylinder segmentation and set all the parameters
                  seg_cylinder.setOptimizeCoefficients(true);
                  seg_cylinder.setMethodType(pcl::SAC_RANSAC);
                  seg_cylinder.setModelType(pcl::SACMODEL_CYLINDER);
                  seg_cylinder.setNormalDistanceWeight(0.1);
                  seg_cylinder.setMaxIterations(10000);
                  seg_cylinder.setDistanceThreshold(2.5);
                  seg_cylinder.setRadiusLimits(0, 30);
                  seg_cylinder.setInputCloud(cloud_pointer);
                  seg_cylinder.setInputNormals(cloud_normals);
                  break;

              case Shape::SHAPE_SPHERE:
                  // Create the segmentation object for cylinder seg_spherementation and set all the parameters
                 seg_sphere.setOptimizeCoefficients(true);
                  seg_sphere.setMethodType(pcl::SAC_RANSAC);
                  seg_sphere.setModelType(pcl::SACMODEL_SPHERE);
                  seg_sphere.setNormalDistanceWeight(0.01);
                  seg_sphere.setMaxIterations(10000);
                  seg_sphere.setDistanceThreshold(2);
                  seg_sphere.setRadiusLimits(0.5, 20);
                  seg_sphere.setInputCloud(cloud_pointer);
                  seg_sphere.setInputNormals(cloud_normals);
                  break;

              case Shape::SHAPE_PLANE:
                  // Create the seg_spherementation object for cylinder segmentation and set all the parameters
                  seg_plane.setOptimizeCoefficients(true);
                  seg_plane.setMethodType(pcl::SAC_RANSAC);
                  seg_plane.setModelType(pcl::SACMODEL_NORMAL_PLANE);
                  seg_plane.setNormalDistanceWeight(0.1);
                  seg_plane.setMaxIterations(10000);
                  seg_plane.setDistanceThreshold(0.88);
                  seg_plane.setRadiusLimits(0.5, 10);
                  seg_plane.setInputCloud(cloud_pointer);
                  seg_plane.setInputNormals(cloud_normals);
                  break;
              }

              std::stringstream msg_general5;

                    time.tic();
                  pcl::PointCloud<PointT>::Ptr cloud_fitted(new pcl::PointCloud<PointT>());
                  pcl::PointCloud<PointT>::Ptr cloud_plane2(new pcl::PointCloud<PointT>());
                  pcl::PointCloud<PointT>::Ptr cloud_plane3(new pcl::PointCloud<PointT>());

                  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ff(new pcl::PointCloud<pcl::PointXYZ>), cloud_f(new pcl::PointCloud<pcl::PointXYZ>);

                  std::array<pcl::PointCloud<pcl::PointXYZ>::Ptr, 3> plane_array;
                  plane_array[0] = cloud_fitted;
                  plane_array[1] = cloud_plane2;
                  plane_array[2] = cloud_plane3;

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

                      switch (myShape) {
                        case Shape::SHAPE_CYLINDER:
                                                  std::cerr << myShape << "; Get ratio of: CYLINDER" << std::endl;
                                                  //Obtain the inliers of cylinder
                                                  seg_cylinder.segment(*inliers_cylinder, *coefficients_cylinder);

                                                  // Save the cylinder inliers
                                                  extract_cylinder.setInputCloud(cloud_pointer);
                                                  extract_cylinder.setIndices(inliers_cylinder);
                                                  extract_cylinder.setNegative(false);
                                                  extract_cylinder.filter(*cloud_fitted);

                                                  //point cloud ratio
                                                  cylinderRatio = (double)cloud_fitted->points.size() / (double)cloud_pointer->points.size() * 100;

                                                  msg_cylinder4 << myShape << "; -----> Cylinder ratio: " << cylinderRatio <<  "%" << std::endl;
                                                  std::cout << msg_cylinder4.str();
                                                  break;

                        case Shape::SHAPE_SPHERE:
                                                  std::cerr << myShape << "; Get ratio of: SPHERE" << std::endl;
                                                  //obtian inliers of sphere
                                                  seg_sphere.segment(*inliers_sphere, *coefficients_sphere);

                                                  // Save the sphere inliers
                                                  extract_sphere.setInputCloud(cloud_pointer);
                                                  extract_sphere.setIndices(inliers_sphere);
                                                  extract_sphere.setNegative(false);
                                                  extract_sphere.filter(*cloud_fitted);

                                                  //couldpoint ratio
                                                  sphereRatio = (double)cloud_fitted->points.size() / (double)cloud_pointer->points.size() * 100;
                                                  msg_sphere4 << myShape << "; -----> Sphere ratio: " << sphereRatio  << "%" << std::endl;
                                                  std::cout << msg_sphere4.str();
                                                  break;

                                              case Shape::SHAPE_PLANE:
                                            //_______________first plane
                                            std::cerr << myShape << "; PLANE" << std::endl;
                                            std::array<pcl::PointIndices::Ptr, 3 >inliers_array;
                                            inliers_array[0] = inliers_plane1;
                                            inliers_array[1] = inliers_plane2;
                                            inliers_array[2] = inliers_plane3;

                                            int nPoints = cloud_pointer->points.size();

                                            double ratio_planes[3];
                                            ratio_planes[0] = 0;
                                            ratio_planes[1] = 0;
                                            ratio_planes[2] = 0;

                                            while (cloud_pointer->points.size() > nPoints * 0.3 && plane_counter < 3)
                                            {
                                                std::cout << "___________________Starting while loop______________nr. " << (plane_counter+1) << " _____________" << std::endl;

                                                seg_plane.setInputCloud(cloud_pointer);
                                                seg_plane.setInputNormals(cloud_normals);
                                                seg_plane.segment(*inliers_array[plane_counter], *plane_coe_array[plane_counter]);

                                                if (inliers_array[plane_counter]->indices.size() == 0) {

                                                    msg_plane5[plane_counter] << myShape << "; Error: cound not estimate a planer model. ||  Amount of Points: " << cloud_pointer->points.size() << std::endl;
                                                    std::cout << msg_plane5[plane_counter].str();
                                                    break;
                                                }

                                                ////////////////////////////PCA

                                                Eigen::Matrix3f covariance_matrix;

                                                Eigen::Vector4f xyz_centroid;

                                                compute3DCentroid (*cloud_pointer, xyz_centroid);

                                                computeCovarianceMatrix (*cloud_pointer, xyz_centroid, covariance_matrix);


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

                                                pcl::ExtractIndices<pcl::PointXYZ> extract_plane;
                                                extract_plane.setInputCloud(cloud_pointer);
                                                extract_plane.setIndices(inliers_array[plane_counter]);
                                                extract_plane.setNegative(false);
                                                extract_plane.filter(*plane_array[plane_counter]);

                                                ratio_planes[plane_counter] = (double)plane_array[plane_counter]->points.size() / (double)nPoints * 100;

                                                std::ostringstream oss;

                                                Eigen::Vector4f centroid;
                                                pcl::compute3DCentroid(*cloud_pointer, centroid);

                                                plane_centerPoint_x[plane_counter] = centroid[0];
                                                plane_centerPoint_y[plane_counter] = centroid[1];
                                                plane_centerPoint_z[plane_counter] = centroid[2];

                                                pcl::ExtractIndices<pcl::PointXYZ> extract_planes;
                                                extract_planes.setInputCloud(cloud_pointer);
                                                extract_planes.setIndices(inliers_array[plane_counter]);
                                                extract_planes.setNegative(true);
                                                extract_planes.filter(*cloud_pointer);

                                                pcl::ExtractIndices<pcl::Normal> extract_normals_box;
                                                extract_normals_box.setInputCloud(cloud_normals);
                                                extract_normals_box.setIndices(inliers_array[plane_counter]);
                                                extract_normals_box.setNegative(true);
                                                extract_normals_box.filter(*cloud_normals);

                                                ++plane_counter;
                                            }
                                            boxRatio = (double)ratio_planes[0] + (double)ratio_planes[1] + (double)ratio_planes[2];
                                            msg_plane8 << myShape << "; -----> Cuboid ratio: " << boxRatio << "%" << std::endl;
                                            std::cout << msg_plane8.str();
                          break;
                      }

                      std::stringstream msg_general6;

                          pcl::ModelCoefficients::Ptr corrected_coefs_cylinder(new pcl::ModelCoefficients);

                          std::stringstream msg_cylinder6;
                          std::stringstream msg_sphere6;
                          std::stringstream msg_plane10;
                          int cmRatio = 100;

                          switch (myShape) {
                          case Shape::SHAPE_CYLINDER:
                              cylinder_diameter = (coefficients_cylinder->values[6] * cmRatio * 2);
                              cylinder_height = cylinder_height / 10;
                              msg_cylinder6 << myShape << "; Cylinder; radius: " << cylinder_diameter << " cm, height: " << cylinder_height << " cm" << std::endl;
                              std::cout << msg_cylinder6.str();
                              break;
                          case Shape::SHAPE_SPHERE:
                              sphere_daimeter = coefficients_sphere->values[3];
                              sphere_height = sphere_daimeter;
                              msg_sphere6 << myShape << "; Sphere; radius: " << sphere_daimeter << " cm,  height: " << sphere_height << " cm" << std::endl;
                              std::cout << msg_sphere6.str();
                              break;
                          case Shape::SHAPE_PLANE:
                              box_radius;
                              box_height;

                              msg_plane10 << myShape << "; Cuboid; radius: " << hexahedron_dimensions[0][0] << " cm,  height: " << hexahedron_dimensions[0][1] << " cm" << std::endl;
                              std::cout << msg_plane10.str();
                              break;
                          }
  myShape++;
  }

    int cylinder_case = 0;
    int sphere_case = 1;
    int box_case = 2;

    double shape_ratio[3];
    shape_ratio[0] = cylinderRatio;
    shape_ratio[1] = sphereRatio;
    shape_ratio[2] = boxRatio;
    int arraySize = sizeof(shape_ratio) / sizeof(shape_ratio[0]);
    int bestRatio_id;

    bestRatio_id = bestGraspRatio(shape_ratio, arraySize);

    double obj_radius[3];
    obj_radius[0] = cylinder_diameter;
    obj_radius[1] = sphere_daimeter;
    obj_radius[2] = box_radius;

    std::cout << "______Best Grasp______" << std::endl;
    double max = shape_ratio[0];

    for (int i = 0; i < arraySize - 1; i++) {
        if (shape_ratio[i + 1] > max) {
            shape_id = i + 1;
            max = shape_ratio[i + 1];
            std::cout << "i: " << shape_id << ", value: " << max << std::endl;
        }
    }

        double ObjSize = 0;
        double MinSize = 0;
        double maxObjSize = 10;
         objDiameter = objDiameter * 100 * 2;
        std::cout << objDiameter << std::endl;

        if (shape_id == 0) {
            if (objDiameter > 5) {
                graspTypeCase = 1;
                std::cout << "________ CYLINDER: Selected grasp type: Medium wrap ________" << std::endl;

            }
            else {
                graspTypeCase = 2;
                std::cout << "________ CYLINDER: Selected grasp type: Lateral tripod________" << std::endl;
            }
        }

        else if (shape_id == 1) {
            ObjSize = objDiameter * 100 * 2;
            if (ObjSize > 55) {
                graspTypeCase = 3;
                std::cout << "________ SPHERE: Selected grasp type: Power sphere ________" << std::endl;
            }
            else  {
                graspTypeCase = 4;
                std::cout << "________ SPHERE: Selected grasp type: Tripod ________" << std::endl;
            }
        }

        else if (shape_id == 2) {
         //   if (objDiameter > 4) {
               if (hexahedron_dimensions[0][0] > 4){
                graspTypeCase = 5;
                std::cout << "________ BOX: Selected grasp type: Medium wrap ________" << std::endl;
            }
            else {
                graspTypeCase = 6;
                std::cout << "________ BOX: Selected grasp type: Lateral ________" << std::endl;
            }
        }

        else {
          std::cout << "________ Selected grasp type: palm grasp - Object is to big for one hand diameter" << std::endl;
        }
    std::cout << std::endl;
    std::cout << shape_id << std::endl;

}

int bestGraspRatio(double shape_ratio[], int arraySize) {
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
/*
void debug_point_cloud(){
  cloud_pointer->width = 60;
  cloud_pointer->height = 60;
  cloud_pointer->is_dense = false ;
  cloud_pointer->points.resize (cloud_pointer->width * cloud_pointer->height);
  for (int i = 0; i < cloud_pointer->width; i ++){
    for (int j = 0; j < cloud_pointer->height; j ++){
        cloud_pointer->points[i].x = 1024 * rand ()/(RAND_MAX + 1.0f );
        cloud_pointer->points[i].y = 1024 * rand ()/(RAND_MAX + 1.0f );
        cloud_pointer->points[i].z = 1024 * rand ()/(RAND_MAX + 1.0f );
      }
    }
  start_here();
}
*/
/*void depth_handler(const sensor_msgs::ImageConstPtr &msg){
  int div = 1;
  int stride = 4;
  float factor = 1;
  ROS_INFO("depth_handler------------------------------------------");

      cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_16UC1);
      //cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_8UC3);
      cv::Mat depth = cv_ptr->image;
      //generate point cloud
      cloud_pointer->width = depth.cols;
      cloud_pointer->height = depth.rows;
      cloud_pointer->is_dense = false ;
      cloud_pointer->points.resize (cloud_pointer->width * cloud_pointer->height);
      float fx = 1.0;  //_intrinsics(0, 0);
      float fy = 1.0;  //_intrinsics(1, 1);
      float cx = 1.0;  //_intrinsics(0, 2);
      float cy = 1.0;  //_intrinsics(1, 2);
      for (int i = 0; i < depth.rows; i += stride){
        for (int j = 0; j < depth.cols; j += stride){
            float Z = depth.at<uint16_t>(i, j) / 1;
            if(Z){
            cloud_pointer->points[i].x = (i - cx) * Z / fx / div;
            cloud_pointer->points[i].y = (j - cy) * Z / fy / div;
            cloud_pointer->points[i].z = Z / div;
        }
      }
      pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
      viewer->setBackgroundColor (0, 0, 0);
      viewer->addPointCloud<pcl::PointXYZ> (cloud, "sample cloud");
      viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");
      viewer->addCoordinateSystem (1.0);
      viewer->initCameraParameters ();
      while (!viewer->wasStopped ()){
        viewer->spinOnce (100);
        boost::this_thread::sleep (boost::posix_time::microseconds (100000));
      }

}
start_here();
}
*/
int main(int argc, char** argv)
{
  std::chrono::system_clock::now().time_since_epoch();
  ros::init(argc, argv, "sub_pcl");
  ros::NodeHandle nh;
  ROS_INFO("Ransac node started------------------------------------------");

if (pcl::io::loadPCDFile<pcl::PointXYZ> ("H1T1.pcd", *cloud_pointer) == -1)
{
      PCL_ERROR ("Couldn't read file test_pcd.pcd \n");
      return (-1);
    }

  start_here();

ros::spin();
}
