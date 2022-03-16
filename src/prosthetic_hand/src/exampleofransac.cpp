#include <pcl/registration/icp.h>
//#include <pcl/visualization/pcl_visualizer.h>
//#include <pcl/console/time.h>   // TicToc


#include <algorithm>
#include <iomanip>
#include <iostream>
#include <thread>
#include <sstream>                                                              //print statmets
#include <mutex>
#include <chrono>
#include <string>
#include <cmath>

//LNK1104  cannot open file 'Dependencies: royale.lib'                         http://pointclouds.org/documentation/tutorials/index.php
//Point Cloud Includes
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
//#include <pcl/segmentation/lccp_segmentation.h>                                 // http://docs.pointclouds.org/trunk/classpcl_1_1_l_c_c_p_segmentation.html
#include <pcl/visualization/pcl_visualizer.h>                                   //http://pointclouds.org/documentation/tutorials/pcl_visualizer.php#pcl-visualizer
#include <pcl/console/time.h>
#include <pcl/sample_consensus/ransac.h>                                        //Random Sample Consensus model


//royale lib
#include <royale.hpp>
#include <sample_utils/PlatformResources.hpp>


typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

bool next_iteration = false;

using namespace sample_utils;
using namespace std;

//gaspIdentifier variables
int objectShape;
double dimensions;
int graspTypeCase;

int setMinRatio = 20;       //30%
double cylinderRatio = 0;
double sphereRatio = 0;
double boxRatio = 0;
int shape_id = 0;

double cylinder_diameter;
double sphere_daimeter;
double box_radius;
double cylinder_hight;
double sphere_hight;
double box_hight;

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
int myShape;

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

float dotProduct(pcl::PointXYZ, pcl::PointXYZ);
float normPointT(pcl::PointXYZ);
std::array<float, 6> getPointCloudBoundaries(const pcl::PointCloud<PointT>&);

int graspIdentifier(double size);
int bestGraspRatio(double shape_ratio[], int arraySize);

void correctCylShape(pcl::ModelCoefficients&, const pcl::ModelCoefficients&, const pcl::PointCloud<PointT>&);
void correctSphereShape(pcl::ModelCoefficients& sphere, const pcl::ModelCoefficients& coefficients, const pcl::PointCloud<PointT>& cloud);
std::array<float, 2> getPointCloudExtremesSphere(const pcl::PointCloud<PointT>& cloud, pcl::PointXYZ center);
double* boxDimentions(int amountOfPlanes, const pcl::ModelCoefficients& plane0, const pcl::ModelCoefficients& plane1, const pcl::ModelCoefficients& plane2);

//my listener add here
class MyListener : public royale::IDepthDataListener
{
    /**
    * Data that has been received in onNewData, and will be printed in the paint() method.
    */
    struct MyFrameData
    {
        std::vector<uint32_t> exposureTimes;
        std::vector<std::string> asciiFrame;
    };


public:
    /**
    * Creates a listener which will have callbacks from two sources - the Royale framework, when a
    * new frame is received, and the UI toolkit, when the graphics are ready to repaint.
    */
    explicit MyListener(const royale::Vector<royale::StreamId>& streamIds) :            //??? probaly something about the matrix possition and the matrix value
        m_streamIds(streamIds)
    {
    }

    /**
    * This callback is called for each depth frame that is captured.  In a mixed-mode use case
    * (a use case with multiple streams), each callback refers to data from a single stream.
    */


    void callThreafFunc(const royale::DepthData* data, int myShape)
    {
        std::stringstream msg_general0;
        msg_general0 << "_______________ " << myShape << " -  loop starting _______________" << std::endl;
        std::cout << msg_general0.str();
        /*
        if (!isViewer) {
            viewer = initViewer();
            isViewer = true;
        }
        viewer->removeAllShapes();                                                          //removez all points
        viewer->removeAllPointClouds();                                                     //removes more stuff?
        */

        // Pointcloud objects
        pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);                    //creat a object cloud

        //Datasets
        pcl::PointCloud<PointT>::Ptr cloud_filtered(new pcl::PointCloud<PointT>);           //creat the cloud_filtered
        pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);  //create a object could_narmals

        // Transform rs2::pointcloud into pcl::PointCloud<PointT>::Ptr                      rs2?? whaaat
        pcl::console::TicToc time;                                                          //create a variable called time
        pcl::console::TicToc tloop;                                                         //
        pcl::console::TicToc Ttest;
        pcl::console::TicToc Tseg;
        time.tic();
        tloop.tic();
        cloud = points_to_pcl(data, 242); //127(50),204(80),229(90),242(95),252(99) //this->depth_confidence                             ?????what
        std::stringstream msg_general1;
        msg_general1 << "\n " << myShape << "; Read pointcloud from " << cloud->size() << " data points (in " << time.toc() << " ms).\n" << std::endl;
        std::cout << msg_general1.str();

        // http://pointclouds.org/documentation/tutorials/cylinder_segmentation.php

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

        //delcare cefficients and inliers's
        pcl::ModelCoefficients::Ptr coefficients_plane_c(new pcl::ModelCoefficients), coefficients_cylinder(new pcl::ModelCoefficients);
        pcl::PointIndices::Ptr inliers_plane_c(new pcl::PointIndices), inliers_cylinder(new pcl::PointIndices);

        pcl::ModelCoefficients::Ptr coefficients_plane_S(new pcl::ModelCoefficients), coefficients_sphere(new pcl::ModelCoefficients);
        pcl::PointIndices::Ptr inliers_plane_S(new pcl::PointIndices), inliers_sphere(new pcl::PointIndices);

        pcl::ModelCoefficients::Ptr coefficients_plane_p(new pcl::ModelCoefficients), coefficients_planes1(new pcl::ModelCoefficients);
        pcl::PointIndices::Ptr inliers_plane_p(new pcl::PointIndices), inliers_plane1(new pcl::PointIndices);
        pcl::ModelCoefficients::Ptr coefficients_plane_p2(new pcl::ModelCoefficients), coefficients_planes2(new pcl::ModelCoefficients);
        pcl::PointIndices::Ptr inliers_plane_p2(new pcl::PointIndices), inliers_plane2(new pcl::PointIndices);
        pcl::ModelCoefficients::Ptr coefficients_plane_p3(new pcl::ModelCoefficients), coefficients_planes3(new pcl::ModelCoefficients);
        pcl::PointIndices::Ptr inliers_plane_p3(new pcl::PointIndices), inliers_plane3(new pcl::PointIndices);


        // Build a passthrough filter to remove unwanted points
        time.tic();
        pass.setInputCloud(cloud);
        pass.setFilterFieldName("x");
        pass.setFilterLimits(filter_lims[0], filter_lims[1]);
        pass.filter(*cloud_filtered);
        //
        pass.setInputCloud(cloud_filtered);
        pass.setFilterFieldName("y");
        pass.setFilterLimits(filter_lims[2], filter_lims[3]);
        pass.filter(*cloud_filtered);
        //
        pass.setInputCloud(cloud_filtered);
        pass.setFilterFieldName("z");
        pass.setFilterLimits(filter_lims[4], filter_lims[5]);
        pass.filter(*cloud_filtered);

        std::stringstream msg_general2;
        msg_general2 << myShape << "; PointCloud after filtering: " << cloud_filtered->points.size() << " data points (" << time.toc() << ")." << std::endl;
        std::cout << msg_general2.str();

        // Downsampling the filtered point cloud
        pcl::VoxelGrid<pcl::PointXYZ> dsfilt;
        dsfilt.setInputCloud(cloud_filtered);
        dsfilt.setLeafSize(this->filt_leaf_size, this->filt_leaf_size, this->filt_leaf_size);
        dsfilt.filter(*cloud_filtered);

        std::stringstream msg_general3;
        msg_general3 << myShape << "; PointCloud after downsampling: " << cloud_filtered->width * cloud_filtered->height << " data points (" << time.toc() << ")." << std::endl;
        std::cout << msg_general3.str();


        //visualizor
        /*
        // Draw filtered PointCloud
        pcl::visualization::PointCloudColorHandlerCustom<PointT> cloud_filtered_in_color_h(cloud_filtered, (int)255 * txt_gray_lvl, (int)255 * txt_gray_lvl,
            (int)255 * txt_gray_lvl);
        viewer->addPointCloud(cloud_filtered, cloud_filtered_in_color_h, "cloud_filtered_in " + myShape, vp);
        */

        // Estimate point normals
        time.tic();
        ne.setSearchMethod(tree);
        ne.setInputCloud(cloud_filtered);
        ne.setKSearch(50);
        ne.compute(*cloud_normals);
        std::stringstream msg_general4;
        msg_general4 << myShape << "; Estimating point normals (" << time.toc() << ")." << std::endl;
        std::cout << msg_general4.str();

        Tseg.tic();

        std::stringstream msg_cylinder1;
        std::stringstream msg_sphere1;
        std::stringstream msg_plane1;

        //std::cerr << "SetModelType to: ";
        //swich to set the right ModeType
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
            seg_cylinder.setInputCloud(cloud_filtered);
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
            seg_sphere.setInputCloud(cloud_filtered);
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
            //seg_plane.setInputCloud(cloud_filtered);
            //seg_plane.setInputNormals(cloud_normals);
            msg_plane1 << myShape << "; SetModelType to: PLANE" << std::endl;
            std::cout << msg_plane1.str();
            break;
        }
        std::stringstream msg_general5;
        msg_general5 << myShape << "; segmentation time: " << Tseg.toc() << " ms" << std::endl;
        std::cout << msg_general5.str();


        // Creating variables to store the %of how good the points fit to the given shape. ratio is not the proper name right
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

        //swich to Obtain the inliers and coefficients, saving the cylinder inliers and saving ratio
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
            extract_cylinder.setInputCloud(cloud_filtered);
            extract_cylinder.setIndices(inliers_cylinder);
            extract_cylinder.setNegative(false);
            extract_cylinder.filter(*cloud_fitted);

            //couldpoint ratio                                                                  nice
            cylinderRatio = (double)cloud_fitted->points.size() / (double)cloud_filtered->points.size() * 100;
            //std::cerr << "cylinderPoints_size: " << cloud_fitted->points.size() << "  ;  filteredPoints_size: " << cloud_filtered->points.size() << std::endl;
            //std::cerr << " - Cylinder ratio: " << cylinderRatio << " CylinderRadius: " << coefficients_cylinder->values[6]*1000 << " mm" << std::endl;               

            msg_cylinder3 << myShape << "; cylinderPoints_size: " << cloud_fitted->points.size() << "  ;  filteredPoints_size: " << cloud_filtered->points.size() << std::endl;
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
            extract_sphere.setInputCloud(cloud_filtered);
            extract_sphere.setIndices(inliers_sphere);
            extract_sphere.setNegative(false);
            extract_sphere.filter(*cloud_fitted);

            //couldpoint ratio                                                                  nice
            sphereRatio = (double)cloud_fitted->points.size() / (double)cloud_filtered->points.size() * 100;
            //std::cerr << "spherePoints_size: " << cloud_fitted->points.size() << "  ;  filteredPoints_size: " << cloud_filtered->points.size() << std::endl;
            //std::cerr << "Sphere ratio: " << sphereRatio << "  |  SphereRadius: " << coefficients_sphere->values[3]*1000 << " mm" <<std::endl;
            msg_sphere3 << myShape << "; -----> spherePoints_size: " << cloud_fitted->points.size() << "  ;  filteredPoints_size: " << cloud_filtered->points.size() << std::endl;
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

            int nPoints = cloud_filtered->points.size();

            double ratio_planes[3];
            ratio_planes[0] = 0;
            ratio_planes[1] = 0;
            ratio_planes[2] = 0;

            while (cloud_filtered->points.size() > nPoints * 0.3 && plane_counter < 3)
            {
                //std::cout << "___________________while loop starts______________nr. " << (plane_counter+1) << " _____________" << std::endl;
                msg_plane2[plane_counter] << myShape << "; ______ Plane2Box while loop|| nr. " << (plane_counter) << " ______" << std::endl;
                std::cout << msg_plane2[plane_counter].str();

                //std::cout << "Original amount of points: " << cloud_filtered->points.size() << std::endl;
                msg_plane3[plane_counter] << myShape << "; Original amount of points: " << cloud_filtered->points.size() << std::endl;
                std::cout << msg_plane3[plane_counter].str();


                seg_plane.setInputCloud(cloud_filtered);
                seg_plane.setInputNormals(cloud_normals);
                seg_plane.segment(*inliers_array[plane_counter], *plane_coe_array[plane_counter]);
                //std::cerr << "Before extracting, inliers_plane.size: " << inliers_array[plane_counter]->indices.size() << std::endl;

                msg_plane4_coe[plane_counter] << myShape << "; plane coefficients: " << *plane_coe_array[plane_counter] << std::endl;
                std::cout << msg_plane4_coe[plane_counter].str();
                msg_plane4[plane_counter] << myShape << "; Before extracting, inliers_plane.size: " << inliers_array[plane_counter]->indices.size() << std::endl;
                std::cout << msg_plane4[plane_counter].str();

                if (inliers_array[plane_counter]->indices.size() == 0) {                                                                                             //prev inliers
                    //std::cout << "Error: cound not estimate a planer model. ||  Amount of Points: " << cloud_filtered->points.size() << std::endl;
                    msg_plane5[plane_counter] << myShape << "; Error: cound not estimate a planer model. ||  Amount of Points: " << cloud_filtered->points.size() << std::endl;
                    std::cout << msg_plane5[plane_counter].str();
                    break;
                }

                // Save plane_1 inliers  
                pcl::ExtractIndices<pcl::PointXYZ> extract_plane;
                extract_plane.setInputCloud(cloud_filtered);
                extract_plane.setIndices(inliers_array[plane_counter]);
                extract_plane.setNegative(false);
                extract_plane.filter(*plane_array[plane_counter]);

                //couldpoint ratio                                                                  nice
                ratio_planes[plane_counter] = (double)plane_array[plane_counter]->points.size() / (double)nPoints * 100;
                //std::cerr << "Extracted Points: " << plane_array[plane_counter]->points.size() << " | Total points: " << (double)cloud_filtered->points.size() << " Plane ratio: " << ratio_planes[plane_counter] << std::endl;
                msg_plane6[plane_counter] << myShape << "; ---> Extracted Points: " << plane_array[plane_counter]->points.size() << " | Total points: " << (double)cloud_filtered->points.size() << " Plane ratio: " << ratio_planes[plane_counter] << std::endl;
                std::cout << msg_plane6[plane_counter].str();


                //get center
                std::ostringstream oss;
                oss << "merged_fitpc2.pcd" + plane_counter;
                //reader.read(oss, *plane_array[plane_counter]);
                Eigen::Vector4f centroid;
                pcl::compute3DCentroid(*cloud_filtered, centroid);
                //cout<<centroid[0] << endl << centroid[1] << endl << centroid[2] << endl;
                msg_planeCentroid[plane_counter] << myShape << "-" << plane_counter << " | CENTROID x: " << centroid[0] << " | CENTROID y: " << centroid[1] << " | CENTROID z: " << centroid[2] << std::endl;
                std::cout << msg_planeCentroid[plane_counter].str();

                //float test = centroid[0];
                plane_centerPoint_x[plane_counter] = centroid[0];
                plane_centerPoint_y[plane_counter] = centroid[1];
                plane_centerPoint_z[plane_counter] = centroid[2];


                // Save plane_1 inliers  
                pcl::ExtractIndices<pcl::PointXYZ> extract_planes;
                extract_planes.setInputCloud(cloud_filtered);
                extract_planes.setIndices(inliers_array[plane_counter]);
                extract_planes.setNegative(true);
                extract_planes.filter(*cloud_filtered);

                //std::cerr << "Extracted negative points: " << cloud_filtered->points.size() << std::endl;
                //std::cerr << "inliers_plane.size for the negative plane: " << inliers_array[plane_counter]->indices.size() << std::endl;
                msg_plane7[plane_counter] << myShape << "; Extracted negative points: " << cloud_filtered->points.size() << " || inliers_plane.size for the negative plane: " << inliers_array[plane_counter]->indices.size() << std::endl;
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
        msg_general6 << myShape << "; extracting of the object finisch in " << time.toc() << " ms" << std::endl;
        std::cout << msg_general6.str();

        pcl::ModelCoefficients::Ptr corrected_coefs_cylinder(new pcl::ModelCoefficients);

        //visulization
        /*
        if (cloud_fitted->points.empty()) {                                                                   // add othere shape checkers here
            //std::cerr << "\nCan't find a object. ";                         // , checks for sphere
            std::stringstream msg_general7;
            msg_general7 << myShape << "; \nCan't find a object. ";
            std::cout << msg_general7.str();
            return;                                                         //make it stop
        }
        else {                                                                                                                                          //change the cyl thing to more general
            //std::cerr << "PointCloud: " << cloud_fitted->points.size() << " data points (in " << time.toc() << " ms)." << std::endl;
            std::stringstream msg_general8;
            msg_general8 << myShape << "; PointCloud: " << cloud_fitted->points.size() << " data points (in " << time.toc() << " ms)." << std::endl;
            std::cout << msg_general8.str();

            // ICP aligned point cloud is red
            pcl::visualization::PointCloudColorHandlerCustom<PointT> cloud_cylinder_color_h(cloud_fitted, 180, 20, 20);
            viewer->addPointCloud(cloud_fitted, cloud_cylinder_color_h, "cloud_cylinder", vp);                                                          //eks here

        }

        if ((setMinRatio < cylinderRatio) || (setMinRatio < sphereRatio) || (setMinRatio < boxRatio))               //update plane1Ratio
        {

            pcl::ModelCoefficients::Ptr corrected_coefs_plane(new pcl::ModelCoefficients);

            std::stringstream msg_cylinder5;
            std::stringstream msg_sphere5;
            std::stringstream msg_plane9;
            //swich to set the right ModeType
            switch (myShape) {
            case Shape::SHAPE_CYLINDER:
                //std::cerr << "start visulization of CYLINDER" << std::endl;
                msg_cylinder5 << myShape << "; start visulization of CYLINDER" << std::endl;
                std::cout << msg_cylinder5.str();
                correctCylShape(*corrected_coefs_cylinder, *coefficients_cylinder, *cloud_fitted);
                viewer->addCylinder(*corrected_coefs_cylinder, "cylinder");
                //viewer->spinOnce(1, true);
                //objectShape = 1;
                break;
            case Shape::SHAPE_SPHERE:
                //std::cerr << "start visulization of SPHERE" << std::endl;
                msg_sphere5 << myShape << "; start visulization of CYLINDER" << std::endl;
                std::cout << msg_sphere5.str();
                viewer->addSphere(*coefficients_sphere, "Sphere");
                //viewer->spinOnce(1, true);                                                          //pretty shure 1 discrips the object
                //objectShape = 2;
                break;
            case Shape::SHAPE_PLANE:
                //std::cerr << "start visulization of PLANE" << std::endl;
                msg_plane9 << myShape << "; start visulization of CYLINDER" << std::endl;
                std::cout << msg_plane9.str();

                for (int i = 0; i < plane_counter; ++i) {
                    viewer->addPlane(*plane_coe_array[i], "plane" + i);                                   //addCube
                }
                break;
            }

            viewer->spinOnce(1, true);
            //std::cout << "Total loop time: (in " << tloop.toc() << " ms)." << std::endl;
            std::stringstream msg_general9;                                                                                     /////////////////// Fuck
            msg_general9 << myShape << "; Total loop time: (in " << tloop.toc() << " ms)." << std::endl;
            std::cout << msg_general9.str();
        }
        else {          //Error
            std::cout << myShape << "; Error: visulasation failed, dou to ratio (good_filtered_Points/filtered_points) to small." << std::endl;
            std::stringstream msg_general10;
            msg_general10 << myShape << "; Error: visulasation failed, dou to ratio (good_filtered_Points/filtered_points) to small." << std::endl;
            std::cout << msg_general10.str();
        }

        */



        std::stringstream msg_cylinder6;
        std::stringstream msg_sphere6;
        std::stringstream msg_plane10;
        int cmRatio = 100;

        switch (myShape) {
        case Shape::SHAPE_CYLINDER:
            correctCylShape(*corrected_coefs_cylinder, *coefficients_cylinder, *cloud_fitted);
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
            boxDimentions(plane_counter, *plane_coe_array[0], *plane_coe_array[1], *plane_coe_array[2]);
            box_radius;
            box_hight;

            msg_plane10 << myShape << "; box; radius: " << box_radius << " cm,  hight: " << box_hight << " cm" << std::endl;
            std::cout << msg_plane10.str();
            break;
        }
    }

    void onNewData(const royale::DepthData* data)  override     //this function is where we read the data, chage the data and get the points
    {
        pcl::console::TicToc fullcycle;
        fullcycle.tic();
        // case variables
        int cylinder_case = 0;
        int sphere_case = 1;
        int box_case = 2;

        // use threads to call this fuction for the different shaps void MyListener::callThreafFunc(const royale::DepthData * data, int myShape)
        std::thread cylinderThread(&MyListener::callThreafFunc, this, data, cylinder_case);
        std::thread sphereThread(&MyListener::callThreafFunc, this, data, sphere_case);
        std::thread boxThread(&MyListener::callThreafFunc, this, data, box_case);

        //wait un til the threads are stopped
        cylinderThread.join();
        sphereThread.join();
        boxThread.join();


        //make ratio array = ratios
        double shape_ratio[3];
        shape_ratio[0] = cylinderRatio;
        shape_ratio[1] = sphereRatio;
        shape_ratio[2] = boxRatio;
        int arraySize = sizeof(shape_ratio) / sizeof(shape_ratio[0]);
        int bestRatio_id;

        //call bestGrasp
        bestRatio_id = bestGraspRatio(shape_ratio, arraySize);

        //restuckture variables for graspIdentifier
        double obj_radius[3];
        obj_radius[0] = cylinder_diameter;
        obj_radius[1] = sphere_daimeter;
        obj_radius[2] = box_radius;

        double obj_hight[3];
        obj_hight[0] = cylinder_hight;
        obj_hight[1] = sphere_hight;
        obj_hight[2] = box_hight;


        //call graspIdentifier(double size);
        graspIdentifier(obj_radius[shape_id], obj_hight[shape_id], shape_id);
        fullcycle.toc();
        std::cout << "|-------->>  full cycle time: " << fullcycle.toc() << " ms <<--------|" << std::endl;
    }
    //____________________________________________________________________________________________________________________________________________________


//Finds the best ratio, by comparring the different shap ratios
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


    //grasp identifier
    double graspIdentifier(double objDiameter, double objHight, int shape_id) {
        double ObjSize;
        double MinSize = 0;
        double maxObjSize = 10;
        // objDiameter = objDiameter * 100 * 2;
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
            else {									    //box <= diameter 20mm:	
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


    pcl::visualization::PCLVisualizer::Ptr simpleVis(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud)
    {
        // --------------------------------------------
        // -----Open 3D viewer and add point cloud-----
        // --------------------------------------------
        pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
        viewer->setBackgroundColor(0, 0, 0);
        viewer->addPointCloud<pcl::PointXYZ>(cloud, "sample cloud");
        viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");
        //viewer->addCoordinateSystem (1.0, "global");
        viewer->initCameraParameters();
        return (viewer);
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
    std::map<royale::StreamId, MyFrameData> m_receivedData;
    std::mutex m_lockForReceivedData;


    //bool isViewer = false;
    //pcl::visualization::PCLVisualizer::Ptr viewer;

    int vp = 0; // Default viewport
    float bckgr_gray_level = 1.0;  // Black:=0.0
    float txt_gray_lvl = 1.0 - bckgr_gray_level;
    std::array<float, 6> filter_lims = { -0.050, 0.050, -0.050, 0.050, 0.000, 0.300 }; // picoflexx depth z-axis (Min ? m)
    float filt_leaf_size = 0.005;

    pcl::visualization::PCLVisualizer::Ptr initViewer()
    {
        pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
        viewer->createViewPort(0.0, 0.0, 1.0, 1.0, vp);
        viewer->setCameraPosition(0.0, 0.0, -0.5, 0.0, -1.0, 0.0, vp);
        viewer->setSize(800, 600);
        viewer->setBackgroundColor(bckgr_gray_level, bckgr_gray_level, bckgr_gray_level, vp);
        viewer->addCoordinateSystem(0.25); // Global reference frame (on-camera)

        return viewer;
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr MyListener::points_to_pcl(const royale::DepthData* data, uint8_t depthConfidence)
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


//################################-----MAIN-----##############################################
int main(int argc, char* argv[])
{
    /*
    * we could make a while loop or a switch statement(scane, grasp, open [showing text), wating for the user input
    */
    //std::cout << "Select a shape, which you like to scane for; your options are; SHAPE_CYLINDER | SHAPE_SPHERE | SHAPE_PLANE " << std::endl;
    //std::cin >> myShape;


    // Windows requires that the application allocate these, not the DLL.  We expect typical
    // Royale applications to be using a GUI toolkit such as Qt, which has its own equivalent of this
    // PlatformResources class (automatically set up by the toolkit).
    PlatformResources resources;

    // This is the data listener which will receive callbacks.  It's declared
    // before the cameraDevice so that, if this function exits with a 'return'
    // statement while the camera is still capturing, it will still be in scope
    // until the cameraDevice's destructor implicitly deregisters the listener.
    unique_ptr<MyListener> listener;

    // this represents the main camera device object
    unique_ptr<royale::ICameraDevice> cameraDevice;

    // if non-null, load this file instead of connecting to a camera
    unique_ptr<royale::String> rrfFile;                         //unique_pointer is a object with deletes stuff from the memory
    // if non-null, choose this use case instead of the default
    unique_ptr<royale::String> commandLineUseCase;

    //
    if (argc > 1)     //0=errors
    {
        // Files recorded with RoyaleViewer have this filename extension
        const auto FILE_EXTENSION = royale::String(".rrf");

        auto arg = std::unique_ptr<royale::String>(new royale::String(argv[1]));
        if (*arg == "--help" || *arg == "-h" || *arg == "/?" || argc > 2)
        {
            cout << argv[0] << ":" << endl;
            cout << "--help, -h, /? : this help" << endl;
            cout << endl;
            cout << "With no command line arguments: opens a camera and retrieves data using the default use case" << endl;
            cout << endl;
            cout << "With an argument that ends \".rrf\": assumes the argument is the filename of a recording, and plays it" << endl;
            cout << "When playing back a recording, the only use case available is the one that was recorded." << endl;
            cout << endl;
            cout << "With an argument that doesn't end \".rrf\": assumes the argument is the name of a use-case" << endl;
            cout << "It will open the camera, and if there's a use case with that exact name will use it." << endl;
            return 0;
        }
        else if (arg->size() > FILE_EXTENSION.size() && 0 == arg->compare(arg->size() - FILE_EXTENSION.size(), FILE_EXTENSION.size(), FILE_EXTENSION))
        {
            cout << "Assuming command-line argument is the filename of an RRF recording" << endl;
            rrfFile = move(arg);
        }
        else
        {
            cout << "Assuming command-line argument is the name of a use case, as it does not end \".rrf\"" << endl;
            commandLineUseCase = move(arg);
        }
    }

    // the camera manager will either open a recording or query for a connected camera
    if (rrfFile)                                         //if we use the monkey run it that way
    {
        royale::CameraManager manager;
        cameraDevice = manager.createCamera(*rrfFile);
    }
    else                                                        //if there is no 3d file run the camerra
    {  //
        royale::CameraManager manager;

        auto camlist = manager.getConnectedCameraList();
        cout << "Detected " << camlist.size() << " camera(s)." << endl;
        if (!camlist.empty())
        {
            cout << "CamID for first device: " << camlist.at(0).c_str() << " with a length of (" << camlist.at(0).length() << ")" << endl;
            cameraDevice = manager.createCamera(camlist[0]);
        }
    }//
    // the camera device is now available and CameraManager can be deallocated here

    if (cameraDevice == nullptr)                                                //if no camera connected 
    {
        cerr << "Cannot create the camera device" << endl;
        return 1;
    }

    // IMPORTANT: call the initialize method before working with the camera device 
    if (cameraDevice->initialize() != royale::CameraStatus::SUCCESS)
    {
        cerr << "Cannot initialize the camera device" << endl;
        return 1;
    }

    royale::Vector<royale::String> useCases;                                    // to get the framerate and distance
    auto status = cameraDevice->getUseCases(useCases);

    if (status != royale::CameraStatus::SUCCESS || useCases.empty())            //error message
    {
        cerr << "No use cases are available" << endl;
        cerr << "getUseCases() returned: " << getErrorString(status) << endl;
        return 1;
    }

    // choose a use case
    auto selectedUseCaseIdx = 0u;                                               //0u is the frame and distance case
    if (commandLineUseCase)
    {
        auto useCaseFound = false;

        for (auto i = 0u; i < useCases.size(); ++i)
        {
            if (*commandLineUseCase == useCases[i])
            {
                selectedUseCaseIdx = i;
                useCaseFound = true;
                break;
            }
        }

        if (!useCaseFound)
        {
            cerr << "Error: the chosen use case is not supported by this camera" << endl;
            cerr << "A list of supported use cases is printed by sampleCameraInfo" << endl;
            return 1;
        }
    }
    else
    {
        // choose the first use case
        selectedUseCaseIdx = 0;
    }

    // set an operation mode/ usecase
    if (cameraDevice->setUseCase(useCases.at(selectedUseCaseIdx)) != royale::CameraStatus::SUCCESS)
    {
        cerr << "Error setting use case" << endl;
        return 1;
    }

    // Retrieve the IDs of the different streams
    royale::Vector<royale::StreamId> streamIds;
    if (cameraDevice->getStreams(streamIds) != royale::CameraStatus::SUCCESS)           //error if the stream are not sending
    {
        cerr << "Error retrieving streams" << endl;
        return 1;
    }

    cout << "Stream IDs : ";            //this id got this value 
    for (auto curStream : streamIds)
    {
        cout << curStream << ", ";
    }
    cout << endl;

    // register a data listener
    listener.reset(new MyListener(streamIds));                                                              //we start the recording
    if (cameraDevice->registerDataListener(listener.get()) != royale::CameraStatus::SUCCESS)                //error/bjarne 
    {
        cerr << "Error registering data listener" << endl;
        return 1;
    }

    // start capture mode
    if (cameraDevice->startCapture() != royale::CameraStatus::SUCCESS)                                     //error
    {
        cerr << "Error starting the capturing" << endl;
        return 1;
    }

    //let the camera capture for some time                                                             
    //this make the cood run for x sec
    this_thread::sleep_for(chrono::seconds(5));//wait 



    // Change the exposure time for the first stream of the use case (Royale will limit this to an
    // eye-safe exposure time, with limits defined by the use case).  The time is given in
    // microseconds.
    //
    // Non-mixed mode use cases have exactly one stream, mixed mode use cases have more than one.
    // For this example we only change the first stream.
    if (cameraDevice->setExposureTime(200, streamIds[0]) != royale::CameraStatus::SUCCESS)
    {
        cerr << "Cannot set exposure time for stream" << streamIds[0] << endl;                           //error
    }
    else
    {
        cout << "Changed exposure time for stream " << streamIds[0] << " to 200 microseconds ..." << endl;      //bjarne 
    }


    // let the camera capture for some time
    this_thread::sleep_for(chrono::seconds(300));                                                           //change the capture time




    // stop capture mode
    if (cameraDevice->stopCapture() != royale::CameraStatus::SUCCESS)
    {
        cerr << "______ capturing stopping _______" << endl;
        return 1;
    }

    return (0);
}


float dotProduct(pcl::PointXYZ a, pcl::PointXYZ b)
{
    return a.x * b.x + a.y * b.y + a.z * b.z;
}

float normPointT(pcl::PointXYZ c)
{
    return std::sqrt(c.x * c.x + c.y * c.y + c.z * c.z);
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


double* boxDimentions(const int amountOfPlanes, const pcl::ModelCoefficients& plane0, const pcl::ModelCoefficients& plane1, const pcl::ModelCoefficients& plane2) {
    //for loop with runs the amount of plane
    double boxDim[2];                   //boxDim[0] = width; BoxDim[1] = hight
    boxDim[0] = 0;
    boxDim[1] = 0;

    //    pcl::PointXYZ point_on_axis(coefficients.values[0], coefficients.values[1], coefficients.values[2]);

    const int plane_amounts = amountOfPlanes;

    double planes[3][4];

    if (amountOfPlanes > 2) {                   //if the amount of planes is more then 2: 3, 4, 5, do this
        //frist plane
        planes[0][0] = plane0.values[0];
        planes[0][1] = plane0.values[1];
        planes[0][2] = plane0.values[2];
        planes[0][3] = plane0.values[3];
        //secound plane
        planes[1][0] = plane1.values[0];
        planes[1][1] = plane1.values[1];
        planes[1][2] = plane1.values[2];
        planes[1][3] = plane1.values[3];
        //thrid plane
        planes[2][0] = plane2.values[0];
        planes[2][1] = plane2.values[1];
        planes[2][2] = plane2.values[2];
        planes[2][3] = plane2.values[3];
    }
    else {                                      //if there are2 or less planes do this.
        //frist plane
        planes[0][0] = plane0.values[0];
        planes[0][1] = plane0.values[1];
        planes[0][2] = plane0.values[2];
        planes[0][3] = plane0.values[3];
        //secound plane
        planes[1][0] = plane1.values[0];
        planes[1][1] = plane1.values[1];
        planes[1][2] = plane1.values[2];
        planes[1][3] = plane1.values[3];
    }

    int loopCounter = amountOfPlanes - 1;

    for (int i = 0; i < loopCounter; ++i) {                    //3 planes = 2 runs

        //define normal vecorts
        long double plane1_nx;
        long double plane1_ny;
        long double plane1_nz;
        long double plane2_nx;
        long double plane2_ny;
        long double plane2_nz;

        //define length and angle variables
        long double c_angle;
        long double c_length;
        long double a_angle;
        long double a_length;

        //find normal vecotrs posssitions
        plane1_nx = planes[i][0] / (sqrt(pow(planes[i][0], 2) + pow(planes[i][1], 2) + pow(planes[i][2], 2)));
        plane1_ny = planes[i][1] / (sqrt(pow(planes[i][0], 2) + pow(planes[i][1], 2) + pow(planes[i][2], 2)));
        plane1_nz = planes[i][2] / (sqrt(pow(planes[i][0], 2) + pow(planes[i][1], 2) + pow(planes[i][2], 2)));

        plane2_nx = planes[i][0] / (sqrt(pow(planes[i + 1][0], 2) + pow(planes[i + 1][1], 2) + pow(planes[i + 1][2], 2)));
        plane2_ny = planes[i][1] / (sqrt(pow(planes[i + 1][0], 2) + pow(planes[i + 1][1], 2) + pow(planes[i + 1][2], 2)));
        plane2_nz = planes[i][2] / (sqrt(pow(planes[i + 1][0], 2) + pow(planes[i + 1][1], 2) + pow(planes[i + 1][2], 2)));
        std::stringstream boxDim0;
        boxDim0 << myShape << "-" << i << "; 1nx" << plane1_nx << "; 1ny: " << plane1_ny << "; 1nz: " << plane1_nz << "; 2nx: " << plane2_nx << "; 2ny: " << plane2_ny << "; 2nz: " << plane2_nz << std::endl;
        std::cout << boxDim0.str();                 //good


        //find angle of C
        //C_angle = invCos((n_A ° n_B )  /  (n_A| * | n_B| ))
        //note ° = dotproduct [ a.x* b.x + a.y * b.y + a.z * b.z ]

        long double plane_dotProduct = (plane1_nx * plane2_nx) + (plane1_ny * plane2_ny) + (plane1_nz * plane2_nz);             //seams good
        long double plane1_length = sqrt((pow(plane1_nx, 2) + pow(plane1_ny, 2) + pow(plane1_nz, 2)));                       //
        long double plane2_length = sqrt((pow(plane2_nx, 2) + pow(plane2_ny, 2) + pow(plane2_nz, 2)));
        long double before_aSin = (plane_dotProduct / ((plane1_length * plane2_length)));
        c_angle = asin(before_aSin);
        std::stringstream boxDim1;
        boxDim1 << myShape << "-" << i << "; dotProduct: " << plane_dotProduct << " ; plane_1 lenght: " << plane1_length << " ; plane_2 length: " << plane2_length << " ; before_aCos: " << before_aSin << "; angle C: " << c_angle << std::endl;
        std::cout << boxDim1.str();


        //find length of c
        //c_length = sqrt( ( (X_B)^2 -(X_A)^2 + ( (y_B)^2 - (Y_A)^2) ) * 2
        //plane_center_x[plane_counter];
        //plane_center_y[plane_counter];
        //plane_center_z[plane_counter];
        //plane_center_n[plane_counter];

        //Plane_centerPoint[plane_counter];
        float plane1_center_x = plane_centerPoint_x[i];
        float plane1_center_y = plane_centerPoint_y[i];
        float plane1_center_z = plane_centerPoint_z[i];

        float plane2_center_x = plane_centerPoint_x[i + 1];
        float plane2_center_y = plane_centerPoint_y[i + 1];
        float plane2_center_z = plane_centerPoint_z[i + 1];

        /*
        c_length = sqrt( ( pow(plane2_center_x, 2) - pow(plane1_center_x, 2) ) +
                         ( pow(plane2_center_y, 2) - pow(plane1_center_y, 2) ) +
                         ( pow(plane2_center_z, 2) - pow(plane1_center_z, 2) ) );
        */
        long double c_length_powx = pow(plane2_center_x - plane1_center_x, 2);
        long double c_length_powy = pow(plane2_center_y - plane1_center_y, 2);
        long double c_length_powz = pow(plane2_center_z - plane1_center_z, 2);

        long double c_length_sub = c_length_powx + c_length_powy + plane2_center_z;

        c_length = sqrt(c_length_sub);

        std::stringstream boxDim2;
        boxDim2 << myShape << "-" << i << "; pow_x: " << c_length_powx << "; pow_y: " << c_length_powy << "; pow_z: " << c_length_powz << "; c_length: " << c_length << std::endl;
        std::cout << boxDim2.str();


        //find Anothere angle (asuming A=B)
        a_angle = (180 - c_angle) / 2;
        std::stringstream boxDim3;
        boxDim3 << myShape << "-" << i << "; a_angle: " << a_angle << std::endl;
        std::cout << boxDim3.str();

        //find lengh of a
        a_length = (c_length * sin(a_angle)) / sin(c_angle);


        std::stringstream boxDim4;
        boxDim4 << myShape << "-" << i << "; a_length: " << a_length << std::endl;
        std::cout << boxDim4.str();

        boxDim[i] = a_length * 10 * 2;                                         // *100 to get in cm


    }
    if (boxDim[0] != boxDim[0]) {
        //nan error
        box_radius = 0;
    }
    else {
        box_radius = boxDim[0];
    }
    if (boxDim[1] != boxDim[1]) {
        //nan error
        box_hight = 0;
    }
    else {
        box_hight = boxDim[1];
    }


    return boxDim;
}



