#include "modelling/poissonReconstruction.h"

#include <pcl/common/common.h>
#include <pcl/common/transforms.h>

#include <pcl/surface/mls.h>
#include <pcl/surface/poisson.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/filters/passthrough.h>

namespace object_modeller
{
namespace modelling
{

void PoissonReconstruction::applyConfig(Config &config)
{

}

PoissonReconstruction::PoissonReconstruction()
{
}

pcl::PolygonMesh::Ptr PoissonReconstruction::process(std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> input)
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud = input[0];

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr filtered(new pcl::PointCloud<pcl::PointXYZRGB>());
    pcl::PassThrough<pcl::PointXYZRGB> filter;
    filter.setInputCloud(cloud);
    filter.filter(*filtered);

    // cout << "begin moving least squares" << endl;
    pcl::MovingLeastSquares<pcl::PointXYZRGB, pcl::PointXYZRGB> mls;
    mls.setInputCloud(filtered);
    mls.setSearchRadius(0.01);
    mls.setPolynomialFit(true);
    mls.setPolynomialOrder(2);
    mls.setUpsamplingMethod(pcl::MovingLeastSquares<pcl::PointXYZRGB, pcl::PointXYZRGB>::SAMPLE_LOCAL_PLANE);
    mls.setUpsamplingRadius(0.005);
    mls.setUpsamplingStepSize(0.003);

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_smoothed (new pcl::PointCloud<pcl::PointXYZRGB>());
    mls.process(*cloud_smoothed);
    // cout << "MLS complete" << endl;

    pcl::NormalEstimationOMP<pcl::PointXYZRGB, pcl::Normal> ne;
    ne.setNumberOfThreads(8);
    ne.setInputCloud(cloud_smoothed);
    ne.setRadiusSearch(0.01);
    Eigen::Vector4f centroid;
    pcl::compute3DCentroid(*cloud_smoothed, centroid);
    ne.setViewPoint(centroid[0], centroid[1], centroid[2]);

    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>());
    ne.compute(*cloud_normals);

    for(size_t i = 0; i < cloud_normals->size(); ++i){
        cloud_normals->points[i].normal_x *= -1;
        cloud_normals->points[i].normal_y *= -1;
        cloud_normals->points[i].normal_z *= -1;
    }

    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_smoothed_normals(new pcl::PointCloud<pcl::PointXYZRGBNormal>());
    pcl::concatenateFields(*cloud_smoothed, *cloud_normals, *cloud_smoothed_normals);

    pcl::Poisson<pcl::PointXYZRGBNormal> poisson;
    poisson.setDepth(9);
    poisson.setInputCloud(cloud_smoothed_normals);
    pcl::PolygonMesh::Ptr mesh(new pcl::PolygonMesh());
    poisson.reconstruct(*mesh);

    return mesh;
}

}
}
