#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>


pcl::PointCloud<FeatureInT>::Ptr features (new pcl::PointCloud<FeatureInT>);

// Remove NaNs
pcl::PointIndices::Ptr nan_points (new pcl::PointIndices);
for(size_t j = 0; j < features->points.size(); j++)
{
    if(!pcl_isfinite(features->at(j).descriptor[0])) {
        PCL_WARN("Point %d is NaN\n", (int)j);
        nan_points->indices.push_back(j);
    }
}

// Remove NaN feature points
if(nan_points->indices.size() > 0)
{
    pcl::ExtractIndices<FeatureInT> extract;
    extract.setInputCloud(features);
    extract.setIndices(nan_points);
    extract.setNegative(true);
    extract.filter(*features);
}

PointInTPtr keypoints (new pcl::PointCloud<PointInT>);

// Remove keypoints corresponding to NaN feature points
if(nan_points->indices.size() > 0)
{
    pcl::ExtractIndices<PointInT> extract;
    extract.setInputCloud(keypoints);
    extract.setIndices(nan_points);
    extract.setNegative(true);
    extract.filter(*keypoints);
}
