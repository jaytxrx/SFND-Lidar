#ifndef RANSAC_H
#define RANSAC_H

#include <unordered_set>
#include "../../processPointClouds.h"

template<typename PointT>
std::unordered_set<int> RansacPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceTol);

#endif
