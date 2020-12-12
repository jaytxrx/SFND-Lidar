#ifndef SFNDPRJ_H
#define SFNDPRJ_H

#include "processPointClouds.h"

template <typename PointT>
class SFND_Ransac {
    public:
        std::unordered_set<int> Ransac3d(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceTol);
};

#endif /* SFNDPRJ_H */