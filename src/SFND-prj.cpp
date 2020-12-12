#include "SFND-prj.h"

template<typename PointT>
std::unordered_set<int> SFND_Ransac<PointT>::Ransac3d(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceTol)
{
    std::unordered_set<int> inliersResult;
    srand(time(NULL));
    
    // TODO: Fill in this function
    //find the total number of cloud points
    auto no_of_cloud_points = cloud->points.size();

    //std::cout << "no_of_cloud_points" << no_of_cloud_points <<std::endl;

    while(maxIterations) { // For max iterations 

        std::unordered_set<int> inliers;
        auto max_inliers = 0;

        while(inliers.size() < 3)
            inliers.insert(rand() % no_of_cloud_points);

        //point coordinates for the line
        auto index_ptr = inliers.begin();
        auto x1 = cloud->points[*index_ptr].x;
        auto y1 = cloud->points[*index_ptr].y;
        auto z1 = cloud->points[*index_ptr].z;
        index_ptr++;
        auto x2 = cloud->points[*index_ptr].x;
        auto y2 = cloud->points[*index_ptr].y;
        auto z2 = cloud->points[*index_ptr].z;
        index_ptr++;
        auto x3 = cloud->points[*index_ptr].x;
        auto y3 = cloud->points[*index_ptr].y;
        auto z3 = cloud->points[*index_ptr].z;

        //generate cross product values
        auto i = ((y2-y1)*(z3-z1))-((z2-z1)*(y3-y1));
        auto j = ((z2-z1)*(x3-x1))-((x2-x1)*(z3-z1));
        auto k = ((x2-x1)*(y3-y1))-((y2-y1)*(x3-x1));

        //generate the palne parametes Ax+By+Cz+D=0
        auto A = i;
        auto B = j;
        auto C = k;
        auto D = -((i*x1)+(j*y1)+(k*z1));

        //iterate through all the points and find if they are inliers
        for(int index = 0; index < cloud->points.size(); index++)
        {
            if(inliers.count(index)>0) //if the point(index - passed as count parameter) is already in our inlier list
                continue;

            auto i = cloud->points[index];
            //std::cout<< "Selected point for distance measurement is x=" << i.x << " and y=" << i.y << std::endl;

            //find the distance d = |Ax+By+C|/sqrt(A^2+B^2)
            auto distance = fabs((A*i.x) + (B*i.y) + (C*i.z) + D)/ sqrt((A*A)+(B*B)+(C*C));

            std::cout << "distance from the point to the plane is " << distance << std::endl;

            if( distance < distanceTol) //inside tolerance
            {
                inliers.insert(index); //save the index
            }
        }

        if (inliers.size() > inliersResult.size())
        {
            inliersResult = inliers; //this is the plane with maximum inliers. So save it
        }
        
        // Measure distance between every point and fitted line
        // If distance is smaller than threshold count it as inlier

        maxIterations--; //decrease the iteration
    }

    // Return indicies of inliers from fitted line with most inliers
    
    return inliersResult;

}
