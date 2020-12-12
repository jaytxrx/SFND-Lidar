/* \author Aaron Brown */
// Quiz on implementing simple RANSAC line fitting

#include "../../render/render.h"
#include <unordered_set>
#include "../../processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "../../processPointClouds.cpp"

pcl::PointCloud<pcl::PointXYZ>::Ptr CreateData()
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
      // Add inliers
      float scatter = 0.6;
      for(int i = -5; i < 5; i++)
      {
          double rx = 2*(((double) rand() / (RAND_MAX))-0.5);
          double ry = 2*(((double) rand() / (RAND_MAX))-0.5);
          pcl::PointXYZ point;
          point.x = i+scatter*rx;
          point.y = i+scatter*ry;
          point.z = 0;

          cloud->points.push_back(point);
      }
      // Add outliers
      int numOutliers = 10;
      while(numOutliers--)
      {
          double rx = 2*(((double) rand() / (RAND_MAX))-0.5);
          double ry = 2*(((double) rand() / (RAND_MAX))-0.5);
          pcl::PointXYZ point;
          point.x = 5*rx;
          point.y = 5*ry;
          point.z = 0;

          cloud->points.push_back(point);

      }
      cloud->width = cloud->points.size();
      cloud->height = 1;

      return cloud;

}

pcl::PointCloud<pcl::PointXYZ>::Ptr CreateData3D()
{
    ProcessPointClouds<pcl::PointXYZ> pointProcessor;
    return pointProcessor.loadPcd("../../../sensors/data/pcd/simpleHighway.pcd");
}


pcl::visualization::PCLVisualizer::Ptr initScene()
{
    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer ("2D Viewer"));
    viewer->setBackgroundColor (0, 0, 0);
    viewer->initCameraParameters();
    viewer->setCameraPosition(0, 0, 15, 0, 1, 0);
    viewer->addCoordinateSystem (1.0);
    return viewer;
}

std::unordered_set<int> RansacPlane(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIterations, float distanceTol)
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


std::unordered_set<int> Ransac(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIterations, float distanceTol)
{
    std::unordered_set<int> inliersResult;
    std::unordered_set<int> tmp_inliersResult;
    srand(time(NULL));
    
    // TODO: Fill in this function
    //find the total number of cloud points
    auto no_of_cloud_points = cloud->points.size();
    auto max_inliers = 0; 

    //std::cout << "no_of_cloud_points" << no_of_cloud_points <<std::endl;

    while(maxIterations) { // For max iterations 

        auto count_inliers = 0;  

        // Randomly sample subset and fit line
        auto random_index1 = rand() % no_of_cloud_points;
        auto random_index2 = rand() % no_of_cloud_points;

        //if both points are the same, retry until different
        while (random_index1 == random_index2) { 
            std::cout<< "Randomly generated index was same. Retrying" << std::endl;
            random_index2 = rand() % no_of_cloud_points;
        }

        //std::cout << "random point index are " << random_index1 << " and " << random_index2 << std::endl;
        //std::cout << "random points are " << cloud->points[random_index1] << " and " << cloud->points[random_index2] << std::endl;

        //point coordinates for the line
        auto x1 = cloud->points[random_index1].x;
        auto y1 = cloud->points[random_index1].y;
        auto x2 = cloud->points[random_index2].x;
        auto y2 = cloud->points[random_index2].y;

        //generate the line parametes Ax+By+C=0
        auto A = y1-y2;
        auto B = x2-x1;
        auto C = (x1*y2)-(x2*y1);

        //iterate through all the points and find if they are inliers
        for(int index = 0; index < cloud->points.size(); index++)
        {
            //index of our line points will also come here. We will add them here

            auto i = cloud->points[index];
            std::cout<< "Selected point for distance measurement is x=" << i.x << " and y=" << i.y << std::endl;

            //find the distance d = |Ax+By+C|/sqrt(A^2+B^2)
            auto distance = fabs((A*i.x) + (B*i.y) + C)/ sqrt((A*A)+(B*B));

            std::cout << "distance from the point to the line is " << distance << std::endl;

            if( distance < distanceTol) //inside tolerance
            {
                tmp_inliersResult.insert(index); //save the index
                count_inliers++;
            }
        }

        if (count_inliers > max_inliers)
        {
            max_inliers = count_inliers; //reset the max
            inliersResult = tmp_inliersResult;
        }
        else
        {
            /* clear it as it is not the line with maximum inliers*/
            tmp_inliersResult.clear();
        }
        
        // Measure distance between every point and fitted line
        // If distance is smaller than threshold count it as inlier

        maxIterations--;
    }

    // Return indicies of inliers from fitted line with most inliers
    
    return inliersResult;

}

int main ()
{

    // Create viewer
    pcl::visualization::PCLVisualizer::Ptr viewer = initScene();

    // Create data
    //pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = CreateData();
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = CreateData3D();
    

    // TODO: Change the max iteration and distance tolerance arguments for Ransac function
    //std::unordered_set<int> inliers = Ransac(cloud, 50, 0.5);
    std::unordered_set<int> inliers = RansacPlane(cloud, 100, 0.2);

    pcl::PointCloud<pcl::PointXYZ>::Ptr  cloudInliers(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudOutliers(new pcl::PointCloud<pcl::PointXYZ>());

    for(int index = 0; index < cloud->points.size(); index++)
    {
        pcl::PointXYZ point = cloud->points[index];
        if(inliers.count(index))
            cloudInliers->points.push_back(point);
        else
            cloudOutliers->points.push_back(point);
    }


    // Render 2D point cloud with inliers and outliers
    if(inliers.size())
    {
        renderPointCloud(viewer,cloudInliers,"inliers",Color(0,1,0));
          renderPointCloud(viewer,cloudOutliers,"outliers",Color(1,0,1));
    }
      else
      {
          renderPointCloud(viewer,cloud,"data");
      }
    
      while (!viewer->wasStopped ())
      {
        viewer->spinOnce ();
      }
      
}
