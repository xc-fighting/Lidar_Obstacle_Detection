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

std::unordered_set<int> Ransac(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIterations, float distanceTol)
{
	std::unordered_set<int> inliersResult;
	srand(time(NULL));
	
	// TODO: Fill in this function
    // z coordinate will be 0
	int totalPoints = cloud->points.size();
	// For max iterations 
    for( int iteration = 0; iteration < maxIterations; iteration++ ) {
		 //use a temp buffer for inlier storage
		 std::vector<int> indices;
         // Randomly sample subset and fit line, in this case, choose two point
		 int index1 = rand() % totalPoints;
		 int index2 = rand() % totalPoints;
		 while( index2 == index1 ) {
	  		 index2 = rand() % totalPoints;
		 }
		 std::cout<<"choose index:"<<index1<<",index:"<<index2<<std::endl;
		 indices.push_back(index1);
		 indices.push_back(index2);
		 pcl::PointXYZ point1 = cloud->points[index1];
		 pcl::PointXYZ point2 = cloud->points[index2];
		 float x1 = point1.x;
		 float y1 = point1.y;
		 float x2 = point2.x;
		 float y2 = point2.y;
         // A = y1-y2
		 float A = y1 - y2;
		 // B = x2 - x1
		 float B = x2 - x1;
		 // C = x1*y2 - x2*y1
		 float C = x1*y2 - x2*y1;
         float sqrtAB = sqrt(A*A + B*B);
		 for( int index = 0; index < totalPoints; index++ ) {
              // Measure distance between every point and fitted line
	          // If distance is smaller than threshold count it as inlier
			  if( index == index1 || index == index2 ) {
				  //no need to compute again
				  continue;
			  }
			  float curX = cloud->points[index].x;
		      float curY = cloud->points[index].y;
			  float distance = abs(A*curX + B*curY + C) / sqrtAB;
			  if( distance <= distanceTol ) {
				   indices.push_back(index);
			  }
		 }
	     if( indices.size() > inliersResult.size() ) {
			 inliersResult.clear();
			 for( int elem: indices ) {
				 inliersResult.insert(elem);
			 }
			 indices.clear();
		 }
	    
	}
	
	// Return indicies of inliers from fitted line with most inliers
	return inliersResult;

}


std::unordered_set<int> RansacPlane(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIterations, float distanceTol) {
   
   std::unordered_set<int> inliersResult;
   srand(time(NULL));
   
   int pointSize = cloud->points.size();
   for( int iteration = 0; iteration < maxIterations; iteration++ ) {
	   //indices for candidate inliers
	   std::vector<int> indices;
	   //generate three random point index for plane generation
	   int index1 = rand() % pointSize;
	   int index2 = rand() % pointSize;
	   while( index2 == index1 ) {
		   index2 = rand() % pointSize;
	   }
	   int index3 = rand() % pointSize;
	   while( index3 == index1 || index3 == index2 ) {
		   index3 = rand() % pointSize;
	   }
	   //push the three points into indices
	   indices.push_back(index1);
	   indices.push_back(index2);
	   indices.push_back(index3);

       //first point x,y,z
	   float x1,y1,z1;
	   x1 = cloud->points[index1].x;
	   y1 = cloud->points[index1].y;
	   z1 = cloud->points[index1].z;
       
	   float x2,y2,z2;
	   x2 = cloud->points[index2].x;
	   y2 = cloud->points[index2].y;
	   z2 = cloud->points[index2].z;

	   float x3,y3,z3;
	   x3 = cloud->points[index3].x;
	   y3 = cloud->points[index3].y;
	   z3 = cloud->points[index3].z;
 
       // from 1 -> 2
       float vector1_x, vector1_y, vector1_z;
	   vector1_x = x2 - x1;
	   vector1_y = y2- y1;
	   vector1_z = z2 - z1;
	   // from 1 -> 3
	   float vector2_x, vector2_y, vector2_z;
       vector2_x = x3 - x1;
	   vector2_y = y3 - y1;
	   vector2_z = z3 - z1;
       // normal vector of vector 1 and 2
	   float i = vector1_y * vector2_z - vector1_z * vector2_y;
	   float j = vector1_z * vector2_x - vector1_x * vector2_z;
	   float k = vector1_x  * vector2_y - vector1_y * vector2_x;
       
	   float A = i;
	   float B = j;
	   float C = k;
	   float D = -( i*x1 + j*y1 + k*z1 );

	   float sqrtABC = sqrt(A*A + B*B + C*C);
	   for( int idx = 0; idx < pointSize; idx++ ) {
            if( idx == index1 || idx == index2 || idx == index3 ) {
				continue;
			}
			float x = cloud->points[idx].x;
			float y = cloud->points[idx].y;
			float z = cloud->points[idx].z;
			float distance = abs(A*x + B*y + C*z + D) / sqrtABC;
			if( distance <= distanceTol ) {
				indices.push_back(idx);
			}
	   }

	   if( indices.size() > inliersResult.size() ) {
		   inliersResult.clear();
		   for( int elem: indices ) {
			   inliersResult.insert(elem);
		   }
		   indices.clear();
	   }
   }

   return inliersResult;
}
int main ()
{

	// Create viewer
	pcl::visualization::PCLVisualizer::Ptr viewer = initScene();

	// Create data
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = CreateData3D();//CreateData();
	

	// TODO: Change the max iteration and distance tolerance arguments for Ransac function
	//after check 100, 0.5 seems best to filter out road
	std::unordered_set<int> inliers = RansacPlane(cloud,100,0.5);//Ransac(cloud, 10, 1.0);

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
  		renderPointCloud(viewer,cloudOutliers,"outliers",Color(1,0,0));
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
