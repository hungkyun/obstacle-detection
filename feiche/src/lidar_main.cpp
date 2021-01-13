/*
  author: linhq
*/
#include "lidar_main.h"
using namespace Eigen;
using namespace std;
//显示对象创建
boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("viewer"));
class Lidar_node{
public:
	Lidar_node();
	void process(const sensor_msgs::PointCloud2 &scan);
private:
	ros::NodeHandle node_handle_;
	ros::Subscriber sub;
	ros::Publisher pub;
	ros::Publisher fpub;
};
Lidar_node::Lidar_node(){
	pub = node_handle_.advertise<sensor_msgs::PointCloud2 >("raw",10);
	fpub = node_handle_.advertise<sensor_msgs::PointCloud2 >("exract",10);
	sub = node_handle_.subscribe("/rfans_driver/rfans_points", 1028, &Lidar_node::process, this);
}
void aabb(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud,string name,pcl::PointXYZI minPt,pcl::PointXYZI maxPt,bool position)//点云AABB包围盒
{
	//绘制AABB包围盒
	if(position>0)
		viewer->addCube(minPt.x, maxPt.x, minPt.y, maxPt.y, minPt.z, maxPt.z, 0.0, 1.0, 0.0, name);
	else
		viewer->addCube(minPt.x, maxPt.x, minPt.y, maxPt.y, minPt.z, maxPt.z, 1.0, 0.0, 0.0, name);
	viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION, pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME, name);
	// viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR,1,1,0,name);
}
void clustering(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud1,pcl::ModelCoefficients::Ptr planetmp){
	vector<pcl::PointIndices> cluster_indices;
	pcl::search::KdTree<pcl::PointXYZI>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZI>);
	pcl::EuclideanClusterExtraction<pcl::PointXYZI> ece;
	pcl::PointXYZI minPt, maxPt, center;
	bool position;
	ece.setInputCloud(cloud1);
	ece.setClusterTolerance(1);
	ece.setMinClusterSize(100);
	ece.setMaxClusterSize(2000);
	ece.setSearchMethod(tree);
	ece.extract(cluster_indices);
	pcl::ExtractIndices<pcl::PointXYZI> ext;
	ext.setInputCloud(cloud1);
	cout<<"ok"<<cluster_indices.size()<<endl;
	for (vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it) {
		pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZI>);
		for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
			cloud_cluster->points.push_back (cloud1->points[*pit]);
		pcl::getMinMax3D (*cloud_cluster, minPt, maxPt);
		center.x=(minPt.x+maxPt.x)/2;
		center.y=(minPt.y+maxPt.y)/2;
		center.z=(minPt.z+maxPt.z)/2;
		position = ((planetmp->values[0]*(center.x+planetmp->values[1]*center.y+planetmp->values[3])/planetmp->values[2]+center.z)>0?1:0); //(ax+by+d)/c+z
		aabb(cloud_cluster,boost::to_string(*it),minPt,maxPt,position);
	}
}
 pcl::ModelCoefficients::Ptr get_plane(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud1,pcl::PointCloud<pcl::PointXYZI>::Ptr cloud2){ //cloud输入点云，cloud2去除平面的点（这个只是用来显示，要参数再另外加）
	//创建分割对象 -- 检测平面参数
	pcl::PointCloud<pcl::PointXYZI>::Ptr output(new pcl::PointCloud<pcl::PointXYZI>);
	pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients()); //存储输出的模型的系数
	pcl::PointIndices::Ptr inliers(new pcl::PointIndices); //存储内点
	pcl::SACSegmentation<pcl::PointXYZI> seg;
	pcl::ExtractIndices<pcl::PointXYZI> extract;
	//参数
	seg.setOptimizeCoefficients(true);
	seg.setModelType(pcl::SACMODEL_PLANE); //设置模型类型，检测平面
	seg.setMethodType(pcl::SAC_RANSAC);      //设置方法
	seg.setMaxIterations(1000);
	seg.setDistanceThreshold(0.2);
	seg.setInputCloud(cloud1);
	seg.segment(*inliers, *coefficients);    //分割操作
	extract.setInputCloud(cloud1);
	extract.setNegative(true);
	extract.setIndices(inliers);
	extract.filter(*cloud2);//提取对于索引的点云 内点
	cout<<cloud2->points.size()<<endl;
	// std::cerr << "Model coefficients: " << coefficients->values[0] << " " << coefficients->values[1] << " "
	// 					<< coefficients->values[2] << " " << coefficients->values[3] << std::endl;
	return coefficients;
}
void Lidar_node::process(const sensor_msgs::PointCloud2 &scan/*,const sensor_msgs::PointCloud2 &scan2*/){
	
	// pcl_conversions::toPCL(scan,pcl_pc);
	// pcl_conversions::toPCL(scan,pcl_pc);
	pcl::PointCloud<pcl::PointXYZI>::Ptr cloud32(new pcl::PointCloud<pcl::PointXYZI>);
	pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
	// pcl::fromPCLPointCloud2(pcl_pc,*cloud);
	pcl::fromROSMsg(scan,*cloud);
	pcl::PointCloud<pcl::PointXYZI>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZI>);
	pcl::PointCloud<pcl::PointXYZI>::Ptr groundinput(new pcl::PointCloud<pcl::PointXYZI>);
	pcl::PointCloud<pcl::PointXYZI>::Ptr groundoutput(new pcl::PointCloud<pcl::PointXYZI>);
	pcl::ModelCoefficients::Ptr plane(new pcl::ModelCoefficients());
	pcl::ModelCoefficients model;
	pcl::VoxelGrid<pcl::PointXYZI> sor;
	for (size_t i = 0; i < cloud->points.size(); ++i) {
			pcl::PointXYZI points=cloud->points[i];
			if(((fabs(points.x))<8 && (fabs(points.y)<8)) && ((fabs(points.x))>1 || (fabs(points.y)>1))  && points.z<4)
				temp_cloud->points.push_back(points);
	}
	sor.setInputCloud(temp_cloud);
	sor.setLeafSize(0.1f,0.1f,0.1f);
	sor.filter(*groundinput);
	plane = get_plane(groundinput,groundoutput); //这里改成了去除平面
	clustering(groundoutput,plane);
	std::cerr << "Model coefficients: " << plane->values[0] << " " << plane->values[1] << " "
						<< plane->values[2] << " " << plane->values[3] << std::endl;
	viewer->addPointCloud<pcl::PointXYZI>(cloud, "pc");
	viewer->spinOnce(1);
	viewer->removeAllPointClouds();
	viewer->removeAllShapes();
  //publish points
	sensor_msgs::PointCloud2 raw;
	pcl::toROSMsg(*groundinput,raw);
	raw.header.frame_id = "world";
	pub.publish(raw);
	sensor_msgs::PointCloud2 extract;
	pcl::toROSMsg(*groundoutput,extract);
	extract.header.frame_id = "world";
	fpub.publish(extract);
}
int main(int argc, char **argv)
{
	srand(time(NULL));
  ros::init(argc,argv,"Lidar_node");
	Lidar_node node;
	viewer->setBackgroundColor(0, 0, 0);
	viewer->addCoordinateSystem(1.0);
	cout<<"standby11!"<<endl;
	ros::spin();
  return 0;
}
