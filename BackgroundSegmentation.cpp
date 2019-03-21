#include <iostream>
#include <fstream>
#include <string>
#include <sstream>
#include <pcl/point_types.h>
#include <boost/asio.hpp>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/search/search.h>
#include <pcl/segmentation/segment_differences.h>
#include <pcl/filters/radius_outlier_removal.h>

std::string read_data(boost::asio::ip::tcp::socket & socket)
{
    boost::asio::streambuf buf;
    boost::asio::read_until( socket, buf, "\n");
    // boost::system::error_code error;
    // boost::asio::read(socket, buf, boost::asio::transfer_all(), error);
    std::string data = boost::asio::buffer_cast<const char*>(buf.data());
    return data;
}

void send_data(boost::asio::ip::tcp::socket & socket, const std::string& message)
{
    const std::string msg = message + "\n";
    boost::asio::write( socket, boost::asio::buffer(message) );
}

int main()
{
    // cloud store the background point cloud and target
    pcl::PointCloud<pcl::PointXYZ>::Ptr background (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr frontground (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr target (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr target2 (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr target_filtered (new pcl::PointCloud<pcl::PointXYZ>);
    
    //Color handlers for red, green, blue and yellow color
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> red(background,255,0,0);
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> blue(frontground,0,0,255);
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> green(target,0,255,0);
    //pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> yellow(target2,255,255,0);
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> yellow(target_filtered,255,255,0);
    
    // read file name
    
    int frame_number = 10;
    std::string source_frontground = "../multi_person/frame-32.txt";
    // read file
    std::ifstream infile_background(source_frontground);
    //read stream line by line
    for(frame_number ; frame_number < 21 ; frame_number++)
    {
        std::string source = "../multi_person/frame-";
        std::string file_type = ".txt";
        std::string file = source + std::to_string(frame_number) + file_type;
        std::ifstream infile(file);
        for(std::string line; std::getline(infile, line);)
        {
            // create structure
            pcl::PointXYZ goalPosition;
            std::istringstream in (line);
            in >> goalPosition.x >> goalPosition.y >> goalPosition.z;
            background->push_back(goalPosition);
        }
    }
    for(std::string line; std::getline(infile_background, line);)
    {
        pcl::PointXYZ goalPosition;
        std::istringstream in (line);
        in >> goalPosition.x >> goalPosition.y >> goalPosition.z;
        frontground->push_back(goalPosition);
    }
    
    //std::string output = std::to_string(background->points[0].x) + " " + std::to_string(background->points[0].y) + " " + std::to_string(background->points[0].z);
    //std::cout << output << std::endl;
    //Segment differences
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
    pcl::SegmentDifferences<pcl::PointXYZ> sdiff;
    sdiff.setTargetCloud(background);
    sdiff.setInputCloud(frontground);
    sdiff.setSearchMethod(tree);
    sdiff.setDistanceThreshold(0);
    sdiff.segment(*target);
    
    std::cout << *target;
    // Get point cloud difference
    //pcl::getPointCloudDifference<pcl::PointXYZ> (*background,*frontground,1,tree,*target2);
    //std::cout << "\n\n" << *target2;
    
    // filter outliers with RadiusOutlier removal
    pcl::RadiusOutlierRemoval<pcl::PointXYZ> outrem;
    // build the filter
    outrem.setInputCloud(target);
    outrem.setRadiusSearch(0.1);
    outrem.setMinNeighborsInRadius (45);
    // apply filter
    outrem.filter (*target_filtered);
    
    //Visualiztion
    //pcl::visualization::PCLVisualizer vis("3D View");
    //vis.addPointCloud(background,red,"background",0);
    //vis.addPointCloud(frontground,green,"frontground",0);
    //vis.addPointCloud(target,blue,"target",0);
    //vis.addPointCloud(target2,yellow,"target2",0);
    //vis.addPointCloud(target_filtered,yellow,"target_filtered",0);
    //vis.addCoordinateSystem(100);
    //OpenWindow
    /*while(!vis.wasStopped())
    {
        vis.spinOnce();
    }*/
    // create tcp/ip server to send
    
    // put target_cloud to sendString
    std::stringstream ss;
    for (size_t i = 0; i < target_filtered->points.size(); ++i)
    {
        ss << std::to_string(target_filtered->points[i].x) + " " + std::to_string(target_filtered->points[i].y) + " " + std::to_string(target_filtered->points[i].z) + " ";
    }
    // create io instance
    boost::asio::io_context _io_service;
    // create listener for new connection
    boost::asio::ip::tcp::acceptor _acceptor(_io_service, boost::asio::ip::tcp::endpoint(boost::asio::ip::tcp::v4(), 8000));
    // create socket
    boost::asio::ip::tcp::socket _socket(_io_service);
    // waiting for connection
    _acceptor.accept(_socket);
    //read operation
    while(1)
    {
        //std::cout << "server read" << std::endl;
        //std::string message = read_data(_socket);
        //std::cout << message << std::endl;
        //write operation
        ss << "\n";
        std::string msg = ss.str();
        std::cout << "server send " << msg << std::endl;
        send_data(_socket, msg);
        //sleep(1);
        std::cout << "end of while" << std::endl;
        //sleep(1);
    }
    
    return 0;
    /*pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
    
    // Fill in the cloud data
    cloud->width  = 5;
    cloud->height = 1;
    cloud->points.resize (cloud->width * cloud->height);
    
    for (size_t i = 0; i < cloud->points.size (); ++i)
    {
        cloud->points[i].x = 1024 * rand () / (RAND_MAX + 1.0f);
        cloud->points[i].y = 1024 * rand () / (RAND_MAX + 1.0f);
        cloud->points[i].z = 1024 * rand () / (RAND_MAX + 1.0f);
    }
    
    std::cerr << "Cloud before filtering: " << std::endl;
    for (size_t i = 0; i < cloud->points.size (); ++i)
    std::cerr << "    " << cloud->points[i].x << " "
    << cloud->points[i].y << " "
    << cloud->points[i].z << std::endl;
    
    // Create the filtering object
    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud (cloud);
    pass.setFilterFieldName ("z");
    pass.setFilterLimits (0.0, 1.0);
    //pass.setFilterLimitsNegative (true);
    pass.filter (*cloud_filtered);
    
    std::cerr << "Cloud after filtering: " << std::endl;
    for (size_t i = 0; i < cloud_filtered->points.size (); ++i)
    std::cerr << "    " << cloud_filtered->points[i].x << " "
    << cloud_filtered->points[i].y << " "
    << cloud_filtered->points[i].z << std::endl;
    
    return (0);*/
}
