#include "camera_multi_tag/cam_multi_tag_node.hpp"

Cam_Node::Cam_Node(ros::NodeHandle *nh) 
{
    cam_data_pub = nh->advertise<std_msgs::Float32MultiArray>("/april_data_multi", 10);
    img_raw_sub = nh->subscribe("/camera/color/image_raw", 10, &Cam_Node::img_raw_callback, this);
    
};

void Cam_Node::img_raw_callback(const sensor_msgs::Image::ConstPtr& msg)
{
    const float markerLength = 0.035;
    cv::Mat imageCopy;
    cv_ptr = cv_bridge::toCvCopy(msg, msg->encoding);
    cv_ptr->image.copyTo(imageCopy);
    std::vector<int> ids;
    std::vector<std::vector<cv::Point2f> > corners;
    // Detect this type of aruco
    cv::aruco::detectMarkers(cv_ptr->image, dictionary, corners, ids);
    int size = ids.size();
    auto ms = std_msgs::Float32MultiArray();
    ms.data.clear();
    // if at least one marker detected
    if (ids.size() > 0)
    {
        ms.layout.dim.push_back(std_msgs::MultiArrayDimension());
        ms.layout.dim.push_back(std_msgs::MultiArrayDimension());
        ms.layout.dim[0].label = "height";
        ms.layout.dim[1].label = "width";
        ms.layout.dim[0].size = size;
        ms.layout.dim[1].size = W;
        ms.layout.dim[0].stride = size*W;
        ms.layout.dim[1].stride = W;
        ms.layout.data_offset = markerLength;
        std::vector<float> vec(size*W, 0);
        cv::aruco::drawDetectedMarkers(imageCopy, corners, ids);

        // Create a vector to store rvecs and tvecs
        std::vector<cv::Vec3d> rvecs, tvecs;
        cv::aruco::estimatePoseSingleMarkers(corners, markerLength, cameraMatrix, distCoeffs, rvecs, tvecs);
        for(int i=0; i<size; i++)
        {
            cv::aruco::drawAxis(imageCopy, cameraMatrix, distCoeffs, rvecs[i], tvecs[i], 0.2);
            cv::Vec3d tvec = tvecs[i];
            cv::Vec3d rvec = rvecs[i];

            float id = ids[i];
            float x = tvec[0];  // robot frame - left(-) & right(+)
            float y = tvec[1];  // robot frame - up(-) & down(+)
            float z = tvec[2];  // robot frame - foreward(+) & backward(-)
            
            cv::Mat1d R(3,3);
	        Rodrigues(rvec, R);
            float euler_y = -atan2(-R(2,0),sqrt(pow(R(2,1),2)+pow(R(2,2),2)))*180/3.14159;
            // std::cout<<"x-"<<atan2(R(2,1),R(2,2))*180/3.14159<<std::endl;
            // std::cout<<"y-"<<atan2(-R(2,0),sqrt(pow(R(2,1),2)+pow(R(2,2),2)))*180/3.14159<<std::endl;
            // std::cout<<"z-"<<atan2(R(1,0),R(0,0))*180/3.14159<<std::endl;

            vec[W*i] = id;
            vec[W*i + 1] = x;
            vec[W*i + 2] = y;
            vec[W*i + 3] = z; 
            vec[W*i + 4] = euler_y;

        }

        ms.data = vec;
        cam_data_pub.publish(ms);
    }

    // cv::imshow(OPENCV_WINDOW, imageCopy);
    // cv::waitKey(1);


}




int main(int argc, char **argv)
{
    ros::init(argc, argv, "camera_multi_tag");
    ros::NodeHandle nh;
    Cam_Node camnode = Cam_Node(&nh);

    // if (nh.getParam("/params/ros__parameters/markerlength", camnode.markerLength))
    // {
    //     ROS_INFO("Got camera width param");
    // }
    // else 
    // {
    //     ROS_ERROR("Failed to get camera width param");
    // }

    while(ros::ok())
    {
        ros::spinOnce();
    }
    
    return 0;
}
