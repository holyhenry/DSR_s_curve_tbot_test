#include "camera/cam_node.hpp"

Cam_Node::Cam_Node(ros::NodeHandle *nh) 
{
    std::string ns = ros::this_node::getNamespace();
    cam_data_pub = nh->advertise<geometry_msgs::Point>(ns + "/april_data", 1);
    img_raw_sub = nh->subscribe("/camera/color/image_raw", 1, &Cam_Node::img_raw_callback, this);
    
};

void Cam_Node::img_raw_callback(const sensor_msgs::Image::ConstPtr& msg)
{
    const float markerLength = 0.04;
    cv::Mat imageCopy;
    cv_ptr = cv_bridge::toCvCopy(msg, msg->encoding);
    cv_ptr->image.copyTo(imageCopy);
    std::vector<int> ids;
    std::vector<std::vector<cv::Point2f> > corners;

    auto ms = geometry_msgs::Point();
    // Detect this type of aruco
    cv::aruco::detectMarkers(cv_ptr->image, dictionary, corners, ids);
    int size = ids.size();
    
    if (size > 0)
    {    
        cv::aruco::drawDetectedMarkers(imageCopy, corners, ids);

        // Create a vector to store rvecs and tvecs
        std::vector<cv::Vec3d> rvecs, tvecs;
        cv::aruco::estimatePoseSingleMarkers(corners, markerLength, cameraMatrix, distCoeffs, rvecs, tvecs);
        
        cv::aruco::drawAxis(imageCopy, cameraMatrix, distCoeffs, rvecs[0], tvecs[0], 0.2);
        cv::Vec3d tvec = tvecs[0];
        cv::Vec3d rvec = rvecs[0];
        float id = ids[0];


        float x = tvec[0];  //left and right
        float y = tvec[1];  //depth into camera .... offset is relative to coordinate frame of april tag
        float z = tvec[2];  // .... offset is relative to coordinate frame of april tag

        ms.x = x;
        ms.y = y;
        ms.z = z;

        cam_data_pub.publish(ms);
        
        // std::cout << "Z direction: "<< z << "\n"; 
        // float dist = sqrt(pow(x,2) + pow(y,2) + pow(z,2));
        // float angle = std::atan2(y, x);


    }
    // cv::imshow(OPENCV_WINDOW, imageCopy);
    cv::waitKey(1);

}




int main(int argc, char **argv)
{
    ros::init(argc, argv, "camera");
    ros::NodeHandle nh;
    Cam_Node camnode = Cam_Node(&nh);

    while(ros::ok())
    {
        ros::spinOnce();
    }
    
    return 0;
}
