#include <cstdio>
#include <cstdlib>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <std_msgs/Empty.h>
#include <sensor_msgs/image_encodings.h>
#include <bender_msgs/BlobList.h>
#include <bender_msgs/Rect.h>
#include <bender_srvs/ImageService.h>
#include <bender_srvs/BlobList.h>
#include <bender_srvs/DetectEmerg.h>

#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>

#include <cv.h>
//#include "cvaux.h"
#include <highgui.h>


#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/objdetect/objdetect.hpp>
#include <opencv2/highgui/highgui.hpp>

using namespace std;

ros::ServiceServer server;
ros::ServiceClient get_blobs;
ros::ServiceClient im_client;
ros::ServiceClient bl_client;


const float prop_dist = float(2*73);
const float prop_seat = float(36);
bool display;

void displayResult(bender_srvs::ImageService img_srv, bender_msgs::Rect rect) {

    cv::Mat image_buffer;
    cv_bridge::CvImagePtr cv_ptr;

    try {
        cv_ptr = cv_bridge::toCvCopy(img_srv.response.im, sensor_msgs::image_encodings::BGR8);
        image_buffer=cv_ptr->image.clone();

    } catch (cv_bridge::Exception& e) {

        ROS_ERROR("Cannot transform color image for display. cv_bridge exception: %s", e.what());
        return;
    }

    cv::rectangle(
        image_buffer,
        cv::Point(rect.x, rect.y),
        cv::Point(rect.x + rect.width, rect.y + rect.height),
        cv::Scalar(255, 0, 255)
    );

	cv::imshow("emergency display", image_buffer);
	cv::waitKey(10);
}

bool DetectEmergency(bender_srvs::DetectEmerg::Request &req,  bender_srvs::DetectEmerg::Response &res) {

    // get image
    bender_srvs::ImageService img_srv;
    if(!im_client.call(img_srv)) {
        ROS_ERROR("Failed to call service camera_service");
        return false;
    }

    // get image blobs
    bender_srvs::BlobList blob_srv;
    if(!bl_client.call(blob_srv)) {
        ROS_ERROR("Failed to call service get_blobs");
        return false;
    }

    // initialize data
    int closest_idx = -1;
    int closest_pixels = -1;

    // look for a proper box
    for(unsigned int i=0; i < blob_srv.response.blob_list.BBoxes.size(); i++) {

    	int W = blob_srv.response.blob_list.BBoxes[i].width;
    	int H = blob_srv.response.blob_list.BBoxes[i].height;
    	int pixels = blob_srv.response.blob_list.nPixels[i];

        if( W < H && H < 2*W ) {

            if( pixels > closest_pixels || closest_pixels == -1) {

                closest_pixels = pixels;
                closest_idx=i;
                res.found=true;
            }
        }
    }

    // not found
    // - - - - - - - - - - - -
    if(closest_idx < 0) {
    	res.state = "notfound";
    	return true;
    }

    // found
    // - - - - - - - - - - - -
	bender_msgs::Rect closest = blob_srv.response.blob_list.BBoxes[closest_idx];

	res.distance = prop_dist/closest.height;
	float thresh = 70;

	// stand vs seat
	if( closest.y > 396 - 36*(res.distance-1) - thresh) {
		res.state = "seat";
	} else {
		res.state = "stand";
	}

	// display result if necessary
	if (display) {
		displayResult(img_srv, closest);
	}

    return true;
}

int main(int argc, char **argv) {

    ros::init(argc, argv, "detect_emergency");
    ros::NodeHandle nh("~");

    std::string cam_name;
    nh.param("cam_name",cam_name,std::string("default_camera"));
    nh.param("display",display,bool(true));
    
    im_client = nh.serviceClient<bender_srvs::ImageService>("/bender/sensors/" + cam_name + "/get_image");
    while ( ros::ok() && !im_client.waitForExistence(ros::Duration(3.0)) );

    bl_client = nh.serviceClient<bender_srvs::BlobList>("/bender/vision/blob_filter/get_blobs");
    while ( ros::ok() && !bl_client.waitForExistence(ros::Duration(3.0)) );

    server = nh.advertiseService("detect_emergency", DetectEmergency);


    // to prevent execution if ctrl+c is used while waiting
    if (!ros::ok())
    exit(1);

    ros::spin();
}
