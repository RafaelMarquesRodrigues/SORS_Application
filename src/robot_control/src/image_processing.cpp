#include "../include/robot_control/image_processing.h"

ImageProcessing::ImageProcessing(ros::NodeHandle n, char* type): node(n), found(0),
	processImageServer(n, PROCESS_IMAGE_ACTION, boost::bind(&ImageProcessing::processImage, this, _1), false){
	camera_sub = node.subscribe(CAMERA_TOPIC(type), 1, &ImageProcessing::handleImage, this);

	processImageServer.start();
}

ImageProcessing::~ImageProcessing(){}

bool ImageProcessing::processImage(const robot_control::processImageGoalConstPtr& image_goal){
	ros::Rate r(10.0);
	
	while(ros::ok()){
		if(found == 20)
			break;
		r.sleep();
	}

	ROS_INFO("found !");

	robot_control::processImageResult result;

    processImageServer.setSucceeded(result);

    return true;
}

void ImageProcessing::handleImage(const sensor_msgs::ImageConstPtr& img){
 	cv_bridge::CvImagePtr cv_ptr;
    cv::Mat img_bgr;

    try{
      cv_ptr = cv_bridge::toCvCopy(img, "bgr8");
      img_bgr = cv_ptr -> image;
    }
    catch(cv_bridge::Exception& e){
      ROS_ERROR("cv_bridge exception: %s", e.what());

      return;
    }

    unsigned int red_pixels = 0;
    cv::Mat img_binary_0;
    cv::Mat img_binary_1;
    cv::Mat img_hsv;
    int x, y;
  	//filter 
    //cv::medianBlur(img_bgr, img_bgr, 3);
	cv::cvtColor(img_bgr, img_hsv, CV_BGR2HSV);

	// Perform thresholding
	cv::inRange(img_hsv, cv::Scalar(0, 100, 100), cv::Scalar(10, 255, 255), img_binary_0);
	//cv::inRange(img_hsv, cv::Scalar(160, 100, 100), cv::Scalar(179, 255, 255), img_binary_1);

	// Loop over the image
	for(y = 0; y < img_binary_0.rows; y++){
		for(x = 0; x < img_binary_0.cols; x++){
			if(img_binary_0.at<unsigned char>(y,x) == 255){
				++red_pixels; // Count ’red’ pixels
			}
		}
	}
/*
	for(y = 0; y < img_binary_1.rows; y++){
		for(x = 0; x < img_binary_1.cols; x++){
			if(img_binary_1.at<unsigned char>(y,x) == 255){
				++red_pixels; // Count ’red’ pixels
			}
		}
	}
*/
	if(red_pixels){
		found++;
		//ROS_INFO("red image %d", found);
		//cv::imshow("HSV image", img_hsv);
		//cv::waitKey(0);
	}
	else
		found = 0;
}

int main(int argc, char **argv) {

    //Initializes ROS, and sets up a node
    ros::init(argc, argv, IMAGE_PROCESSING_NODE);

    if(argc < 2){
        ROS_INFO("Robot type not specified. Shuting down...");
        return -1;
    }

    ros::NodeHandle node;

    ImageProcessing *processor = new ImageProcessing(node, argv[1]);

    ROS_INFO("Image Processing started.");

    ros::spin();

    delete processor;

    return 0;
}