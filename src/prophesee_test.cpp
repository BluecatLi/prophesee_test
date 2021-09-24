#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <iostream>
#include <metavision/sdk/driver/camera_generation.h>
#include <metavision/sdk/core/events/event_cd.h>
#include "metavision/sdk/core/utils/cd_frame_generator.h"
#include <opencv2/opencv.hpp>


#include <metavision/sdk/driver/prophesee_driver.h>

//File includes
#include <iostream>
#include <fstream>


class ImageGenerator {
public:
	ImageGenerator(int width,int height){
		width_ = width;
		height_ = height;
		frame_ = cv::Mat(height_, width_, CV_16SC1, cv::Scalar(0));
		frame_ptr = (short*)frame_.data;
	}
	cv::Mat frame_;
	short* frame_ptr;
	int width_, height_;
	std::vector<Metavision::EventCD> event_buffer_;
};



int main(int argc, char** argv) {
	ros::init(argc, argv, "prophesee_publisher");
	ros::NodeHandle nH;

    image_transport::ImageTransport ita(nH);
	ros::NodeHandle nh_("~");
	// Load Parameters

    std::string camera_name_, publish_iv_, raw_file_to_read_;
    nh_.getParam("camera_name", camera_name_);
    nh_.getParam("publish_iv", publish_iv_);
    nh_.getParam("raw_file_to_read", raw_file_to_read_);
	image_transport::Publisher iv_pub = ita.advertise("/topic_iv_frame", 1);

	Metavision::Camera cam;
	// cam = Metavision::Camera::from_file("/home/yufan/Documents/Prophesee/x_1.raw");
	cam = Metavision::Camera::from_file("/home/yufan/Documents/Prophesee/x_1.raw");

	std::cout << "Kitty on your lap!" << std::endl;


	//unsigned int max_event_rate_(6000);
	//if (!cam.set_max_event_rate_limit(max_event_rate_)) {
	//	std::cout << "maxev_rate" << std::endl;
	//}
			// get camera resolution
	int camera_width = cam.geometry().width();
	int camera_height = cam.geometry().height();

	ImageGenerator ig(camera_width, camera_height);
	int event_cnt = 0;
	std::time_t prev = std::time(nullptr);
	// auto accumulation_duration;
	Metavision::CallbackId cd_callback =
		cam.cd().add_callback([&ig](const Metavision::EventCD *ev_begin, const Metavision::EventCD *ev_end) {
			

				// ig.frame_.setTo(cv::Scalar(0));
		auto inserter = std::back_inserter(ig.event_buffer_);
			{
				/** When there is not activity filter **/
				std::copy(ev_begin, ev_end, inserter);
			}

			// std::cout<<"Once"<<std::endl;
			for (const Metavision::EventCD *it = std::addressof(ig.event_buffer_[0]);
				it != std::addressof(ig.event_buffer_[ig.event_buffer_.size()]); ++it) {
				// ig.frame_.setTo(cv::Scalar(0));
				if (it->p == 1)
				{
					//Use array access instead of cv::Mat::at function (Slow)
					//ig.frame_.at<int16_t>(it->y, it->x) += 1;
					ig.frame_ptr[it->y *ig.width_ + it->x] += 1;
				}
				else {
					//				cd_frame_generator_.frame_.at<int16_t>(it->y, it->x) += -1;
					ig.frame_ptr[it->y *ig.width_ + it->x]-= 1;
				}
				// event_cnt += ig.event_buffer_.size();
			}

			//setto scalar
			(ig.event_buffer_).clear();

	});

	const std::string window_name = "Metavision SDK Get Started";
	// start the camera
	
	cam.start();

	cv::Mat cd_frame,cdshow;
	const int threshold = 10000;//please set appropriate value
	const int fps=25;

	const int wait_time = 10;//ms
	while (cam.is_running()) {
		// display the frame if it's not empty
		if (!ig.frame_.empty()) {
			// ig.frame_.copyTo(cd_frame);
			// cd_frame.convertTo(cdshow,CV_8UC1, 5.0, 128.0);
			// cv::imshow(window_name, cdshow);
			cv::Mat frame_Part = ig.frame_(cv::Range(160, 460),cv::Range(160,430));
			// cv::Scalar eventsum = cv::sum(cv::abs(ig.frame_));
			cv::Scalar eventsum = cv::sum(cv::abs(frame_Part));
            if(eventsum(0) > threshold){
				sensor_msgs::ImagePtr msg;
				msg = cv_bridge::CvImage(std_msgs::Header(), "16SC1", ig.frame_).toImageMsg();
				iv_pub.publish(msg);
				ig.frame_.setTo(cv::Scalar(0));
				// eventsum = 0;
            }
			// cv::waitKey(0);
		}

		// if the user presses the `q` key, quit the loop
		if ((cv::waitKey(wait_time) & 0xff) == 'q') {
			break;
		}
	}

	
	return 0;
}

// 	const int wait_time = static_cast<int>(std::round(1.f / fps * 1000)); // how much we should wait between two frames
// 	// keep running while the camera is on or the video is not finished
// 	while (cam.is_running()) {
// 		// display the frame if it's not empty
// 		if (!ig.frame_.empty()) {
// 			// ig.frame_.copyTo(cd_frame);
// 			// cd_frame.convertTo(cdshow,CV_8UC1, 5.0, 128.0);
// 			// cv::imshow(window_name, cdshow);
//             sensor_msgs::ImagePtr msg;
//             msg = cv_bridge::CvImage(std_msgs::Header(), "16SC1", ig.frame_).toImageMsg();
// 			iv_pub.publish(msg);
// 			cv::Scalar eventsum = cv::sum(cv::abs(ig.frame_));

//         	std::fstream eventfile("/home/yufan/Pictures/0922/evtNum_z3.txt",std::fstream::app);
// 			eventfile<<eventsum(0)<<std::endl;

// 			ig.frame_.setTo(cv::Scalar(0));
// 			// cv::waitKey(0);
// 		}
// 		// if the user presses the `q` key, quit the loop
// 		if ((cv::waitKey(wait_time) & 0xff) == 'q') {
// 			break;
// 		}
// 	}	
// 	return 0;
// }