#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/video/background_segm.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/video/video.hpp>

using namespace cv;
using namespace std;

static const std::string OPENCV_WINDOW = "Image window";

	class NC_BackgroundSubtractor
	{
		ros::NodeHandle nh_;
		image_transport::ImageTransport it_;
		image_transport::Subscriber image_sub_;
		image_transport::Publisher image_pub_;
		cv::Mat rgb_original_frame;
		BackgroundSubtractorMOG2 pMOG2;

		public:
			NC_BackgroundSubtractor()
				: it_(nh_)
		{
			// Subscribe to input video feed and publish output video feed
			image_sub_ = it_.subscribe("multisense/camera/left/image_raw", 1, 
			&NC_BackgroundSubtractor::imageCb, this);
			image_pub_ = it_.advertise("/background_subtractor/output_video", 1);


			cv::namedWindow(OPENCV_WINDOW);

			// set parameters related to the background subtraction methods.  Should tweak to reduce noise.
			// see webpage:  http://docs.opencv.org/2.4.8/modules/video/doc/motion_analysis_and_object_tracking.html#float%20varThresholdGen
			pMOG2.setDouble("fVarInit" , 30.0);
			pMOG2.setDouble("backgroundRatio", .999);
			pMOG2.setDouble("varThresholdGen", 1);
		}
		  
		~NC_BackgroundSubtractor()
		{
			cv::destroyWindow(OPENCV_WINDOW);
		}

		
		// callback when data arrives from subscription 
		void imageCb(const sensor_msgs::ImageConstPtr& msg)
		{
			cv_bridge::CvImagePtr cv_ptr;
			cv_bridge::CvImagePtr cv_out_ptr;

			try
			{
				// conver ROS image to OpenCV image
				cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
				
				// get first frame.  not really used right now but could be useful if we want to compare new frames against original.
				if (rgb_original_frame.empty())
					rgb_original_frame = cv_ptr->image;
			}
			catch (cv_bridge::Exception& e)
			{
				ROS_ERROR("cv_bridge exception: %s", e.what());
				return;
			}

			cv::Mat fgMaskM0G2;			// new foreground mask generated by M0G2 method

			// get the latest image from robot camera
			cv::Mat rgb_frame;
			rgb_frame = cv_ptr->image;
		
			// call MOG2 method and return foreground mask (i.e. the part of the image that changed from the previous image)					
			pMOG2(rgb_frame, fgMaskM0G2);
	
			// Update GUI Window
			//cv::imshow(OPENCV_WINDOW, threshold_frame);
			if (!fgMaskM0G2.empty())
			{
				cv::imshow(OPENCV_WINDOW, fgMaskM0G2);
				cv::waitKey(3);
			}

			//cv_out_ptr = cv_bridge::toImageMessage(fgMaskM0G2);
			//cv_out_ptr->image = fgMaskM0G2;

			// Output modified video stream
			//image_pub_.publish(cv_out_ptr->toImageMsg());
		}
	};

	int main(int argc, char** argv)
	{
		
		ros::init(argc, argv, "background_subtractor");
		NC_BackgroundSubtractor bs;
		ros::spin();

		return 0;
	}