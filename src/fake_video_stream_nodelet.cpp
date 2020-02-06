#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <thread>
#include <sensor_msgs/CameraInfo.h>
#include <pluginlib/class_list_macros.h>

namespace fake_video_stream_nodelet {

class FakeVideoStreamNodelet : public nodelet::Nodelet {
public:
    image_transport::Publisher image_publisher;
    std::unique_ptr<image_transport::ImageTransport> img_transport;
    std::unique_ptr<ros::Rate> publish_rate;
    cv::Mat image_to_publish;
    ros::Publisher camera_info_publisher;

    std::vector<double> distortion_coeffs;
    std::vector<double> camera_matrix;
    std::vector<double> projection_matrix;
    std::vector<double> rectification_matrix;

    int current_frame_number;
    int image_width;
    int image_height;

    bool keep_publishing;

    std::shared_ptr<std::thread> video_publisher_thread;
    std::shared_ptr<std::thread> cam_info_publisher_thread;

    void load_array_param(std::string param_name, int param_index, std::vector<double>& param_destination) {
        ROS_INFO_STREAM("Loading camera parameter " << param_name << " from parameter list index " << param_index);
        ros::NodeHandle& node_handle = getPrivateNodeHandle();

        XmlRpc::XmlRpcValue param_value;
        bool value_exists = node_handle.getParam(param_name, param_value);

        if (!value_exists) {
            throw std::runtime_error("Coult not load list of lists param value!");
        }

        for (int i = 0; i < param_value[param_index].size(); ++i) {
            param_destination.push_back(param_value[param_index][i]);
        }
    }

    void onInit() {
        keep_publishing = true;
        current_frame_number = 0;
        ros::NodeHandle& private_node_handle = getPrivateNodeHandle();
        img_transport = std::unique_ptr<image_transport::ImageTransport>(new image_transport::ImageTransport(private_node_handle));

        image_publisher = img_transport->advertise("fake_video_stream/image_raw", 5);

        std::string image_path;
        private_node_handle.getParam("fake_video_image_path", image_path);
        ROS_INFO_STREAM("Loading fake video stream image from path " << image_path);

        image_to_publish = cv::imread(image_path, CV_LOAD_IMAGE_GRAYSCALE);

        int rate;
        private_node_handle.getParam("publish_rate", rate);

        publish_rate = std::unique_ptr<ros::Rate>(new ros::Rate(rate));

        private_node_handle.getParam("image_width", image_width);
        private_node_handle.getParam("image_height", image_height);

        load_array_param("distortion_coeffs", 0, distortion_coeffs);
        load_array_param("intrinsic_coeffs", 0, camera_matrix);
        load_array_param("rectification_coeffs", 0, rectification_matrix);
        load_array_param("projection_coeffs", 0, projection_matrix);

        ROS_INFO_STREAM("Got distortion coeffs: ");
        for (auto coeff : distortion_coeffs) {
            ROS_INFO_STREAM("    " << coeff);
        }

        ROS_INFO_STREAM("Got camera matrix: ");
        for (auto coeff : camera_matrix) {
            ROS_INFO_STREAM("    " << coeff);
        }

        camera_info_publisher = private_node_handle.advertise<sensor_msgs::CameraInfo>("fake_video_stream/camera_info", 5);

        video_publisher_thread = std::shared_ptr<std::thread>(new std::thread(std::bind(&FakeVideoStreamNodelet::image_publish_loop, this)));
        cam_info_publisher_thread = std::shared_ptr<std::thread>(new std::thread(std::bind(&FakeVideoStreamNodelet::cam_info_publish_loop, this)));
    }

    void cam_info_publish_loop() {
        while (keep_publishing) {
            sensor_msgs::CameraInfo info_msg;
            info_msg.header.stamp = ros::Time::now();
            info_msg.width = image_width;
            info_msg.height = image_height;

            info_msg.distortion_model = "plumb_bob";
            info_msg.D = distortion_coeffs;

            for (int i = 0; i < camera_matrix.size(); ++i) {
                info_msg.K[i] = camera_matrix[i];
            }

            for (int i = 0; i < rectification_matrix.size(); ++i) {
                info_msg.R[i] = rectification_matrix[i];
            }

            for (int i = 0; i < projection_matrix.size(); ++i) {
                info_msg.P[i] = projection_matrix[i];
            }

            camera_info_publisher.publish(info_msg);

            publish_rate->sleep();
        }
    }

    void image_publish_loop() {
        while (keep_publishing) {
            sensor_msgs::ImagePtr msg = cv_bridge::CvImage(
                std_msgs::Header(),
                "mono8",
                image_to_publish
            ).toImageMsg();

            msg->header.stamp = ros::Time::now();
            msg->header.seq = current_frame_number;
            msg->header.frame_id = "cam_0_optical_frame";

            image_publisher.publish(msg);

            ++current_frame_number;
            publish_rate->sleep();
        }
    }

    ~FakeVideoStreamNodelet() {
        keep_publishing = false;
        video_publisher_thread->join();
        cam_info_publisher_thread->join();
    }
};

PLUGINLIB_EXPORT_CLASS(fake_video_stream_nodelet::FakeVideoStreamNodelet, nodelet::Nodelet);
} // namespace fake_video_stream_nodelet
