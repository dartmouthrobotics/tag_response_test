#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <image_transport/image_transport.h>
#include <pluginlib/class_list_macros.h>
#include <ar_track_alvar_msgs/AlvarMarkers.h>
#include <memory>
#include <numeric>


namespace fake_video_stream_nodelet {

class MarkerSubscriberNodelet : public nodelet::Nodelet {

public:
    std::vector<float> delay_data;
    int number_frames_to_collect;
    int number_frames_collected;
    std::unique_ptr<ros::Subscriber> marker_subscriber_ptr;
    std::string ar_tag_topic;

    void marker_message_callback(const ar_track_alvar_msgs::AlvarMarkers& markers_message) {
        if (number_frames_collected < number_frames_to_collect) {
            auto collected_at = ros::Time::now();
            auto marker_stamp = markers_message.header.stamp;

            float delay = (collected_at - marker_stamp).toSec();
            std::cout << "Frame " << markers_message.header.seq << " delay " << delay << std::endl; 

            delay_data.push_back(delay);
            ++number_frames_collected;
        } else {
            auto average_delay = std::accumulate(delay_data.begin(), delay_data.end(), 0.0) / float(delay_data.size());
            std::cout << "Finished collecting. Average delay: " << average_delay << std::endl;
        }
    }

    void onInit() {
        number_frames_collected = 0;

        ros::NodeHandle& private_node_handle = getPrivateNodeHandle();
        private_node_handle.getParam("number_frames", number_frames_to_collect);
        private_node_handle.getParam("ar_tag_topic", ar_tag_topic);

        marker_subscriber_ptr = std::unique_ptr<ros::Subscriber>(
            new ros::Subscriber(private_node_handle.subscribe(
                ar_tag_topic,
                1,
                &MarkerSubscriberNodelet::marker_message_callback,
                this
            ))
        );
    }
}; // class MarkerSubscriberNodelet

} // namespace fake_video_stream_nodelet

PLUGINLIB_EXPORT_CLASS(fake_video_stream_nodelet::MarkerSubscriberNodelet, nodelet::Nodelet);
