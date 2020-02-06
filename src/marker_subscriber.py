#! /usr/bin/python2

import rospy
import ar_track_alvar_msgs.msg
import collections

CsvRow = collections.namedtuple("CsvRow", ["frame_number", "received_time", "sent_time", "delay_seconds", "number_markers"])

collected_data = []
output_file = ""
number_frames = 0

number_frames_collected = 0

def marker_callback(marker_message):
    global collected_data, number_frames_collected, number_frames

    if number_frames_collected > number_frames:
        print "Got all frames. Exiting."
        rospy.signal_shutdown(5)

    received_time = rospy.Time.now()
    delay = (received_time - marker_message.header.stamp).to_sec()
    print "Frame {marker_seq} at {received_time} delay: {delay}".format(marker_seq=marker_message.header.seq, received_time=received_time, delay=delay)

    data_row = CsvRow(
        frame_number=marker_message.header.seq,
        received_time=received_time,
        delay_seconds=delay,
        sent_time=marker_message.header.stamp,
        number_markers=len(marker_message.markers)
    )
    collected_data.append(data_row)

    number_frames_collected += 1


def shutdown_handler():
    print "Average delay: {}".format(
        sum([row.delay_seconds for row in collected_data]) / float(len(collected_data))
    )

    with open(output_file, "w") as output_stream:
        output_stream.write(",".join(CsvRow._fields) + "\n")

        for row in collected_data:
            output_stream.write(",".join(map(str, [getattr(row, field) for field in CsvRow._fields])) + "\n")


def main():
    global output_file, collected_data, number_frames

    rospy.init_node("publisher_node")
    rospy.loginfo("Starting data logger.")
    rospy.on_shutdown(shutdown_handler)

    output_file = rospy.get_param("~output_file")

    marker_topic = rospy.get_param("~ar_tag_topic")

    number_frames = rospy.get_param("~number_frames")
    rospy.Subscriber(marker_topic, ar_track_alvar_msgs.msg.AlvarMarkers, marker_callback)

    rospy.spin()

if __name__ == "__main__":
    main()
