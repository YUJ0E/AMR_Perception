#!/usr/bin/env python3
import rospy
import numpy as np
import cv2
import os
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2D
from std_msgs.msg import String
from builtins import print


class Bonus:
    def __init__(self, seq, render=True, matric_number="A0276853N"):
        file_path = os.path.dirname(
            os.path.dirname(os.path.dirname(
                os.path.dirname(os.path.dirname(os.path.realpath(__file__)))))) + '/Homework1_Perception/data/Task 1'
        ft = np.loadtxt(os.path.join(file_path, seq, 'firsttrack.txt'), delimiter=',').astype(int)
        self.gt = list(np.loadtxt(os.path.join(file_path, seq, 'groundtruth.txt'), delimiter=',').astype(int))
        self.x_f, self.y_f, self.w, self.h = ft[0], ft[1], ft[2], ft[3]
        self.ok = None
        self.bbox = None
        self.current_gt = None
        self.render = render
        self.matric_number = matric_number
        cv2.destroyAllWindows()
        rospy.Subscriber("/me5413/image_raw", Image, self.image_callback)
        self.tracked_pub = rospy.Publisher("/me5413/tracked", Detection2D, queue_size=10)
        self.ground_truth_pub = rospy.Publisher("/me5413/gt", Detection2D, queue_size=10)
        self.matric_number_pub = rospy.Publisher("/me5413/matric_number", String, queue_size=10)
        rospy.spin()

    def image_callback(self, msg):
        if msg.encoding == '8UC3':
            # Convert the ROS image message to a NumPy array
            dtype = np.dtype("uint8")  # 8-bit unsigned integer
            image_np = np.frombuffer(msg.data, dtype=dtype)
            image_np = image_np.reshape((msg.height, msg.width, 3))  # Reshape to (height, width, channels)
            image = np.copy(image_np)
            if self.ok is None:
                print("Initializing tracker")
                self.tracker = cv2.TrackerKCF_create()
                self.ok = self.tracker.init(image, (self.x_f, self.y_f, self.w, self.h))
                self.current_gt = self.gt.copy()
            self.ok, bbox = self.tracker.update(image)
            if self.ok:
                # publish tracked object detection with vision_msgs/Detection2D
                tracked = Detection2D()
                tracked.header.stamp = rospy.Time.now()
                tracked.header.frame_id = "object_tracking_tracked"
                tracked.bbox.size_x = bbox[2]
                tracked.bbox.size_y = bbox[3]
                tracked.bbox.center.x = bbox[0] + bbox[2] / 2
                tracked.bbox.center.y = bbox[1] + bbox[3] / 2
                tracked.bbox.center.theta = 0
                self.tracked_pub.publish(tracked)
                rospy.loginfo('size_x: {}, size_y: {}, center_x: {}, center_y: {}'.format(tracked.bbox.size_x,
                                                                                          tracked.bbox.size_y,
                                                                                          tracked.bbox.center.x,
                                                                                          tracked.bbox.center.y))

                # publish ground truth object detection with vision_msgs/Detection2D
                gt = Detection2D()
                gt.header.stamp = rospy.Time.now()
                gt.header.frame_id = "object_tracking_gt"
                current_gt = self.current_gt.pop(0)
                gt.bbox.size_x = current_gt[2]
                gt.bbox.size_y = current_gt[3]
                gt.bbox.center.x = current_gt[0] + current_gt[2] / 2
                gt.bbox.center.y = current_gt[1] + current_gt[3] / 2
                gt.bbox.center.theta = 0
                self.ground_truth_pub.publish(gt)

                self.matric_number_pub.publish(matric_number)

                # update image with tracked and gt and show result
                if self.render:
                    p1 = (int(bbox[0]), int(bbox[1]))
                    p2 = (int(bbox[0] + bbox[2]), int(bbox[1] + bbox[3]))
                    cv2.rectangle(image, p1, p2, (255, 0, 0), 2, 1)
                    cv2.rectangle(image, (current_gt[0], current_gt[1]),
                                  (current_gt[0] + current_gt[2], current_gt[1] + current_gt[3]), (0, 255, 0), 2, 1)
                    cv2.imshow("Tracking", image)
                    cv2.waitKey(1)

                if self.current_gt == []:
                    self.ok = None
                    self.current_gt = None
                    if self.render:
                        cv2.destroyAllWindows()

        else:
            rospy.loginfo("Unsupported image encoding: {}".format(msg.encoding))


if __name__ == '__main__':
    rospy.init_node('Bonus_task', anonymous=True)
    matric_number = 'A0276853N'
    render = False
    BT = Bonus(seq='seq_1', render=render, matric_number=matric_number)
