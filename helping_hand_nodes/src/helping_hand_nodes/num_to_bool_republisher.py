import rospy

from std_msgs.msg import Bool, Float64


def num_2_bin_array(num):
    bin_array = [int(x) for x in list('{0:0b}'.format(int(num)))]
    bin_array.reverse()
    return bin_array


class NumberToBooleanRepublisher(object):

    def __init__(self, bit_loc, topic_from, topic_to):

        rospy.init_node("num2bool_rep")

        self.publisher = rospy.Publisher(topic_to, Bool, queue_size=10)

        rospy.Subscriber(topic_from, Float64, callback=self._sub_64_cb, queue_size=10)

        self.bit_loc = bit_loc

        rospy.spin()

    def _sub_64_cb(self, data):
        bin_array = 64*[0]
        num = data.data

        tmp_array = num_2_bin_array(num)
        bin_array[0:len(tmp_array)] = tmp_array

        self.publisher.publish(Bool(bin_array[self.bit_loc]))
