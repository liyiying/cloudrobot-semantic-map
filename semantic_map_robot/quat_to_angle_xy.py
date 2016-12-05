import rospy
from geometry_msgs.msg import Twist, Point, Quaternion
import tf
from rbx1_nav.transform_utils import quat_to_angle, normalize_angle
from math import radians, copysign, sqrt, pow, pi
from std_msgs.msg import Float64


class QuatToAngle():
    def __init__(self):
        # Give the node a name
        rospy.init_node('quat_to_angle', anonymous=False)

        # Publisher to control the robot's speed
        self.turtlebot_angle = rospy.Publisher('/turtlebot_angle', Float64, queue_size=5)
        self.turtlebot_posex = rospy.Publisher('/turtlebot_posex', Float64, queue_size=5)
        self.turtlebot_posey = rospy.Publisher('/turtlebot_posey', Float64, queue_size=5)

        #goal.target_pose.pose = Pose(Point(float(data["point"]["x"]), float(data["point"]["y"]), float(data["point"]["z"])), Quaternion(float(data["quat"]["x"]), float(data["quat"]["y"]), float(data["quat"]["z"]), float(data["quat"]["w"])))
        
        # How fast will we update the robot's movement?
        rate = 20
        
        # Set the equivalent ROS rate variable
        r = rospy.Rate(rate)

        # Initialize the tf listener
        self.tf_listener = tf.TransformListener()
        
        # Give tf some time to fill its buffer
        rospy.sleep(2)
        
        # Set the odom frame
        self.odom_frame = '/odom'
        
        # Find out if the robot uses /base_link or /base_footprint
        try:
            self.tf_listener.waitForTransform(self.odom_frame, '/base_footprint', rospy.Time(), rospy.Duration(1.0))
            self.base_frame = '/base_footprint'
        except (tf.Exception, tf.ConnectivityException, tf.LookupException):
            try:
                self.tf_listener.waitForTransform(self.odom_frame, '/base_link', rospy.Time(), rospy.Duration(1.0))
                self.base_frame = '/base_link'
            except (tf.Exception, tf.ConnectivityException, tf.LookupException):
                rospy.loginfo("Cannot find transform between /odom and /base_link or /base_footprint")
                rospy.signal_shutdown("tf Exception")  
        
        # Initialize the position variable as a Point type
        position = Point()
        while not rospy.is_shutdown():
            (position, rotation) = self.get_odom()
            print(position)
            self.turtlebot_angle.publish(rotation)
            #print(str(position).split('x: ')[1].split('\ny:')[0])
            x = float(str(position).split('x: ')[1].split('\ny:')[0])
            y = float(str(position).split('y: ')[1].split('\nz:')[0])
            self.turtlebot_posex.publish(x)
            self.turtlebot_posey.publish(y)
            #print(rotation)
            rospy.sleep(5)

    def get_odom(self):
        # Get the current transform between the odom and base frames
        try:
            (trans, rot)  = self.tf_listener.lookupTransform(self.odom_frame, self.base_frame, rospy.Time(0))
        except (tf.Exception, tf.ConnectivityException, tf.LookupException):
            rospy.loginfo("TF Exception")
            return
        return (Point(*trans), quat_to_angle(Quaternion(*rot)))

if __name__ == '__main__':
    try:
        QuatToAngle()
    except Exception as e:
        print(e)
        rospy.loginfo("Out-and-Back node terminated.")


