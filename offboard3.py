import rospy
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode

current_state = State()

def state_cb(msg):
    global current_state
    current_state = msg

rospy.init_node("offboard_takeoff")

state_sub = rospy.Subscriber("mavros/state", State, state_cb)

local_pos_pub = rospy.Publisher(
    "mavros/setpoint_position/local",
    PoseStamped,
    queue_size=10
)

rospy.wait_for_service("/mavros/cmd/arming")
arming_client = rospy.ServiceProxy("mavros/cmd/arming", CommandBool)

rospy.wait_for_service("/mavros/set_mode")
set_mode_client = rospy.ServiceProxy("/mavros/set_mode", SetMode)

rate = rospy.Rate(20)

while not rospy.is_shutdown() and not current_state.connected:
    rate.sleep()

pose = PoseStamped()
pose.pose.position.x = 0
pose.pose.position.y = 0
pose.pose.position.z = 1

# 先发送100帧setpoint
for i in range(100):
    local_pos_pub.publish(pose)
    rate.sleep()

set_mode_client(base_mode=0, custom_mode="OFFBOARD")
arming_client(True)

rospy.loginfo("OFFBOARD enabled and armed")

# 再发送50帧保证稳定
for i in range(50):
    local_pos_pub.publish(pose)
    rate.sleep()

rospy.loginfo("Takeoff trigger finished")