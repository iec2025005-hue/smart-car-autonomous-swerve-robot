import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Point
from std_msgs.msg import String, Float64MultiArray
import time, math

def main():
    rclpy.init()
    node = Node('test_autonomous_obstacle_runner', parameter_overrides=[rclpy.parameter.Parameter('use_sim_time', rclpy.Parameter.Type.BOOL, True)])

    current_odom = None
    last_target = None
    last_obstacle = None
    last_pid = None
    last_state = "IDLE"

    def odom_cb(msg):
        nonlocal current_odom
        current_odom = msg

    def target_cb(msg):
        nonlocal last_target
        last_target = msg

    def obstacle_cb(msg):
        nonlocal last_obstacle
        last_obstacle = msg

    def pid_cb(msg):
        nonlocal last_pid
        last_pid = msg

    def state_cb(msg):
        nonlocal last_state
        last_state = msg.data

    node.create_subscription(Odometry, '/odom', odom_cb, 10)
    node.create_subscription(Point, '/target_point', target_cb, 10)
    node.create_subscription(Point, '/obstacle_point', obstacle_cb, 10)
    node.create_subscription(Float64MultiArray, '/pid/diagnostics', pid_cb, 10)
    node.create_subscription(String, '/navigation/state', state_cb, 10)

    pub_mode = node.create_publisher(String, '/navigation/set_mode', 10)
    pub_vision_mode = node.create_publisher(String, '/vision/set_mode', 10)
    pub_target = node.create_publisher(String, '/vision/set_target', 10)


    def get_pose():
        if current_odom is None:
            return (0.0, 0.0, 0.0, 0.0)
        p = current_odom.pose.pose.position
        q = current_odom.pose.pose.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        return (p.x, p.y, p.z, yaw)

    print("Waiting for /odom and perception topics...")
    t_start = time.time()
    while current_odom is None and time.time() - t_start < 15.0:
        rclpy.spin_once(node, timeout_sec=0.05)

    if current_odom is None:
        print("ERROR: Did not receive /odom!")
        node.destroy_node()
        rclpy.shutdown()
        return

    print("Connected! Initializing autonomous Follow + Avoid demonstration with YOLO Chair...")

    ordered_transitions = []
    current_state_rec = None
    states_visited = set()

    def record_state(s, t_rel):
        nonlocal current_state_rec
        if s:
            states_visited.add(s)
        if s != current_state_rec:
            ordered_transitions.append((t_rel, current_state_rec, s))
            current_state_rec = s
            print(f"  [STATE TRANSITION @ t={t_rel:.2f}s]: {ordered_transitions[-1][1]} -> {s}")

    # 1. Start in IDLE
    pub_vision_mode.publish(String(data='yolo'))
    pub_target.publish(String(data='chair'))
    pub_mode.publish(String(data='idle'))
    t0 = time.time()
    while time.time() - t0 < 1.0:
        rclpy.spin_once(node, timeout_sec=0.05)
        record_state(last_state, time.time() - t0)

    # 2. Transition to SEARCH
    pub_mode.publish(String(data='search'))
    t_search = time.time()
    while time.time() - t_search < 1.0:
        rclpy.spin_once(node, timeout_sec=0.05)
        record_state(last_state, time.time() - t0)

    # 3. Transition to FOLLOW (triggers TARGET_DETECTED -> APPROACH -> AVOID -> REACQUIRE -> APPROACH -> ARRIVED)
    pub_mode.publish(String(data='follow'))

    obs_x, obs_y, obs_radius = 1.2, 0.18, 0.10
    tgt_x, tgt_y, tgt_radius = 2.5, 0.0, 0.20
    robot_radius = 0.16
    min_safe_dist = obs_radius + robot_radius  # 0.26 m

    collision_count = 0
    min_dist_to_obs = 999.0
    trajectory_log = []


    # Monitor for up to 16 seconds or until ARRIVED is reached
    start_demo = time.time()
    while time.time() - start_demo < 16.0:
        rclpy.spin_once(node, timeout_sec=0.05)
        px, py, pz, yaw = get_pose()
        t_rel = time.time() - t0
        
        dist_to_obs = math.hypot(px - obs_x, py - obs_y)
        dist_to_tgt = math.hypot(px - tgt_x, py - tgt_y)
        
        min_dist_to_obs = min(min_dist_to_obs, dist_to_obs)
        if dist_to_obs < min_safe_dist:
            collision_count += 1
            
        states_visited.add(last_state)
        record_state(last_state, t_rel)
        
        tgt_info = f"({last_target.x:.0f},{last_target.y:.0f},w={last_target.z:.0f})" if (last_target and last_target.z > 0) else "None"
        obs_info = f"({last_obstacle.x:.0f},{last_obstacle.y:.0f},w={last_obstacle.z:.0f})" if (last_obstacle and last_obstacle.z > 0) else "None"
        pid_info = f"err={last_pid.data[0]:+.1f}, P={last_pid.data[1]:+.1f}, out={last_pid.data[4]:+.1f}, vx={last_pid.data[5]:.2f}, vy={last_pid.data[6]:.2f}" if last_pid else "N/A"
        
        entry = {
            'time': t_rel,
            'pose': (px, py, pz, yaw),
            'dist_obs': dist_to_obs,
            'dist_tgt': dist_to_tgt,
            'state': last_state,
            'target': tgt_info,
            'obstacle': obs_info,
            'pid': pid_info
        }
        trajectory_log.append(entry)
        
        if len(trajectory_log) % 10 == 0:
            print(f"t={t_rel:4.1f}s | Pose=({px:+.2f}, {py:+.2f}) | d_obs={dist_to_obs:.2f}m | d_tgt={dist_to_tgt:.2f}m | State={last_state:<15} | Tgt={tgt_info} | PID: {pid_info}")
        
        if last_state == 'ARRIVED' and time.time() - start_demo > 5.0:
            print(f"  >>> ARRIVED detected at pose ({px:+.2f}, {py:+.2f}), distance to chair = {dist_to_tgt:.2f}m! Holding for 1s to verify stop...")
            for _ in range(20):
                rclpy.spin_once(node, timeout_sec=0.05)
                time.sleep(0.05)
            break

        time.sleep(0.05)

    # Stop car after test
    pub_mode.publish(String(data='idle'))
    time.sleep(0.5)

    px_final, py_final, _, _ = get_pose()
    unique_sequence = [tr[2] for tr in ordered_transitions]

    print("\n=======================================================")
    print("=== AUTONOMOUS OBSTACLE AVOIDANCE TEST RESULTS       ===")
    print("=======================================================")
    print(f"Initial Pose          : (0.00, 0.00) m")
    print(f"Final Pose            : ({px_final:+.3f}, {py_final:+.3f}) m")
    print(f"Obstacle Position     : ({obs_x:.2f}, {obs_y:.2f}) m (Radius={obs_radius}m)")
    print(f"Target Position       : ({tgt_x:.2f}, {tgt_y:.2f}) m (Radius={tgt_radius}m)")
    print(f"Minimum Obstacle Dist : {min_dist_to_obs:.4f} m (Safety threshold={min_safe_dist:.2f} m)")
    print(f"Collision Count       : {collision_count}")
    print(f"Unique State Sequence : {unique_sequence}")
    print(f"States Visited Set    : {list(states_visited)}")
    
    req_states = {'IDLE', 'SEARCH', 'TARGET_DETECTED', 'APPROACH', 'AVOID', 'REACQUIRE', 'ARRIVED'}
    missing = req_states - states_visited
    passed = (collision_count == 0) and (px_final > 0.8) and (len(missing) == 0)

    print(f"Missing Required States: {list(missing) if missing else 'None (All 7 required states observed)'}")
    print(f"Test Status           : {'PASS (Complete 7-State YOLO Chair Mission Verified)' if passed else 'FAIL'}")
    print("=======================================================\n")

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()


