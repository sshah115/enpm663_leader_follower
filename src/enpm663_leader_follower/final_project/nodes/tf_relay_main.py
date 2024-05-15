#!/usr/bin/env python3

import rclpy
from rclpy.executors import SingleThreadedExecutor
from final_project.tf_relay_interface import TFRelay, TFStaticRelay

def main():
    rclpy.init()
    executor = SingleThreadedExecutor()
    leader_namespace = "leader"
    follower_namespace = "follower"

    tf_relays = []
    tf_static_relays = []

    try:
        tf_relay_leader = TFRelay(namespace=leader_namespace)
        tf_relays.append(tf_relay_leader)
        
        tf_static_relay_leader = TFStaticRelay(namespace=leader_namespace)
        tf_static_relays.append(tf_static_relay_leader)
        
        tf_relay_follower = TFRelay(namespace=follower_namespace)
        tf_relays.append(tf_relay_follower)
        
        tf_static_relay_follower = TFStaticRelay(namespace=follower_namespace)
        tf_static_relays.append(tf_static_relay_follower)
        

        for tf_relay in tf_relays:
            executor.add_node(tf_relay)

        for tf_static_relay in tf_static_relays:
            executor.add_node(tf_static_relay)

        try:
            executor.spin()
        finally:
            executor.shutdown()

    except KeyboardInterrupt:
        executor.shutdown()
        
if __name__ == "__main__":
    main()