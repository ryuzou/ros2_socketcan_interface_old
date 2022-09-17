//
// Created by ryuzo on 2022/08/08.
//

#ifndef ROS2_MASTER_SOCKETCAN_TX_NODE_HPP
#define ROS2_MASTER_SOCKETCAN_TX_NODE_HPP

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <vector>
#include <net/if.h>
#include <sys/socket.h>

#include <linux/can.h>

#include "socketcan_interface_msg/msg/socketcan_if.hpp"
#include "visibility.h"

using LNI = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface;

namespace socketcan_interface {

    class SocketcanInterface final : public rclcpp_lifecycle::LifecycleNode {
    private:
        rclcpp::Subscription<socketcan_interface_msg::msg::SocketcanIF>::SharedPtr _subscription;
        rclcpp::TimerBase::SharedPtr _pub_timer;

        int64_t interval_ms;

        std::string can_interface_name;
        std::map<uint16_t, rclcpp::Publisher<socketcan_interface_msg::msg::SocketcanIF>::SharedPtr> known_id_rx_publisher;
        struct sockaddr_can addr{};
        struct ifreq ifr{};
        int s;

        rclcpp::QoS _qos = rclcpp::QoS(40).keep_all();

        void _publisher_callback();
        void _subscriber_callback(socketcan_interface_msg::msg::SocketcanIF msg);
    public:
        ROS2_SOCKETCAN_INTERFACE_PUBLIC
        explicit SocketcanInterface(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

        LNI::CallbackReturn on_configure(const rclcpp_lifecycle::State & state) override;

        LNI::CallbackReturn on_activate(const rclcpp_lifecycle::State & state) override;

        LNI::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state) override;

        LNI::CallbackReturn on_cleanup(const rclcpp_lifecycle::State & state) override;

        LNI::CallbackReturn on_shutdown(const rclcpp_lifecycle::State & state) override;
    };
}

#endif //ROS2_MASTER_SOCKETCAN_TX_NODE_HPP
