//
// Created by ryuzo on 2022/08/08.
//
#include <rclcpp/rclcpp.hpp>
#include <lifecycle_msgs/msg/state.hpp>
#include <net/if.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <cerrno>
#include <cstring>
#include <fcntl.h>
#include <cstdio>

#include "socketcan_interface_msg/msg/socketcan_if.hpp"
#include "ros2_socketcan_interface/socketcan_node.hpp"

namespace socketcan_interface {

    SocketcanInterface::SocketcanInterface(const rclcpp::NodeOptions &options)
            : rclcpp_lifecycle::LifecycleNode("socket_can_node", options) {
        using namespace std::chrono_literals;

        declare_parameter("interval_ms", 1);
        can_interface_name = this->declare_parameter<std::string>("can_if", "vcan0");
        interval_ms = this->get_parameter("interval_ms").as_int();
    }

    void SocketcanInterface::_publisher_callback() {
        if (this->get_current_state().id() != lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE){
            return;
        }

        auto msg = std::make_shared<socketcan_interface_msg::msg::SocketcanIF>();

        int nbytes;
        struct canfd_frame frame{};
        errno = 0;

        while (true) {
            nbytes = read(s, &frame, sizeof(struct canfd_frame));
            if (nbytes < 0) {
                if (errno !=
                    (EAGAIN | EWOULDBLOCK)) {   // these errors occur when nothing can be read in non-blocking mode, so ignore.
                    RCLCPP_ERROR(this->get_logger(), "Unexpected read error.");
                }
                break;
            }
            RCLCPP_INFO(this->get_logger(), "Published ID:0x%03X [%d] ", frame.can_id, frame.len);
            msg->canid = frame.can_id;
            msg->candlc = frame.len;
            for (int i = 0; i < frame.len; i++) {
                msg->candata[i] = frame.data[i];
            }

            if (known_id_rx_publisher.find(frame.can_id) == known_id_rx_publisher.end()) {
                known_id_rx_publisher[frame.can_id] = this->create_publisher<socketcan_interface_msg::msg::SocketcanIF>(
                        std::string("/can_rx_" + std::to_string(frame.can_id)), _qos);
            }
            known_id_rx_publisher[frame.can_id]->publish(*msg);
        }
    }

    void SocketcanInterface::_subscriber_callback(const socketcan_interface_msg::msg::SocketcanIF msg) {
        if (this->get_current_state().id() != lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE){
            return;
        }

        struct canfd_frame frame{};

        frame.can_id = msg.canid;
        if ((msg.candlc > 0) and (msg.candlc <= 64)) {
            frame.len = msg.candlc;
        } else {
            frame.len = 64;
        }
        std::string can_data_print;
        for (int i = 0; i < frame.len; ++i) {
            frame.data[i] = msg.candata[i];

            char str[64];
            sprintf(str, "0x%03X, ", msg.candata[i]);
            can_data_print = can_data_print + str;
        }
        //RCLCPP_INFO(this->get_logger(), "Sending to can bus ID: 0x%03X, can data: %s", msg.canid, can_data_print.c_str());

        if (write(s, &frame, sizeof(struct canfd_frame)) != sizeof(struct canfd_frame)) {
            perror("Write frame0");
            RCLCPP_ERROR(this->get_logger(), "Write error");
        }
    }

    LNI::CallbackReturn SocketcanInterface::on_configure(const rclcpp_lifecycle::State &state) {
        RCLCPP_INFO(this->get_logger(), "on config.");

        _pub_timer = this->create_wall_timer(
                std::chrono::milliseconds(interval_ms),
                [this] { _publisher_callback(); }
        );
        _subscription = this->create_subscription<socketcan_interface_msg::msg::SocketcanIF>(
                "/can_tx",
                _qos,
                std::bind(&SocketcanInterface::_subscriber_callback, this, std::placeholders::_1)
        );

        return LNI::CallbackReturn::SUCCESS;
    }

    LNI::CallbackReturn SocketcanInterface::on_activate(const rclcpp_lifecycle::State &state) {
        RCLCPP_INFO(this->get_logger(), "on activate.");

        if ((s = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0) {
            RCLCPP_ERROR(this->get_logger(), "Socket error");
            return LNI::CallbackReturn::ERROR;
        }

        strcpy(ifr.ifr_name, can_interface_name.c_str());
        ioctl(s, SIOCGIFINDEX, &ifr);

        memset(&addr, 0, sizeof(addr));
        addr.can_family = AF_CAN;
        addr.can_ifindex = ifr.ifr_ifindex;

        if (ioctl(s, SIOCGIFMTU, &ifr) < 0) {
            RCLCPP_ERROR(this->get_logger(), "SIOCGIFMTU");
            return LNI::CallbackReturn::ERROR;
        }
        int mtu = ifr.ifr_mtu;

        if (mtu != CANFD_MTU) {
            RCLCPP_ERROR(this->get_logger(), "CAN interface is not CAN FD capable");
            return LNI::CallbackReturn::ERROR;
        }

        int enable_canfd = 1;
        /* interface is ok - try to switch the socket into CAN FD mode */
        setsockopt(s, SOL_CAN_RAW, CAN_RAW_FD_FRAMES, &enable_canfd, sizeof(enable_canfd));

        if (bind(s, (struct sockaddr *) &addr, sizeof(addr)) < 0) {
            RCLCPP_ERROR(this->get_logger(), "Error at Binding socket: %s", strerror(errno));
            return LNI::CallbackReturn::ERROR;
        }

        if (fcntl(s, F_SETFL, O_NONBLOCK) < 0) {    //Set can socket to non-blocking mode.
            RCLCPP_ERROR(this->get_logger(), "Error at Setting CAN: %s", strerror(errno));
            return LNI::CallbackReturn::ERROR;
        }
        return LNI::CallbackReturn::SUCCESS;
    }

    LNI::CallbackReturn SocketcanInterface::on_deactivate(const rclcpp_lifecycle::State &state) {
        RCLCPP_INFO(this->get_logger(), "on deactivate.");
        if (close(s) < 0) {
            RCLCPP_ERROR(this->get_logger(), "Error at Closing Socket: %s", strerror(errno));
            return LNI::CallbackReturn::ERROR;
        }
        return LNI::CallbackReturn::SUCCESS;
    }

    LNI::CallbackReturn SocketcanInterface::on_cleanup(const rclcpp_lifecycle::State &state) {
        RCLCPP_INFO(this->get_logger(), "on cleanup.");
        _pub_timer->reset();
        _subscription.reset();
        for (const auto& [key, value] : known_id_rx_publisher){
            known_id_rx_publisher[key].reset();
        }
        return LNI::CallbackReturn::SUCCESS;
    }

    LNI::CallbackReturn SocketcanInterface::on_shutdown(const rclcpp_lifecycle::State &state) {
        RCLCPP_INFO(this->get_logger(), "on shutdown.");
        _pub_timer->reset();
        _subscription.reset();
        for (const auto& [key, value] : known_id_rx_publisher){
            known_id_rx_publisher[key].reset();
        }
        return LNI::CallbackReturn::SUCCESS;
    }
}
#include <rclcpp_components/register_node_macro.hpp>

RCLCPP_COMPONENTS_REGISTER_NODE(socketcan_interface::SocketcanInterface)