#ifndef UDP_DATA_H
#define UDP_DATA_H

#include <string>
#include <mutex>
#include <atomic>
#include <thread>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <fcntl.h>
#include <unistd.h>
#include <iostream>
#include "aair.h"

class AAIRReceiver {
public:
    AAIRReceiver(const std::string& ip, int port, const std::string& send_ip, int send_port)
        : udp_ip(ip), udp_port(port), send_ip(send_ip), send_port(send_port), stop_flag(false), sockfd(-1) {}

    ~AAIRReceiver() {
        stop();
    }

    void start() {
        stop_flag = false;
        receive_thread = std::thread(&AAIRReceiver::udp_data_thread, this);
    }

    void stop() {
        stop_flag = true;
        if (receive_thread.joinable()) {
            receive_thread.join();
        }
        if (sockfd != -1) {
            close(sockfd);
        }
    }

    AAIR getCurAAIR() {
        std::lock_guard<std::mutex> lock(aair_mutex);
        return global_aair;
    }

    // 主线程调用，发送 GPS 数据
    void sendGpsData(float latitude, float longitude, float altitude) {
        // 构造 GPS 数据
        std::string gps_data = "GPS: " + std::to_string(latitude) + ", " 
            + std::to_string(longitude) + ", " + std::to_string(altitude);

        // 发送 GPS 数据
        ssize_t n = sendto(sockfd, gps_data.c_str(), gps_data.size(), 0, (const struct sockaddr*)&send_addr, sizeof(send_addr));
        if (n < 0) {
            std::cerr << "Error sending GPS data: " << strerror(errno) << std::endl;
        } else {
            std::cout << "Sent GPS data: " << gps_data << std::endl;
        }
    }

private:
    std::string udp_ip;      // 接收数据的 IP 地址
    int udp_port;            // 接收数据的端口
    std::string send_ip;     // 发送数据的目标 IP 地址
    int send_port;           // 发送数据的目标端口
    std::atomic<bool> stop_flag;
    std::thread receive_thread;
    int sockfd;
    struct sockaddr_in send_addr;
    AAIR global_aair;
    std::mutex aair_mutex;

    void udp_data_thread() {
        char buffer[sizeof(AAIR)];

        // 创建UDP套接字
        if ((sockfd = socket(AF_INET, SOCK_DGRAM, 0)) < 0) {
            std::cerr << "Socket creation failed\n";
            return;
        }

        // 设置套接字为非阻塞模式
        int flags = fcntl(sockfd, F_GETFL, 0);
        fcntl(sockfd, F_SETFL, flags | O_NONBLOCK);

        // 配置接收端地址
        struct sockaddr_in server_addr;
        memset(&server_addr, 0, sizeof(server_addr));
        server_addr.sin_family = AF_INET;
        server_addr.sin_port = htons(udp_port);
        server_addr.sin_addr.s_addr = inet_addr(udp_ip.c_str());

        // 绑定套接字
        if (bind(sockfd, (const struct sockaddr*)&server_addr, sizeof(server_addr)) < 0) {
            std::cerr << "Bind failed\n";
            close(sockfd);
            sockfd = -1;
            return;
        }

        // 配置发送端地址
        memset(&send_addr, 0, sizeof(send_addr));
        send_addr.sin_family = AF_INET;
        send_addr.sin_port = htons(send_port);
        send_addr.sin_addr.s_addr = inet_addr(send_ip.c_str());

        while (!stop_flag) {
            ssize_t n = recvfrom(sockfd, buffer, sizeof(buffer), 0, nullptr, nullptr);
            if (n < 0) {
                if (errno != EAGAIN && errno != EWOULDBLOCK) {
                    std::cerr << "Error receiving UDP packet: " << strerror(errno) << std::endl;
                    break;
                }
            } else if (n == sizeof(AAIR)) {
                AAIR received_aair;
                memcpy(&received_aair, buffer, sizeof(AAIR));

                {
                    std::lock_guard<std::mutex> lock(aair_mutex);
                    global_aair = received_aair;
                }

                std::cout << "Received AAIR: Lat=" << received_aair.lat
                          << ", Lng=" << received_aair.lng
                          << ", Height=" << received_aair.height
                          << ", Yaw=" << received_aair.yaw
                          << ", Pitch=" << received_aair.pitch
                          << ", Roll=" << received_aair.roll
                          << ", Angle=" << received_aair.angle
                          << std::endl;
            } else {
                std::cerr << "Invalid packet size: " << n << " bytes.\n";
            }

            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }

        std::cout << "UDP listener stopped." << std::endl;
    }
};

#endif // UDP_DATA_H
