/**
 * Payload Data Structure from locqt Server (bytes)
 * 0-3      ->      frame_number
 * 4        ->      frame_ID
 * 5-12     ->      translation.x
 * 13-20    ->      translation.y   
 * 21-28    ->      translation.z
 * 29-36    ->      rotation.quaternion.x
 * 37-44    ->      rotation.quaternion.y
 * 45-52    ->      rotation.quaternion.z
 * 53-60    ->      rotation.quaternion.scalar
 */

#include "external_position_comm/communicator.hpp"

Communicator::Communicator() : Node("external_position_comm")
{
    // 33Hz
    this->timer_ = this->create_wall_timer(std::chrono::milliseconds(30), std::bind(&Communicator::timer_callback, this));

    // Publishers
    this->odometry_pub_ = this->create_publisher<px4_msgs::msg::VehicleOdometry>("/fmu/in/uwb_position", 10);
}

void Communicator::timer_callback()
{
    // Signal Handler
    signal(SIGINT, [](int sig_num){exit(sig_num);});

    ssize_t rx_length = recvfrom(sock, (char *) buffer, sizeof(buffer), MSG_WAITALL, (struct sockaddr *) &server, &socket_length);
    buffer[rx_length] = '\0';

    // Unpack from locqt Server
    memcpy(&frame_number, buffer+0, 4);
    memcpy(&frame_ID, buffer+4, 1);

    memcpy(&position.translation[0], buffer+5, 8);
    memcpy(&position.translation[1], buffer+13, 8);
    memcpy(&position.translation[2], buffer+21, 8);

    memcpy(&position.rotation_q[0], buffer+29, 8);
    memcpy(&position.rotation_q[1], buffer+37, 8);
    memcpy(&position.rotation_q[2], buffer+45, 8);
    memcpy(&position.rotation_q[3], buffer+53, 8);

    // ROS2 publishing
    px4_msgs::msg::VehicleOdometry msg{};
    msg.timestamp = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::system_clock::now().time_since_epoch()).count();

    msg.pose_frame = px4_msgs::msg::VehicleOdometry::POSE_FRAME_FRD;
    msg.position = {
        static_cast<float>(position.translation[0]),
        static_cast<float>(-position.translation[1]),
        static_cast<float>(-position.translation[2]) 
    };
    msg.q = {
        static_cast<float>(position.rotation_q[3]),
        static_cast<float>(position.rotation_q[0]),
        static_cast<float>(-position.rotation_q[1]),
        static_cast<float>(-position.rotation_q[2])
    };
    msg.position_variance = {0.001, 0.001, 0.001};

    odometry_pub_->publish(msg);
}

void Communicator::connect()
{
    // Socket init
    if( (sock = socket(AF_INET, SOCK_DGRAM, 0)) < 0) 
    {
        perror("Socket creation failed"); 
        exit(EXIT_FAILURE); 
    }
    else
    {
        std::cout << "Socket created" << std::endl;
    }

    // Zero-out server address structure
    std::memset(&server, 0, sizeof(server));

    server.sin_addr.s_addr = INADDR_ANY; //inet_addr("192.168.50.56");;
    server.sin_family = AF_INET;
    server.sin_port = htons(PORT);
    socket_length = sizeof(server);

    // Bind the socket to any valid IP address and a specific port
    if( bind(sock, (const struct sockaddr *)&server, sizeof(server)) < 0 ) 
    { 
        perror("Bind failed"); 
        exit(EXIT_FAILURE); 
    }
    else
    {
        std::cout << "Bind completed" << std::endl;
    }
}

void Communicator::disconnect()
{
    close(sock);
}

int main(int argc, char *  argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Communicator>();

    node->connect();
    rclcpp::spin(node);
    node->disconnect();

    rclcpp::shutdown();
    return 0;
}