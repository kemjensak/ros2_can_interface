#include "ros2_can_interface/can_interface.hpp"
#define CAN_FRAME_MAX_LEN 8 

using std::placeholders::_1;
class CanInterface : public rclcpp::Node

{
  public:
    CanInterface();
    ~CanInterface();
    
    int MainLoop();
    void receiveThread();

  protected:
    void command_callback(const can_msgs::msg::Frame & msg);
    int InitCanInterface(const char *ifname);
    int TransmitCanFrame(const int &sock, const uint32_t &id, const uint8_t *data, const size_t data_len);
    int ReceiveCanFrame(const int sock);
    
    rclcpp::Subscription<can_msgs::msg::Frame>::SharedPtr subscription_;
    rclcpp::Publisher<can_msgs::msg::Frame>::SharedPtr publisher_;
    uint8_t can_data_[CAN_FRAME_MAX_LEN] = { 0xa2, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
    uint32_t can_id_;
    int sock_;

};

CanInterface::CanInterface()
:Node("CanConnector"), can_id_(0x001)
{
  subscription_ = this->create_subscription<can_msgs::msg::Frame>(
    "can_command", 10, std::bind(&CanInterface::command_callback, this, _1));
  publisher_ = this->create_publisher<can_msgs::msg::Frame>("can_feedback", 10);

  sock_ = InitCanInterface("can0");
  if (sock_ < 0) {
    RCLCPP_ERROR(this->get_logger(), "CAN conection failed");
  }

  struct timeval tv;
  tv.tv_sec = 0;
  tv.tv_usec = 100;
  setsockopt(sock_, SOL_SOCKET, SO_RCVTIMEO, (const char*)&tv, sizeof tv);

}

CanInterface::~CanInterface()
{
  
}

int CanInterface::InitCanInterface(const char *ifname)
{
  int sock = socket(PF_CAN, SOCK_RAW, CAN_RAW);
  if (sock == -1) {
  RCLCPP_ERROR(this->get_logger(), "Fail to create can socket for %s - %m\n", ifname);
  // printf("Fail to create can socket for %s - %m\n", ifname);
    return -1;
  }
  RCLCPP_INFO(this->get_logger(), "Success to create can socket for %s\n", ifname);
  // printf("Success to create can socket for %s\n", ifname);

  struct ifreq ifr;
  strcpy(ifr.ifr_name, ifname);
  if (ioctl(sock, SIOCGIFINDEX, &ifr) == -1) {
    perror("Fail to get can interface index -");
    return -1;
  }
  printf("Success to get can interface index: %d\n", ifr.ifr_ifindex);

  struct sockaddr_can addr;
  addr.can_family = AF_CAN;
  addr.can_ifindex = ifr.ifr_ifindex;
  if (bind(sock, (struct sockaddr *)&addr, sizeof(addr)) == -1) {
    perror("Fail to bind can socket -");
    return -1;
  }
  RCLCPP_INFO(this->get_logger(), "Success to bind socket for \n");

  return sock;
}


void CanInterface::command_callback(const can_msgs::msg::Frame & msg)
{
  uint32_t can_id_ = msg.id;
  can_data_[0] = (msg.data[0]);
  can_data_[1] = (msg.data[1]);
  can_data_[2] = (msg.data[2]);
  can_data_[3] = (msg.data[3]);
  can_data_[4] = (msg.data[4]);
  can_data_[5] = (msg.data[5]);
  can_data_[6] = (msg.data[6]);
  can_data_[7] = (msg.data[7]);

  this->TransmitCanFrame(sock_, can_id_, can_data_, sizeof(can_data_));
}


int CanInterface::TransmitCanFrame(const int &sock, const uint32_t &id, const uint8_t *data, const size_t data_len)
{
  struct can_frame frame;
  frame.can_id = id & 0x1ff;
  frame.can_id |= (1 << 13);
  memcpy(frame.data, data, data_len);
  frame.can_dlc = data_len;

  int tx_bytes = write(sock, &frame, sizeof(frame));
  if (tx_bytes == -1) {
    perror("Fail to transmit can frame -");
    RCLCPP_ERROR(this->get_logger(), "CAN send failed");
    return -1;
  } 
  // printf("Success to transmit can frame - %d bytes is transmitted\n", tx_bytes);
  // RCLCPP_INFO(this->get_logger(), "Sent CAN frame");
  return 0;
}

int CanInterface::ReceiveCanFrame(const int sock)
{
  struct can_frame frame;
  int rx_bytes = read(sock, &frame, sizeof(frame));
  if (rx_bytes < 0) {
    // RCLCPP_ERROR(this->get_logger(), "Fail to receive can frame");
    // perror("Fail to receive can frame - ");
    return -1;
  } else if (rx_bytes < (int)sizeof(struct can_frame)) {
    printf("Incomplete can frame is received - rx_bytes: %d\n", rx_bytes);
    return -1;
  } else if (frame.can_dlc > CAN_FRAME_MAX_LEN) {
    printf("Invalid dlc: %u\n", frame.can_dlc);
    return -1;
  }

  if (((frame.can_id >> 29) & 1) == 1) {
    printf("Error frame is received\n");
  } else if (((frame.can_id >> 30) & 1) == 1) {
    printf("RTR frame is received\n");
  } else {
    if (((frame.can_id >> 31) & 1) == 1) {
      // printf("11bit long std can frame is received\n");
    } else {
      // printf("29bit long ext can frame is received\n");
      // std::cout << frame.can_id << std::endl;
    }
  }
  auto feedback = can_msgs::msg::Frame();
  feedback.id = frame.can_id ;
  for(uint8_t i = 0; i<=7; i++){
    feedback.data[i] = frame.data[i];
  }

  publisher_->publish(feedback);
  return 0;
}

void CanInterface::receiveThread()
  {
      while (rclcpp::ok())
      {
          ReceiveCanFrame(sock_);
      }
  }


int CanInterface::MainLoop()
{
    // ReceiveCanFrame(sock_);
    rclcpp::spin_some(shared_from_this());
  return 0;
}

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  auto CAN = std::make_shared<CanInterface>();
  std::thread receive_thread(&CanInterface::receiveThread, CAN);
  while(rclcpp::ok()){
    CAN->MainLoop();
  }
  receive_thread.join();
  rclcpp::shutdown();
  return 0;
}