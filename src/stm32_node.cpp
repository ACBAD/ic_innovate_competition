#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <unistd.h>
#include <termios.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <csignal>
#include <poll.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/Int32.h>

#define SERIAL_PORT "/dev/ttyS3"
#define MAX_LINEAR_SPEED 10.0
#define MAX_ANGULAR_SPEED 2.0
#define READ_STR_LENGTH 500

geometry_msgs::Twist global_vel_msg;
uint32_t rwheel_ticks = 0;
uint32_t lwheel_ticks = 0;
uint8_t cover_cmd = 0;
bool cover_state = false;
uint8_t clogging_state = 'N';

void updateVel(const geometry_msgs::Twist& msg) {global_vel_msg = msg;}

class SerialDevice {
  int serial_port = -1;
  std::vector<uint8_t> buffer;
  std::vector<uint8_t> recv_buffer;
public:
  SerialDevice() {
    serial_port = open(SERIAL_PORT, O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (serial_port < 0) {
      ROS_ERROR("Failed to open serial port");
      return;
    }
    termios tty{};
    if (tcgetattr(serial_port, &tty) != 0) {
      ROS_ERROR("Failed to set serial port: tcgetattr failed");
      close(serial_port);
    }
    cfsetispeed(&tty, B115200);
    cfsetospeed(&tty, B115200);
    tty.c_iflag &= ~(BRKINT | IMAXBEL);
    tty.c_cflag |= (CLOCAL | CREAD);
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;
    tty.c_cflag &= ~PARENB;
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CRTSCTS;
    tty.c_lflag &= ~(ECHO | ECHOE | ECHOK | ECHOCTL | ECHOKE | ICANON);
    tty.c_cc[VMIN] = 1;
    tty.c_cc[VTIME] = 0;
    if (tcsetattr(serial_port, TCSANOW, &tty) != 0) {
      ROS_ERROR("Failed to set serial port: tcsetattr failed");
      close(serial_port);
    }
    ROS_WARN("Serial Open");
    int size = READ_STR_LENGTH;
    ioctl(serial_port, TIOCSWINSZ, &size);
    ROS_WARN("Set buffer size: %d", size);
    buffer.resize(READ_STR_LENGTH);
  }
  ~SerialDevice() {
    if (serial_port >= 0)
      close(serial_port);
  }
  const std::vector<uint8_t>* tread(const int timeout_ms) {
    if (serial_port < 0) {
      ROS_WARN("Read failed: serial not open");
      return nullptr;
    }
    pollfd fds{};
    fds.fd = serial_port;
    fds.events = POLLIN;
    const int ret = poll(&fds, 1, timeout_ms);
    if (ret <= 0) {
      if (ret == 0)
        ROS_WARN("Read timeout");
      else
        ROS_WARN("Poll error");
      return nullptr;
    }
    if (!(fds.revents & POLLIN)) {
      ROS_WARN("Poll event not triggered");
      return nullptr;
    }
    const ssize_t read_count = read(serial_port, buffer.data(), buffer.size());
    if (read_count <= 0) {
      ROS_WARN("Read error, count = %ld", read_count);
      return nullptr;
    }
    ROS_DEBUG("Read %ld bytes", read_count);
    recv_buffer.insert(recv_buffer.end(), buffer.begin(), buffer.begin() + read_count);
    return &recv_buffer;
  }
  std::vector<uint8_t> sread() {
    std::vector<uint8_t> last_payload;
    std::vector<uint8_t> current_frame;
    bool in_frame = false;
    bool escape = false;
    size_t last_end_index = 0;
    tread(200);
    for (size_t i = 0; i < recv_buffer.size(); ++i) {
      uint8_t byte = recv_buffer[i];
      if (byte == 0xC0) {
        if (in_frame && !current_frame.empty()) {
          last_payload = current_frame;
          last_end_index = i + 1;
        }
        current_frame.clear();
        in_frame = true;
        escape = false;
        continue;
      }
      if (!in_frame) {
        continue;
      }
      if (escape) {
        if (byte == 0xDC)
          current_frame.push_back(0xC0);
        else if (byte == 0xDD)
          current_frame.push_back(0xDB);
        escape = false;
      } else if (byte == 0xDB) {
        escape = true;
      } else {
        current_frame.push_back(byte);
      }
    }
    if (last_end_index > 0) {
      recv_buffer.erase(recv_buffer.begin(), recv_buffer.begin() + static_cast<int>(last_end_index));
    }
    return last_payload;
  }
  ssize_t send(const char *d) const {
    if (serial_port < 0)
      return -10;
    tcflush(serial_port, TCIOFLUSH);
    const ssize_t write_count = write(serial_port, d, strlen(d));
    if (write_count == static_cast<ssize_t>(strlen(d)))
      ROS_DEBUG("Send: %s", d);
    else
      ROS_WARN("Send failed: %s (sent %ld)", d, write_count);
    return write_count;
  }
  ssize_t ssend(const std::vector<uint8_t>& payload) const {
    if (serial_port < 0)
      return -10;
    std::vector<uint8_t> encoded;
    encoded.reserve(payload.size() + 10);
    for (uint8_t byte : payload) {
      if (byte == 0xC0) {
        encoded.push_back(0xDB);
        encoded.push_back(0xDC);
      } else if (byte == 0xDB) {
        encoded.push_back(0xDB);
        encoded.push_back(0xDD);
      } else {
        encoded.push_back(byte);
      }
    }
    encoded.push_back(0xC0);
    tcflush(serial_port, TCIOFLUSH);
    const ssize_t write_count = write(serial_port, encoded.data(), encoded.size());
    if (write_count == static_cast<ssize_t>(encoded.size())) {
      ROS_DEBUG("Sent SLIP frame with payload size: %lu", payload.size());
    } else {
      ROS_WARN("Partial SLIP frame sent: expected %lu, sent %ld", encoded.size(), write_count);
    }
    return write_count;
  }
};

std::vector<uint8_t> packDatas() {
  std::vector<uint8_t> buffer({0, 0, 0});
  const auto normalized_x_vel = static_cast<int8_t>( global_vel_msg.linear.x > MAX_LINEAR_SPEED ?
                                                       MAX_LINEAR_SPEED : global_vel_msg.linear.x / MAX_LINEAR_SPEED * 100.0);
  const auto normalized_r_vel = static_cast<int8_t>( global_vel_msg.angular.z > MAX_ANGULAR_SPEED ?
                                                       MAX_ANGULAR_SPEED : global_vel_msg.angular.z / MAX_ANGULAR_SPEED * 100.0);
  buffer[0] |= normalized_x_vel;
  buffer[1] |= normalized_r_vel;
  buffer[2] |= cover_cmd;
  return buffer;
}

bool decodeDatas(const std::vector<uint8_t>& bin_datas) {
  for (int i = 0; i < 3; i++) {
    rwheel_ticks |= bin_datas[i];
    rwheel_ticks <<= 8;
  }
  rwheel_ticks |= bin_datas[3];
  for (int i = 0; i < 3; i++) {
    lwheel_ticks |= bin_datas[i + 4];
    lwheel_ticks <<= 8;
  }
  lwheel_ticks |= bin_datas[7];
  if(bin_datas[8])cover_state = true;
  else cover_state = false;
  clogging_state = bin_datas[9];
  return true;
}

int main(int argc, char* argv[]) {
  ros::init(argc, argv, "stm32_node");
  ros::NodeHandle node_handle;
  const ros::Publisher rw_pub = node_handle.advertise<std_msgs::Int32>("/rwheel_ticks", 1);
  const ros::Publisher lw_pub = node_handle.advertise<std_msgs::Int32>("/lwheel_ticks", 1);
  const ros::Publisher cover_pub = node_handle.advertise<std_msgs::UInt8>("/cover_state", 1);
  const ros::Publisher clog_pub = node_handle.advertise<std_msgs::UInt8>("clogging_state", 1);
  ros::Subscriber vel_sub = node_handle.subscribe("/cmd_vel", 1, updateVel);
  ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info);
  ros::Rate rate(30);
  // ReSharper disable once CppTooWideScope
  SerialDevice serial_device;
  global_vel_msg.angular.x = 0;
  global_vel_msg.angular.y = 0;
  global_vel_msg.angular.z = 0;
  global_vel_msg.linear.x = 0;
  global_vel_msg.linear.y = 0;
  global_vel_msg.linear.z = 0;
  while (ros::ok()) {
    ros::spinOnce();
    serial_device.ssend(packDatas());
    // ReSharper disable once CppDFAConstantConditions
    if(decodeDatas(serial_device.sread())) {
      std_msgs::Int32 send_msg;
      send_msg.data = static_cast<int32_t>(rwheel_ticks);
      rw_pub.publish(send_msg);
      send_msg.data = static_cast<int32_t>(lwheel_ticks);
      lw_pub.publish(send_msg);
      std_msgs::UInt8 u8_msg;
      u8_msg.data = cover_state;
      cover_pub.publish(u8_msg);
      u8_msg.data = clogging_state;
      clog_pub.publish(u8_msg);
    }
    rate.sleep();
  }
}

