#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <unistd.h>
#include <termios.h>
#include <fcntl.h>
#include <csignal>
#include <thread>
#include <sys/ioctl.h>
#include <vector>
#include <cstdint>
#include <algorithm>
#include <std_msgs/UInt8.h>
#include <std_msgs/Int32.h>

#define SERIAL_PORT "/dev/ttyS3"
#define MAX_LINEAR_SPEED 10.0
#define MAX_ANGULAR_SPEED 2.0
#define READ_STR_LENGTH 500
#define WHEEL_SEPRATION 0.155
#define METER_PER_ROTATE 0.207

geometry_msgs::Twist global_vel_msg;
uint32_t rwheel_ticks = 0;
uint32_t lwheel_ticks = 0;
uint8_t cover_cmd = 0;
bool cover_state = false;

void updateVel(const geometry_msgs::Twist& msg) {global_vel_msg = msg;}

void vectorToHexString(const std::vector<uint8_t>& vec, std::string& output) {
  std::ostringstream oss;
  for (const unsigned char i : vec) {
    oss << "0x"
        << std::setw(2)  // Ensure two digits for each byte
        << std::setfill('0')  // Fill with zeros if needed
        << std::hex  // Use hexadecimal format
        << static_cast<int>(i // Convert uint8_t to int for correct printing
        )  // Convert uint8_t to int for correct printing
        << " ";  // Add space between hex bytes
  }
  output = oss.str();  // Assign the string to the output parameter
}


class SerialDevice {
  int serial_port = -1;
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
    cfmakeraw(&tty);
    cfsetispeed(&tty, B115200);
    cfsetospeed(&tty, B115200);
    tty.c_iflag &= ~IMAXBEL;
    tty.c_cflag |= CLOCAL | CREAD;
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;
    tty.c_cflag &= ~PARENB;
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CRTSCTS;
    tty.c_lflag &= ~(ECHO | ECHOE | ECHOK | ECHOCTL | ECHOKE | ICANON);
    tty.c_cc[VMIN] = 0;
    tty.c_cc[VTIME] = 0;
    if (tcsetattr(serial_port, TCSANOW, &tty) != 0) {
      ROS_ERROR("Failed to set serial port: tcsetattr failed");
      close(serial_port);
    }
    ROS_WARN("Serial Open");
    recv_buffer.resize(READ_STR_LENGTH);
  }
  ~SerialDevice() {
    if (serial_port >= 0)
      close(serial_port);
  }
  int tread(const int timeout_ms) {
    if (serial_port < 0) {
      ROS_WARN("Read failed: serial not open");
      return 0;
    }
    int bytes;
    const auto start = std::chrono::steady_clock::now();
    while (true) {
      auto now  = std::chrono::steady_clock::now();
      if(std::chrono::duration_cast<std::chrono::milliseconds>(now - start).count() > timeout_ms)
        return 0;
      ioctl(serial_port, TIOCINQ, &bytes);
      if(bytes > 0)break;
      // std::this_thread::sleep_for(std::chrono::microseconds(10));
    }
    char buffer[READ_STR_LENGTH];
    const ssize_t read_count = read(serial_port, buffer, READ_STR_LENGTH);
    if (read_count < 0) {
      ROS_WARN("Read error, count = %ld", read_count);
      return 0;
    }
    ROS_DEBUG("Read %ld bytes", read_count);
    for (uint8_t i = 0; i < read_count; i++)
      recv_buffer.push_back(buffer[i]);
    return static_cast<int>(read_count);
  }
  std::vector<uint8_t> sread(const size_t payload_size, const int timeout_ms = 5) {
    std::vector<uint8_t> payload;
    std::vector<uint8_t> current_frame;
    bool in_frame = false;
    bool escape = false;
    while (true) {
      ROS_DEBUG("Now recv_buffer length is %ld", recv_buffer.size());
      if(tread(timeout_ms) == 0) {
        ROS_WARN("Read timeout!");
        break;
      }
      for (unsigned char byte : recv_buffer) {
        if (byte == 0xC0) {
          if (in_frame && !current_frame.empty())
            payload = current_frame;
          current_frame.clear();
          in_frame = true;
          escape = false;
          continue;
        }
        if (!in_frame)
          continue;
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
      if (payload.size() == payload_size) {
        recv_buffer.clear();
        ROS_DEBUG("Decode successfully");
        return payload;
      }
    }
    return {};
  }
  ssize_t ssend(const std::vector<uint8_t>& payload) const {
    if (serial_port < 0)
      return -10;
    std::vector<uint8_t> encoded;
    encoded.reserve(payload.size() * 2 + 4);
    encoded.push_back(0xC0);
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
    tcflush(serial_port, TCOFLUSH);
    const ssize_t write_count = write(serial_port, encoded.data(), encoded.size());
    tcdrain(serial_port);
    if (write_count == static_cast<ssize_t>(encoded.size())) {
      ROS_DEBUG("Sent SLIP frame with payload size: %lu", payload.size());
      std::string hex;
      vectorToHexString(encoded, hex);
      ROS_DEBUG("Sent raw content: %s", hex.c_str());
    } else {
      ROS_WARN("Partial SLIP frame sent: expected %lu, sent %ld", encoded.size(), write_count);
    }
    return write_count;
  }
};

inline void appendInt16BE(std::vector<uint8_t>& buf, const int16_t value) {
  buf.push_back(static_cast<uint8_t>(value & 0xFF));
  buf.push_back(static_cast<uint8_t>((value >> 8) & 0xFF));
}

template <typename T>
const T& clamp(const T& val, const T& low, const T& high) {
  return val < low ? low : val > high ? high : val;
}

std::vector<uint8_t> packDatas() {
  std::vector<uint8_t> buffer;
  buffer.reserve(10); // 预留空间以避免频繁扩容
  // 限速到 ±2 m/s
  const double base_vel = clamp(global_vel_msg.linear.x, -2.0, 2.0);
  const double l_vel = base_vel + global_vel_msg.angular.z * WHEEL_SEPRATION / 2.0;
  const double r_vel = base_vel - global_vel_msg.angular.z * WHEEL_SEPRATION / 2.0;
  // 转换为 RPM 并强转为 int16_t（注意范围是否溢出）
  const auto l_rpm = static_cast<int16_t>(l_vel * 60.0 / METER_PER_ROTATE);
  const auto r_rpm = static_cast<int16_t>(r_vel * 60.0 / METER_PER_ROTATE);
  // 写入字节（高位在前，大端格式）
  appendInt16BE(buffer, l_rpm);
  appendInt16BE(buffer, r_rpm);
  buffer.push_back(cover_cmd);
  std::reverse(buffer.begin(), buffer.end());
  // 可以继续添加其他字段
  return buffer;
}

bool decodeDatas(const std::vector<uint8_t>& bin_datas) {
  if(bin_datas.size() < 10)return false;
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
  return true;
}

int main(int argc, char* argv[]) {
  ros::init(argc, argv, "fpga_node");
  ros::NodeHandle node_handle;
  const ros::Publisher rw_pub = node_handle.advertise<std_msgs::Int32>("/rwheel_ticks", 1);
  const ros::Publisher lw_pub = node_handle.advertise<std_msgs::Int32>("/lwheel_ticks", 1);
  const ros::Publisher cover_pub = node_handle.advertise<std_msgs::UInt8>("/cover_state", 1);
  ros::Subscriber vel_sub = node_handle.subscribe("/cmd_vel", 1, updateVel);
  int debug_mode = 0;
  ros::param::get("fpga_debug", debug_mode);
  if(debug_mode)
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug);
  else
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info);
  ros::Rate rate(10);
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
    // if(decodeDatas(serial_device.sread(10))) {
    //   std_msgs::Int32 send_msg;
    //   send_msg.data = static_cast<int32_t>(rwheel_ticks);
    //   rw_pub.publish(send_msg);
    //   send_msg.data = static_cast<int32_t>(lwheel_ticks);
    //   lw_pub.publish(send_msg);
    //   std_msgs::UInt8 u8_msg;
    //   u8_msg.data = cover_state;
    //   cover_pub.publish(u8_msg);
    // }
    // else
      // ROS_DEBUG("Payload size does not match");
    rate.sleep();
  }
}

