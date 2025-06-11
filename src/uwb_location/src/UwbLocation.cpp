#include "sensor_msgs/msg/imu.hpp"
#include "std_msgs/msg/string.hpp"
#include "uwb_location/msg/uwb.hpp"
#include "uwb_location/trilateration.h"
#include <iostream>
#include <rclcpp/rclcpp.hpp>
#include <serial/serial.h>
#include <string.h>
#include <string>
#include "crc16_xmodem.h"

#define MAX_DATA_NUM 1024
#define DataHead 'm'
#define DataHead2 'M'
#define DataTail '\n'
#define PACKET_HEAD0 (0X55)
#define PACKET_HEAD1 (0xAA)

using namespace std;

namespace {
size_t buffer_and_get_packet(const unsigned char* input_data, size_t input_len, unsigned char* output_buf) {
    static std::vector<uint8_t> data_buffer;

    // 清空输出缓冲区
    memset(output_buf, 0, MAX_DATA_NUM);
    
    // 将新数据添加到缓冲区
    data_buffer.insert(data_buffer.end(), input_data, input_data + input_len);
    
    // 查找并返回完整的数据包
    while (data_buffer.size() >= 7) {
      if (data_buffer[0] == PACKET_HEAD0 && data_buffer[1] == PACKET_HEAD1) {
        uint16_t tlv_total_len = *(uint16_t *)&data_buffer[3];
        size_t packet_len = tlv_total_len + 7;
        
        if (data_buffer.size() >= packet_len) {
          // 找到完整数据包，复制到输出缓冲区
          memcpy(output_buf, data_buffer.data(), packet_len);
          data_buffer.erase(data_buffer.begin(), data_buffer.begin() + packet_len);
          return packet_len;
        }
        break;
      } else {
        data_buffer.erase(data_buffer.begin());
      }
    }
    return 0;  // 没有找到完整数据包
  }
}

class UwbLocationNode : public rclcpp::Node {
public:
  UwbLocationNode() : Node("uwb_location_node") {
    // 创建发布者
    uwb_publisher_ =
        this->create_publisher<uwb_location::msg::UWB>("/uwb/data", 10);
    // imu_publisher_ =
    //     this->create_publisher<sensor_msgs::msg::Imu>("/imu/data", 10);

    // 初始化串口
    try {
      serial_port_.setPort("/dev/ttyUSB0");
      serial_port_.setBaudrate(115200);
      serial_port_.setBytesize(serial::eightbits);
      serial_port_.setParity(serial::parity_none);
      serial_port_.setStopbits(serial::stopbits_one);
      serial::Timeout to = serial::Timeout::simpleTimeout(11);
      serial_port_.setTimeout(to);
      serial_port_.open();
    } catch (serial::IOException &e) {
      RCLCPP_ERROR(this->get_logger(), "Unable to open port: %s", e.what());
      return;
    }

    if (serial_port_.isOpen()) {
      RCLCPP_INFO(this->get_logger(), "/dev/ttyUSB0 is opened.");
    }

    // 创建定时器，用于定期检查串口数据
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(10),
        std::bind(&UwbLocationNode::timer_callback, this));
  }

private:
  void timer_callback() {
    if (serial_port_.available()) {
      size_t len = serial_port_.available();
      if (len > 0) {
        unsigned char usart_buf[1024] = {0};
        serial_port_.read(usart_buf, len);

        // unsigned char *pbuf;
        // unsigned char buf[2014] = {0};

        // pbuf = (unsigned char *)usart_buf;
        // memcpy(&buf[0], pbuf, len);
        // int reallength = len;
        // if (reallength != 0) {
        //   for (int i = 0; i < reallength; i++) {
        //     buf_data_from_ctrl_[buf_ctrl_posit_w_] = buf[i];
        //     buf_ctrl_posit_w_ = (buf_ctrl_posit_w_ == (MAX_DATA_NUM - 1))
        //                             ? 0
        //                             : (1 + buf_ctrl_posit_w_);
        //   }
        // }
        // CtrlSerDataDeal();

        // memcpy(&receive_buf_[0], &usart_buf[0], len);
        // receive_buf_len_ = len;
        receive_buf_len_ = buffer_and_get_packet(usart_buf, len, receive_buf_);
        // 处理串口数据
        int ret = receive_deal_func();

        if (ret == 0)
        {
          // 发布UWB数据
          auto uwb_msg = uwb_location::msg::UWB();
          uwb_msg.header.stamp = this->now();
          uwb_msg.distance = distance_;
          uwb_msg.angle = angle_;
          uwb_msg.pitch = pitch_;
          uwb_msg.rssi = rssi_;
          uwb_msg.rssi_len = rssi_len_;
          uwb_publisher_->publish(uwb_msg);
        }

        // // 发布IMU数据
        // auto imu_msg = sensor_msgs::msg::Imu();
        // imu_msg.header.stamp = this->now();
        // imu_msg.header.frame_id = "base_link";
        // imu_msg.linear_acceleration.x = 0.f;
        // imu_msg.linear_acceleration.y = 0.f;
        // imu_msg.linear_acceleration.z = 0.f;
        // imu_msg.angular_velocity.x = 0.f;
        // imu_msg.angular_velocity.y = 0.f;
        // imu_msg.angular_velocity.z = 0.f;
        // imu_msg.orientation.x = 0.f;
        // imu_msg.orientation.y = 0.f;
        // imu_msg.orientation.z = 0.f;
        // imu_msg.orientation.w = 0.f;
        // imu_publisher_->publish(imu_msg);
      }
    }
  }

  uint16_t swap_uint16(uint16_t value)  {
    return (value >> 8) | (value << 8);
  }

  // 原有的数据处理函数保持不变
  int receive_deal_func() {
    // int range[8] = {-1};
    if ((receive_buf_[0] == PACKET_HEAD0) && (receive_buf_[1] == PACKET_HEAD1))
    {
      // uint8_t seq = receive_buf_[2];
      uint16_t tlv_total_len = *(uint16_t *)&receive_buf_[3];
      uint8_t *tlv = receive_buf_ + 5;
      // uint16_t crc = *(uint16_t *)(receive_buf_ + receive_buf_len_ - 2);
      if (static_cast<uint32_t>(tlv_total_len) + 7 != receive_buf_len_)
      {
        printf("receive_buf_len_ = %d\n", receive_buf_len_);
        printf("tlv_total_len = %d\n", tlv_total_len);
        printf("Invalid TLV length\n");
        return -1; 
      }
      uint16_t crc = *(uint16_t *)(receive_buf_ + receive_buf_len_ - 2);
      
      // 调试打印
      // printf("TLV数据: ");
      // for(int i = 0; i < tlv_total_len; i++) {
      //     printf("%02X ", tlv[i]);
      // }
      // printf("\n接收的CRC: %04X\n", crc);

      // 校验CRC
      uint16_t crc_result = 0x0000;  // CRC16-XMODEM初始值为0x0000
      crc_result = crc16_xmodem(crc_result, tlv, tlv_total_len);
      
      // 由于CRC是大端格式，需要交换字节顺序
      crc_result = swap_uint16(crc_result);
      // printf("计算的CRC (after swap): %04X\n", crc_result);

      if (crc_result != crc)
      {
        printf("CRC校验错误: 计算值=0x%04X, 接收值=0x%04X\n", crc_result, crc);
        return -1;
      }

      // printf("uart data %u\n", seq);
      // // print receive_buf_
      // for (int i = 0; i < receive_buf_len_; i++)
      // {
      //   printf("%02x ", receive_buf_[i]);
      // }
      // printf("\n");

      // check crc
      // uint16_t crc_result = 0;
      // crc_result = crc_ccitt(crc_result, tlv, tlv_total_len);
      // if (crc_result != crc)
      // {
      //   printf("CRC Error\n");
      //   return -1;
      // }
      uint8_t tlv_type = tlv[0];

      if (tlv_type == 0xC5) {
        struct __attribute__((packed, aligned(1))) C5Data
        {
          uint8_t type;
          uint8_t len;
          uint32_t sync_cnt;
          uint32_t mac_id;
          uint32_t fob_id;
          uint16_t fob_type;
          float distance;
          float angle;
          float pitch;
          uint8_t rssi_len;
        };
        // printf("c5data size: %d\n", sizeof(C5Data));
        C5Data *uwb_aoa_fob_pkg = (C5Data *)tlv;
        uint8_t *rssi = &tlv[29];
        rssi_.assign(rssi, rssi + uwb_aoa_fob_pkg->rssi_len);

        // uwb_aoa_fob_pkg->sync_cnt = swap_uint32(uwb_aoa_fob_pkg->sync_cnt);
        // uwb_aoa_fob_pkg->mac_id = swap_uint32(uwb_aoa_fob_pkg->mac_id);
        // uwb_aoa_fob_pkg->fob_id = swap_uint32(uwb_aoa_fob_pkg->fob_id);
        // uwb_aoa_fob_pkg->fob_type = swap_uint16(uwb_aoa_fob_pkg->fob_type);
        // uwb_aoa_fob_pkg->angle = swap_float(uwb_aoa_fob_pkg->angle);
        // uwb_aoa_fob_pkg->distance = swap_float(uwb_aoa_fob_pkg->distance);
        // uwb_aoa_fob_pkg->pitch = swap_float(uwb_aoa_fob_pkg->pitch);

        // printf("Sync Count: %u, MAC ID: %u, FOB ID: %u, FOB Type: %u\n",
        //        uwb_aoa_fob_pkg->sync_cnt, uwb_aoa_fob_pkg->mac_id, uwb_aoa_fob_pkg->fob_id, uwb_aoa_fob_pkg->fob_type);
        // printf("Distance: %.3f, Angle: %.3f, Pitch: %.3f, RSSI Length: %d\n",
        //        uwb_aoa_fob_pkg->distance, uwb_aoa_fob_pkg->angle, uwb_aoa_fob_pkg->pitch, rssi_count);

        // print receive_buf_
        // for (int i = 0; i < 4; i++)
        // {
        //   printf("%02x ", ((uint8_t *)&uwb_aoa_fob_pkg->distance)[i]);
        // }
        // for (int i = 0; i < rssi_count; i++)
        // {
        //   printf("RSSI[%d]: %d\n", i, rssi[i]);
        // }

        rssi_len_ = uwb_aoa_fob_pkg->rssi_len;
        angle_ = uwb_aoa_fob_pkg->angle;
        pitch_ = uwb_aoa_fob_pkg->pitch;
        distance_ = uwb_aoa_fob_pkg->distance;
        // printf("rssi_len = %d, dis = %.2fm, angle = %.2f°, pitch = %.2f°\n", rssi_len_, distance_, angle_, pitch_);
        return 0;
      }
      return -1;
    }
#if 0
    if ((receive_buf_[0] == DataHead) && (receive_buf_[1] == 'c')) {
      unsigned int mask, lnum, seq, tid, aid;
      int rangetime;
      char role;
      int data_len = strlen((char *)receive_buf_);
      if (data_len == 106) {
        sscanf((char *)receive_buf_,
               "mc %x %d %d %d %d %d %d %d %d %x %x %d %c%u:%u", &mask,
               &range[0], &range[1], &range[2], &range[3], &range[4], &range[5],
               &range[6], &range[7], &lnum, &seq, &rangetime, &role, &tid,
               &aid);

        printf("mask=0x%02x\nrange[0]=%d(mm)\nrange[1]=%d(mm)\nrange[2]=%d(mm)"
               "\nrange[3]=%d(mm)\nrange[4]=%d(mm)\nrange[5]=%d(mm)\nrange[6]=%"
               "d(mm)\nrange[7]=%d(mm)\r\n",
               mask, range[0], range[1], range[2], range[3], range[4], range[5],
               range[6], range[7]);
      } else if (data_len == 70) {
        sscanf((char *)receive_buf_, "mc %x %d %d %d %d %x %x %d %c%u:%u",
               &mask, &range[0], &range[1], &range[2], &range[3], &lnum, &seq,
               &rangetime, &role, &tid, &aid);

        printf("mask=0x%02x\nrange[0]=%d(mm)\nrange[1]=%d(mm)\nrange[2]=%d(mm)"
               "\nrange[3]=%d(mm)\r\n",
               mask, range[0], range[1], range[2], range[3]);
      } else {
        return;
      }
    }
    // MP0034,0,302,109,287,23,134.2,23.4,23,56
    else if ((receive_buf_[0] == DataHead2) && (receive_buf_[1] == 'P')) {
      char *ptr, *retptr;
      ptr = (char *)receive_buf_;
      char cut_data[30][12];
      int cut_count = 0;

      while ((retptr = strtok(ptr, ",")) != NULL) {
        // printf("%s\n", retptr);
        strcpy(cut_data[cut_count], retptr);
        ptr = NULL;
        cut_count++;
        if (cut_count >= 29)
          break;
      }

      int tag_id = atoi(cut_data[1]);
      printf("tag_id = %d\n", tag_id);

      x_ = (float)atoi(cut_data[2]) / 100.0f;
      printf("x = %.2fm\n", x_);

      y_ = (float)atoi(cut_data[3]) / 100.0f;
      printf("y = %.2fm\n", y_);

      aoa_ = atof(cut_data[7]);
      printf("aoa = %.2f°\n", aoa_);

      distance_ = (float)atoi(cut_data[4]) / 100.0f;
      printf("dis = %.2fm\n", distance_);
    } else if ((receive_buf_[0] == DataHead) && (receive_buf_[1] == 'i')) {
      float rangetime2;
      float acc[3], gyro[3];

      // mi,981.937,0.63,NULL,NULL,NULL,-2.777783,1.655664,9.075048,-0.004788,-0.014364,-0.001596,T0
      // //13
      // mi,3.710,0.55,NULL,NULL,NULL,NULL,NULL,NULL,NULL,-1.327881,0.653174,9.577490,-0.004788,-0.013300,-0.002128,T0
      // //17
      char *ptr, *retptr;
      ptr = (char *)receive_buf_;
      char cut_data[30][12];
      int cut_count = 0;

      while ((retptr = strtok(ptr, ",")) != NULL) {
        // printf("%s\n", retptr);
        strcpy(cut_data[cut_count], retptr);
        ptr = NULL;
        cut_count++;
        if (cut_count >= 29)
          break;
      }

      rangetime2 = atof(cut_data[1]);

      if (cut_count == 13) // 4anchors
      {
        for (int i = 0; i < 4; i++) {
          if (strcmp(cut_data[i + 2], "NULL")) {
            range[i] = atof(cut_data[i + 2]) * 1000;
          } else {
            range[i] = -1;
          }
        }

        for (int i = 0; i < 3; i++) {
          acc[i] = atof(cut_data[i + 6]);
        }

        for (int i = 0; i < 3; i++) {
          gyro[i] = atof(cut_data[i + 9]);
        }

        printf("rangetime = %.3f\n", rangetime2);
        printf("range[0] = %d\n", range[0]);
        printf("range[1] = %d\n", range[1]);
        printf("range[2] = %d\n", range[2]);
        printf("range[3] = %d\n", range[3]);
        printf("acc[0] = %.3f\n", acc[0]);
        printf("acc[1] = %.3f\n", acc[1]);
        printf("acc[2] = %.3f\n", acc[2]);
        printf("gyro[0] = %.3f\n", gyro[0]);
        printf("gyro[1] = %.3f\n", gyro[1]);
        printf("gyro[2] = %.3f\n", gyro[2]);
      } else if (cut_count == 17) // 8anchors
      {
        for (int i = 0; i < 8; i++) {
          if (strcmp(cut_data[i + 2], "NULL")) {
            range[i] = atof(cut_data[i + 2]) * 1000;
          } else {
            range[i] = -1;
          }
        }

        for (int i = 0; i < 3; i++) {
          acc[i] = atof(cut_data[i + 6 + 4]);
        }

        for (int i = 0; i < 3; i++) {
          gyro[i] = atof(cut_data[i + 9 + 4]);
        }

        printf("rangetime = %.3f\n", rangetime2);
        printf("range[0] = %d\n", range[0]);
        printf("range[1] = %d\n", range[1]);
        printf("range[2] = %d\n", range[2]);
        printf("range[3] = %d\n", range[3]);
        printf("range[4] = %d\n", range[4]);
        printf("range[5] = %d\n", range[5]);
        printf("range[6] = %d\n", range[6]);
        printf("range[7] = %d\n", range[7]);
        printf("acc[0] = %.3f\n", acc[0]);
        printf("acc[1] = %.3f\n", acc[1]);
        printf("acc[2] = %.3f\n", acc[2]);
        printf("gyro[0] = %.3f\n", gyro[0]);
        printf("gyro[1] = %.3f\n", gyro[1]);
        printf("gyro[2] = %.3f\n", gyro[2]);
      } else {
        return;
      }
    } else {
      puts("no range message");
      return;
    }
#endif
    return -1;
  }

  void CtrlSerDataDeal() {
    unsigned char middata = 0;
    static unsigned char dataTmp[MAX_DATA_NUM] = {0};

    while (buf_ctrl_posit_r_ != buf_ctrl_posit_w_) {
      middata = buf_data_from_ctrl_[buf_ctrl_posit_r_];
      buf_ctrl_posit_r_ =
          (buf_ctrl_posit_r_ == MAX_DATA_NUM - 1) ? 0 : (buf_ctrl_posit_r_ + 1);

      if (((middata == DataHead) || (middata == DataHead2)) &&
          (rcvsign_ == 0)) //收到头
      {
        rcvsign_ = 1;                      //开始了一个数据帧
        dataTmp[data_record_++] = middata; //数据帧接收中
      } else if ((middata != DataTail) && (rcvsign_ == 1)) {
        dataTmp[data_record_++] = middata;                 //数据帧接收中
      } else if ((middata == DataTail) && (rcvsign_ == 1)) //收到尾
      {
        if (data_record_ != 1) {
          rcvsign_ = 0;
          dataTmp[data_record_++] = middata;
          dataTmp[data_record_] = '\0';

          strncpy((char *)receive_buf_, (char *)dataTmp, data_record_);
          printf("receive_buf = %slen = %d\n", receive_buf_, data_record_);
          receive_deal_func(); /*调用处理函数*/
          bzero(receive_buf_, sizeof(receive_buf_));

          data_record_ = 0;
        }
      }
    }
  }

  rclcpp::Publisher<uwb_location::msg::UWB>::SharedPtr uwb_publisher_;
  // rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  serial::Serial serial_port_;

  // 原有的全局变量移到类成员变量
  unsigned char receive_buf_[MAX_DATA_NUM] = {0};
  uint32_t receive_buf_len_ = 0;
  // float x_ = 0.f;
  // float y_ = 0.f;
  // float aoa_ = 0.f;
  uint8_t rssi_len_ = 0;
  float distance_ = 0.f;
  float angle_ = 0.f;
  float pitch_ = 0.f;
  std::vector<int8_t> rssi_;

  unsigned char buf_data_from_ctrl_[MAX_DATA_NUM] = {0};
  int buf_ctrl_posit_w_ = 0, buf_ctrl_posit_r_ = 0;
  int data_record_ = 0, rcvsign_ = 0;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<UwbLocationNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
