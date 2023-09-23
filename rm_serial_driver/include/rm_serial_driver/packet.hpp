// Copyright (c) 2022 ChenJun
// Licensed under the MIT License.

#ifndef RM_SERIAL_DRIVER__PACKET_HPP_
#define RM_SERIAL_DRIVER__PACKET_HPP_

#include <algorithm>
#include <cstdint>
#include <vector>

namespace rm_serial_driver
{
// 接收数据包
struct ReceivePacket
{
  uint8_t header = 0x5A; // 帧头
  float bullet_speed; // 子弹速度
  float yaw; // yaw角 
  float pitch; // pitch角
  float roll; // roll角
  char enemy_color; // 敌方颜色
  uint16_t checksum = 0; // 校验和
} __attribute__((packed));

// 发送数据包
struct SendPacket
{
  uint8_t header = 0xA5; // 帧头
  uint32_t packat_id = 0; // 数据包ID
  uint8_t suggest_fire = 0; // 建议开火
  float offset_yaw = 0; // 偏航角增量
  float offset_pitch = 0; // 俯仰角增量
  uint16_t checksum = 0; // 校验和
} __attribute__((packed));

// 将vector转换为ReceivePacket数据包
inline ReceivePacket fromVector(const std::vector<uint8_t> & data)
{
  ReceivePacket packet;
  std::copy(data.begin(), data.end(), reinterpret_cast<uint8_t *>(&packet));
  return packet;
}

// 将SendPacket数据包转换为vector
inline std::vector<uint8_t> toVector(const SendPacket & data)
{
  std::vector<uint8_t> packet(sizeof(SendPacket));
  std::copy(
    reinterpret_cast<const uint8_t *>(&data),
    reinterpret_cast<const uint8_t *>(&data) + sizeof(SendPacket), packet.begin());
  return packet;
}

}  // namespace rm_serial_driver

#endif  // RM_SERIAL_DRIVER__PACKET_HPP_
