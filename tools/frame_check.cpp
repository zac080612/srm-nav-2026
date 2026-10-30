#include <nav/frame.hpp>
#include <cstdio>

int main() {
  nav::UpFrame f = nav::pack_velocity_frame(1, 1.0f, 0.0f, 0.0f);
  printf("vx=1 vy=0 wz=0 seq=1: ");
  for (auto b : f) printf("%02X ", b);
  printf("\n协议参考帧:          A5 0C 00 01 26 00 00 80 3F 00 00 00 00 00 00 00 00 6B A3\n\n");

  nav::UpFrame z = nav::pack_velocity_frame(1, 0.0f, 0.0f, 0.0f);
  printf("零速     seq=1: ");
  for (auto b : z) printf("%02X ", b);
  printf("\n协议参考帧:          A5 0C 00 01 26 00 00 00 00 00 00 00 00 00 00 00 00 18 9F\n\n");

  nav::UpFrame g = nav::pack_velocity_frame(1, 1.0f, -1.0f, 3.0f);
  printf("vx=1 vy=-1 wz=3: ");
  for (auto b : g) printf("%02X ", b);
  printf("\n协议参考帧:          A5 0C 00 01 26 00 00 80 3F 00 00 80 BF 00 00 40 40 D3 96\n");
  return 0;
}
