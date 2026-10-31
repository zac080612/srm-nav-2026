// 桩结构验证的几何模拟单测（零 ROS 依赖，Windows/NUC 都能跑）。
// 用「射线求交」模拟激光雷达扫描 3D 场地，场地里放：
//   2 个真桩（下半球 r=0.10 + 上细杆 r=0.01）+ 5 类杂物
// 然后跑与感知节点完全相同的流水线：分带 → 低带找圆 → 结构验证，
// 断言：无论车在 8 字路线的哪个位置，真桩恰好 2 个、杂物 0 个。

#include "nav/pile_detect.hpp"
#include "nav/test_util.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <functional>
#include <unordered_set>
#include <vector>

namespace {

using nav::Pile;
using nav::Vec2;

constexpr double kInf = 1e9;

// ---- 3D 场地模型 ----

// 旋转体：z 处的水平截面是圆心 c、半径 radius(z) 的圆盘
struct Lathe {
  Vec2 c;
  double z_min, z_max;
  std::function<double(double)> radius;   // z → 截面半径
};

// 竖直箱体：z 处截面是矩形
struct Box {
  Vec2 c;
  double hx, hy, z_min, z_max;
};

// 球体（真桩的下半球用）
struct Ball {
  Vec2 c;
  double z_c, r;
};

// 场地：真桩 (0,-1) 下、(0,1) 上，杂物 5 类放 8 字路线外
struct World {
  std::vector<Lathe> lathes;
  std::vector<Box> boxes;
  std::vector<Ball> balls;
  Vec2 down{0.0, -1.0};
  Vec2 up{0.0, 1.0};
};

World build_world() {
  World w;
  // 真桩下半球：球心 (0,0)，半径 0.10，z∈[0,0.10] 截面半径 sqrt(0.01-z^2)
  auto half_sphere = [](double z) {
    return (z >= 0.0 && z <= 0.10) ? std::sqrt(0.01 - z * z) : 0.0;
  };
  // 真桩上半细杆：z∈[0.10,0.85]，半径 0.01
  auto thin_rod = [](double) { return 0.01; };
  for (const Vec2 c : {w.down, w.up}) {
    w.lathes.push_back({c, 0.0, 0.10, half_sphere});
    w.lathes.push_back({c, 0.10, 0.85, thin_rod});
    w.balls.push_back({c, 0.0, 0.10});
  }
  // 杂物1：矮圆柱 r=0.10 高 0.12（低带圆过关，上方 0 点 → 应被结构验证滤掉）
  w.lathes.push_back({{2.6, -0.6}, 0.0, 0.12, [](double) { return 0.10; }});
  // 杂物2：粗柱 r=0.12 高 0.9（半径在桩范围内，上方点太多 → 应被滤掉）
  w.lathes.push_back({{2.8, 1.9}, 0.0, 0.9, [](double) { return 0.12; }});
  // 杂物3：只有半球没有杆（最强干扰：长得像桩底座 → 应被滤掉）
  w.lathes.push_back({{-2.8, 0.6}, 0.0, 0.10, half_sphere});
  // 杂物4：全高细柱 r=0.05（半径小于下限 → 圆检就滤掉）
  w.lathes.push_back({{-2.4, -1.9}, 0.0, 0.9, [](double) { return 0.05; }});
  // 杂物5：箱子 0.3×0.3×0.4（直线面 → 圆拟合残差大 → 滤掉）
  w.boxes.push_back({{1.9, -2.9}, 0.15, 0.15, 0.0, 0.4});
  // 杂物6：测试用"人"（0.3×0.3×0.5 箱子，返程路线外侧 0.25m）
  w.boxes.push_back({{-0.98, -0.85}, 0.15, 0.15, 0.0, 0.5});
  return w;
}

// ---- 射线求交 ----

std::pair<double, double> hit_disk_t(const Vec2& s, const Vec2& d,
                                     const Vec2& c, double r) {
  const double dx = s.x - c.x, dy = s.y - c.y;
  const double proj = dx * d.x + dy * d.y;
  const double lat = std::fabs(dx * d.y - dy * d.x);
  const double rr = r * r;
  if (lat > r) return {kInf, kInf};
  const double half = std::sqrt(rr - lat * lat);
  // 二次方程 |s+t d-c|=r 的根是 -proj ± half
  const double t_enter = -proj - half;
  if (t_enter > 0.0) return {t_enter, -proj + half};
  if (-proj + half > 0.0) return {0.0, -proj + half};   // 起点在圆盘内
  return {kInf, kInf};
}

double hit_sphere(const Vec2& s, double z_s, const Vec2& d, double dz,
                  const Vec2& c, double z_c, double r) {
  const double sx = s.x - c.x, sy = s.y - c.y, sz = z_s - z_c;
  const double b = 2.0 * (sx * d.x + sy * d.y + sz * dz);
  const double a = d.x * d.x + d.y * d.y + dz * dz;
  const double cc = sx * sx + sy * sy + sz * sz - r * r;
  const double disc = b * b - 4.0 * a * cc;
  if (disc <= 0.0) return kInf;
  const double t = (-b - std::sqrt(disc)) / (2.0 * a);
  if (t > 0.0) return t;
  const double t2 = (-b + std::sqrt(disc)) / (2.0 * a);
  return t2 > 0.0 ? 0.0 : kInf;
}

double hit_box(const Vec2& s, double z_s, const Vec2& d, double dz,
               const Vec2& c, double hx, double hy, double z_min, double z_max) {
  double tmin = 0.0, tmax = kInf;
  const double lo_x = c.x - hx, hi_x = c.x + hx;
  const double lo_y = c.y - hy, hi_y = c.y + hy;
  if (std::fabs(d.x) < 1e-12) {
    if (s.x < lo_x || s.x > hi_x) return kInf;
  } else {
    double t1 = (lo_x - s.x) / d.x, t2 = (hi_x - s.x) / d.x;
    if (t1 > t2) std::swap(t1, t2);
    tmin = std::max(tmin, t1);
    tmax = std::min(tmax, t2);
  }
  if (std::fabs(d.y) < 1e-12) {
    if (s.y < lo_y || s.y > hi_y) return kInf;
  } else {
    double t1 = (lo_y - s.y) / d.y, t2 = (hi_y - s.y) / d.y;
    if (t1 > t2) std::swap(t1, t2);
    tmin = std::max(tmin, t1);
    tmax = std::min(tmax, t2);
  }
  if (std::fabs(dz) < 1e-12) {
    if (z_s < z_min || z_s > z_max) return kInf;
  } else {
    double t1 = (z_min - z_s) / dz, t2 = (z_max - z_s) / dz;
    if (t1 > t2) std::swap(t1, t2);
    tmin = std::max(tmin, t1);
    tmax = std::min(tmax, t2);
  }
  return tmin <= tmax ? tmin : kInf;
}

// ---- 扫描模拟 ----

// 车在 car、射线起点高 z_lidar，水平 360° 分 samples 条射线，
// 每条射线俯角 pitch_deg（dz = -tan(pitch)，水平层模拟传 0）。
// 返回 2D 点（车体系）。
std::vector<Vec2> scan(const World& w, const Vec2& car, double z_lidar,
                       double pitch_deg, int samples, double min_range,
                       double max_range) {
  std::vector<Vec2> pts;
  const double dz = -std::tan(pitch_deg * M_PI / 180.0);
  for (int i = 0; i < samples; ++i) {
    const double a = -M_PI + 2.0 * M_PI * i / samples;
    const Vec2 d{std::cos(a), std::sin(a)};
    double t_best = kInf;
    for (const auto& L : w.lathes) {
      const double r = L.radius(z_lidar);   // 水平层：截面半径按层高
      if (r <= 1e-9) continue;
      const auto [t1, t2] = hit_disk_t(car, d, L.c, r);
      if (t1 >= kInf) continue;
      // 带俯角时检查打点高度是否还在柱体高度范围内（顶/底帽误差忽略）
      const double z1 = z_lidar + t1 * dz;
      const double z2 = z_lidar + t2 * dz;
      if (std::min(z1, z2) > L.z_max || std::max(z1, z2) < L.z_min) continue;
      if (t1 > 0.0) t_best = std::min(t_best, t1);
    }
    for (const auto& B : w.balls) {
      t_best = std::min(t_best,
                        hit_sphere(car, z_lidar, d, dz, B.c, B.z_c, B.r));
    }
    for (const auto& B : w.boxes) {
      t_best = std::min(t_best, hit_box(car, z_lidar, d, dz, B.c, B.hx,
                                        B.hy, B.z_min, B.z_max));
    }
    if (t_best > min_range && t_best < max_range) {
      pts.push_back({t_best * d.x, t_best * d.y});   // 车体系坐标（车在原点）
    }
  }
  return pts;
}

// 复刻感知节点的体素降采样（2D 格子 + band 维度）
std::vector<Vec2> voxel_filter(const std::vector<Vec2>& pts, double voxel,
                               int band) {
  const double inv = 1.0 / std::max(voxel, 1e-3);
  std::unordered_set<int64_t> seen;
  std::vector<Vec2> out;
  for (const auto& p : pts) {
    const int gx = static_cast<int>(std::floor(p.x * inv));
    const int gy = static_cast<int>(std::floor(p.y * inv));
    const int64_t key = (static_cast<int64_t>(gx) << 40) ^
                        (static_cast<int64_t>(gy) << 8) ^ band;
    if (!seen.insert(key).second) continue;
    out.push_back(p);
  }
  return out;
}

// ---- 感知流水线（与 perception_node 相同的参数与顺序）----

struct DetParams {
  double min_range = 0.30;
  double max_range = 4.0;
  double voxel = 0.04;
  // 半径是弱特征：短弧 Kasa 拟合连半径都会偏小（实测 0.054），且不同俯角
  // 打点投影半径 0.04~0.095 混合。下限放宽到 0.04；上限 0.11 滤粗柱。
  // 细柱(r=0.05)/粗柱(r=0.12) 主要靠「高带散布大」过滤，半径只是辅助。
  double min_radius = 0.04;
  double max_radius = 0.11;
  int min_points = 3;
  double cluster_dist = 0.20;
  double max_fit_rms = 0.08;
  double pile_low_max = 0.15;
  double pile_high_min = 0.35;
  double high_match_r = 0.10;
  int high_min_pts = 2;
  double high_spread_max = 0.03;
};

// 返回 {验证后的桩, 低带点数, 高带点数}
struct Result {
  std::vector<Pile> piles;
  size_t low_n, high_n;
};

// 用生产参数组装检测器
nav::PileDetectParams det_params(const DetParams& p) {
  nav::PileDetectParams dp;
  dp.min_radius = p.min_radius;
  dp.max_radius = p.max_radius;
  dp.min_points = static_cast<size_t>(p.min_points);
  dp.cluster_dist = p.cluster_dist;
  dp.max_fit_rms = p.max_fit_rms;
  dp.min_arc_deg = 0.0;
  return dp;
}

nav::PileStructureParams struct_params(const DetParams& p) {
  nav::PileStructureParams sp;
  sp.high_match_r = p.high_match_r;
  sp.high_min_pts = p.high_min_pts;
  sp.high_spread_max = p.high_spread_max;
  return sp;
}

// 高低带点 → 结构化桩检测
Result run_detect(const std::vector<Vec2>& low, const std::vector<Vec2>& high,
                  const DetParams& p) {
  const auto piles =
      nav::detect_piles_structured(low, high, det_params(p), struct_params(p));
  return {piles, low.size(), high.size()};
}

// 断言：检测桩只可能是真桩（1~2 个都行，两桩一线被挡时短时只看到 1 个；
// 主程序有锁定+漏检兜底）。绝不允许检测到杂物。
int check_piles(const Result& r, const Vec2& car) {
  int fails = 0;
  if (r.piles.empty()) {
    std::printf("[FAIL] 车(%.2f,%.2f) 一个桩都没看到（低带=%zu 高带=%zu）\n",
                car.x, car.y, r.low_n, r.high_n);
    return 1;
  }
  if (r.piles.size() > 2) {
    std::printf("[FAIL] 车(%.2f,%.2f) 检测到 %zu 个桩（>2，混入杂物）\n",
                car.x, car.y, r.piles.size());
    for (const auto& pl : r.piles) {
      std::printf("        检测到 (%.2f, %.2f) r=%.3f\n", pl.center.x,
                  pl.center.y, pl.radius);
    }
    return 1;
  }
  for (const auto& pl : r.piles) {
    // 检测结果是车体系坐标，换算回世界再与真桩比较
    const double wx = car.x + pl.center.x;
    const double wy = car.y + pl.center.y;
    const double dd = std::hypot(wx - 0.0, wy + 1.0);
    const double du = std::hypot(wx - 0.0, wy - 1.0);
    if (std::min(dd, du) > 0.15) {
      std::printf("[FAIL] 车(%.2f,%.2f) 检测到杂物桩 (%.2f, %.2f) r=%.3f\n",
                  car.x, car.y, wx, wy, pl.radius);
      fails++;
    }
  }
  return fails;
}

// 8 字路线采样点：启动 → 直入 → 绕下桩 → 交叉 → 绕上桩 → 返程
std::vector<Vec2> route_points() {
  std::vector<Vec2> pts;
  pts.push_back({-1.5, -2.5});            // 停车位（启动）
  pts.push_back({-0.75, -1.75});          // StraightIn 中点
  for (int i = 0; i < 8; ++i) {           // 绕下桩 (0,-1)
    const double a = 2.0 * M_PI * i / 8;
    pts.push_back({0.57 * std::cos(a), -1.0 + 0.57 * std::sin(a)});
  }
  pts.push_back({0.0, 0.0});              // 交叉点
  for (int i = 0; i < 8; ++i) {           // 绕上桩 (0,1)
    const double a = 2.0 * M_PI * i / 8;
    pts.push_back({0.57 * std::cos(a), 1.0 + 0.57 * std::sin(a)});
  }
  pts.push_back({-0.75, -0.75});          // StraightOut 中点
  return pts;
}

}  // namespace

int main() {
  const World w = build_world();
  const auto route = route_points();

  // 场景 A：Gazebo 多层水平激光（sim 用的几何）
  //   层 z=0.05/0.06 低带、0.25 中间、0.45/0.80 高带，0.25°/线
  std::printf("== 场景 A：Gazebo 水平层几何 ==\n");
  {
    DetParams p;
    p.voxel = 0.005;
    const std::vector<double> layers{0.05, 0.06, 0.25, 0.45, 0.80};
    int fails = 0;
    for (const auto& car : route) {
      std::vector<Vec2> low, high;
      for (const double z : layers) {
        // 水平层：射线起点高 = 层高，俯角 0
        auto pts = scan(w, car, z, 0.0, 1440, p.min_range, p.max_range);
        const int band = (z <= p.pile_low_max) ? 0 : ((z >= p.pile_high_min) ? 1 : 2);
        if (band == 2) continue;   // 中间带不参与识别（和实车一致）
        // 高带（细杆）不做体素降采样：杆打点散布 < 2cm，0.04 体素会把整根杆并成 1 点
        if (band == 1) {
          for (const auto& q : pts) high.push_back(q);
          continue;
        }
        pts = voxel_filter(pts, p.voxel, 0);
        for (const auto& q : pts) low.push_back(q);
      }
      const auto r = run_detect(low, high, p);
      fails += check_piles(r, car);
    }
    if (fails == 0) std::printf("全部 %zu 个位置通过\n", route.size());
    else std::printf("失败 %d 处\n", fails);
    CHECK_TRUE(fails == 0);
  }

  // 场景 B：俯角多线几何（近似实车 Livox：雷达高 0.2m，仰角 45°~俯角 8°
  //          （负 = 向上，细杆顶 0.85m 在雷达上方，必须仰角才打得到），
  //          0.2°/线，voxel 0.04）
  std::printf("== 场景 B：Livox 俯角多线几何 ==\n");
  {
    DetParams p;
    std::vector<double> pitches;
    for (int i = 0; i < 16; ++i) pitches.push_back(-45.0 + 53.0 * i / 15);
    int fails = 0;
    for (const auto& car : route) {
      std::vector<Vec2> low, high;
      for (const double pth : pitches) {
        auto pts = scan(w, car, 0.20, pth, 1800, p.min_range, p.max_range);
        // 低带 = 打点高度 z(t) 在 0.15 以下、高带 = 0.35 以上。
        // 打点高度 = 0.2 - t*tan(pth)；点是车体系坐标，t = 到车的距离
        std::vector<Vec2> lo, hi;
        for (const auto& q : pts) {
          const double t = std::hypot(q.x, q.y);
          const double z = 0.20 - t * std::tan(pth * M_PI / 180.0);
          if (z <= p.pile_low_max) lo.push_back(q);
          else if (z >= p.pile_high_min) hi.push_back(q);
        }
        // 低带做体素降采样；高带（细杆）不做——杆 r=1cm，打点全在 2cm 圆内，
        // 0.04 体素会把一整根杆并成 1 个点，high_min_pts=2 永远过不了
        lo = voxel_filter(lo, p.voxel, 0);
        low.insert(low.end(), lo.begin(), lo.end());
        high.insert(high.end(), hi.begin(), hi.end());
      }
      const auto r = run_detect(low, high, p);
      fails += check_piles(r, car);
    }
    if (fails == 0) std::printf("全部 %zu 个位置通过\n", route.size());
    else std::printf("失败 %d 处\n", fails);
    CHECK_TRUE(fails == 0);
  }

  return navtest::summary("test_pile_structure");
}
