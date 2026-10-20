// T7：桩系定位测试 —— 不依赖里程计，用两桩当自然信标定位
#include <cmath>

#include "nav/pile_frame.hpp"
#include "nav/test_util.hpp"

int main() {
  // ---- 1) 车在桩系原点正下方 0.55m：下桩在车上方 0.55，上桩在更上方 ----
  // 两桩竖直排列（dx=0），桩间距 2m。桩系 x 轴 = 世界 y 方向。
  {
    const nav::Vec2 b_down{0.0, 0.55};
    const nav::Vec2 b_up{0.0, 2.55};
    const nav::Vec2 c = nav::car_in_pile_frame(b_down, b_up);
    // 车在下桩下方 0.55m → 桩系 x = -0.55, y = 0
    CHECK_NEAR(c.x, -0.55, 1e-9);
    CHECK_NEAR(c.y, 0.0, 1e-9);
  }

  // ---- 2) 车偏离两桩连线（下桩在车后方 0.3m，两桩竖直排列）----
  // 下桩 b_down=(-0.3, 0.55)：下桩在车体系"后 0.3、左 0.55"。
  // 桩系 x 轴 = 车体系 +y（左），y 轴 = 车体系 -x（后）。
  // 车相对下桩 = (0.3, -0.55)，沿桩系 y 轴（后）投影 = -0.3。
  {
    const nav::Vec2 b_down{-0.3, 0.55};
    const nav::Vec2 b_up{-0.3, 2.55};
    const nav::Vec2 c = nav::car_in_pile_frame(b_down, b_up);
    CHECK_NEAR(c.x, -0.55, 1e-9);
    CHECK_NEAR(c.y, -0.3, 1e-9);
  }

  // ---- 3) 往返一致性：pile_to_body 是 car_in_pile_frame 的逆 ----
  // 已知车在桩系位置 c，它对应车体系里的两桩位置。反过来 to_park 应得 0（车就在 p_park）。
  {
    const nav::Vec2 b_down{0.0, 0.55};
    const nav::Vec2 b_up{0.0, 2.55};
    const nav::Vec2 p_park = nav::car_in_pile_frame(b_down, b_up);
    // 车就在 p_park 处，车→停车位 应为 0
    const nav::Vec2 to = nav::to_park_body(b_down, b_up, p_park);
    CHECK_NEAR(to.x, 0.0, 1e-9);
    CHECK_NEAR(to.y, 0.0, 1e-9);
  }

  // ---- 4) 车离开停车位后，to_park_body 应指向回停车位的正确方向 ----
  // 停车位在桩系 (-0.55, 0)。车现在在桩系 (0.55, 0)（下桩正上方 0.55m）。
  // 车→停车位（桩系）= (-0.55-0.55, 0) = (-1.1, 0)。
  // 桩系 x 轴 = 车体系 (0,1)（向上），所以车→停车位 车体系 = (0, -1.1)。
  {
    const nav::Vec2 b_down{0.0, -0.55};   // 下桩在车下方 0.55m（车在桩上方）
    const nav::Vec2 b_up{0.0, 1.45};      // 上桩在车上方 1.45m
    const nav::Vec2 p_park{-0.55, 0.0};   // 停车位（桩系坐标）
    const nav::Vec2 to = nav::to_park_body(b_down, b_up, p_park);
    // 车在桩系 (0.55, 0)，停车位 (-0.55, 0)，方向沿桩系 -x = 车体系 (0,-1)，距离 1.1
    CHECK_NEAR(to.x, 0.0, 1e-9);
    CHECK_NEAR(to.y, -1.1, 1e-9);
  }

  // ---- 5) 两桩重合（退化）应返回 0，不出 NaN ----
  {
    const nav::Vec2 b_down{0.5, 0.5};
    const nav::Vec2 b_up{0.5, 0.5};
    const nav::Vec2 c = nav::car_in_pile_frame(b_down, b_up);
    CHECK_TRUE(std::isfinite(c.x) && std::isfinite(c.y));
    CHECK_NEAR(c.x, 0.0, 1e-9);
    CHECK_NEAR(c.y, 0.0, 1e-9);
  }

  return navtest::summary("test_pile_frame");
}