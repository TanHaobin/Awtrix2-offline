# AWTRIX2 Offline — 8x32 LED 像素时钟

基于 AWTRIX 2.0 Controller 改造的**离线像素时钟**，运行在 ESP8266 (D1 Mini) + 8x32 WS2812B LED 矩阵上。无需服务器，开机即用。

## 功能

### 三个 App，自动轮播
| App | 说明 |
|-----|------|
| **时钟** | HH:MM:SS 大数字显示，冒号闪烁，底部星期指示器 |
| **日历** | 左侧红底白字日期，右侧年月星期滚动 |
| **眼睛** | 可爱的眨眼左右看动画 |

每 10 秒自动切换，也可按钮手动切换。

### 按钮控制（三键：左 / 中 / 右）
| 操作 | 功能 |
|------|------|
| 短按左/右 | 切换 App |
| 长按左 | 增加亮度（每 2 秒一档） |
| 长按右 | 降低亮度（每 2 秒一档） |
| 长按中（3 秒） | 进入 WiFi 配置模式（AP 热点） |

### WiFi & 时间同步
- 开机自动扫描附近**开放（无密码）WiFi** 并连接，优先选择 `Tencent-GuestWiFi`
- 时间同步：NTP 优先（`ntp.tencent.com` / `ntp.aliyun.com`），失败时自动通过 **HTTP Date 头**获取时间（兼容 Captive Portal 网络）
- 无 WiFi 时进入离线模式，显示 `--:--:--`

### 显示优化
- `FASTLED_ALLOW_INTERRUPTS=0` 禁用中断干扰
- `FastLED.setDither(0)` 禁用时序抖动
- 双次 `FastLED.show()` 消除首像素闪烁
- WS2812 时序驱动，兼容 WS2812B / SK6812 面板

## 硬件

| 组件 | 型号 |
|------|------|
| 主控 | ESP8266 D1 Mini |
| LED 矩阵 | 8x32 WS2812B（数据线 D2） |
| 按钮 | 3 个（D0=右键, D4=中键, D8=左键） |
| 可选 | BME280/HTU21D/BMP280 温湿度传感器、APDS9960 手势传感器、LDR 光敏电阻、DFPlayer Mini 音频模块 |

### 接线

```
ESP8266 D2  → LED 矩阵 DIN
ESP8266 D0  → 右按钮
ESP8266 D4  → 中按钮
ESP8266 D8  → 左按钮
ESP8266 5V  → LED 矩阵 VCC
ESP8266 GND → LED 矩阵 GND
```

> **提示**：如果左上角第一个 LED 偶尔闪烁，在 5V 电源输入端并联一个 1000μF 电解电容可彻底解决。

## 编译 & 烧录

需要 [PlatformIO](https://platformio.org/)：

```bash
# 编译
pio run --environment nodemcuv2

# 烧录
pio run --target upload --environment nodemcuv2

# 串口监控
pio device monitor --baud 115200
```

## 致谢

基于 [AWTRIX 2.0 Controller](https://github.com/blueforcer/awtrix2.0-Controller) by Blueforcer & Mazze2000。

## License

[MIT](LICENSE)
