# 樹莓派部署指南（給樹莓派上的 Claude Code）

這份文件說明如何把這個專案從 Windows／WSL2 筆電搬到樹莓派上執行。請依序完成每一節，
每一節最後都有「驗證」步驟，**驗證沒通過就先停下來回報使用者，不要跳過**。

## 0. 背景與工作原則

- 專案把所有感測器包在 Docker 容器裡，用 ROS 2 Humble 傳資料，再透過 `foxglove_bridge`
  （`ws://<樹莓派 IP>:8765`）讓筆電上的 Foxglove 顯示。架構說明見 [README.md](README.md)。
- **主機（樹莓派）不需要安裝 ROS 2**：ROS 全部在容器裡。主機只需要 Docker、USB 驅動和幾個
  小工具。
- 原本的環境是 WSL2：USB 裝置要靠 `usbipd` 轉接進 WSL。樹莓派是原生 Linux，**這些步驟都不需要**，
  README 裡提到 `usbipd` 和 WSLg 的地方可以忽略。
- 需要 `sudo` 的指令，請先向使用者說明用途並取得同意再執行。
- 這套系統在筆電（x86_64）上驗證過：IMU、深度相機、光達、測距儀、遙控接收機都能正常發布資料。
  **在樹莓派（arm64）上還沒有人跑過**，第 9 節列出預期可能出問題的地方。

## 1. 硬體清單與 USB 識別

| 裝置 | USB 晶片（VID:PID） | Linux 驅動 | 裝置節點 | 備註 |
|---|---|---|---|---|
| Yahboom 10 軸 IMU | CP2102（`10c4:ea60`） | `cp210x` | `/dev/ttyUSB*` | 9600 baud，WitMotion 協定 |
| YDLidar | CP2102（`10c4:ea60`） | `cp210x` | `/dev/ttyUSB*` | **跟 IMU 用同一款晶片，序號也都是 `0001`**，無法用 by-id 區分 |
| Orbbec Gemini 深度相機 | `2bc5:0614`（深度）＋ `2bc5:0511`（彩色） | libusb（不需要核心驅動） | `/dev/bus/usb/...` | 需要 USB 3.0 |
| NodeMCU-32S（測距儀＋遙控接收機） | CH340（`1a86:7523`） | `ch341` | `/dev/ttyUSB*` | micro-ROS，115200 baud |
| DDSM Driver HAT (A)（輪子） | CH343（`1a86:55d3`） | `cdc_acm` | `/dev/ttyACM*` | 用 HAT 上標示 **ESP32-USB** 的 Type-C 孔 |

`start.sh` 會自動辨認這些裝置：
- NodeMCU 和 HAT 靠 VID:PID 辨認。
- IMU 和光達用的是同一款晶片，所以 `start.sh` 會打開每個序列埠，看哪一個在送 IMU 封包（`0x55` 開頭）；剩下的就是光達。

## 2. 系統需求

- **64 位元作業系統（arm64／aarch64）是必要條件**：Orbbec SDK 只附 arm64 的函式庫，ROS 的
  Docker 映像檔也不支援 32 位元。用 `uname -m` 檢查，應該要顯示 `aarch64`。
- 建議的作業系統：
  - Raspberry Pi 5：Raspberry Pi OS（Bookworm）64-bit，或 Ubuntu 24.04 arm64。
  - Raspberry Pi 4：以上兩種都可以，Ubuntu 22.04 arm64 也行。
- 儲存空間：至少 **32 GB**，建議用 SSD。Docker 映像檔加起來約 6–10 GB，編譯時還需要額外空間。
- 電源：
  - Pi 5 用官方 27 W（5 V／5 A）電源；Pi 4 用 3 A 電源。
  - 光達、深度相機、IMU、NodeMCU 同時接，USB 耗電可能超過樹莓派的供電能力，**建議用有外接電源的 USB Hub**。
- **深度相機要接在藍色的 USB 3.0 孔**（或 USB 3.0 Hub 上）。

驗證：
```bash
uname -m                 # aarch64
cat /etc/os-release | head -3
df -h /                  # 剩餘空間 > 20 GB
```

## 3. 主機套件

### 3.1 apt 套件

```bash
sudo apt update
sudo apt install -y git curl rsync usbutils udev python3 python3-serial
```

| 套件 | 用途 |
|---|---|
| `git`、`curl`、`rsync` | 下載安裝腳本、搬移專案 |
| `usbutils` | 提供 `lsusb`，`start.sh` 用它偵測深度相機 |
| `udev` | 提供 `udevadm`，`start.sh` 用它讀 USB 的 VID:PID |
| `python3`、`python3-serial` | `start.sh` 在主機上探測哪個序列埠是 IMU |

`python3-venv`、`cmake` 只有要在樹莓派上用 PlatformIO 燒錄 ESP32 時才需要，見第 8 節。

### 3.2 Docker Engine 與 Compose

用 Docker 官方的安裝腳本，Raspberry Pi OS 和 Ubuntu 都適用，會一併裝好 `docker compose` 外掛：
```bash
curl -fsSL https://get.docker.com -o /tmp/get-docker.sh
sudo sh /tmp/get-docker.sh
sudo systemctl enable --now docker
```

### 3.3 使用者群組

```bash
sudo usermod -aG docker,dialout,video,plugdev $USER
```
- `docker`：不用 sudo 就能執行 docker。
- `dialout`：讀寫 `/dev/ttyUSB*`、`/dev/ttyACM*`。`start.sh` 在主機上直接探測序列埠，所以一定要有這個權限。
- `video`、`plugdev`：深度相機的 udev 規則把裝置歸給 `video` 群組。

**加完群組後要登出再登入（或重開機）才會生效。**

驗證：
```bash
groups                   # 應該包含 docker dialout video
docker run --rm hello-world
docker compose version
```

## 4. USB 驅動與 udev 規則

### 4.1 序列埠驅動

`cp210x`、`ch341`、`cdc_acm` 在 Raspberry Pi OS 和 Ubuntu 的核心都有內建，通常不用另外安裝。
**把裝置都接上之後**檢查：
```bash
lsusb                                   # 應該看到第 1 節表格裡的 VID:PID
ls -l /dev/ttyUSB* /dev/ttyACM*         # 群組應該是 dialout
for d in /dev/ttyUSB* /dev/ttyACM*; do
  echo "$d: $(udevadm info -q property -n $d | grep -E 'ID_VENDOR_ID|ID_MODEL_ID' | tr '\n' ' ')"
done
lsmod | grep -E 'cp210x|ch341|cdc_acm'
```
如果某個裝置在 `lsusb` 裡看得到、卻沒有出現對應的 `/dev/tty*`，就用 `sudo modprobe cp210x`
（或 `ch341`、`cdc_acm`）手動載入，再看 `dmesg | tail` 有沒有錯誤訊息。

**注意**：Ubuntu 上的 `brltty`（點字機服務）可能會搶走 CH340 裝置，讓 `/dev/ttyUSB*` 一出現就消失。
`dmesg` 裡如果看到 `brltty`，就執行 `sudo apt remove brltty`。

### 4.2 深度相機的 udev 規則

容器以 privileged 模式執行，理論上不需要這條規則。不過裝上之後，主機上的工具也能直接存取相機，
權限問題比較少：
```bash
sudo cp depth_camera/third_party/pyorbbecsdk/99-obsensor-libusb.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules && sudo udevadm trigger
```
這份規則已經包含這台相機用到的 `2bc5:0614` 和 `2bc5:0511`。

## 5. 把專案搬到樹莓派

這個專案是 GitHub repo [Steven0811/TAMR](https://github.com/Steven0811/TAMR) 裡的 `sensor/` 資料夾。
YDLidar 驅動、YDLidar SDK、pyorbbecsdk 是 git submodule，**clone 時一定要加 `--recurse-submodules`**，
否則這三個資料夾會是空的，Docker 映像檔建置會失敗：

```bash
cd ~
git clone --recurse-submodules git@github.com:Steven0811/TAMR.git
# 用 HTTPS 也可以：git clone --recurse-submodules https://github.com/Steven0811/TAMR.git
cd ~/TAMR/sensor
cp .env.example .env        # 遙控器中位值（RC_THROTTLE_CENTER=1478、RC_STEERING_CENTER=1488）
```

驗證：
```bash
git -C ~/TAMR submodule status          # 三行，開頭不能是 "-"（"-" 代表沒有抓下來）
ls ~/TAMR/sensor/ydlidar/YDLidar-SDK/CMakeLists.txt
```

以下各節的指令都在 `~/TAMR/sensor` 底下執行。repo 不包含 Orbbec 的 SDK 和 Viewer 執行檔，
也不包含任何 x86 的編譯產物；ROS 2 和 Docker 用不到它們，見 `depth_camera/third_party/README.md`。

## 6. 為樹莓派修改專案

### 6.1 光達映像檔改用較小的基底（建議）

[ydlidar/Dockerfile](ydlidar/Dockerfile) 用的是 `osrf/ros:humble-desktop`，內含 RViz 等 GUI 套件，
大約 5 GB，在樹莓派上下載和建置都很慢。主 compose 用的是 `ydlidar_launch.py`，只會啟動驅動程式和
`tf2_ros` 的 static_transform_publisher，**不需要 GUI**。請把第一行改成：
```dockerfile
FROM ros:humble-ros-base
```
然後確認 `ydlidar/Dockerfile` 裡用到的 `python3-colcon-common-extensions`、`cmake`、`pkg-config`
都還在 apt install 清單裡。`ros-base` 已經內含 `tf2_ros`。

`ydlidar/docker-compose.yml`（光達單獨執行、搭配 RViz 的舊設定）裡有 WSLg 專用的環境變數，
在樹莓派上用不到，也不會被主 compose 使用，可以不用管。

### 6.2 確認映像檔都有 arm64 版本

```bash
for img in ros:humble-ros-base microros/micro-ros-agent:humble; do
  echo "$img: $(docker manifest inspect $img | grep -o '"architecture": *"[a-z0-9]*"' | sort -u | tr '\n' ' ')"
done
```
每個映像檔都要列出 `arm64`。如果 `microros/micro-ros-agent:humble` 沒有 arm64 版本，
就改在 `docker/Dockerfile` 裡用 ROS 官方套件 `ros-humble-micro-ros-agent` 自己建一個 agent 映像檔，並先跟使用者確認。

### 6.3 深度相機映像檔

[docker/Dockerfile](docker/Dockerfile) 的 `camera` target 會從原始碼編譯 pyorbbecsdk。它的 CMake
會用 `uname -m` 判斷平台，在樹莓派上會自動選 `sdk/lib/arm64`，**理論上不用修改**。
編譯時間會比筆電長很多。

## 7. 建置與啟動

```bash
cd ~/TAMR/sensor
docker compose build          # 第一次會花比較久（樹莓派上可能要 20–60 分鐘）
./start.sh -d                 # 偵測裝置並在背景啟動
```

`start.sh` 的輸出應該類似這樣（只會列出有接上的裝置）：
```
IMU   : /dev/ttyUSB1
Lidar : /dev/ttyUSB0 (model: ydlidar)
Camera: found
ESP32 : /dev/ttyUSB2 (micro-ROS range finder + RC receiver)
DDSM  : /dev/ttyACM0 (DDSM Driver HAT, wheels 1)
Starting: foxglove imu ydlidar camera micro_ros_agent range_tf rc_teleop ddsm115 twist_mux
```

驗證（每個 topic 都要有資料）：
```bash
docker compose ps                         # 全部都是 Up，不能一直 Restarting
docker exec foxglove_bridge bash -c 'source /opt/ros/humble/setup.bash; ros2 topic list'
docker exec foxglove_bridge bash -c 'source /opt/ros/humble/setup.bash; timeout 6 ros2 topic hz /imu/data'
```

| Topic | 筆電上量到的頻率 |
|---|---|
| `/imu/data` | 約 10 Hz |
| `/scan` | 約 10 Hz |
| `/camera/color/image_raw/compressed`、`/camera/depth/image_raw` | 約 30 Hz（樹莓派上可能較低） |
| `/camera/depth/points` | 約 8–10 Hz（只在有人訂閱時發布） |
| `/rc/pulses` | 約 50 Hz；遙控器開著、搖桿放開時兩個數值約 1480–1490 |
| `/range_finder/range` | 約 20 Hz |

最後請使用者在筆電的 Foxglove 連到 `ws://<樹莓派 IP>:8765`，再匯入
[docker/foxglove_layout.json](docker/foxglove_layout.json)。樹莓派和筆電要在同一個網段，
而且樹莓派的防火牆（如果有開）要放行 TCP 8765。

**輪子**：第一次在樹莓派上測試 DDSM115 時，請提醒使用者**把輪子架空**。速度上限是 60 rpm（`DDSM_MAX_RPM`）。

## 8.（選用）在樹莓派上燒錄 NodeMCU 韌體

韌體也可以繼續在筆電上燒錄。如果要在樹莓派上燒錄：

```bash
sudo apt install -y python3-venv cmake git      # micro-ROS 函式庫建置時需要
curl -fsSL -o /tmp/get-platformio.py https://raw.githubusercontent.com/platformio/platformio-core-installer/master/get-platformio.py
python3 /tmp/get-platformio.py
~/.platformio/penv/bin/pio --version

cd ~/TAMR/sensor
docker compose stop micro_ros_agent              # agent 會占用序列埠
~/.platformio/penv/bin/pio run -e car_io -t upload
docker compose start micro_ros_agent
```
第一次編譯 `car_io` 會下載 ESP32 工具鏈並建置 micro-ROS，在樹莓派上可能要 10–20 分鐘。
各個 environment 的用途見 README 的「ESP32 firmware」一節。

## 9. 預期可能遇到的問題

| 現象 | 可能原因與處理 |
|---|---|
| Pi 5 上某個容器一啟動就 segfault，或出現 `Unsupported system page size`、jemalloc 錯誤 | Raspberry Pi OS 在 Pi 5 上預設用 16K page 的核心，有些 arm64 預編譯程式（例如 Orbbec SDK）只支援 4K page。在 `/boot/firmware/config.txt` 加入 `kernel=kernel8.img`，重開機後用 `getconf PAGESIZE` 確認是 4096 |
| 深度相機 `failed to open camera`，或 log 裡有大量 `usb_request_cancel` | 沒接在 USB 3.0 孔、USB 供電不足，或線材品質不好。先單獨接相機測試。`docker compose logs camera` 裡第一個 `[orbbec_camera]` 開頭的錯誤，才是真正的原因 |
| `start.sh` 把 IMU 和光達的序列埠對調 | 用 `.env` 指定 `IMU_PORT=`、`LIDAR_PORT=`。**不要用 `/dev/serial/by-id`**，兩者的 by-id 名稱一樣 |
| `start.sh` 出現 `cannot probe /dev/ttyUSBx: Permission denied` | 使用者還沒加入 `dialout` 群組，或加入後還沒重新登入 |
| 光達 log 一直出現 `Checksum error`、`Failed to start` | 光達型號的參數檔不對。在 `.env` 設定 `LIDAR_MODEL=`，值要是 `ydlidar/ydlidar_ros2_driver/params/` 裡的檔名（例如 `X4`、`G4`、`Tmini`）。先問使用者光達型號 |
| `/rc/pulses` 全是 0 | 遙控器沒開或沒對頻（接收機的綠燈要恆亮），或訊號線沒接在 GPIO34／35。可以燒 `rc_test` 或 `pin_scan` 診斷（見 README） |
| 筆電的 Foxglove 連不上 | 確認 `docker compose ps` 裡有 `foxglove_bridge`、樹莓派 IP 正確、防火牆放行 8765。也可以在樹莓派上先用 `curl -i http://localhost:8765` 測試 |
| 整體很卡、相機掉幀 | 調高 `.env` 裡的 `CLOUD_STRIDE`（例如 4），或調低 `CLOUD_RATE`，降低點雲的負擔 |

## 10. 完成後回報給使用者

請整理一份簡短的結果給使用者：
- 作業系統、樹莓派型號、`uname -m`、`getconf PAGESIZE`。
- 每個裝置是否偵測到、對應到哪個 `/dev/*`。
- 第 7 節表格裡每個 topic 實際量到的頻率。
- 為了樹莓派做過哪些修改（檔案和原因），以及還沒解決的問題。
