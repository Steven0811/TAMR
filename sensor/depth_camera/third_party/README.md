# depth_camera/third_party

| 資料夾 | 在 repo 裡？ | 用途 |
|---|---|---|
| `pyorbbecsdk/` | ✅ git submodule（[orbbec/pyorbbecsdk](https://github.com/orbbec/pyorbbecsdk) `v1.3.4`） | Python 綁定，內含 OrbbecSDK 1.10.16 的函式庫（`sdk/lib/<平台>`）。Docker 的 `camera` 映像檔用它編譯，**ROS 2 只需要這個** |
| `sdk/` | ❌ 需自行下載 | OrbbecSDK v1.10.37 完整 SDK（C/C++ 範例與工具） |
| `viewer/` | ❌ 需自行下載 | OrbbecViewer v1.10.37 GUI，`../run_orbbec_viewer.sh` 會用到 |

`sdk/` 和 `viewer/` 加起來約 180 MB，ROS 2 / Docker 用不到，所以沒有放進 git。
要用 OrbbecViewer 或 SDK 範例時，從 Orbbec 的 OrbbecSDK v1 GitHub Releases 下載
v1.10.37 的 Linux x64 版本，解壓縮到這裡：

```
third_party/sdk/OrbbecSDK_v1.10.37/
third_party/viewer/OrbbecViewer_v1.10.37_<日期>_linux_x64_release/
```

`run_orbbec_viewer.sh` 裡寫死了 viewer 的資料夾名稱，下載的版本不同時要跟著改。

## 本機的 Python 環境（gemini_viewer.py）

`gemini_viewer.py` 用的是本機的 venv（`../.venv`，沒有放進 git），裡面以 `.pth` 檔指向
`pyorbbecsdk/install/lib`。重建方式：

```bash
cd depth_camera/third_party/pyorbbecsdk
pip install pybind11==2.11.0
cmake -S . -B build -Dpybind11_DIR="$(python3 -m pybind11 --cmakedir)" && cmake --build build -j && cmake --install build
cd ../.. && python3 -m venv .venv && .venv/bin/pip install "numpy<2" opencv-python
echo "$PWD/third_party/pyorbbecsdk/install/lib" > .venv/lib/python3.10/site-packages/pyorbbecsdk_local.pth
```
