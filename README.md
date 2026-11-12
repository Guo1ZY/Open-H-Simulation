# Open-H：双 UR5 内窥镜 MuJoCo 仿真


## 1. 安装并启动

从仓库页面点击 **Code → Download ZIP**，完整解压后，在包含本 README 和 `launch.py` 的目录打开终端。不要直接在压缩包内运行。

推荐 **Python 3.12**

### macOS / Linux

```bash
python3.12 -m venv .venv
./.venv/bin/python -m pip install --upgrade pip
./.venv/bin/python -m pip install -r requirements.txt
./.venv/bin/python launch.py
```

### Windows（PowerShell 或命令提示符）

```powershell
py -3.12 -m venv .venv
.\.venv\Scripts\python.exe -m pip install --upgrade pip
.\.venv\Scripts\python.exe -m pip install -r requirements.txt
.\.venv\Scripts\python.exe launch.py
```

## 2. 键盘与视角

先点击 MuJoCo 窗口，让它获得键盘焦点。默认夹爪闭合。


| 操作                 | 功能                                          |
| -------------------- | --------------------------------------------- |
| 按住空格 + W / S     | 第二只臂带着夹具前进 / 后退                   |
| 按住空格 + 方向键    | 控制内窥镜尖端转向                            |
| G                    | 夹紧 / 松开夹爪                               |
| [ / ]                | 减小 / 增大夹爪闭合位置目标，从而改变接触压力 |
| C                    | 右上角依次切换肠道、夹具、双电机手柄近景      |
| 左侧画面鼠标左键拖动 | 旋转总览视角                                  |
| 左侧画面鼠标右键拖动 | 平移总览视角                                  |
| 左侧画面滚轮         | 缩放                                          |
| Esc 或关闭窗口       | 退出并结束本次记录                            |


## 3. 采集内镜图像与状态

例如采集 60 秒仿真时间的图像和状态：

macOS / Linux：

```bash
./.venv/bin/python launch.py --mode keyboard --seconds 60
```

Windows：

```powershell
.\.venv\Scripts\python.exe launch.py --mode keyboard --seconds 60
```

添加 `--no-video` 可以关闭图像保存，保留状态 CSV。图像为 **640 × 480 PNG 序列**，每仿真秒 30 帧；状态采样为每仿真秒 50 次。图像保存会增加计算和磁盘开销。

每次运行输出到 `sim/recordings_native/日期_时间/`：


| 文件                   | 内容                                                                     |
| ---------------------- | ------------------------------------------------------------------------ |
| `states.csv`           | 输入、仿真时间、实际耗时、机械臂关节、夹爪位置、接触力、滑动、尖端位置等 |
| `metadata.json`        | 本次运行模式及模型说明                                                   |
| `frames/000000.png` 等 | 启用图像保存时生成的内镜帧                                               |

`frame_index` 对应最近保存的图像编号；配合 `sim_time_s` 对齐图像和状态。未保存图像时该值为 −1。帧率和采样率均按**仿真时间**计算，不保证实际运行达到实时速度。

也可以将 `--mode keyboard` 改为 `--mode xbox`：LB 按住使能、左摇杆转向、RT / LT 推进与回拉。当前手柄映射**未用实体 Xbox 手柄验证**，夹爪开合仍通过键盘 G 操作。
