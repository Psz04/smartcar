# smart_car — 智能车图像工程

智能车校内赛的图像处理与控制工程，包含三部分：**车端控制程序**（龙芯 LoongArch 交叉编译）、**x86 上位机调试程序**、**Node.js 图传上位机**。

---

## 一、目录说明

| 目录 | 说明 |
| --- | --- |
| `校内赛图像控制代码-new/` | 车端主工程。C/C++ + CMake，依赖 OpenCV、ncnn、逐飞 ZF 库，交叉编译后上传龙芯板卡 |
| `上位机_调试部分_new/` | x86 上位机调试程序，用于摄像头驱动调试与图像算法验证（C++17 + OpenCV） |
| `图传上位机-new/` | 图传上位机，Node.js + WebSocket，浏览器打开 `web.html` 实时看图 |
| `ver/` | 图像处理算法迭代版本，`main_original.cpp` 为初始版，`v1.cpp` ~ `v3f.cpp` 为后续迭代 |
| `ref/`、`input/` | 裁判系统参考图与测试输入图 |
| `cmp_ref_lines.py`、`ref_info.py`、`ref_neighbor.py`、`ref_rowtable.py`、`ref_verify.py` | Python 侧的 referee 图像分析脚本 |

---

## 二、换机器前必读：路径是硬编码的

工程里多处写死了绝对路径，新机器的目录结构必须与原机一致，否则编译会找不到头文件或库：

- `校内赛图像控制代码-new/CMakeLists.txt` → `include_directories(/home/smartcar/boost)`
- `校内赛图像控制代码-new/cross.cmake` → `OpenCV_DIR = /usr/local/include/opencv/install_loongxin/lib/cmake/opencv4`
- `校内赛图像控制代码-new/cross.cmake` → 工具链 `/opt/loongson-gnu-toolchain-8.3-x86_64-loongarch64-linux-gnu-rc1.3-1/`
- `校内赛图像控制代码-new/build.sh` → 板卡 `board_IP="192.168.31.57"`、`remote_user="root"`、`remote_path="/home/root"`

**最省事的做法：新虚拟机的用户名也创建成 `smartcar`**，家目录就是 `/home/smartcar`，所有硬编码路径自动对上，一行代码都不用改。

如果用户名不同，先全局替换路径：

```bash
cd /path/to/smart_car
grep -rl "/home/smartcar" --include="*.txt" --include="*.cmake" --include="*.sh" .
# 确认无误后替换为新家目录
grep -rl "/home/smartcar" --include="*.txt" --include="*.cmake" --include="*.sh" . \
  | xargs sed -i 's#/home/smartcar#/home/<新用户名>#g'
```

---

## 三、环境依赖（Ubuntu 20.04 focal）

```bash
# 1. 基础编译工具
sudo apt update && sudo apt install -y build-essential cmake git

# 2. OpenCV（上位机直接用系统版即可）
sudo apt install -y libopencv-dev pkg-config

# 3. 龙芯交叉工具链（车端编译必需）
#    解压到 /opt/，保证路径为
#    /opt/loongson-gnu-toolchain-8.3-x86_64-loongarch64-linux-gnu-rc1.3-1/
#    版本不同需同步改 cross.cmake 与 build.sh 里的 TOOLCHAIN_DIR
#    车端还需要龙芯版 OpenCV：交叉编译后装到
#    /usr/local/include/opencv/install_loongxin/

# 4. Boost（工程以纯头文件方式引用）
#    把 boost 源码放到 /home/smartcar/boost 即可（CMakeLists 直接 include 该路径）
#    或用系统版：sudo apt install -y libboost-dev
#    ⚠️ 用系统版时需同步修改 CMakeLists.txt 的 include_directories

# 5. Node.js（图传上位机）
sudo apt install -y nodejs npm && sudo npm i -g pnpm
```

---

## 四、编译与运行

### 1) 车端程序：一键交叉编译并上传板卡

```bash
cd ~/校内赛图像控制代码-new
./build.sh
```

脚本流程：清理 `build/` → `cmake`（走 `cross.cmake`）→ `make -j24` → `scp` 可执行文件与 ncnn 库到板卡。
上传目标在 `build.sh` 顶部「核心配置区」修改：`board_IP` / `remote_user` / `remote_path` / `MAKE_JOBS`。
想改成 x86 本机编译：把 `cross.cmake` 里的 `CROSS_COMPILE` 设为 `0`。

### 2) 上位机调试程序

```bash
cd ~/上位机_调试部分_new
cmake -S . -B build -DCMAKE_BUILD_TYPE=Debug
cmake --build build -j$(nproc)
```

默认 Debug，便于 VSCode 打断点。可用 `-DIMAGE_CPP=... -DIMAGE_H=...` 指定从车端拷过来的 `image.cpp` / `image.h`。

### 3) 图传上位机

```bash
cd ~/图传上位机-new
pnpm install
pnpm start            # 等价于 node run.js
# 浏览器打开 web.html 查看实时图像；ffmpeg.sh 用于推/拉流
```

---

## 五、未纳入仓库的大文件（clone 后需自备）

以下内容已通过 `.gitignore` 排除，避免超出 GitHub 的单文件 100 MB 限制：

| 排除项 | 体积 | 如何恢复 |
| --- | --- | --- |
| `boost/`、`boost.zip` | 843 MB / 192 MB | 自行下载 Boost 源码，放到 `/home/smartcar/boost` |
| `vidframes/` | 152 MB | 视频拆帧中间产物，可重新生成 |
| `图传上位机-new/node_modules/` | 46 MB | `pnpm install` |
| `build/`、`build_stale_20260928/` | — | 编译产物 |
| `output/`、`output_new/`、`output_old_submission/`、`refmatch/` | — | 运行结果，可重新生成 |
| `*.mp4` | 37 MB | 演示视频，仅本地留存 |
| `snap/`、`.cache/`、`.config/` | — | 系统与编辑器缓存 |

> 注意：`校内赛图像控制代码-new/ZF/zf_components/lqncnn/` 下的 `libncnn.so`（约 3 MB ×3）是**已随仓库提交**的，无需另外准备。

---

## 六、日常同步

```bash
git pull --rebase
# ...修改代码...
git add .
git commit -m "说明本次改动"
git push
```
