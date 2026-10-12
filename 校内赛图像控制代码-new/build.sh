#!/bin/bash
# 脚本功能：清理build目录 -> cmake编译 -> make编译 -> 上传可执行文件与ncnn库到远程板卡
# 使用方法：在工程根目录执行 ./build.sh

export PATH=/opt/loongson-gnu-toolchain-8.3-x86_64-loongarch64-linux-gnu-rc1.3-1/bin:$PATH

# ===================== 核心配置区（按需修改） =====================
WORK_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"  # 脚本所在绝对路径（工程根目录）
OUT_DIR="${WORK_DIR}/build"                               # 编译输出目录
USER_DIR="${WORK_DIR}"                                    # CMakeLists.txt 所在目录
source_name="project"                                     # 生成的可执行文件名（对应 CMake 的 project() 名）
target_name="project"                                     # 上传到板卡后的文件名
board_IP="192.168.31.57"                                  # 板卡IP
remote_user="root"                                        # 板卡登录用户
remote_path="/home/root"                                  # 板卡上传路径
MAKE_JOBS=24                                              # make编译线程数
NCNN_LIB_DIR="${WORK_DIR}/ZF/zf_components/lqncnn/ncnn_install/lib"  # ncnn库文件路径

# ===================== 通用函数 =====================
error_exit() {
    echo -e "\033[31m[ERROR] $1\033[0m"
    exit 1
}

info_echo() {
    echo -e "\033[32m[INFO] $1\033[0m"
}

# ===================== 主逻辑 =====================
# 1. 进入build目录，目录不存在则自动创建
info_echo "准备进入目录: ${OUT_DIR}"
mkdir -p "${OUT_DIR}" || error_exit "无法创建 ${OUT_DIR} 目录，请检查路径！"
cd "${OUT_DIR}" || error_exit "无法进入 ${OUT_DIR} 目录，请检查目录是否存在！"

# 2. 清理build目录下所有内容，保证干净编译
info_echo "开始清理当前目录"
find . -mindepth 1 -exec rm -rf {} + || error_exit "目录清理失败，请检查目录权限！"

# 3. 执行cmake，生成Makefile
info_echo "执行cmake: cmake ${USER_DIR}"
cmake "${USER_DIR}" || error_exit "cmake 编译失败，请检查CMakeLists.txt或编译依赖！"

# 4. 执行make多线程编译
info_echo "cmake执行成功，开始执行 make -j${MAKE_JOBS} 编译项目..."
make -j${MAKE_JOBS} || error_exit "make 编译失败，编译日志如上！"

# 5. 校验可执行文件是否生成
[ -f "${source_name}" ] || error_exit "未找到可执行文件 ${source_name}！"
info_echo "待上传文件：${source_name}"

# 6. 上传可执行文件到板卡
info_echo "开始上传文件到 ${remote_user}@${board_IP}:${remote_path}"
scp "${source_name}" "${remote_user}@${board_IP}:${remote_path}/${target_name}" || error_exit "文件上传失败，请检查网络/远程权限/文件是否存在！"

# 7. 上传ncnn共享库并创建软链接（程序运行依赖）
info_echo "上传ncnn共享库到板卡..."
scp "${NCNN_LIB_DIR}/libncnn.so.1.0.20260219" "${remote_user}@${board_IP}:${remote_path}/" || error_exit "ncnn库上传失败！"
ssh "${remote_user}@${board_IP}" "cd ${remote_path} && ln -sf libncnn.so.1.0.20260219 libncnn.so.1 && ln -sf libncnn.so.1 libncnn.so && sync" || error_exit "ncnn软链接创建失败！"

info_echo "所有操作执行完成：清理目录 -> cmake编译 -> make编译 -> 文件上传 均成功！"
exit 0
