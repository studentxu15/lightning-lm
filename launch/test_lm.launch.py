# 导入ROS2 Launch核心依赖（兼容所有版本）
from launch import LaunchDescription
from launch.actions import ExecuteProcess
import os

# 导入包路径工具（规范路径）
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # -------------------------- 路径配置 --------------------------
    lightning_share_dir = get_package_share_directory('lightning')
    config_file = os.path.join(lightning_share_dir, 'config', 'default_livox.yaml')
    executable_path = os.path.join(
        lightning_share_dir.replace('share', 'lib'),  # share -> lib（ROS2安装规则）
        'run_slam_online'
        # 'run_loc_online'
    )

    # 调试：打印路径（可选，确认路径正确）
    print(f"Executable path: {executable_path}")
    print(f"Config file path: {config_file}")

    # 验证文件存在（提前报错，便于调试）
    if not os.path.exists(executable_path):
        raise FileNotFoundError(f"Executable not found: {executable_path}")
    if not os.path.exists(config_file):
        raise FileNotFoundError(f"Config file not found: {config_file}")

    # -------------------------- 进程配置 --------------------------
    slam_online_process = ExecuteProcess(
        cmd=[
            executable_path,
            '--config', config_file
        ],
        output='log',          # 终端输出日志
        emulate_tty=True,         # 优化输出格式（解决日志乱码/不实时问题）
    )

    # -------------------------- 组装Launch描述 --------------------------
    ld = LaunchDescription()
    ld.add_action(slam_online_process)

    return ld