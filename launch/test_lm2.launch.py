# 导入ROS2 Launch核心依赖
from launch import LaunchDescription
from launch.actions import ExecuteProcess
import os

# 导入包路径工具
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # -------------------------- 路径配置（关键：补全可执行文件路径） --------------------------
    lightning_share_dir = get_package_share_directory('lightning')
    executable_path = os.path.join(
        lightning_share_dir.replace('share', 'lib'),  # share → lib
        'run_slam_online'
    )
    config_file = os.path.join(lightning_share_dir, 'config', 'default_livox.yaml')

    # 调试：打印路径
    print(f"Executable path: {executable_path}")
    print(f"Config file path: {config_file}")

    # 验证文件存在
    if not os.path.exists(executable_path):
        raise FileNotFoundError(f"可执行文件不存在: {executable_path}")
    if not os.path.exists(config_file):
        raise FileNotFoundError(f"配置文件不存在: {config_file}")

    # -------------------------- 进程配置（无额外ROS2参数） --------------------------
    slam_online_process = ExecuteProcess(
        cmd=[executable_path, '--config', config_file],
        output='log',          # 日志输出到终端
        emulate_tty=False,         # 优化日志格式
        name='run_slam_online'
    )

    ld = LaunchDescription()
    ld.add_action(slam_online_process)

    return ld