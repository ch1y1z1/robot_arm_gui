import numpy as np
import matplotlib.pyplot as plt

# 定义机械臂参数（单位：mm和度）
L1 = 213  # 基座到第一关节的高度
L2 = 150  # 第一关节到第二关节的长度
L3 = 180  # 第二关节到末端执行器的长度
L4 = 0  # 水平偏移量
L5 = 0  # 水平偏移量
# 初始角度（单位：度），内部计算时会转换为弧度
A1_START_ANGLE_DEG = 0
A2_START_ANGLE_DEG = 82.3
A3_START_ANGLE_DEG = 27.3
# 转换为弧度
A1_START_ANGLE = np.radians(A1_START_ANGLE_DEG)
A2_START_ANGLE = np.radians(A2_START_ANGLE_DEG)
A3_START_ANGLE = np.radians(A3_START_ANGLE_DEG)


def joint_angle_to_descartes(theta1, theta2, theta3):
    """
    关节角度转笛卡尔坐标

    参数:
        theta1, theta2, theta3: 三个关节的角度（弧度）

    返回:
        各关节位置的坐标数组
    """
    # 计算实际关节坐标（弧度）
    a1_coord = A1_START_ANGLE + theta1
    a2_coord = A2_START_ANGLE + theta2
    a3_coord = A3_START_ANGLE + theta3 - a2_coord

    # 基座位置
    base_pos = np.array([0, 0, 0])

    # A1关节位置
    a1_pos = np.array([0, 0, L1])

    # A2关节位置
    a2_x = (L5 + L4) * np.cos(a1_coord)
    a2_y = (L5 + L4) * np.sin(a1_coord)
    a2_z = L1
    a2_pos = np.array([a2_x, a2_y, a2_z])

    # A3关节位置
    a3_x = ((L5 + L4) + L2 * np.cos(a2_coord)) * np.cos(a1_coord)
    a3_y = ((L5 + L4) + L2 * np.cos(a2_coord)) * np.sin(a1_coord)
    a3_z = L1 + L2 * np.sin(a2_coord)
    a3_pos = np.array([a3_x, a3_y, a3_z])

    # 末端执行器位置
    end_x = (
        (L5 + L4) + L2 * np.cos(a2_coord) + L3 * np.cos(a2_coord + a3_coord)
    ) * np.cos(a1_coord)
    end_y = (
        (L5 + L4) + L2 * np.cos(a2_coord) + L3 * np.cos(a2_coord + a3_coord)
    ) * np.sin(a1_coord)
    end_z = L1 + L2 * np.sin(a2_coord) + L3 * np.sin(a2_coord + a3_coord)
    end_pos = np.array([end_x, end_y, end_z])

    return base_pos, a1_pos, a2_pos, a3_pos, end_pos


def visualize_robot_arm(theta1, theta2, theta3, ax):
    """
    可视化机械臂，并添加末端执行器到三个基面的投影

    参数:
        theta1, theta2, theta3: 三个关节的角度（弧度）
        ax: matplotlib 3D axes 对象

    返回:
        None (直接在提供的 axes 对象上绘图)
    """
    # 计算各关节位置
    base_pos, a1_pos, a2_pos, a3_pos, end_pos = joint_angle_to_descartes(
        theta1, theta2, theta3
    )

    # 绘制机械臂连杆
    # 基座到A1
    ax.plot(
        [base_pos[0], a1_pos[0]],
        [base_pos[1], a1_pos[1]],
        [base_pos[2], a1_pos[2]],
        "k-",
        linewidth=3,
    )
    # A1到A2
    ax.plot(
        [a1_pos[0], a2_pos[0]],
        [a1_pos[1], a2_pos[1]],
        [a1_pos[2], a2_pos[2]],
        "r-",
        linewidth=3,
    )
    # A2到A3
    ax.plot(
        [a2_pos[0], a3_pos[0]],
        [a2_pos[1], a3_pos[1]],
        [a2_pos[2], a3_pos[2]],
        "g-",
        linewidth=3,
    )
    # A3到末端
    ax.plot(
        [a3_pos[0], end_pos[0]],
        [a3_pos[1], end_pos[1]],
        [a3_pos[2], end_pos[2]],
        "b-",
        linewidth=3,
    )

    # 绘制关节点
    ax.scatter(
        [base_pos[0]], [base_pos[1]], [base_pos[2]], color="k", s=100, label="Base"
    )
    ax.scatter(
        [a1_pos[0]], [a1_pos[1]], [a1_pos[2]], color="r", s=100, label="Joint A1"
    )
    ax.scatter(
        [a2_pos[0]], [a2_pos[1]], [a2_pos[2]], color="g", s=100, label="Joint A2"
    )
    ax.scatter(
        [a3_pos[0]], [a3_pos[1]], [a3_pos[2]], color="b", s=100, label="Joint A3"
    )
    ax.scatter(
        [end_pos[0]], [end_pos[1]], [end_pos[2]], color="m", s=100, label="End Effector"
    )

    # 绘制末端执行器的投影点
    ax.scatter(
        end_pos[0],
        end_pos[1],
        0,
        color="gray",
        s=50,
        label="End Effector Projection on XY",
    )  # XY平面
    ax.scatter(
        end_pos[0],
        250,
        end_pos[2],
        color="gray",
        s=50,
        label="End Effector Projection on XZ",
    )  # XZ平面
    ax.scatter(
        0,
        end_pos[1],
        end_pos[2],
        color="gray",
        s=50,
        label="End Effector Projection on YZ",
    )  # YZ平面

    # 绘制末端执行器的投影线
    ax.plot(
        [end_pos[0], end_pos[0]],
        [end_pos[1], end_pos[1]],
        [end_pos[2], 0],
        color="gray",
        linestyle="--",
    )  # XY平面
    ax.plot(
        [end_pos[0], end_pos[0]],
        [end_pos[1], 250],
        [end_pos[2], end_pos[2]],
        color="gray",
        linestyle="--",
    )  # XZ平面
    ax.plot(
        [end_pos[0], 0],
        [end_pos[1], end_pos[1]],
        [end_pos[2], end_pos[2]],
        color="gray",
        linestyle="--",
    )  # YZ平面

    # 设置标签
    ax.set_xlabel("X Axis")
    ax.set_ylabel("Y Axis")
    ax.set_zlabel("Z Axis")
    ax.set_title(
        f"Robot Arm Visualization\nA1: {np.degrees(theta1):.2f}°, A2: {np.degrees(theta2):.2f}°, A3: {np.degrees(theta3):.2f}°"
    )

    # 添加图例
    # ax.legend()

    # 设置等比例
    ax.set_box_aspect([1, 1, 1])

    # 显示网格
    ax.grid(True)

    plt.tight_layout()
    ax.view_init(elev=15, azim=-70)
    ax.set_xlim(-50, 300)
    ax.set_ylim(-200, 200)
    ax.set_zlim(0, 500)


def forward_kinematics(theta2, theta3):
    """
    正向运动学：计算r和Z

    参数:
        theta2, theta3: 关节角度（弧度）

    返回:
        r: 水平距离
        z: 垂直高度
    """
    # 计算实际关节坐标（弧度）
    a2_coord = A2_START_ANGLE + theta2
    a3_coord = A3_START_ANGLE + theta3 - a2_coord

    # 计算水平距离r和垂直高度z
    r = (L5 + L4) + L2 * np.cos(a2_coord) + L3 * np.cos(a2_coord + a3_coord)
    z = L1 + L2 * np.sin(a2_coord) + L3 * np.sin(a2_coord + a3_coord)

    return r, z


def compute_jacobian(theta2, theta3):
    """
    计算雅克比矩阵

    参数:
        theta2, theta3: 关节角度（弧度）

    返回:
        J: 2x2雅克比矩阵
    """
    # 计算实际关节坐标（弧度）
    a2_coord = A2_START_ANGLE + theta2
    a3_coord = A3_START_ANGLE + theta3 - a2_coord

    # 计算雅克比矩阵元素
    j11 = -L2 * np.sin(a2_coord) - L3 * np.sin(a2_coord + a3_coord)
    j12 = -L3 * np.sin(a2_coord + a3_coord)
    j21 = L2 * np.cos(a2_coord) + L3 * np.cos(a2_coord + a3_coord)
    j22 = L3 * np.cos(a2_coord + a3_coord)

    return np.array([[j11, j12], [j21, j22]])


def inverse_kinematics(x, y, z, max_iterations=100, tolerance=1e-6, learning_rate=0.1):
    """
    逆运动学: 给定末端位置，计算关节角度
    使用雅克比矩阵进行迭代求解

    参数:
        x, y, z: 目标位置坐标
        max_iterations: 最大迭代次数
        tolerance: 收敛容差
        learning_rate: 学习率

    返回:
        theta1, theta2, theta3: 三个关节的角度（弧度）
    """
    # 计算theta1
    theta1 = np.arctan2(y, x)

    # 目标水平距离
    r_target = np.sqrt(x**2 + y**2)

    # 初始猜测（弧度）
    theta2 = 0  # 初始猜测值
    theta3 = 0  # 初始猜测值

    for i in range(max_iterations):
        # 使用当前的关节角度计算末端位置
        r_current, z_current = forward_kinematics(theta2, theta3)

        # 计算误差
        error_r = r_target - r_current
        error_z = z - z_current
        error = np.array([error_r, error_z])

        # 检查是否收敛
        if np.linalg.norm(error) < tolerance:
            print(f"逆运动学求解收敛，迭代次数: {i + 1}")
            break

        # 计算雅克比矩阵
        J = compute_jacobian(theta2, theta3)

        # 计算关节角度更新
        try:
            delta = np.linalg.solve(J, error)
        except np.linalg.LinAlgError:
            # 使用伪逆
            J_pinv = np.linalg.pinv(J)
            delta = J_pinv @ error

        # 更新关节角度（使用学习率控制步长）
        theta2 += delta[0] * learning_rate
        theta3 += delta[1] * learning_rate

        if i == max_iterations - 1:
            print("警告：逆运动学求解未收敛，达到最大迭代次数")

    return theta1, theta2, theta3
