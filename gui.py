from nicegui import ui
from robot_arm_kinematics import inverse_kinematics, visualize_robot_arm


class Data:
    def __init__(self):
        self.x = 180
        self.y = 0
        self.z = 440


data = Data()
ax = None


def update_arm():
    if ax is not None:
        theta1, theta2, theta3 = inverse_kinematics(data.x, data.y, data.z)
        ax.clear()
        visualize_robot_arm(theta1, theta2, theta3, ax)
        mpl.update()


with ui.grid(columns=3).classes("w-full"):
    ui.number(label="X").bind_value(data, "x")
    ui.number(label="Y").bind_value(data, "y")
    ui.number(label="Z").bind_value(data, "z")
    ui.slider(min=0, max=360, step=5, on_change=update_arm).bind_value(data, "x")
    ui.slider(min=-360, max=360, step=5, on_change=update_arm).bind_value(data, "y")
    ui.slider(min=0, max=500, step=5, on_change=update_arm).bind_value(data, "z")


with ui.matplotlib(figsize=(8, 8)) as mpl:
    fig = mpl.figure
    ax = fig.add_subplot(111, projection="3d")
    theta1, theta2, theta3 = inverse_kinematics(data.x, data.y, data.z, 20)
    visualize_robot_arm(theta1, theta2, theta3, ax)


mpl.update()
ui.run()
