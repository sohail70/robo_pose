#!/usr/bin/env python3
from PyQt5.QtWidgets import QApplication, QWidget, QPushButton, QLineEdit, QVBoxLayout
import rclpy
from rclpy.node import Node
from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import Parameter, ParameterValue

class ConfigGUI(Node, QWidget):
    def __init__(self):
        Node.__init__(self, 'config_gui_node')
        QWidget.__init__(self)

        self.init_ui()

    def init_ui(self):
        self.param_input = QLineEdit(self)
        self.param_input.setPlaceholderText("Enter parameter value")

        update_button = QPushButton('Update Parameter', self)
        update_button.clicked.connect(self.update_parameter)

        layout = QVBoxLayout()
        layout.addWidget(self.param_input)
        layout.addWidget(update_button)

        self.setLayout(layout)

        self.show()

    def update_parameter(self):
        value = self.param_input.text()

        # Use ROS 2 Python API to communicate with your C++ node and update the parameter
        param_client = self.create_client(SetParameters, '/Filter/set_parameters')
        request = SetParameters.Request()
        
        # Instantiate ParameterValue without keyword argument
        request.parameters.append(Parameter(name='use_sim_time', value=ParameterValue(string_value=value)))

        future = param_client.call_async(request)

        # Wait for the service response
        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None:
            self.get_logger().info('Parameter updated successfully')
        else:
            self.get_logger().info('Failed to update parameter')

def main():
    rclpy.init()
    app = QApplication([])
    gui = ConfigGUI()
    app.exec_()
    rclpy.shutdown()

if __name__ == '__main__':
    main()