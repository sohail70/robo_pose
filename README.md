# Fusemuse ROS2 Package

## Overview

The Fusemuse ROS2 Package is a comprehensive collection of tools and utilities designed to streamline the integration and collaboration of various models and components within a unified framework. This library aims to facilitate the seamless fusion of diverse elements, making it easier to develop, deploy, and manage complex robotic applications.

## Current Features

- **User Model Integration:** Users can create their motion models and integrate them into the fusion process.

- **Arbitrary Number of Sensors:** The package supports the integration of an arbitrary number of sensors. Currently IMU data and odometry data can be fused.

## Getting Started

Here is an overview of the entire package:

![Fusemuse ROS2 Package](https://drive.usercontent.google.com/download?id=1YoiWyYLB_H4QnVc5sR48ACLma5Rwx07V&export=view&authuser=0)

### Installation

To install the Fusemuse ROS2 Package, follow these steps:

1. Clone the repository to your ROS2 workspace:

    ```bash
    git@github.com:sohail70/fusemuse.git
    ```

2. Build the package using colcon:

    ```bash
    colcon build
    ```

3. Source your ROS2 workspace:

    ```bash
    source install/setup.bash
    ```
3. Source your ROS2 workspace:

    ```bash
    ros2 run fusion fusion --ros-args --params-file src/fusion/params/ekf.yaml
    ```

For detailed usage and customization instructions, refer to the [full documentation](docs/).

## Contributing

Contributions are welcome! See [CONTRIBUTING.md](CONTRIBUTING.md) for details on how to get started.

## License

This project is licensed under the [MIT License](LICENSE).
