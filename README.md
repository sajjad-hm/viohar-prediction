
# viohar-prediction

## Overview

**viohar-prediction** is a real-time Human Activity Recognition (HAR) prediction system that integrates the vioHAR dataset with the vins-mono visual-inertial odometry framework. This project provides a web interface for predicting human activities based on sensor data.

## Features

- **Real-time HAR Prediction**: Utilize the vioHAR dataset for accurate activity classification.
- **Integration with vins-mono**: Leverage visual-inertial odometry for enhanced sensor data processing.
- **Web Interface**: Interactive UI for users to input data and receive predictions.

## Project Structure

```
viohar-prediction/
├── launch/
├── scripts/
├── srv/
├── web/
├── CMakeLists.txt
└── package.xml
```

- **launch/**: Contains launch files for initializing the system.
- **scripts/**: Includes Python scripts for data processing and model inference.
- **srv/**: Defines service files for communication between components.
- **web/**: Holds the frontend code for the web interface.
- **CMakeLists.txt**: Build configuration file.
- **package.xml**: ROS package configuration.

## Installation

### Prerequisites

- ROS (Robot Operating System)
- Python 3.x
- Node.js and npm (for frontend)
- Dependencies as specified in `package.xml` and `requirements.txt`

### Steps

1. Clone the repository:

   ```bash
   git clone https://github.com/sajjad-hm/viohar-prediction.git
   cd viohar-prediction
   ```

2. Install ROS dependencies:

   ```bash
   rosdep install --from-paths src --ignore-src -r -y
   ```

3. Build the workspace:

   ```bash
   catkin_make
   ```

4. Install Python dependencies:

   ```bash
   pip install -r requirements.txt
   ```

5. Install frontend dependencies:

   ```bash
   cd web
   npm install
   ```

## Usage

1. Launch the system:

   ```bash
   roslaunch viohar_prediction main.launch
   ```

2. Access the web interface at `http://localhost:8080`.

3. Input sensor data and view real-time HAR predictions.

## Contributing

Feel free to fork the repository, submit issues, and create pull requests. Contributions are welcome!

## License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

## Acknowledgements

- [vioHAR Dataset](https://github.com/viothings/viohar)
- [vins-mono](https://github.com/HKUST-Aerial-Robotics/VINS-Mono)
