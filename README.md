# Weed Rooba: Intelligent Weed Detection and Plant Health Monitoring Robot

## Overview

Weed Rooba is an innovative agricultural robot that combines Simultaneous Localization and Mapping (SLAM) technology with advanced computer vision to revolutionize weed detection and plant health monitoring in agricultural settings. By leveraging cutting-edge mapping and image processing techniques, Weed Rooba offers farmers a more efficient and sustainable solution for crop management.

## Key Features

- **SLAM-based Mapping**: Weed Rooba creates accurate, real-time maps of agricultural fields, enabling precise navigation and data collection.
- **Computer Vision for Weed Detection**: Utilizes state-of-the-art machine learning algorithms to identify and locate weeds with high accuracy.
- **Plant Health Monitoring**: Advanced sensors and image analysis to assess crop health, detecting signs of disease, nutrient deficiencies, or pest infestations.
- **Efficient Path Planning**: Optimizes its route through fields to minimize time and energy consumption while maximizing coverage.
  
## Technologies Used

- **Robot Operating System (ROS)**: For overall robot control and integration of various modules.
- **OpenCV**: For image processing and computer vision tasks.
- **TensorFlow**: For implementing and running machine learning models for weed detection and plant health assessment.
- **RTABMap**: For SLAM-based mapping and localization.
- **Python**: Primary programming language for software development.
- **React**: For building the user interface of the data analytics dashboard.

## Getting Started

### Prerequisites

- ROS Noetic
- Python 3.8+
- OpenCV 4.5+
- TensorFlow 2.5+
- RTABMap

### Installation

1. Clone the repository:
   ```
   git clone https://github.com/your-organization/weed-rooba.git
   cd weed-rooba
   ```

2. Install dependencies:
   ```
   pip install -r requirements.txt
   ```

3. Build the ROS packages:
   ```
   catkin_make
   ```

4. Source the setup file:
   ```
   source devel/setup.bash
   ```

## Usage

1. Launch the Weed Rooba core system:
   ```
   roslaunch weed_rooba_bringup weed_rooba_system.launch
   ```

2. Start the mapping and navigation module:
   ```
   roslaunch weed_rooba_navigation start_mapping.launch
   ```

3. Initiate the weed detection and plant health monitoring:
   ```
   roslaunch weed_rooba_vision start_detection.launch
   ```

4. Access the data analytics dashboard:
   ```
   cd weed_rooba_dashboard
   npm start
   ```

## Contributing

We welcome contributions to the Weed Rooba project! Please read our [CONTRIBUTING.md](CONTRIBUTING.md) file for guidelines on how to submit pull requests, report issues, and suggest improvements.

## License

This project is licensed under the MIT License - see the [LICENSE.md](LICENSE.md) file for details.

## Acknowledgments

- Thanks to the open-source community for providing invaluable tools and libraries.
- Special thanks to agricultural experts who provided insights into weed management and plant health assessment.

![alt text](images/rviz)
![alt text](images/map)
