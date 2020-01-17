# MiniKiwiFormation

## Lidar

- [ ] Display the scan data
  - [x] Updated config.yml
  - [x] Deactivate raw data relay
  - [x] Put to zero angle with an error
  - [x] Transmit angle by angle to serial
  - [x] Print data with binserial
  - [x] Display data in a plot
  - [x] Fix quality and distance inversion in the label
  - [x] Change plot to polar coordinates
  - [ ] Generalize SerialInterface
    - [ ] Read and plot any type of data
      - [x] Choose number of points per plot lines
      - [x] Choose between polar or xy plot
      - [ ] Option to give x data or not
    - [ ] Write any type of data with Qt widgets
      - [ ] Make an array with the widget use to send data
      - [ ] Make slot for widget to update data to send
- [ ] WS2812 to display obstacle
  - [ ] [Non-Blocking WS2812 LED Library - PJRC](https://www.pjrc.com/non-blocking-ws2812-led-library/)
- [ ] Arduino code
  - [x] Use customPID
  - [x] Handle data error
  - [x] Clean code
  - [ ] Add method to configure lidar
  - [ ] Develop algorithm for data manipulation
    - [ ] Idea from [LidarObstacleDetection](https://github.com/enginBozkurt/LidarObstacleDetection)
      - Filtering
      - Segmentation
      - Clustering
      - Bound Box

### Data format

Variable | Type | Description
-------- | ---- | -----------
motor_rpm | uint16_t | lidar rpm
aryDist | uint16_t[4] | 4 distance
aryQuality | uint16_t[4] | 4 signal strength
angle | uint16_t | First angle measured [0,4,…,356]

## Software

- [ ] Add position control
- [x] Modify Motor to include Encoder and PID
- [x] Add correction ratio to odometry

## Documents de référence à écrire

- [ ] Documentation carte miniKiwi
- [ ] Documentation et fichiers .stl supports de formation PID, et explications sur la préparation des supports de formations
- [ ] Documentation de base sur l'utilisation du firmware de jeelabs, et l'utilisation en pont uart/wifi

## Resources

- [XV_Lidar_Controller](https://github.com/getSurreal/XV_Lidar_Controller)
