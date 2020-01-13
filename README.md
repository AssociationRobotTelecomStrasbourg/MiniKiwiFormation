# MiniKiwiFormation

## Software

- [ ] Lidar
  - [ ] Use SerialInterface to display the lidar data
  - [ ] Motor/PID
    - [ ] Use customPID
    - [ ] Set reference
    - [ ] Set kp, ki, kd
    - [ ] Set min, max pwm
    - [ ] Set sample time
  - [ ] Handle data error
  - [ ] Set which angle to ignore
  - [ ] Initialize the variable in lidar.cpp
  - [ ] Develop algorithm for data manipulation
    - [ ] Idea from [LidarObstacleDetection](https://github.com/enginBozkurt/LidarObstacleDetection)
      - Filtering
      - Segmentation
      - Clustering
      - Bound Box
- [ ] Add position control
- [x] Modify Motor to include Encoder and PID
- [x] Add correction ratio to odometry

## Documents de référence à écrire

- [ ] Documentation carte miniKiwi
- [ ] Documentation et fichiers .stl supports de formation PID, et explications sur la préparation des supports de formations
- [ ] Documentation de base sur l'utilisation du firmware de jeelabs, et l'utilisation en pont uart/wifi

## Resources

- [XV_Lidar_Controller](https://github.com/getSurreal/XV_Lidar_Controller)
