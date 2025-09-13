# IMU

# I. Description:
The project uses MPU9250 or MPU6500 (without magnetometer) with examples of using Kalman filters and Quaternion (which is not my implementation)

# II. Structure:
The solution project has been divided into three parts:
- Project
  - Filters,
  - MPU9250,
  - Settings
- Tests
  - Conan,
  - Hardware,
  - Middleware

# III. Configuration:
- Python 2.11.6 with required packages,
- Conan 2.2.2
- Git 2.40.1
- CMake 3.16.3
- Visual Studio Code 1.88.0

# IV. Builidng:
- Go to 'Conan' folder and open git bash console,
- Type 'conan install . --build=gtest/cci.20210126',
- Type 'conan source .',
- Type 'conan build .'
