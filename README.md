# PDR Navigation System

A Java implementation of a Pedestrian Dead Reckoning (PDR) system for indoor navigation, based on smartphone inertial sensors.

## Overview

PDR (Pedestrian Dead Reckoning) is a technique for indoor positioning when GPS signals are unavailable. This system uses data from a smartphone's accelerometer, gyroscope, and orientation sensors to:

1. Detect steps
2. Estimate step length
3. Determine heading direction
4. Track the user's position

## Features

- Step detection using peak-valley analysis of accelerometer data
- Step length estimation using various methods (Weinberg, Scarlet, Kim)
- Heading estimation with Kalman filtering and map-aided corrections
- Position tracking using dead reckoning

## Project Structure

```
src/
├── main/
│   ├── java/
│   │   └── com/
│   │       └── pdrnavigation/
│   │           ├── Main.java                  # Main application entry point
│   │           ├── data/                      # Data handling classes
│   │           │   ├── SensorData.java        # Class for storing sensor data
│   │           │   ├── SensorDataLoader.java  # Loads data from files
│   │           │   └── NavigationResult.java  # Stores navigation results
│   │           ├── filters/                   # Signal processing filters
│   │           │   ├── KalmanFilter.java      # Kalman filter implementation
│   │           │   ├── LowPassFilter.java     # Low-pass filter
│   │           │   └── FIRFilter.java         # FIR filter implementation
│   │           ├── math/                      # Mathematical utilities
│   │           │   ├── Quaternion.java        # Quaternion operations
│   │           │   ├── DCM.java               # Direction Cosine Matrix
│   │           │   ├── EulerAngles.java       # Euler angle utilities
│   │           │   └── MathUtils.java         # Mathematical utilities
│   │           ├── steps/                     # Step detection and analysis
│   │           │   ├── StepDetector.java      # Detects steps from acceleration
│   │           │   ├── StepLength.java        # Estimates step length
│   │           │   └── PeakValleyDetector.java # Peak-valley detection algorithm
│   │           ├── orientation/               # Orientation estimation
│   │           │   ├── HeadingEstimator.java  # Estimates heading direction
│   │           │   ├── AttitudeEstimator.java # Estimates device attitude
│   │           │   └── CornerDetector.java    # Detects turns using gyroscope
│   │           ├── location/                  # Position tracking
│   │           │   ├── PDRNavigator.java      # Main PDR algorithm
│   │           │   ├── ParticleFilter.java    # Particle filter implementation
│   │           │   └── MapMatcher.java        # Map matching algorithm
│   │           └── utils/                     # Utility classes
│   │               ├── Constants.java         # Constants used in the system
│   │               ├── Vector3D.java          # 3D vector operations
│   │               └── FileUtils.java         # File handling utilities
│   └── resources/
│       └── config.properties                  # Configuration properties
└── test/
    └── java/
        └── com/
            └── pdrnavigation/
                ├── steps/
                │   └── StepDetectorTest.java  # Tests for step detection
                ├── filters/
                │   └── KalmanFilterTest.java  # Tests for Kalman filter
                └── orientation/
                    └── HeadingEstimatorTest.java # Tests for heading estimation
```

## Getting Started

### Prerequisites

- Java JDK 11 or higher
- Maven

### Building the Project

```bash
mvn clean package
```

This will generate a JAR file with dependencies in the `target` directory.

### Running the Application

```bash
java -jar target/pdr-navigation-1.0-SNAPSHOT-jar-with-dependencies.jar [dataDirectory]
```

Where `dataDirectory` is the directory containing the sensor data files (`acc.txt`, `gyr.txt`, `ang.txt`).

### Input Data Format

The system expects three text files:

1. `acc.txt` - Accelerometer data (3 columns: X, Y, Z)
2. `gyr.txt` - Gyroscope data (3 columns: X, Y, Z)
3. `ang.txt` - Orientation data (3 columns: Yaw, Roll, Pitch) in degrees

Each file should contain data samples with one sample per line.

### Output

The system generates a text file `navigation_result.txt` with the following information:

- Step count
- Step lengths
- Step orientations
- North and East positions

## Algorithms

### Step Detection

The system detects steps by applying a low-pass filter to the accelerometer magnitude signal and then identifying peaks and valleys that exceed a certain threshold.

### Step Length Estimation

Three methods are implemented:

1. **Weinberg**: `SL = K * (amax - amin)^0.25`
2. **Scarlet**: `SL = K * (aavg - amin)/(amax - amin)`
3. **Kim**: `SL = K * aavg^(1/3)`

Where `K` is a calibration constant, and `amax`, `amin`, and `aavg` are the maximum, minimum, and average acceleration magnitudes during a step.

### Heading Estimation

The system uses a Kalman filter to estimate heading direction from gyroscope and orientation sensor data. Map-aided corrections are applied at corners to reduce drift.

## License

This project is licensed under the MIT License - see the LICENSE file for details.

## Acknowledgments

- This implementation is based on algorithms described in academic papers on PDR systems
- The original MATLAB code served as a reference for the algorithms