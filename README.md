# Calibration-and-Augmented-Reality

## Overview
This project focuses on developing a system capable of analyzing and enhancing video streams in real-time using sophisticated techniques such as chessboard corner extraction, Harris corner recognition, and camera calibration. The ultimate goal is to enable accurate projection of virtual objects onto a 2D video feed, enhancing the augmented reality (AR) experience.

## Features
Corner Detection: Utilizing chessboard patterns for robust corner detection with sub-pixel accuracy.

Camera Calibration: Accurate mapping from pixel coordinates to real-world coordinates.

Augmented Reality: Projecting 3D virtual objects onto a 2D video feed, maintaining correct orientation and position.

Feature Detection: Implementing Harris corner detection for robust feature identification.

Real-Time Processing: Ensuring real-time performance for video stream analysis and AR applications.

## Tasks
### Task 1: Detect and Extract Target Corners
Corner Detection: Utilized OpenCV's findChessboardCorners function.
Corner Refinement: Applied cornerSubPix for improved accuracy.
Draw Detected Corners: Visual feedback using drawChessboardCorners.

### Task 2: Select Calibration Images
World Coordinates Calculation: Calculate and store corresponding world coordinates for detected corners.
Storage of Coordinates: Organize data for camera calibration.
Adding Data to Lists: Prepare data for calibration by storing multiple sets of corner and world coordinates.

### Task 3: Calibrate the Camera
Intrinsic Parameter Estimation: Use calibrateCamera to estimate camera characteristics.
Reprojection Error Minimization: Optimize calibration parameters for accurate geometric image correction.
Save Calibration Data: Store calibration results for future use.

### Task 4: Compute Features for Each Major Region
Position Estimation: Estimate camera position using rotation and translation matrices.
Rotation and Translation Matrices: Calculate and interpret matrices to understand camera movement.

### Task 5: Project Outside Corners or 3D Axes
Input Parameters: Utilize calibrated camera matrix and distortion coefficients.
Coordinate System: Define and project 3D axes points onto the 2D image plane.
Draw Axes Lines: Visualize camera orientation relative to the scene.

### Task 6: Create a Virtual Object
Projection Parameters: Establish parameters like camera matrix, distortion coefficients, and camera pose.
Definition of Virtual Objects: Define shapes such as cylinders, pyramids, and cubes in 3D space.
Projection Process: Transform and project 3D vertices onto the 2D image plane.

### Task 7: Detect Robust Features
Corner Detection: Apply Harris corner detection to identify robust features.
Normalizing: Normalize detected corners and convert to suitable format.
Visualization: Visualize detected corners on the original image for representation.

## Key Implementations
Chessboard Corner Extraction: Accurate detection and refinement of corners.
Harris Corner Detection: Robust feature detection method.
Camera Calibration: Estimation of intrinsic parameters and minimization of reprojection error.
Augmented Reality: Projection of 3D virtual objects onto 2D video feed.

## Usage
Key Commands
q - Quit the program
s - Save the current calibration frame and perform calibration if frames >= 5
c - Save the current calibration in a CSV file
x - Display 3D axes at the origin of world coordinates
d - Display 3D objects
h - Print the number of Harris Corners detected

## Conclusion
This project showcases the integration of computer vision techniques to enhance real-time video streams with augmented reality. The system's ability to accurately detect, calibrate, and project virtual objects onto a video feed opens up various possibilities for AR applications.





