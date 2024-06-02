/*
Tejasri Kasturi & Veditha Gudapati
CS 5330 Computer Vision
Spring 2024
Project 4

main() CPP function for looking at an input video stream, detecting feature points on a target and placing virtual 3D objects on the target.
*/

#include <iostream>

#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "virtual.h"
#include "csv_util.h"

// Task 1- Detect and Extract Target Corners

/*
 Description: Detects corners in the 9x6 checkerboard grid of an image frame and draws them.
 Parameters:
     src: Input image frame
     dst: Output image frame with corners drawn
     corners: Vector to store the pixel coordinates of detected corners
     drawCorners: Flag indicating whether to draw corners on the output image
 Returns:
     bool: True if corners are found, false otherwise
 Given a cv::Mat of the image frame, cv::Mat for the output and vector of points
 */

// Function to extract corners from an input image and optionally draw them on the output image.
bool CornersExtract(cv::Mat &src, cv::Mat &dst, std::vector<cv::Point2f> &corners, bool drawCorners)
{
    // Make a copy of the source image.
    dst = src.clone();

    // Attempt to find chessboard corners in the source image.
    bool found = cv::findChessboardCorners(src, cv::Size(9, 6), corners);

    // Convert the source image to grayscale.
    cv::Mat gray;
    cv::cvtColor(src, gray, cv::COLOR_BGR2GRAY);

    // Refine corner locations if chessboard corners are found.
    if (found == true)
    {
        cv::cornerSubPix(gray, corners, cv::Size(5, 5), cv::Size(-1, -1), cv::TermCriteria(cv::TermCriteria::COUNT | cv::TermCriteria::EPS, 30, 0.1));
    }

    // Draw chessboard corners on the output image if requested.
    if (drawCorners)
    {
        cv::drawChessboardCorners(dst, cv::Size(6, 9), corners, found);
    }

    // Return whether chessboard corners are found in the image.
    return found;
}

// Task 2- Select Calibration Images
/*
 Function: selectCalibrationImg
 Description: Populates vectors with the pixel coordinates of detected corners and their corresponding world coordinates.
 Parameters:
     corners: Vector containing pixel coordinates of detected corners
     corners_list: Vector to store multiple sets of corner coordinates
     points: Vector to store world coordinates of the checkerboard target
     points_list: Vector to store multiple sets of world coordinates
 Returns:
     int: 0 indicating success
*/

// Function to select calibration images, compute grid points and corner locations, and store them.
int selectCalibrationImg(std::vector<cv::Point2f> &corners, std::vector<std::vector<cv::Point2f>> &corners_list,
                         std::vector<cv::Vec3f> &points, std::vector<std::vector<cv::Vec3f>> &points_list)
{
    // Number of columns in the chessboard grid.
    int cols = 9;

    // Loop through detected corners and compute corresponding grid points.
    for (int k = 0; k < corners.size(); k++)
    {
        // Calculate the column index of the current corner.
        float i = (float)(k % cols);
        // Calculate the row index of the current corner.
        float j = (float)(-1 * k / cols);
        // Create a 3D point corresponding to the grid location of the corner.
        cv::Vec3f point(i, j, 0);
        // Store the 3D point in the list of points.
        points.push_back(point);
    }

    // Store the detected corner locations and computed grid points.
    corners_list.push_back(corners);
    points_list.push_back(points);

    // Return 0 to indicate successful execution.
    return 0;
}

// Task 3- Calibrate the Camera

/*
 Function: calibrateCamera
 Description: Performs camera calibration and calculates the camera matrix and distortion coefficients.
 Parameters:
     points_list: Vector containing sets of world coordinates
     corners_list: Vector containing sets of pixel coordinates of detected corners
     camera_matrix: Output parameter to store the calibrated camera matrix
     dist_coeff: Output parameter to store the distortion coefficients
 Returns:
     float: Reprojection error
*/

// Function to calibrate the camera using the provided grid points and detected corner locations.
float calibrateCamera(std::vector<std::vector<cv::Vec3f>> &points_list, std::vector<std::vector<cv::Point2f>> &corners_list,
                      cv::Mat &camera_matrix, cv::Mat &dist_coeff)
{
    // Vectors to store rotation and translation vectors for each calibration image.
    std::vector<cv::Mat> rot, trans;

    // Perform camera calibration and compute reprojection error.
    float error = cv::calibrateCamera(points_list,                                                                            // List of 3D points for each calibration image.
                                      corners_list,                                                                           // List of 2D corner points for each calibration image.
                                      cv::Size(1280, 720),                                                                    // Size of the calibration images.
                                      camera_matrix,                                                                          // Output camera matrix.
                                      dist_coeff,                                                                             // Output distortion coefficients.
                                      rot,                                                                                    // Output rotation vectors.
                                      trans,                                                                                  // Output translation vectors.
                                      cv::CALIB_FIX_ASPECT_RATIO,                                                             // Fix the aspect ratio during calibration.
                                      cv::TermCriteria(cv::TermCriteria::MAX_ITER + cv::TermCriteria::EPS, 30, DBL_EPSILON)); // Termination criteria for the iterative optimization algorithm.

    // Return the computed reprojection error.
    return error;
}

/*
 Function: saveCalibration
 Description: Saves the camera calibration data to a CSV file.
 Parameters:
     camera_matrix: Camera matrix containing intrinsic parameters
     dist_coeff: Distortion coefficients
 Returns:
     int: 0 indicating success
*/
// Function to save camera calibration data to a CSV file.
int saveCalibration(cv::Mat &camera_matrix, cv::Mat &dist_coeff)
{
    // Define the filename for the CSV file.
    std::string fileName = "checker_data.csv";
    // Convert the filename to a C-style string.
    char *fileName_char = new char[fileName.length() + 1];
    strcpy(fileName_char, fileName.c_str());

    // Define the label for the camera matrix data in the CSV file.
    std::string columnName = "camera_matrix";
    // Convert the label to a C-style string.
    char *labelName_char = new char[columnName.length() + 1];
    strcpy(labelName_char, columnName.c_str());

    // Vector to store camera matrix data.
    std::vector<float> camVector;
    // Iterate over each element in the camera matrix.
    for (int i = 0; i < camera_matrix.rows; i++)
    {
        for (int j = 0; j < camera_matrix.cols; j++)
        {
            // Convert each double value to float and push it to the vector.
            float f_val = (float)camera_matrix.at<double>(i, j);
            camVector.push_back(f_val);
        }
    }
    // Write camera matrix data to the CSV file.
    append_image_data_csv(fileName_char, labelName_char, camVector);

    // Define the label for the distortion coefficient data in the CSV file.
    columnName = "distortion_coefficient";
    // Convert the label to a C-style string.
    char *label_char = new char[columnName.length() + 1];
    strcpy(label_char, columnName.c_str());

    // Vector to store distortion coefficient data.
    std::vector<float> distVector;
    // Iterate over each element in the distortion coefficients matrix.
    for (int i = 0; i < dist_coeff.rows; i++)
    {
        for (int j = 0; j < dist_coeff.cols; j++)
        {
            // Convert each double value to float and push it to the vector.
            float f_val = (float)dist_coeff.at<double>(i, j);
            distVector.push_back(f_val);
        }
    }
    // Write distortion coefficient data to the CSV file.
    append_image_data_csv(fileName_char, label_char, distVector);

    // Return 0 to indicate successful execution.
    return 0;
}

/*
 main function
 */
int main(int argc, char *argv[])
{
    cv::VideoCapture *capdev; // Pointer to a VideoCapture object

    // Open the video device
    capdev = new cv::VideoCapture("/dev/video1"); // Dynamically allocate a VideoCapture object
    if (!capdev->isOpened())                      // Check if the video capture is open
    {
        printf("Unable to open video frame\n"); // Print error message
        return (-1);                            // Return -1 to indicate failure
    }

    // Set properties of the video capture
    capdev->set(cv::CAP_PROP_FRAME_WIDTH, 960);  // Set frame width
    capdev->set(cv::CAP_PROP_FRAME_HEIGHT, 540); // Set frame height
    // Get the expected frame size
    cv::Size refS((int)capdev->get(cv::CAP_PROP_FRAME_WIDTH), (int)capdev->get(cv::CAP_PROP_FRAME_HEIGHT)); // Get frame size
    printf("Expected size: %d %d\n", static_cast<int>(refS.width), static_cast<int>(refS.height));          // Print expected frame size

    // Create a named window
    cv::namedWindow("Video", 1); // Create a window to display video

    // Initialize global variables for different tasks
    cv::Mat frame;    // Matrix to store each frame
    int frameNo = 1;  // Variable to keep track of frame number
    int frameCal = 1; // Variable to keep track of frame calibration
    cv::Mat output;   // Matrix for output

    // Flags for different functionalities
    bool drawCorners = true; // Flag to draw corners
    bool DispAxes = false;   // Flag to display axes
    bool DispObject = false; // Flag to display object
    bool robust = false;     // Flag for robustness

    // Lists for storing points and corners
    std::vector<std::vector<cv::Vec3f>> points_list;    // Vector of vectors to store points
    std::vector<std::vector<cv::Point2f>> corners_list; // Vector of vectors to store corners

    // Camera matrix and distortion coefficients
    double cammat[] = {1, 0, (double)frame.cols / 2, 0, 1, (double)frame.rows / 2, 0, 0, 1}; // Define camera matrix
    cv::Mat camera_matrix(cv::Size(3, 3), CV_64FC1, &cammat);                                // Initialize camera matrix
    cv::Mat dist_coeff;                                                                      // Matrix for distortion coefficients
    cv::Mat rot, trans;                                                                      // Matrices for rotation and translation

    // Start live feed from the video device
    while (true) // Infinite loop for live video feed
    {
        *capdev >> frame;  // Get a new frame from the camera, treat as a stream
        if (frame.empty()) // Check if the frame is empty
        {
            printf("Frame is empty\n"); // Print error message
            break;                      // Break the loop if frame is empty
        }

        // Vector to store detected corners
        std::vector<cv::Point2f> corners; // Vector to store detected corners
        // Vector to store detected points
        std::vector<cv::Vec3f> points; // Vector to store detected points

        // Task 1 - Extract corners from chessboard
        bool found = CornersExtract(frame, output, corners, drawCorners);

        // Display axes if enabled and corners are found
        if (DispAxes && found)
        {
            // Task 2 - Select calibration image
            selectCalibrationImg(corners, corners_list, points, points_list);

            // Task 4 - Calculate current position of the camera
            cameraCalcPosition(points, corners, camera_matrix, dist_coeff, rot, trans);
            // Display rotation matrix
            std::cout << std::endl
                      << "rotation_matrix: " << rot << std::endl;
            // Display translation matrix
            std::cout << std::endl
                      << "translation_matrix: " << trans << std::endl;

            // Task 5 - Project 3D axes
            draw3dAxes(output, camera_matrix, dist_coeff, rot, trans);
        }

        // Check if displaying a virtual object is enabled and corners were found in the frame
        if (DispObject && found)
        {
            // Select the current frame as a calibration image based on detected corners, updating lists of corners and 3D points
            selectCalibrationImg(corners, corners_list, points, points_list);

            // Calculate the current position (pose) of the camera relative to the chessboard
            // This involves computing the rotation and translation matrices
            cameraCalcPosition(points, corners, camera_matrix, dist_coeff, rot, trans);
            // Print the rotation matrix to the console for debugging or information purposes
            std::cout << std::endl
                      << "rotation_matrix: " << rot << std::endl;
            // Print the translation matrix to the console for debugging or information purposes
            std::cout << std::endl
                      << "translation_matrix: " << trans << std::endl;

            // Create and display a virtual object in the output frame
            // The object's position and orientation are determined by the camera's pose
            draw3dObject(output, camera_matrix, dist_coeff, rot, trans);
        }

        // Display the current frame (with any overlays like the virtual object) in the "Video" window
        cv::imshow("Video", output);

        // Wait for a keystroke with a short delay (10 milliseconds)
        // This function also processes window events, allowing the displayed image to update
        char key = cv::waitKey(10);

        // Check if the 'q' key was pressed, which is designated to quit the loop/program
        if (key == 'q')
        {
            break; // Exit the loop, leading to the termination of the program or moving to the next block of code
        }
        // Press 's' to save current calibration frame and perform calibration if frames >= 5
        else if (key == 's' && found && !DispAxes && !DispObject && drawCorners)
        {
            // Task 2 - Select calibration images
            selectCalibrationImg(corners, corners_list, points, points_list);

            // Print message indicating saving of calibration image
            printf("Saving calibration image...\n");

            // Generate filename for calibration image
            std::string fname = "calibration-frame-";
            fname += std::to_string(frameCal) + ".jpg";

            // Save the calibration frame
            imwrite(fname, output);

            // Print information about the saved calibration image
            std::cout << "---------------------------------------------------------------------------" << std::endl;
            std::cout << "Calibration image " << frameCal << " saved" << std::endl;
            std::cout << "Point set " << frameCal << " \t Corners set " << frameCal << std::endl;

            // Loop through each corner point and print its world coordinates and image coordinates
            for (int i = 0; i < points.size(); i++)
            {
                cv::Vec3s point = points[i];
                cv::Point2f corner = corners[i];

                // Print world coordinates of the corner point
                printf("[%d %d %d] \t\t", point[0], point[1], point[2]);
                // Print image coordinates of the corner point
                printf("[%f, %f] \n", corner.x, corner.y);
            }
        
        // Print a separator line
        std::cout << "---------------------------------------------------------------------------" << std::endl;

        // Require at least 5 frames for calibration
        if (frameCal >= 5)
        {
            // Task 3 - Calibrate the camera
            std::cout << "Performing calibration with " << frameCal << " frames..." << std::endl;
            // Print initial camera matrix
            std::cout << "initial camera matrix:" << std::endl;
            std::cout << camera_matrix << std::endl;

            // Calibrate the camera and calculate reprojection error
            float reprojErr = calibrateCamera(points_list, corners_list, camera_matrix, dist_coeff);

            // Print the calibration statistics for the user
            std::cout << "calibrated camera matrix:" << std::endl;
            std::cout << camera_matrix << std::endl;
            std::cout << "re-projection error: " << reprojErr << std::endl;
            std::cout << "distortion coefficients: " << dist_coeff << std::endl;
        }

            // Increment the frame count for calibration
            frameCal = frameCal + 1;
        }
        // Press 'c' to save current calibration in a csv file to be read later
        else if (key == 'c' && found && !DispAxes && !DispObject && drawCorners)
        {
            // Save current calibration in a csv file
            std::cout << std::endl
                      << "Saving performed calibration..." << std::endl;
            saveCalibration(camera_matrix, dist_coeff);
        }

        // Press 'x' to display 3d axes at the origin of world coordinates
        else if (key == 'x' && found)
        {
            // Toggle the display of axes
            DispAxes = !DispAxes;
            if (DispObject)
            {
                DispObject = !DispObject;
            }
            // Toggle the drawing of corners
            drawCorners = !(DispAxes || DispObject);

            // Read calibration to display axes
            std::string fileName = "checker_data.csv";
            readCalibration(fileName, camera_matrix, dist_coeff);
            std::cout << std::endl
                      << "retrieved calibrated camera matrix:" << std::endl;
            std::cout << camera_matrix << std::endl;
            std::cout << "distortion coefficients: " << dist_coeff << std::endl;
        }
        // press 'd' to display 3d objects
        else if (key == 'd' && found)
        {
            if (DispAxes)
            {
                DispAxes = !DispAxes;
            }
            DispObject = !DispObject;
            drawCorners = !(DispObject || DispAxes);

            // read calibration to display virtual object
            std::string fileName = "checker_data.csv";
            readCalibration(fileName, camera_matrix, dist_coeff);
            std::cout << std::endl
                      << "retrieved calibrated camera matrix:" << std::endl;
            std::cout << camera_matrix << std::endl;
            std::cout << "distortion coefficients: " << dist_coeff << std::endl;
        }
    }

    delete capdev;

    return (0);
}
