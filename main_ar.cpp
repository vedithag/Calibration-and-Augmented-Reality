/*
Tejasri Kasturi & Veditha Gudapati
CS 5330 Computer Vision
Spring 2024
Project 4

main() CPP function for extensions to the AR system.
*/

#include <iostream>

// OpenCV headers
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

// User-defined header
#include "extension.h"

// Main function
int main(int argc, char *argv[])
{
    cv::VideoCapture *capdev; // Pointer to VideoCapture object

    // Open the video device
    capdev = new cv::VideoCapture("/dev/video1");
    if (!capdev->isOpened())
    {
        printf("Unable to open video device\n");
        return (-1);
    }

    // Set properties of the image
    capdev->set(cv::CAP_PROP_FRAME_WIDTH, 960);
    capdev->set(cv::CAP_PROP_FRAME_HEIGHT, 540);
    cv::Size refS((int)capdev->get(cv::CAP_PROP_FRAME_WIDTH),
                  (int)capdev->get(cv::CAP_PROP_FRAME_HEIGHT));
    printf("Expected size: %d %d\n", refS.width, refS.height);

    // Create a window to display video
    cv::namedWindow("Video", 1);

    // Initialize variables
    cv::Mat frame;    // Matrix to hold each frame
    int frameNo = 1;  // Frame number
    int frameCal = 1; // Calibration frame number
    cv::Mat output;   // Output image

    bool drawCenters = true;                            // Boolean flag for drawing centers
    std::vector<std::vector<cv::Vec3f>> points_list;    // List to store points
    std::vector<std::vector<cv::Point2f>> centers_list; // List to store centers

    // Camera matrix initialization
    double cammat[] = {1, 0, (double)frame.cols / 2, 0, 1, (double)frame.rows / 2, 0, 0, 1};
    cv::Mat camera_matrix(cv::Size(3, 3), CV_64FC1, &cammat);
    cv::Mat dist_coefficient; // Distortion coefficients
    cv::Mat rot, trans;       // Rotation and translation matrices
    bool DispAxes = false;    // Boolean flag for displaying axes
    bool DispObject = false;  // Boolean flag for displaying object

    bool canvas = false; // Boolean flag for canvas mode

    // Start live feed from the video device
    while (true)
    {
        *capdev >> frame; // Get a new frame from the camera, treat as a stream
        if (frame.empty())
        {                               // Check if the frame is empty
            printf("frame is empty\n"); // Print message indicating the frame is empty
            break;                      // Exit the loop if the frame is empty
        }

        std::vector<cv::Point2f> centers; // Vector to store detected centers
        std::vector<cv::Vec3f> points;    // Vector to store detected points

        // Extracting corners from circle-grid
        bool found = circleExtractCenters(frame, output, centers, drawCenters);

        // Display axes
        if (DispAxes && found)
        {
            selectCalibrationImg(centers, centers_list, points, points_list); // Select calibration images

            // Calculate current position of the camera
            calcCameraPosition(points, centers, camera_matrix, dist_coefficient, rot, trans);
            std::cout << std::endl
                      << "rotation matrix: " << rot << std::endl; // Print rotation matrix
            std::cout << std::endl
                      << "translation matrix: " << trans << std::endl; // Print translation matrix

            // Project 3D axes
            draw3dAxes(output, camera_matrix, dist_coefficient, rot, trans);
        }

        // Display virtual object
        if (DispObject && found)
        {
            selectCalibrationImg(centers, centers_list, points, points_list); // Select calibration images

            // Calculate current position of the camera
            calcCameraPosition(points, centers, camera_matrix, dist_coefficient, rot, trans);
            std::cout << std::endl
                      << "rotation matrix: " << rot << std::endl; // Print rotation matrix
            std::cout << std::endl
                      << "translation matrix: " << trans << std::endl; // Print translation matrix

            // Create a virtual object
            draw3dObject(output, camera_matrix, dist_coefficient, rot, trans);
        }

        // Transform target into image canvas if canvas mode is enabled and circles are found
        if (canvas && found)
        {
            selectCalibrationImg(centers, centers_list, points, points_list); // Select calibration images

            // Calculate current position of the camera
            calcCameraPosition(points, centers, camera_matrix, dist_coefficient, rot, trans);
            std::cout << std::endl
                      << "Rotation matrix: " << rot << std::endl; // Print rotation matrix
            std::cout << std::endl
                      << "Translation matrix: " << trans << std::endl; // Print translation matrix

            std::string imageFilename = "nature.jpeg"; // Define filename for the image to be placed on the target
            // Draw image contents on the target
            drawOnTarget(frame, output, camera_matrix, dist_coefficient, rot, trans, imageFilename);
        }

        // Display the current frame
        cv::imshow("Video", output); // Show the current frame on a window titled "Video"

        // Check if there is a waiting keystroke
        char key = cv::waitKey(10);

        // Press 'q' to quit
        if (key == 'q')
        {
            break; // Exit the loop if 'q' key is pressed
        }

        // Press 's' to save current calibration frame and perform calibration if frames >= 5
        else if (key == 's' && found && !DispAxes && !DispObject && drawCenters)
        {
            // Select calibration images
            selectCalibrationImg(centers, centers_list, points, points_list);

            printf("Saving calibration image...\n");    // Print message indicating saving of calibration image
            std::string fname = "calibration-frame-";   // Create filename prefix for calibration frame
            fname += std::to_string(frameCal) + ".jpg"; // Append frame number to filename prefix
            imwrite(fname, output);                     // Save calibration frame as an image

            // Print the corner points in world coordinates with corresponding image coordinates
            std::cout << "---------------------------------------------------------------------------" << std::endl;
            std::cout << "Calibration image " << frameCal << " is saved" << std::endl;
            std::cout << "Point set " << frameCal << " \t Corners set " << frameCal << std::endl;
            for (int i = 0; i < points.size(); i++)
            {
                cv::Vec3s point = points[i];                             // Get a corner point in world coordinates
                cv::Point2f corner = centers[i];                         // Get the corresponding corner in image coordinates
                printf("[%d %d %d] \t\t", point[0], point[1], point[2]); // Print world coordinates
                printf("[%f, %f] \n", corner.x, corner.y);               // Print image coordinates
            }
            std::cout << "---------------------------------------------------------------------------" << std::endl;

            // Require at least 5 frames for calibration
            if (frameCal >= 5)
            {
                // Calibrate the camera
                std::cout << "Performing calibration with " << frameCal << " frames..." << std::endl;
                std::cout << "Initial camera matrix:" << std::endl;
                std::cout << camera_matrix << std::endl;

                float reprojErr = calibrateCamera(points_list, centers_list, camera_matrix, dist_coefficient);

                // Print the calibration stats for the user
                std::cout << "Calibrated camera matrix:" << std::endl;
                std::cout << camera_matrix << std::endl;
                std::cout << "Re-projection error: " << reprojErr << std::endl;
                std::cout << "Distortion coefficients: " << dist_coefficient << std::endl;
            }

            frameCal++; // Increment the frame counter
        }
        // Press 'c' to save current calibration in a csv file to be read later
        else if (key == 'c' && found && !DispAxes && !DispObject && drawCenters)
        {
            // Save current calibration in a csv file
            std::cout << std::endl
                      << "Saving performed calibration..." << std::endl;
            saveCalibration(camera_matrix, dist_coefficient);
        }

        // Press 'x' to display 3D axes at the origin of world coordinates
        else if (key == 'x' && found)
        {
            DispAxes = !DispAxes; // Toggle display of axes
            if (DispObject)
            {
                DispObject = !DispObject; // Disable display of virtual object if enabled
            }
            drawCenters = !(DispAxes || DispObject); // Enable drawing of centers if axes or object are not displayed
            if (canvas)
            {
                canvas = !canvas; // Disable canvas transformation if enabled
            }

            // Read calibration to display axes
            std::string fileName = "circlegrid.csv";                    // File containing calibration data
            readCalibration(fileName, camera_matrix, dist_coefficient); // Read calibration data from file
            std::cout << std::endl
                      << "Retrieved calibrated camera matrix:" << std::endl;
            std::cout << camera_matrix << std::endl;                                   // Print retrieved camera matrix
            std::cout << "Distortion coefficients: " << dist_coefficient << std::endl; // Print distortion coefficients
        }

        // Press 'o' to display 3D objects
        else if (key == 'o' && found)
        {
            if (DispAxes)
            {
                DispAxes = !DispAxes; // Disable display of axes if enabled
            }
            DispObject = !DispObject;                // Toggle display of virtual object
            drawCenters = !(DispObject || DispAxes); // Enable drawing of centers if object or axes are not displayed
            if (canvas)
            {
                canvas = !canvas; // Disable canvas transformation if enabled
            }

            // Read calibration to display virtual object
            std::string fileName = "circlegrid.csv";                    // File containing calibration data
            readCalibration(fileName, camera_matrix, dist_coefficient); // Read calibration data from file
            std::cout << std::endl
                      << "Retrieved calibrated camera matrix:" << std::endl;
            std::cout << camera_matrix << std::endl;                                   // Print retrieved camera matrix
            std::cout << "Distortion coefficients: " << dist_coefficient << std::endl; // Print distortion coefficients
        }

        // Press 't' to place image canvas on target
        else if (key == 't' && found)
        {
            canvas = !canvas; // Toggle canvas transformation
            if (DispAxes)
            {
                DispAxes = !DispAxes; // Disable display of axes if enabled
            }
            if (DispObject)
            {
                DispObject = !DispObject; // Disable display of virtual object if enabled
            }
            drawCenters = !(DispObject || DispAxes || canvas); // Enable drawing of centers if object, axes, or canvas are not displayed

            // Read calibration to transform target
            std::string fileName = "circlegrid.csv";                    // File containing calibration data
            readCalibration(fileName, camera_matrix, dist_coefficient); // Read calibration data from file
            std::cout << std::endl
                      << "Retrieved calibrated camera matrix:" << std::endl;
            std::cout << camera_matrix << std::endl;                                   // Print retrieved camera matrix
            std::cout << "Distortion coefficients: " << dist_coefficient << std::endl; // Print distortion coefficients
        }

        // Press 'p' to take a snapshot of the current frame
        else if (key == 'p')
        {
            printf("Saving image\n");                  // Print message indicating image saving
            std::string fname = "frame-";              // File name prefix
            fname += std::to_string(frameNo) + ".jpg"; // Append frame number to file name
            imwrite(fname, output);                    // Save the current frame as an image
            frameNo++;                                 // Increment frame number
        }
    }

    delete capdev; // Delete the video capture device object
    return (0);    // Return 0 to indicate successful execution
}
