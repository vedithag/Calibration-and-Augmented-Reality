/*
Tejasri Kasturi & Veditha Gudapati
CS 5330 Computer Vision
Spring 2024
Project 4

This file contains the implementation of Harris corners
*/



#include <opencv2/opencv.hpp> // Include OpenCV library
#include <iostream>           // Include input/output stream library

using namespace cv;  // Using OpenCV namespace
using namespace std; // Using standard namespace for C++

int main()
{
    // Open the default camera
    VideoCapture capture(0); // Initialize a VideoCapture object with default camera index
    if (!capture.isOpened())
    {                                                   // Check if camera is opened successfully
        cerr << "Error: Failed to open camera" << endl; // Display error message if camera failed to open
        return -1;                                      // Return with error code
    }

    // Create a window to display the detected corners
    namedWindow("Detected Corners", WINDOW_NORMAL); // Create a window with resizable option
    resizeWindow("Detected Corners", 800, 600);     // Resize the window to specific dimensions

    Mat frame, gray; // Declare Mat objects for storing frames and grayscale images
    while (true)
    {
        // Capture frame from the camera
        capture >> frame;  // Capture frame from the camera
        if (frame.empty()) // Check if the frame is empty
            break;         // Break the loop if frame is empty

        // Convert frame to grayscale
        cvtColor(frame, gray, COLOR_BGR2GRAY); // Convert BGR image to grayscale

        // Detect Harris corners
        Mat dst, dst_norm;                                   // Declare Mat objects for storing corner detection results
        int blockSize = 2;                                   // Size of the neighborhood considered for corner detection
        int apertureSize = 3;                                // Aperture parameter for the Sobel operator
        double k = 0.04;                                     // Harris detector free parameter
        cornerHarris(gray, dst, blockSize, apertureSize, k); // Apply Harris corner detection algorithm

        // Normalize
        normalize(dst, dst_norm, 0, 255, NORM_MINMAX, CV_32FC1, Mat()); // Normalize the output of Harris corner detection
        convertScaleAbs(dst_norm, dst_norm);                            // Convert the normalized image to 8-bit unsigned integer format

        // Use goodFeaturesToTrack to actually find the corners
        vector<Point2f> corners;                           // Declare a vector to store detected corner points
        goodFeaturesToTrack(gray, corners, 500, 0.01, 10); // Apply the Shi-Tomasi corner detection algorithm to find good features

        // Draw circles around detected corners
        for (size_t i = 0; i < corners.size(); ++i)
        {                                                             // Loop through all detected corners
            circle(frame, corners[i], 5, Scalar(0, 255, 0), 2, 8, 0); // Draw a green circle around each detected corner
        }

        // Display the frame with detected corners
        imshow("Detected Corners", frame); // Display the frame with detected corners in the window

        // Check for the 'h' key press to print the number of corners
        char key = waitKey(30); // Wait for a key press for 30 milliseconds
        if (key == 'h')
        {                                                                     // Check if 'h' key is pressed
            cout << "Number of corners detected: " << corners.size() << endl; // Print the number of detected corners
        }

        // Check for the Esc key press to exit
        if (key == 27) // Check if Esc key is pressed
            break;     // Break the loop if Esc key is pressed
    }

    return 0; // Return with success code
}
