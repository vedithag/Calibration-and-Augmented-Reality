/*
Tejasri Kasturi & Veditha Gudapati
CS 5330 Computer Vision
Spring 2024
Project 4

Functions for projecting 3D points in world coordinates to 2D image pixel coordinates.
*/

#include "virtual.h"
#include "csv_util.h"

/*
 * Read Calibration
 * Given the CSV file containing calibration statistics,
 * this function retrieves the calibrated camera matrix and distortion coefficients.
 */

int readCalibration(std::string csv_filename, cv::Mat &camera_matrix, cv::Mat &dist_coeff)
{
    char *fname_char = new char[csv_filename.length() + 1]; // Convert string to char array
    strcpy(fname_char, csv_filename.c_str());               // Copy contents of string to char array

    // Read calibration data from CSV file
    std::cout << "Retrieving saved calibration..." << std::endl; // Print message indicating calibration data retrieval
    std::vector<char *> featureName;                             // Define vector for feature names
    std::vector<std::vector<float>> data;                        // Define vector of vectors for calibration data
    read_image_data_csv(fname_char, featureName, data, 0);       // Read calibration data from CSV file

    // Extract camera matrix and distortion coefficients from data
    camera_matrix.at<double>(0, 0) = (double)data[0][0]; // Assign value from data to camera matrix element
    camera_matrix.at<double>(0, 1) = (double)data[0][1]; // Assign value from data to camera matrix element
    camera_matrix.at<double>(0, 2) = (double)data[0][2]; // Assign value from data to camera matrix element
    camera_matrix.at<double>(1, 0) = (double)data[0][3]; // Assign value from data to camera matrix element
    camera_matrix.at<double>(1, 1) = (double)data[0][4]; // Assign value from data to camera matrix element
    camera_matrix.at<double>(1, 2) = (double)data[0][5]; // Assign value from data to camera matrix element
    camera_matrix.at<double>(2, 0) = (double)data[0][6]; // Assign value from data to camera matrix element
    camera_matrix.at<double>(2, 1) = (double)data[0][7]; // Assign value from data to camera matrix element
    camera_matrix.at<double>(2, 2) = (double)data[0][8]; // Assign value from data to camera matrix element

    dist_coeff = cv::Mat(1, 5, CV_32F); // Initialize distortion coefficients matrix

    for (int i = 0; i < 5; i++)
    {
        dist_coeff.at<float>(0, i) = data[1][i]; // Assign values from data to distortion coefficients matrix
    }

    return (0); // Return 0 indicating successful calibration data retrieval
}

/*
 * Calculate Camera Position
 * Given below vectors containing current corner set and point set, calibrated camera matrix and distortion coefficients,
 * where the function estimates the position of the camera with respect to the target and populates arrays with rotation and translation data.
 */
int cameraCalcPosition(std::vector<cv::Vec3f> &points, std::vector<cv::Point2f> &corners, cv::Mat &camera_matrix, cv::Mat &dist_coeff, cv::Mat &rot, cv::Mat &trans)
{
    // Estimate camera position using solvePnP function
    cv::solvePnP(points, corners, camera_matrix, dist_coeff, rot, trans); // Solve PnP problem to estimate camera position

    return (0); // Return 0 indicating successful calculation of camera position
}

/*
 Given a cv::Mat of the image frame, calibrated camera matrix, distortion coefficients, rotation and translation data
 from the current estimated camera position, this function projects 3D world coordinates of axes to image pixel
 coordinates on the image frame and draws lines between these points to generate the 3D axes at origin.
 */
int draw3dAxes(cv::Mat &src, cv::Mat &camera_matrix, cv::Mat &dist_coeff, cv::Mat &rot, cv::Mat &trans)
{
    std::vector<cv::Vec3f> points;           // Define vector to store 3D points
    points.push_back(cv::Vec3f({0, 0, 0}));  // Add origin point to the vector
    points.push_back(cv::Vec3f({2, 0, 0}));  // Add point along X-axis to the vector
    points.push_back(cv::Vec3f({0, -2, 0})); // Add point along Y-axis to the vector
    points.push_back(cv::Vec3f({0, 0, 2}));  // Add point along Z-axis to the vector

    std::vector<cv::Point2f> corners; // Define vector to store projected 2D points

    // Project 3D points onto 2D image plane
    cv::projectPoints(points, rot, trans, camera_matrix, dist_coeff, corners);

    // Draw X-axis arrow on the source image
    cv::arrowedLine(src, corners[0], corners[1], cv::Scalar(0, 0, 255), 5); // Draw X-axis arrow in red

    // Draw Y-axis arrow on the source image
    cv::arrowedLine(src, corners[0], corners[2], cv::Scalar(0, 255, 0), 5); // Draw Y-axis arrow in green

    // Draw Z-axis arrow on the source image
    cv::arrowedLine(src, corners[0], corners[3], cv::Scalar(255, 0, 0), 5); // Draw Z-axis arrow in blue

    return (0); // Return 0 indicating successful drawing of 3D axes on the source image
}

/*
 Given a cv::Mat of the image frame, calibrated camera matrix, distortion coefficients, rotation and translation data
 from the current estimated camera position, this function projects 3D world vertices of virtual shapes to image pixel
 coordinates on the image frame and draws lines between them to generate 3D virtual objects on the target.
 */
int draw3dObject(cv::Mat &src, cv::Mat &camera_matrix, cv::Mat &dist_coeff, cv::Mat &rot, cv::Mat &trans)
{

    // CYLINDER
    std::vector<cv::Vec3f> points; // Define vector to store 3D points

    // Define the parameters of the cylinder
    float radius = 1.0f;   // Define radius of the cylinder
    float height = 3.0f;   // Define height of the cylinder
    int num_segments = 30; // Number of segments to approximate the circle

    // Calculate the vertices for the cylinder
    for (int i = 0; i < num_segments; ++i)
    {
        float theta = 2.0f * M_PI * i / num_segments; // Calculate angle for each segment
        float x = radius * cosf(theta);               // Calculate x-coordinate of the vertex
        float y = radius * sinf(theta);               // Calculate y-coordinate of the vertex

        // Top circle
        points.push_back(cv::Vec3f({x, y, height / 2.0f})); // Add vertex to represent top circle
        // Bottom circle
        points.push_back(cv::Vec3f({x, y, -height / 2.0f})); // Add vertex to represent bottom circle
    }

    std::vector<cv::Point2f> corners; // Define vector to store projected 2D points

    // Project 3D points to 2D image coordinates
    cv::projectPoints(points, rot, trans, camera_matrix, dist_coeff, corners);

    // Connect the top and bottom vertices to form the cylinder
    for (int i = 0; i < num_segments; ++i)
    {
        // Connect top circle vertices
        cv::line(src, corners[i], corners[(i + 1) % num_segments], cv::Scalar(255, 0, 0), 2); // Connect vertices with blue lines
        // Connect bottom circle vertices
        cv::line(src, corners[i + num_segments], corners[((i + 1) % num_segments) + num_segments], cv::Scalar(255, 0, 0), 2); // Connect vertices with blue lines
        // Connect top and bottom circle vertices skipping every other vertex
        if (i % 2 == 0)
        {
            cv::line(src, corners[i], corners[i + num_segments], cv::Scalar(255, 0, 0), 2); // Connect vertices with blue lines
        }
    }

    points.clear();  // Clear the vector containing 3D points
    corners.clear(); // Clear the vector containing projected 2D points

    // PYRAMID

    // Define 3D points for a pyramid
    points.push_back(cv::Vec3f({2, -2, 3})); // Center
    points.push_back(cv::Vec3f({3, -1, 0})); // Top right
    points.push_back(cv::Vec3f({3, -3, 0})); // Bottom right
    points.push_back(cv::Vec3f({1, -3, 0})); // Bottom left
    points.push_back(cv::Vec3f({1, -1, 0})); // Top left

    // Project 3D points to 2D image coordinates
    cv::projectPoints(points, rot, trans, camera_matrix, dist_coeff, corners);

    // Connect vertices to form the pyramid
    cv::line(src, corners[0], corners[1], cv::Scalar(0, 255, 255), 5); // Connect center to top right (yellow)
    cv::line(src, corners[0], corners[2], cv::Scalar(0, 255, 255), 5); // Connect center to bottom right (yellow)
    cv::line(src, corners[0], corners[3], cv::Scalar(0, 255, 255), 5); // Connect center to bottom left (yellow)
    cv::line(src, corners[0], corners[4], cv::Scalar(0, 255, 255), 5); // Connect center to top left (yellow)
    cv::line(src, corners[1], corners[2], cv::Scalar(0, 255, 255), 5); // Connect top right to bottom right (yellow)
    cv::line(src, corners[2], corners[3], cv::Scalar(0, 255, 255), 5); // Connect bottom right to bottom left (yellow)
    cv::line(src, corners[3], corners[4], cv::Scalar(0, 255, 255), 5); // Connect bottom left to top left (yellow)
    cv::line(src, corners[4], corners[1], cv::Scalar(0, 255, 255), 5); // Connect top left to top right (yellow)

    points.clear();  // Clear the 3D points vector
    corners.clear(); // Clear the projected 2D points vector

    // CUBE

    // Define 3D points for a cube
    points.push_back(cv::Vec3f({8, -5, 0})); // Bottom right base
    points.push_back(cv::Vec3f({8, -5, 2})); // Top right base
    points.push_back(cv::Vec3f({6, -5, 0})); // Bottom left base
    points.push_back(cv::Vec3f({6, -5, 2})); // Top left base
    points.push_back(cv::Vec3f({8, -3, 0})); // Bottom right top
    points.push_back(cv::Vec3f({8, -3, 2})); // Top right top
    points.push_back(cv::Vec3f({6, -3, 0})); // Bottom left top
    points.push_back(cv::Vec3f({6, -3, 2})); // Top left top

    // Project 3D points to 2D image coordinates
    cv::projectPoints(points, rot, trans, camera_matrix, dist_coeff, corners);

    // Connect vertices to form the cube
    cv::line(src, corners[0], corners[2], cv::Scalar(255, 255, 0), 5); // Connect bottom right base to bottom left base (blue)
    cv::line(src, corners[4], corners[6], cv::Scalar(255, 255, 0), 5); // Connect top right base to top left base (blue)
    cv::line(src, corners[0], corners[4], cv::Scalar(255, 255, 0), 5); // Connect bottom right base to bottom right top (blue)
    cv::line(src, corners[2], corners[6], cv::Scalar(255, 255, 0), 5); // Connect bottom left base to top left base (blue)
    cv::line(src, corners[1], corners[3], cv::Scalar(255, 255, 0), 5); // Connect top right base to top right top (blue)
    cv::line(src, corners[5], corners[7], cv::Scalar(255, 255, 0), 5); // Connect top left base to top left top (blue)
    cv::line(src, corners[1], corners[5], cv::Scalar(255, 255, 0), 5); // Connect top right base to top left base (blue)
    cv::line(src, corners[3], corners[7], cv::Scalar(255, 255, 0), 5); // Connect top right top to top left top (blue)
    cv::line(src, corners[0], corners[1], cv::Scalar(255, 255, 0), 5); // Connect bottom right base to top right base (blue)
    cv::line(src, corners[2], corners[3], cv::Scalar(255, 255, 0), 5); // Connect bottom left base to top left base (blue)
    cv::line(src, corners[4], corners[5], cv::Scalar(255, 255, 0), 5); // Connect bottom right top to top right top (blue)
    cv::line(src, corners[6], corners[7], cv::Scalar(255, 255, 0), 5); // Connect bottom left top to top left top (blue)

    points.clear();  // Clear the 3D points vector
    corners.clear(); // Clear the projected 2D points vector

    return (0);
}
