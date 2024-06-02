/*
Tejasri Kasturi & Veditha Gudapati
CS 5330 Computer Vision
Spring 2024
Project 4

Function implementations for various steps used during the calibration of camera and projecting 3D points in world coordinates to 2D image pixel coordinates.
*/

#include "extension.h"
#include "csv_util.h"

/*******************************Extension -1  Detect circle corners*****************************************************/
/*
 Given a cv::Mat of the image frame, cv::Mat for the output and vector of points
 the function detects the circles centers present in the circle grid and draws them.
 This function also populates given vector with image pixel coordinates of centers detected.
 */
bool circleExtractCenters(cv::Mat &src, cv::Mat &dst, std::vector<cv::Point2f> &centers, bool drawCenters)
{
    dst = src.clone();

    bool found = cv::findCirclesGrid(src, cv::Size(4, 11), centers, cv::CALIB_CB_ASYMMETRIC_GRID + cv::CALIB_CB_CLUSTERING);

    // std::cout << "No. of corners detected:- " << centers.size() << std::endl;
    // std::cout << "Co-ordinate of top left corner:- " << centers[0].x << " " << centers[0].y << std::endl;

    if (drawCenters)
    {
        cv::drawChessboardCorners(dst, cv::Size(4, 11), centers, found);
    }

    return (found);
}

/*
 Given a vector of points having image pixel coordinates of detected circle centers,
 this function populates the vector of points in world coordinates for the checkerboard target.
 It also populates the vectors for storing multiple point sets and center sets to be used in calibration.
 */
int selectCalibrationImg(std::vector<cv::Point2f> &corners, std::vector<std::vector<cv::Point2f>> &corners_list, std::vector<cv::Vec3f> &points, std::vector<std::vector<cv::Vec3f>> &points_list)
{
    // Populate world coordinates for the checkerboard target
    points.push_back(cv::Vec3f(10, 7, 0)); // Push back a world coordinate point to the vector
    points.push_back(cv::Vec3f(10, 5, 0)); // Push back a world coordinate point to the vector
    points.push_back(cv::Vec3f(10, 3, 0)); // Push back a world coordinate point to the vector
    points.push_back(cv::Vec3f(10, 1, 0));
    points.push_back(cv::Vec3f(9, 6, 0));
    points.push_back(cv::Vec3f(9, 4, 0));
    points.push_back(cv::Vec3f(9, 2, 0));
    points.push_back(cv::Vec3f(9, 0, 0));
    points.push_back(cv::Vec3f(8, 7, 0));
    points.push_back(cv::Vec3f(8, 5, 0));
    points.push_back(cv::Vec3f(8, 3, 0));
    points.push_back(cv::Vec3f(8, 1, 0));
    points.push_back(cv::Vec3f(7, 6, 0));
    points.push_back(cv::Vec3f(7, 4, 0));
    points.push_back(cv::Vec3f(7, 2, 0));
    points.push_back(cv::Vec3f(7, 0, 0));
    points.push_back(cv::Vec3f(6, 7, 0));
    points.push_back(cv::Vec3f(6, 5, 0));
    points.push_back(cv::Vec3f(6, 3, 0));
    points.push_back(cv::Vec3f(6, 1, 0));
    points.push_back(cv::Vec3f(5, 6, 0));
    points.push_back(cv::Vec3f(5, 4, 0));
    points.push_back(cv::Vec3f(5, 2, 0));
    points.push_back(cv::Vec3f(5, 0, 0));
    points.push_back(cv::Vec3f(4, 7, 0));
    points.push_back(cv::Vec3f(4, 5, 0));
    points.push_back(cv::Vec3f(4, 3, 0));
    points.push_back(cv::Vec3f(4, 1, 0));
    points.push_back(cv::Vec3f(3, 6, 0));
    points.push_back(cv::Vec3f(3, 4, 0));
    points.push_back(cv::Vec3f(3, 2, 0));
    points.push_back(cv::Vec3f(3, 0, 0));
    points.push_back(cv::Vec3f(2, 7, 0));
    points.push_back(cv::Vec3f(2, 5, 0));
    points.push_back(cv::Vec3f(2, 3, 0));
    points.push_back(cv::Vec3f(2, 1, 0));
    points.push_back(cv::Vec3f(1, 6, 0));
    points.push_back(cv::Vec3f(1, 4, 0));
    points.push_back(cv::Vec3f(1, 2, 0));
    points.push_back(cv::Vec3f(1, 0, 0));
    points.push_back(cv::Vec3f(0, 7, 0));
    points.push_back(cv::Vec3f(0, 5, 0));
    points.push_back(cv::Vec3f(0, 3, 0));
    points.push_back(cv::Vec3f(0, 1, 0));

    // Store corners and points for calibration
    corners_list.push_back(corners);
    points_list.push_back(points);

    return (0); // Return 0 indicating successful selection of calibration image
}

/*
 Given vectors having a list of point sets and center sets, an initial camera matrix,
 this function generates the calibration and calculates the calibrated camera matrix and distortion coefficients.
 This function also returns the reprojection error after performing calibration.
 */
float calibrateCamera(std::vector<std::vector<cv::Vec3f>> &points_list, std::vector<std::vector<cv::Point2f>> &centers_list, cv::Mat &camera_matrix, cv::Mat &dist_coeff)
{
    std::vector<cv::Mat> rot, trans; // Define vectors to store rotation and translation matrices

    // Perform camera calibration using provided point sets and center sets
    float error = cv::calibrateCamera(points_list,                                                                            // Vector containing multiple point sets
                                      centers_list,                                                                           // Vector containing multiple center sets
                                      cv::Size(1280, 720),                                                                    // Size of the image frame
                                      camera_matrix,                                                                          // Initial camera matrix
                                      dist_coeff,                                                                             // Initial distortion coefficients
                                      rot,                                                                                    // Vector to store rotation matrices
                                      trans,                                                                                  // Vector to store translation matrices
                                      cv::CALIB_FIX_ASPECT_RATIO,                                                             // Flag to fix aspect ratio during calibration
                                      cv::TermCriteria(cv::TermCriteria::MAX_ITER + cv::TermCriteria::EPS, 30, DBL_EPSILON)); // Termination criteria for the iterative algorithm

    return (error); // Return reprojection error after calibration
}

/*
 Given camera matrix and distance coefficients,
 this function saves the current calibration into a CSV file to be retrieved later for calculating camera pose.
 */
int saveCalibration(cv::Mat &camera_matrix, cv::Mat &dist_coeff)
{
    std::string fileName = "circlegrid.csv";               // Define the filename for the CSV file
    char *fileName_char = new char[fileName.length() + 1]; // Convert string to char array
    strcpy(fileName_char, fileName.c_str());               // Copy contents of string to char array

    std::string columnName = "camera_matrix";                 // Define label for camera matrix data in CSV
    char *labelName_char = new char[columnName.length() + 1]; // Convert string to char array
    strcpy(labelName_char, columnName.c_str());               // Copy contents of string to char array

    std::vector<float> camVector; // Define vector to store flattened camera matrix values
    for (int i = 0; i < camera_matrix.rows; i++)
    {
        for (int j = 0; j < camera_matrix.cols; j++)
        {
            float f_val = (float)camera_matrix.at<double>(i, j); // Convert camera matrix element to float
            camVector.push_back(f_val);                          // Add camera matrix element to the vector
        }
    }
    append_image_data_csv(fileName_char, labelName_char, camVector); // Append camera matrix data to CSV file

    columnName = "distortion_coeff";                      // Define label for distortion coefficients data in CSV
    char *label_char = new char[columnName.length() + 1]; // Convert string to char array
    strcpy(label_char, columnName.c_str());               // Copy contents of string to char array

    std::vector<float> distVector; // Define vector to store flattened distortion coefficients values
    for (int i = 0; i < dist_coeff.rows; i++)
    {
        for (int j = 0; j < dist_coeff.cols; j++)
        {
            float f_val = (float)dist_coeff.at<double>(i, j); // Convert distortion coefficients element to float
            distVector.push_back(f_val);                      // Add distortion coefficients element to the vector
        }
    }
    append_image_data_csv(fileName_char, label_char, distVector); // Append distortion coefficients data to CSV file

    return (0); // Return 0 indicating successful saving of calibration data
}

/*
 Given the CSV with calibration stats,
 this function retrieves the calibrated camera matrix and distortion coefficients.
 */
int readCalibration(std::string csv_filename, cv::Mat &camera_matrix, cv::Mat &dist_coeff)
{
    char *fname_char = new char[csv_filename.length() + 1]; // Convert CSV filename to char array
    strcpy(fname_char, csv_filename.c_str());               // Copy contents of the CSV filename to char array

    std::cout << "Retrieving saved calibration..." << std::endl; // Print message indicating calibration data retrieval

    std::vector<char *> featureName;                    // Define vector to store feature names
    std::vector<std::vector<float>> data;               // Define vector of vectors for calibration data
    read_image_data_csv(fname_char, featureName, data); // Read calibration data from CSV file

    // Extract camera matrix from data
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

    // Extract distortion coefficients from data
    for (int i = 0; i < 5; i++)
    {
        dist_coeff.at<float>(0, i) = data[1][i]; // Assign values from data to distortion coefficients matrix
    }

    return (0); // Return 0 indicating successful calibration data retrieval
}

/*
 Given vector containing current point set and center set, calibrated camera matrix and distortion coefficients,
 this function estimates the position of the camera relative to the target and populates arrays with rotation and translation data.
 */
int calcCameraPosition(std::vector<cv::Vec3f> &points, std::vector<cv::Point2f> &centers, cv::Mat &camera_matrix, cv::Mat &dist_coeff, cv::Mat &rot, cv::Mat &trans)
{
    cv::solvePnP(points, centers, camera_matrix, dist_coeff, rot, trans); // Solve PnP problem to estimate camera position

    return (0); // Return 0 indicating successful calculation of camera position
}

/*
 Given a cv::Mat of the image frame, calibrated camera matrix, distortion coefficients, rotation and translation data
 from the current estimated camera position, this function projects 3D world coordinates of axes to image pixel
 coordinates on the image frame and draws lines between these points to generate the 3D axes at origin.
 */
int draw3dAxes(cv::Mat &src, cv::Mat &camera_matrix, cv::Mat &dist_coeff, cv::Mat &rot, cv::Mat &trans)
{
    std::vector<cv::Vec3f> points;          // Define a vector to store 3D points
    points.push_back(cv::Vec3f({0, 0, 0})); // Add origin point to the vector
    points.push_back(cv::Vec3f({2, 0, 0})); // Add point along X-axis to the vector
    points.push_back(cv::Vec3f({0, 2, 0})); // Add point along Y-axis to the vector
    points.push_back(cv::Vec3f({0, 0, 2})); // Add point along Z-axis to the vector

    std::vector<cv::Point2f> centers; // Define a vector to store projected 2D points

    // Project 3D points onto 2D image plane
    cv::projectPoints(points, rot, trans, camera_matrix, dist_coeff, centers);

    // Draw X-axis arrow on the source image
    cv::arrowedLine(src, centers[0], centers[1], cv::Scalar(0, 0, 255), 5); // Draw X-axis arrow in red
    // Draw Y-axis arrow on the source image
    cv::arrowedLine(src, centers[0], centers[2], cv::Scalar(0, 255, 0), 5); // Draw Y-axis arrow in green
    // Draw Z-axis arrow on the source image
    cv::arrowedLine(src, centers[0], centers[3], cv::Scalar(255, 0, 0), 5); // Draw Z-axis arrow in blue

    return (0); // Return 0 indicating successful drawing of 3D axes on the source image
}

/*
 Given a cv::Mat of the image frame, calibrated camera matrix, distortion coefficients, rotation and translation data
 from the current estimated camera position, this function projects 3D world vertices of virtual shapes to image pixel
 coordinates on the image frame and draws lines between them to generate 3D virtual objects on the target.
 */
int draw3dObject(cv::Mat &src, cv::Mat &camera_matrix, cv::Mat &dist_coeff, cv::Mat &rot, cv::Mat &trans)
{

    // CUBE
    std::vector<cv::Vec3f> points; // Define a vector to store 3D points

    // Define vertices for a cube
    points.push_back(cv::Vec3f({8, 5, 0})); // Bottom right base
    points.push_back(cv::Vec3f({8, 5, 2})); // Top right base
    points.push_back(cv::Vec3f({6, 5, 0})); // Bottom left base
    points.push_back(cv::Vec3f({6, 5, 2})); // Top left base
    points.push_back(cv::Vec3f({8, 3, 0})); // Bottom right top
    points.push_back(cv::Vec3f({8, 3, 2})); // Top right top
    points.push_back(cv::Vec3f({6, 3, 0})); // Bottom left top
    points.push_back(cv::Vec3f({6, 3, 2})); // Top left top

    std::vector<cv::Point2f> centers; // Define a vector to store projected 2D points

    // Project 3D points to 2D image coordinates
    cv::projectPoints(points, rot, trans, camera_matrix, dist_coeff, centers);

    // Connect vertices to form the cube
    cv::line(src, centers[0], centers[2], cv::Scalar(255, 255, 0), 5); // Connect bottom right base to bottom left base (blue)
    cv::line(src, centers[4], centers[6], cv::Scalar(255, 255, 0), 5); // Connect top right base to top left base (blue)
    cv::line(src, centers[0], centers[4], cv::Scalar(255, 255, 0), 5); // Connect bottom right base to bottom right top (blue)
    cv::line(src, centers[2], centers[6], cv::Scalar(255, 255, 0), 5); // Connect bottom left base to top left base (blue)
    cv::line(src, centers[1], centers[3], cv::Scalar(255, 255, 0), 5); // Connect top right base to top right top (blue)
    cv::line(src, centers[5], centers[7], cv::Scalar(255, 255, 0), 5); // Connect top left base to top left top (blue)
    cv::line(src, centers[1], centers[5], cv::Scalar(255, 255, 0), 5); // Connect top right base to top left base (blue)
    cv::line(src, centers[3], centers[7], cv::Scalar(255, 255, 0), 5); // Connect top right top to top left top (blue)
    cv::line(src, centers[0], centers[1], cv::Scalar(255, 255, 0), 5); // Connect bottom right base to top right base (blue)

    points.clear();
    centers.clear();

    return (0);
}

//********************************************Extension 2- Replace the target with an image****************************************/

/*
 Given a cv::Mat of the image frame, a cv::Mat of the output frame, calibrated camera matrix, distortion coefficients, rotation & translation data and filename for artwork image,
 this function reads the artwork image and draws artwork image on the target using perspective transformation.
 */
int drawOnTarget(cv::Mat &src, cv::Mat &dst, cv::Mat &camera_matrix, cv::Mat &dist_coeff, cv::Mat &rot, cv::Mat &trans, std::string img_filename)
{
    // Define arrays to hold input and output quadrilaterals for perspective transformation
    cv::Point2f inputQuad[4];
    cv::Point2f outputQuad[4];

    // Create matrices for perspective transformation and canvas (artwork image)
    cv::Mat lambda(2, 4, CV_32FC1);
    cv::Mat canvas;

    // Read the artwork image from the provided filename
    canvas = cv::imread(img_filename, 1);

    // Initialize lambda matrix with zeros
    lambda = cv::Mat::zeros(canvas.rows, canvas.cols, canvas.type());

    // Define the input quad points (corners) for perspective transformation
    inputQuad[0] = cv::Point2f(0, 0);
    inputQuad[1] = cv::Point2f(canvas.cols, 0);
    inputQuad[2] = cv::Point2f(canvas.cols - 1, canvas.rows - 1);
    inputQuad[3] = cv::Point2f(0, canvas.rows - 1);

    // Define 3D points representing the target
    std::vector<cv::Vec3f> points;
    points.push_back(cv::Vec3f({-3, 9, 0}));
    points.push_back(cv::Vec3f({13, 9, 0}));
    points.push_back(cv::Vec3f({13, -2, 0}));
    points.push_back(cv::Vec3f({-3, -2, 0}));

    // Project 3D points of the target onto the image plane
    std::vector<cv::Point2f> centers;
    cv::projectPoints(points, rot, trans, camera_matrix, dist_coeff, centers);

    // Define output quad points based on the projected 3D points
    outputQuad[0] = centers[0];
    outputQuad[1] = centers[1];
    outputQuad[2] = centers[2];
    outputQuad[3] = centers[3];

    // Create vertices and polygons for filling the target area with black color
    std::vector<cv::Point> vertices{outputQuad[0], outputQuad[1], outputQuad[2], outputQuad[3]};
    std::vector<std::vector<cv::Point>> pts{vertices};
    cv::fillPoly(src, pts, cv::Scalar(0, 0, 0));

    // Calculate the perspective transformation matrix
    lambda = getPerspectiveTransform(inputQuad, outputQuad);

    // Apply perspective transformation to the artwork image
    warpPerspective(canvas, dst, lambda, dst.size());

    // Replace the target area in the output frame with the warped artwork image
    src.copyTo(dst, src);

    return (0);
}
