/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/

#include <F2FTransform.h>
#include <Random.h>

#include <OrbSlam/Optimizer.h>
#include <OrbSlam/ORBmatcher.h>
#include <Eigen/Geometry>
#include <thread>

float F2FTransform::OpticalFlowMatch(const cv::Mat&            f1Gray,
                                     const cv::Mat&            f2Gray,
                                     std::vector<cv::KeyPoint>& kp1,
                                     std::vector<cv::Point2f>&  p1,
                                     std::vector<cv::Point2f>&  p2)
{
    cv::TermCriteria criteria(cv::TermCriteria::COUNT | cv::TermCriteria::EPS, 10, 0.03);

    std::vector<cv::Point2f> tmp1;
    std::vector<cv::Point2f> tmp2;
    tmp1.reserve(kp1.size());
    tmp2.reserve(kp1.size());
    for (int i = 0; i < kp1.size(); i++)
    {
        tmp1.push_back(kp1[i].pt);
    }
    std::vector<uchar>    status;
    std::vector<float>    err;
    cv::Size winSize(15, 15);

    cv::calcOpticalFlowPyrLK(
      f1Gray,
      f2Gray,
      tmp1,                      // Previous and current keypoints coordinates.The latter will be
      tmp2,                      // expanded if more good coordinates are detected during OptFlow
      status,                    // Output vector for keypoint correspondences (1 = match found)
      err,                       // Error size for each flow
      winSize,                   // Search window for each pyramid level
      1,                         // Max levels of pyramid creation
      criteria,                  // Configuration from above
      0,                         // Additional flags
      0.001);                    // Minimal Eigen threshold

    float motionAmplitude = 0;
    for (unsigned int i = 0; i < status.size(); i++)
    {
        if (status[i] && err[i] < 5)
        {
            motionAmplitude += cv::norm(tmp1[i] - tmp2[i]);
            tmp1[i].x -= f1Gray.rows/2;
            tmp2[i].x -= f1Gray.rows/2;
            tmp1[i].y -= f1Gray.cols/2;
            tmp2[i].y -= f1Gray.cols/2;
            p1.push_back(tmp1[i]);
            p2.push_back(tmp2[i]);
        }
    }
    return motionAmplitude / p1.size();
}

bool CheckCoherentRotation(cv::Mat_<double>& R)
{
    float detR = fabs(determinant(R));
    if (detR - 1.0 < 1e-07 && detR - 1.0 > -1e-07)
    {
        cerr << "det(R) != +-1.0, this is not a rotation matrix" << endl;
        return false;
    }
    return true;
}

bool F2FTransform::FindTransform(const cv::Mat            K,
                                 std::vector<cv::Point2f> p1,
                                 std::vector<cv::Point2f> p2,
                                 cv::Mat&                 tcw)
{
    std::vector<char> inliers;
    cv::Mat           F;

    if (p1.size() < 10)
        return false;

    F = cv::findFundamentalMat(
      p1, p2, // matching points
      cv::FM_RANSAC,              // RANSAC method
      1.0,                        // distance to epipolar line
      0.98);                      // confidence probability

    // Compute Essential Matrix from Fundamental Matrix
    cv::Mat E = K.t() * F * K;
    
    cv::SVD svd(E,cv::SVD::MODIFY_A);
    cv::Matx33d W(0,-1,0, 1,0,0, 0,0,1);

    cv::Mat_<double> R = svd.u * cv::Mat(W) * svd.vt; //HZ 9.19
    cv::Mat_<double> t = svd.u.col(2); //u3
    if (!CheckCoherentRotation(R))
        return false;
    tcw = cv::Mat::eye(4, 4, CV_32F);
    R.copyTo(tcw.rowRange(0, 3).colRange(0, 3));
    t.copyTo(tcw.rowRange(0, 3).col(3));

    return true;
}
/*
Mat_<double> LinearLSTriangulation(
  Point3d u,  //homogenous image point (u,v,1)
  Matx34d P,  //camera 1 matrix
  Point3d u1, //homogenous image point in 2nd camera
  Matx34d P1  //camera 2 matrix
)
{
    //build A matrix
    Matx43d A(u.x * P(2, 0) - P(0, 0), u.x * P(2, 1) - P(0, 1), u.x * P(2, 2) - P(0, 2),
              u.y * P(2, 0) - P(1, 0), u.y * P(2, 1) - P(1, 1), u.y * P(2, 2) - P(1, 2),
              u1.x * P1(2, 0) - P1(0, 0), u1.x * P1(2, 1) - P1(0, 1), u1.x * P1(2, 2) - P1(0, 2),
              u1.y * P1(2, 0) - P1(1, 0), u1.y * P1(2, 1) - P1(1, 1), u1.y * P1(2, 2) - P1(1, 2));
    //build B vector
    Matx41d B(-(u.x * P(2, 3) - P(0, 3)),
              -(u.y * P(2, 3) - P(1, 3)),
              -(u1.x * P1(2, 3) - P1(0, 3)),
              -(u1.y * P1(2, 3) - P1(1, 3)));
    //solve for X
    Mat_<double> X;
    solve(A, B, X, DECOMP_SVD);
    return X;
}

double TriangulatePoints(
  const vector<KeyPoint>& pt_set1,
  const vector<KeyPoint>& pt_set2,
  const Mat&              Kinv,
  const Matx34d&          P,
  const Matx34d&          P1,
  vector<Point3d>&        pointcloud)
{
    vector<double> reproj_error;
    for (unsigned int i = 0; i < pts_size; i++)
    {
        //convert to normalized homogeneous coordinates
        Point2f      kp = pt_set1[i].pt;
        Point3d      u(kp.x, kp.y, 1.0);
        Mat_<double> um  = Kinv * Mat_<double>(u);
        u                = um.at<Point3d>(0);
        Point2f      kp1 = pt_set2[i].pt;
        Point3d      u1(kp1.x, kp1.y, 1.0);
        Mat_<double> um1 = Kinv * Mat_<double>(u1);
        u1               = um1.at<Point3d>(0);
        //triangulate
        Mat_<double> X = LinearLSTriangulation(u, P, u1, P1);
        //calculate reprojection error
        Mat_<double> xPt_img = K * Mat(P1) * X;
        Point2f      xPt_img_(xPt_img(0) / xPt_img(2), xPt_img(1) / xPt_img(2));
        reproj_error.push_back(norm(xPt_img_ - kp1));
        //store 3D point
        pointcloud.push_back(Point3d(X(0), X(1), X(2)));
    }
    //return mean reprojection error
    Scalar me = mean(reproj_error);
    return me[0];
}
*/

cv::Mat eigen2cv(Eigen::Matrix3f m)
{
    cv::Mat r;
    r = (cv::Mat_<float>(3, 3) << m(0), m(3), m(6), m(1), m(4), m(7), m(2), m(5), m(8));
    return r;
}

cv::Mat eigen2cv4(Eigen::Matrix4f m)
{
    cv::Mat r;
    r = (cv::Mat_<double>(4, 4) << m(0), m(4), m(8), m(12), m(1), m(5), m(9), m(13), m(3), m(7), m(11), m(15));
    return r;
}

bool F2FTransform::EstimateRot(const cv::Mat            K,
                               std::vector<cv::Point2f> p1,
                               std::vector<cv::Point2f> p2,
                               cv::Mat&                 tcw)
{
    if (p1.size() < 20)
        return false;

    cv::Mat H = estimateAffine2D(p1, p2);

    float zrot = asin(min(H.at<double>(1, 0), 1.0));
    float dx = H.at<double>(0, 2);
    float dy = -H.at<double>(1, 2);

    Eigen::Vector3f v1(0, 0, K.at<double>(0, 0));
    Eigen::Vector3f v2(dx, dy, K.at<double>(0, 0));
    v1.normalize();
    v2.normalize();

    float xyrot = acos(v1.dot(v2));
    Eigen::Vector3f axis = v1.cross(v2).normalized();

    Eigen::Matrix3f m;
    m = Eigen::AngleAxisf(xyrot, axis);// * Eigen::AngleAxisf(zrot, Eigen::Vector3f::UnitZ());

    cv::Mat R = eigen2cv(m);

    tcw = cv::Mat::eye(4, 4, CV_32F);
    R.copyTo(tcw.rowRange(0, 3).colRange(0, 3));

    return true;
}

/*
bool F2FTransform::EstimateRot(const cv::Mat            K,
                               std::vector<cv::Point2f> p1,
                               std::vector<cv::Point2f> p2,
                               cv::Mat&                 tcw)
{
    if (p1.size() < 20)
        return false;

    cv::Mat H = estimateAffine2D(p1, p2);

    float c = H.at<double>(0, 0);
    float s = H.at<double>(1, 0);
    float dx = H.at<double>(0, 2);
    float dy = -H.at<double>(1, 2);

    c = c < 1.0 ? c : 1.0; 
    s = s < 1.0 ? s : 1.0; 
    c = c > -1.0 ? c : -1.0; 
    s = s > -1.0 ? s : -1.0;

    Eigen::Vector3f v1(0, 0, 1.0);
    Eigen::Vector3f vx(dx, 0, K.at<double>(0, 0));
    Eigen::Vector3f vy(0, dy, K.at<double>(0, 0));
    //Eigen::Vector3f vx(c * dx, s * dx, K.at<double>(0, 0));
    //Eigen::Vector3f vy(-s * dy, c * dy, K.at<double>(0, 0));

    vx.normalize();
    vy.normalize();

    float cx = v1.dot(vy);
    float cy = v1.dot(vx);
    Eigen::Vector3f ax = v1.cross(vy).normalized();
    Eigen::Vector3f ay = v1.cross(vx).normalized();
    Eigen::Vector3f az = Eigen::Vector3f::UnitZ();

    float xrot = acos(cx);
    float yrot = acos(cy);
    float zrot = asin(s);

    Eigen::Matrix3f m;
    m = Eigen::AngleAxisf(-xrot, ax) *
        Eigen::AngleAxisf(-yrot, ay) *
        Eigen::AngleAxisf(-zrot, az);

    cv::Mat R = eigen2cv(m);

    tcw = cv::Mat::eye(4, 4, CV_32F);
    R.copyTo(tcw.rowRange(0, 3).colRange(0, 3));

    return true;
}
*/
