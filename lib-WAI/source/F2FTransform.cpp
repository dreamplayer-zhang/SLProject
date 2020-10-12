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

#include <thread>

bool F2FTransform::FindTransform(const WAIFrame&           f1,
                                 const WAIFrame&           f2,
                                 int                       matchWindow,
                                 cv::Mat&                  tcw,
                                 std::vector<cv::Point3f>& vP3D,
                                 std::vector<bool>&        vbTriangulated,
                                 int                       nbIterations,
                                 float                     sigma)
{
    std::vector<Match> m = featureMatch(f1, f2, matchWindow);
    int N = m.size();

    if (N < 20)
        return false;

    // Indices for minimum set selection
    std::vector<size_t> vAllIndices;
    vAllIndices.reserve(N);
    std::vector<size_t> vAvailableIndices;

    for (int i = 0; i < N; i++)
    {
        vAllIndices.push_back(i);
    }

    // Generate sets of 8 points for each RANSAC iteration
    std::vector<std::vector<size_t>> vSets; //Set of 8 points
    vSets = std::vector<std::vector<size_t>>(nbIterations, std::vector<size_t>(8, 0));

    for (int it = 0; it < nbIterations; it++)
    {
        vAvailableIndices = vAllIndices;

        // Select a minimum set
        for (size_t j = 0; j < 8; j++)
        {
            int randi = DUtils::Random::RandomInt(0, (int)vAvailableIndices.size() - 1);
            int idx   = (int)vAvailableIndices[randi];

            vSets[it][j] = idx;

            vAvailableIndices[randi] = vAvailableIndices.back();
            vAvailableIndices.pop_back();
        }
    }

    // Launch threads to compute in parallel a fundamental matrix and a homography
    std::vector<bool> inliersH, inliersF;
    float             SH, SF;
    cv::Mat           H, F;

   // thread threadH(&F2FTransform::FindHomography, ref(m), ref(vSets), ref(f1.mvKeysUn), ref(f2.mvKeysUn), ref(inliersH), ref(SH), ref(H), 50, 1.0);
   // thread threadF(&F2FTransform::FindHomography, ref(m), ref(vSets), ref(f1.mvKeysUn), ref(f2.mvKeysUn), ref(inliersF), ref(SF), ref(F), 50, 1.0);

    F2FTransform::FindHomography(m,vSets,f1.mvKeysUn,f2.mvKeysUn,inliersH,SH,H, 50, 1.0);
    F2FTransform::FindHomography(m,vSets,f1.mvKeysUn,f2.mvKeysUn,inliersF,SF,F, 50, 1.0);
    // Wait until both threads have finished
    //threadH.join();
    //threadF.join();

    // Compute ratio of scores
    float RH = SH / (SH + SF);

    cv::Mat R = cv::Mat::eye(4, 4, CV_32F);
    cv::Mat T = cv::Mat::eye(4, 4, CV_32F);

    // Try to reconstruct from homography or fundamental depending on the ratio (0.40-0.45)
    bool ret;
    if (RH > 0.40)
        ret = ReconstructH(m, inliersH, f1.mvKeysUn, f2.mvKeysUn, H, f1.mK, R, T, vP3D, vbTriangulated, 1.0, 10, sigma);
    else //if(pF_HF>0.6)
        ret = ReconstructF(m, inliersF, f1.mvKeysUn, f2.mvKeysUn, F, f1.mK, R, T, vP3D, vbTriangulated, 1.0, 10, sigma);

    tcw = cv::Mat::eye(4, 4, CV_32F);
    if (ret)
    {
        R.copyTo(tcw.rowRange(0, 3).colRange(0, 3));
        T.copyTo(tcw.rowRange(0, 3).col(3));
    }

    return ret;
}

bool F2FTransform::FindTransform(const WAIFrame&           f1,
                                 const WAIFrame&           f2,
                                 int                       matchWindow,
                                 cv::Mat&                  tcw,
                                 int                       nbIterations,
                                 float                     sigma)
{

    std::vector<cv::Point3f> vP3D;
    std::vector<bool>        vbTriangulated;
    return FindTransform(f1, f2, 10, tcw, vP3D, vbTriangulated, 100, 1.0);
}

void F2FTransform::FindHomography(std::vector<Match>&               m,
                                  std::vector<std::vector<size_t>>& vSets,
                                  const std::vector<cv::KeyPoint>&  vKeysUn1,
                                  const std::vector<cv::KeyPoint>&  vKeysUn2,
                                  std::vector<bool>&                inliers,
                                  float&                            score,
                                  cv::Mat&                          H,
                                  int                               iter,
                                  float                             sigma)
{
    const int N = (int)m.size();

    // Normalize coordinates
    std::vector<cv::Point2f> vPn1, vPn2;
    cv::Mat                  T1, T2;
    Normalize(vKeysUn1, vPn1, T1);
    Normalize(vKeysUn2, vPn2, T2);

    cv::Mat T2inv = T2.inv();

    // Best Results variables
    score            = 0.0;
    inliers = std::vector<bool>(N, false);

    // Iteration variables
    std::vector<cv::Point2f> vPn1i(8);
    std::vector<cv::Point2f> vPn2i(8);
    cv::Mat                  H21i, H12i;
    std::vector<bool>        vbCurrentInliers(N, false);
    float                    currentScore;

    // Perform all RANSAC iterations and save the solution with highest score
    for (int it = 0; it < iter; it++)
    {
        // Select a minimum set
        for (size_t j = 0; j < 8; j++)
        {
            int idx = (int)vSets[it][j];
            vPn1i[j] = vPn1[m[idx].first];
            vPn2i[j] = vPn2[m[idx].second];
        }

        cv::Mat Hn = ComputeH21(vPn1i, vPn2i);
        H21i       = T2inv * Hn * T1;
        H12i       = H21i.inv();

        currentScore = CheckHomography(m, vKeysUn1, vKeysUn2, H21i, H12i, vbCurrentInliers, sigma);

        if (currentScore > score)
        {
            H       = H21i.clone();
            inliers = vbCurrentInliers;
            score   = currentScore;
        }
    }
}

void F2FTransform::FindFundamental(std::vector<Match>&               m,
                                   std::vector<std::vector<size_t>>& vSets,
                                   const std::vector<cv::KeyPoint>&  vKeysUn1,
                                   const std::vector<cv::KeyPoint>&  vKeysUn2,
                                   std::vector<bool>&                inliers,
                                   float&                            score,
                                   cv::Mat&                          F21,
                                   int                               iter,
                                   float                             sigma)
{
    // Number of putative matches
    const int N = (int)m.size();

    // Normalize coordinates
    std::vector<cv::Point2f> vPn1, vPn2;
    cv::Mat                  T1, T2;
    Normalize(vKeysUn1, vPn1, T1);
    Normalize(vKeysUn2, vPn2, T2);
    cv::Mat T2t = T2.t();

    // Best Results variables
    score            = 0.0;
    inliers = std::vector<bool>(N, false);

    // Iteration variables
    std::vector<cv::Point2f> vPn1i(8);
    std::vector<cv::Point2f> vPn2i(8);
    cv::Mat                  F21i;
    std::vector<bool>        vbCurrentInliers(N, false);
    float                    currentScore;

    // Perform all RANSAC iterations and save the solution with highest score
    for (int it = 0; it < iter; it++)
    {
        // Select a minimum set
        for (int j = 0; j < 8; j++)
        {
            int idx = (int)vSets[it][j];

            vPn1i[j] = vPn1[m[idx].first];
            vPn2i[j] = vPn2[m[idx].second];
        }

        cv::Mat Fn = ComputeF21(vPn1i, vPn2i);

        F21i = T2t * Fn * T1;

        currentScore = CheckFundamental(m, vKeysUn1, vKeysUn2, F21i, vbCurrentInliers, sigma);

        if (currentScore > score)
        {
            F21     = F21i.clone();
            inliers = vbCurrentInliers;
            score   = currentScore;
        }
    }
}

cv::Mat F2FTransform::ComputeH21(const std::vector<cv::Point2f>& vP1, const std::vector<cv::Point2f>& vP2)
{
    const int N = (int)vP1.size();

    cv::Mat A(2 * N, 9, CV_32F);

    for (int i = 0; i < N; i++)
    {
        const float u1 = vP1[i].x;
        const float v1 = vP1[i].y;
        const float u2 = vP2[i].x;
        const float v2 = vP2[i].y;

        A.at<float>(2 * i, 0) = 0.0;
        A.at<float>(2 * i, 1) = 0.0;
        A.at<float>(2 * i, 2) = 0.0;
        A.at<float>(2 * i, 3) = -u1;
        A.at<float>(2 * i, 4) = -v1;
        A.at<float>(2 * i, 5) = -1;
        A.at<float>(2 * i, 6) = v2 * u1;
        A.at<float>(2 * i, 7) = v2 * v1;
        A.at<float>(2 * i, 8) = v2;

        A.at<float>(2 * i + 1, 0) = u1;
        A.at<float>(2 * i + 1, 1) = v1;
        A.at<float>(2 * i + 1, 2) = 1;
        A.at<float>(2 * i + 1, 3) = 0.0;
        A.at<float>(2 * i + 1, 4) = 0.0;
        A.at<float>(2 * i + 1, 5) = 0.0;
        A.at<float>(2 * i + 1, 6) = -u2 * u1;
        A.at<float>(2 * i + 1, 7) = -u2 * v1;
        A.at<float>(2 * i + 1, 8) = -u2;
    }

    cv::Mat u, w, vt;

    cv::SVDecomp(A, w, u, vt, cv::SVD::MODIFY_A | cv::SVD::FULL_UV);

    return vt.row(8).reshape(0, 3);
}

cv::Mat F2FTransform::ComputeF21(const std::vector<cv::Point2f>& vP1, const std::vector<cv::Point2f>& vP2)
{
    const int N = (int)vP1.size();

    cv::Mat A(N, 9, CV_32F);

    for (int i = 0; i < N; i++)
    {
        const float u1 = vP1[i].x;
        const float v1 = vP1[i].y;
        const float u2 = vP2[i].x;
        const float v2 = vP2[i].y;

        A.at<float>(i, 0) = u2 * u1;
        A.at<float>(i, 1) = u2 * v1;
        A.at<float>(i, 2) = u2;
        A.at<float>(i, 3) = v2 * u1;
        A.at<float>(i, 4) = v2 * v1;
        A.at<float>(i, 5) = v2;
        A.at<float>(i, 6) = u1;
        A.at<float>(i, 7) = v1;
        A.at<float>(i, 8) = 1;
    }

    cv::Mat u, w, vt;

    cv::SVDecomp(A, w, u, vt, cv::SVD::MODIFY_A | cv::SVD::FULL_UV);

    cv::Mat Fpre = vt.row(8).reshape(0, 3);

    cv::SVDecomp(Fpre, w, u, vt, cv::SVD::MODIFY_A | cv::SVD::FULL_UV);

    w.at<float>(2) = 0;

    return u * cv::Mat::diag(w) * vt;
}

float F2FTransform::CheckHomography(std::vector<Match>&              m,
                                    const std::vector<cv::KeyPoint>& vKeysUn1,
                                    const std::vector<cv::KeyPoint>& vKeysUn2,
                                    const cv::Mat&                   H21,
                                    const cv::Mat&                   H12,
                                    std::vector<bool>&               inliers,
                                    float                            sigma)
{
    const int N = (int)m.size();

    const float h11 = H21.at<float>(0, 0);
    const float h12 = H21.at<float>(0, 1);
    const float h13 = H21.at<float>(0, 2);
    const float h21 = H21.at<float>(1, 0);
    const float h22 = H21.at<float>(1, 1);
    const float h23 = H21.at<float>(1, 2);
    const float h31 = H21.at<float>(2, 0);
    const float h32 = H21.at<float>(2, 1);
    const float h33 = H21.at<float>(2, 2);

    const float h11inv = H12.at<float>(0, 0);
    const float h12inv = H12.at<float>(0, 1);
    const float h13inv = H12.at<float>(0, 2);
    const float h21inv = H12.at<float>(1, 0);
    const float h22inv = H12.at<float>(1, 1);
    const float h23inv = H12.at<float>(1, 2);
    const float h31inv = H12.at<float>(2, 0);
    const float h32inv = H12.at<float>(2, 1);
    const float h33inv = H12.at<float>(2, 2);

    inliers.resize(N);

    float score = 0;

    const float th = 5.991f;

    const float invSigmaSquare = 1.0f / (sigma * sigma);

    for (int i = 0; i < N; i++)
    {
        bool bIn = true;

        const cv::KeyPoint& kp1 = vKeysUn1[m[i].first];
        const cv::KeyPoint& kp2 = vKeysUn2[m[i].second];

        const float u1 = kp1.pt.x;
        const float v1 = kp1.pt.y;
        const float u2 = kp2.pt.x;
        const float v2 = kp2.pt.y;

        // Reprojection error in first image
        // x2in1 = H12*x2

        const float w2in1inv = 1.0f / (h31inv * u2 + h32inv * v2 + h33inv);
        const float u2in1    = (h11inv * u2 + h12inv * v2 + h13inv) * w2in1inv;
        const float v2in1    = (h21inv * u2 + h22inv * v2 + h23inv) * w2in1inv;

        const float squareDist1 = (u1 - u2in1) * (u1 - u2in1) + (v1 - v2in1) * (v1 - v2in1);

        const float chiSquare1 = squareDist1 * invSigmaSquare;

        if (chiSquare1 > th)
            bIn = false;
        else
            score += th - chiSquare1;

        // Reprojection error in second image
        // x1in2 = H21*x1

        const float w1in2inv = 1.0f / (h31 * u1 + h32 * v1 + h33);
        const float u1in2    = (h11 * u1 + h12 * v1 + h13) * w1in2inv;
        const float v1in2    = (h21 * u1 + h22 * v1 + h23) * w1in2inv;

        const float squareDist2 = (u2 - u1in2) * (u2 - u1in2) + (v2 - v1in2) * (v2 - v1in2);

        const float chiSquare2 = squareDist2 * invSigmaSquare;

        if (chiSquare2 > th)
            bIn = false;
        else
            score += th - chiSquare2;

        if (bIn)
            inliers[i] = true;
        else
            inliers[i] = false;
    }

    return score;
}

float F2FTransform::CheckFundamental(std::vector<Match>&              m,
                                     const std::vector<cv::KeyPoint>& vKeysUn1,
                                     const std::vector<cv::KeyPoint>& vKeysUn2,
                                     const cv::Mat&                   F21,
                                     std::vector<bool>&               inliers,
                                     float                            sigma)
{
    const int N = (int)m.size();

    const float f11 = F21.at<float>(0, 0);
    const float f12 = F21.at<float>(0, 1);
    const float f13 = F21.at<float>(0, 2);
    const float f21 = F21.at<float>(1, 0);
    const float f22 = F21.at<float>(1, 1);
    const float f23 = F21.at<float>(1, 2);
    const float f31 = F21.at<float>(2, 0);
    const float f32 = F21.at<float>(2, 1);
    const float f33 = F21.at<float>(2, 2);

    inliers.resize(N);

    float score = 0;

    const float th      = 3.841f;
    const float thScore = 5.991f;

    const float invSigmaSquare = 1.0f / (sigma * sigma);

    for (int i = 0; i < N; i++)
    {
        bool bIn = true;

        const cv::KeyPoint& kp1 = vKeysUn1[m[i].first];
        const cv::KeyPoint& kp2 = vKeysUn2[m[i].second];

        const float u1 = kp1.pt.x;
        const float v1 = kp1.pt.y;
        const float u2 = kp2.pt.x;
        const float v2 = kp2.pt.y;

        // Reprojection error in second image
        // l2=F21x1=(a2,b2,c2)

        const float a2 = f11 * u1 + f12 * v1 + f13;
        const float b2 = f21 * u1 + f22 * v1 + f23;
        const float c2 = f31 * u1 + f32 * v1 + f33;

        const float num2 = a2 * u2 + b2 * v2 + c2;

        const float squareDist1 = num2 * num2 / (a2 * a2 + b2 * b2);

        const float chiSquare1 = squareDist1 * invSigmaSquare;

        if (chiSquare1 > th)
            bIn = false;
        else
            score += thScore - chiSquare1;

        // Reprojection error in second image
        // l1 =x2tF21=(a1,b1,c1)

        const float a1 = f11 * u2 + f21 * v2 + f31;
        const float b1 = f12 * u2 + f22 * v2 + f32;
        const float c1 = f13 * u2 + f23 * v2 + f33;

        const float num1 = a1 * u1 + b1 * v1 + c1;

        const float squareDist2 = num1 * num1 / (a1 * a1 + b1 * b1);

        const float chiSquare2 = squareDist2 * invSigmaSquare;

        if (chiSquare2 > th)
            bIn = false;
        else
            score += thScore - chiSquare2;

        if (bIn)
            inliers[i] = true;
        else
            inliers[i] = false;
    }

    return score;
}

bool F2FTransform::ReconstructF(std::vector<Match>&              m,
                                std::vector<bool>&               inliers,
                                const std::vector<cv::KeyPoint>& vKeysUn1,
                                const std::vector<cv::KeyPoint>& vKeysUn2,
                                cv::Mat&                         F21,
                                const cv::Mat&                   K,
                                cv::Mat&                         R21,
                                cv::Mat&                         t21,
                                std::vector<cv::Point3f>&        vP3D,
                                std::vector<bool>&               vbTriangulated,
                                float                            minParallax,
                                int                              minTriangulated,
                                float                            sigma)
{
    int N = 0;
    for (size_t i = 0, iend = inliers.size(); i < iend; i++)
        if (inliers[i])
            N++;
    

    // Compute Essential Matrix from Fundamental Matrix
    cv::Mat E21 = K.t() * F21 * K;

    cv::Mat R1, R2, t;

    // Recover the 4 motion hypotheses
    DecomposeE(E21, R1, R2, t);

    cv::Mat t1 = t;
    cv::Mat t2 = -t;

    // Reconstruct with the 4 hyphoteses and check
    std::vector<cv::Point3f> vP3D1, vP3D2, vP3D3, vP3D4;
    std::vector<bool>        vbTriangulated1, vbTriangulated2, vbTriangulated3, vbTriangulated4;
    float                    parallax1, parallax2, parallax3, parallax4;
    float                    sigma2 = sigma * sigma;

    int nGood1 = CheckRT(R1, t1, vKeysUn1, vKeysUn2, m, inliers, K, vP3D1, 4.0f * sigma2, vbTriangulated1, parallax1);
    int nGood2 = CheckRT(R2, t1, vKeysUn1, vKeysUn2, m, inliers, K, vP3D2, 4.0f * sigma2, vbTriangulated2, parallax2);
    int nGood3 = CheckRT(R1, t2, vKeysUn1, vKeysUn2, m, inliers, K, vP3D3, 4.0f * sigma2, vbTriangulated3, parallax3);
    int nGood4 = CheckRT(R2, t2, vKeysUn1, vKeysUn2, m, inliers, K, vP3D4, 4.0f * sigma2, vbTriangulated4, parallax4);

    int maxGood = max(nGood1, max(nGood2, max(nGood3, nGood4)));

    R21 = cv::Mat();
    t21 = cv::Mat();

    int nMinGood = max(static_cast<int>(0.9 * N), minTriangulated);

    int nsimilar = 0;
    if (nGood1 > 0.7 * maxGood)
        nsimilar++;
    if (nGood2 > 0.7 * maxGood)
        nsimilar++;
    if (nGood3 > 0.7 * maxGood)
        nsimilar++;
    if (nGood4 > 0.7 * maxGood)
        nsimilar++;

    // If there is not a clear winner or not enough triangulated points reject initialization
    if (maxGood < nMinGood || nsimilar > 1)
    {
        return false;
    }

    // If best reconstruction has enough parallax initialize
    if (maxGood == nGood1)
    {
        if (parallax1 > minParallax)
        {
            vP3D           = vP3D1;
            vbTriangulated = vbTriangulated1;

            R1.copyTo(R21);
            t1.copyTo(t21);
            return true;
        }
    }
    else if (maxGood == nGood2)
    {
        if (parallax2 > minParallax)
        {
            vP3D           = vP3D2;
            vbTriangulated = vbTriangulated2;

            R2.copyTo(R21);
            t1.copyTo(t21);
            return true;
        }
    }
    else if (maxGood == nGood3)
    {
        if (parallax3 > minParallax)
        {
            vP3D           = vP3D3;
            vbTriangulated = vbTriangulated3;

            R1.copyTo(R21);
            t2.copyTo(t21);
            return true;
        }
    }
    else if (maxGood == nGood4)
    {
        if (parallax4 > minParallax)
        {
            vP3D           = vP3D4;
            vbTriangulated = vbTriangulated4;

            R2.copyTo(R21);
            t2.copyTo(t21);
            return true;
        }
    }

    return false;
}

bool F2FTransform::ReconstructH(std::vector<Match>&              m,
                                std::vector<bool>&               inliers,
                                const std::vector<cv::KeyPoint>& vKeysUn1,
                                const std::vector<cv::KeyPoint>& vKeysUn2,
                                cv::Mat&                         H21,
                                const cv::Mat&                   K,
                                cv::Mat&                         R21,
                                cv::Mat&                         t21,
                                std::vector<cv::Point3f>&        vP3D,
                                std::vector<bool>&               vbTriangulated,
                                float                            minParallax,
                                int                              minTriangulated,
                                float                            sigma)
{
    int N = 0;
    for (size_t i = 0, iend = inliers.size(); i < iend; i++)
        if (inliers[i])
            N++;

    // We recover 8 motion hypotheses using the method of Faugeras et al.
    // Motion and structure from motion in a piecewise planar environment.
    // International Journal of Pattern Recognition and Artificial Intelligence, 1988

    cv::Mat invK = K.inv();
    cv::Mat A    = invK * H21 * K;

    cv::Mat U, w, Vt, V;
    cv::SVD::compute(A, w, U, Vt, cv::SVD::FULL_UV);
    V = Vt.t();

    float s = (float)(cv::determinant(U) * cv::determinant(Vt));

    float d1 = w.at<float>(0);
    float d2 = w.at<float>(1);
    float d3 = w.at<float>(2);

    if (d1 / d2 < 1.00001 || d2 / d3 < 1.00001)
    {
        return false;
    }

    std::vector<cv::Mat> vR, vt, vn;
    vR.reserve(8);
    vt.reserve(8);
    vn.reserve(8);

    //n'=[x1 0 x3] 4 posibilities e1=e3=1, e1=1 e3=-1, e1=-1 e3=1, e1=e3=-1
    float aux1 = sqrt((d1 * d1 - d2 * d2) / (d1 * d1 - d3 * d3));
    float aux3 = sqrt((d2 * d2 - d3 * d3) / (d1 * d1 - d3 * d3));
    float x1[] = {aux1, aux1, -aux1, -aux1};
    float x3[] = {aux3, -aux3, aux3, -aux3};

    //case d'=d2
    float aux_stheta = sqrt((d1 * d1 - d2 * d2) * (d2 * d2 - d3 * d3)) / ((d1 + d3) * d2);

    float ctheta   = (d2 * d2 + d1 * d3) / ((d1 + d3) * d2);
    float stheta[] = {aux_stheta, -aux_stheta, -aux_stheta, aux_stheta};

    for (int i = 0; i < 4; i++)
    {
        cv::Mat Rp         = cv::Mat::eye(3, 3, CV_32F);
        Rp.at<float>(0, 0) = ctheta;
        Rp.at<float>(0, 2) = -stheta[i];
        Rp.at<float>(2, 0) = stheta[i];
        Rp.at<float>(2, 2) = ctheta;

        cv::Mat R = s * U * Rp * Vt;
        vR.push_back(R);

        cv::Mat tp(3, 1, CV_32F);
        tp.at<float>(0) = x1[i];
        tp.at<float>(1) = 0;
        tp.at<float>(2) = -x3[i];
        tp *= d1 - d3;

        cv::Mat t = U * tp;
        vt.push_back(t / cv::norm(t));

        cv::Mat np(3, 1, CV_32F);
        np.at<float>(0) = x1[i];
        np.at<float>(1) = 0;
        np.at<float>(2) = x3[i];

        cv::Mat n = V * np;
        if (n.at<float>(2) < 0)
            n = -n;
        vn.push_back(n);
    }

    //case d'=-d2
    float aux_sphi = sqrt((d1 * d1 - d2 * d2) * (d2 * d2 - d3 * d3)) / ((d1 - d3) * d2);

    float cphi   = (d1 * d3 - d2 * d2) / ((d1 - d3) * d2);
    float sphi[] = {aux_sphi, -aux_sphi, -aux_sphi, aux_sphi};

    for (int i = 0; i < 4; i++)
    {
        cv::Mat Rp         = cv::Mat::eye(3, 3, CV_32F);
        Rp.at<float>(0, 0) = cphi;
        Rp.at<float>(0, 2) = sphi[i];
        Rp.at<float>(1, 1) = -1;
        Rp.at<float>(2, 0) = sphi[i];
        Rp.at<float>(2, 2) = -cphi;

        cv::Mat R = s * U * Rp * Vt;
        vR.push_back(R);

        cv::Mat tp(3, 1, CV_32F);
        tp.at<float>(0) = x1[i];
        tp.at<float>(1) = 0;
        tp.at<float>(2) = x3[i];
        tp *= d1 + d3;

        cv::Mat t = U * tp;
        vt.push_back(t / cv::norm(t));

        cv::Mat np(3, 1, CV_32F);
        np.at<float>(0) = x1[i];
        np.at<float>(1) = 0;
        np.at<float>(2) = x3[i];

        cv::Mat n = V * np;
        if (n.at<float>(2) < 0)
            n = -n;
        vn.push_back(n);
    }

    int                      bestGood        = 0;
    int                      secondBestGood  = 0;
    int                      bestSolutionIdx = -1;
    float                    bestParallax    = -1;
    std::vector<cv::Point3f> bestP3D;
    std::vector<bool>        bestTriangulated;
    float                    sigma2 = sigma * sigma;

    // Instead of applying the visibility constraints proposed in the Faugeras' paper (which could fail for points seen with low parallax)
    // We reconstruct all hypotheses and check in terms of triangulated points and parallax
    for (size_t i = 0; i < 8; i++)
    {
        float                    parallaxi;
        std::vector<cv::Point3f> vP3Di;
        std::vector<bool>        vbTriangulatedi;

        int nGood = CheckRT(vR[i],
                            vt[i],
                            vKeysUn1,
                            vKeysUn2,
                            m,
                            inliers,
                            K,
                            vP3Di,
                            4.0f * sigma2,
                            vbTriangulatedi,
                            parallaxi);

        if (nGood > bestGood)
        {
            secondBestGood   = bestGood;
            bestGood         = nGood;
            bestSolutionIdx  = (int)i;
            bestParallax     = parallaxi;
            bestP3D          = vP3Di;
            bestTriangulated = vbTriangulatedi;
        }
        else if (nGood > secondBestGood)
        {
            secondBestGood = nGood;
        }
    }

    if (bestParallax >= minParallax && bestGood > minTriangulated && bestGood > 0.9 * N)
    {
        vR[bestSolutionIdx].copyTo(R21);
        vt[bestSolutionIdx].copyTo(t21);
        vP3D           = bestP3D;
        vbTriangulated = bestTriangulated;

        return true;
    }

    return false;
}

void F2FTransform::Triangulate(const cv::Point& p1,
                              const cv::Point& p2,
                              const cv::Mat&   P1,
                              const cv::Mat&   P2,
                              cv::Mat&         x3D)
{
    cv::Mat A(4, 4, CV_32F);

    A.row(0) = p1.x * P1.row(2) - P1.row(0);
    A.row(1) = p1.y * P1.row(2) - P1.row(1);
    A.row(2) = p2.x * P2.row(2) - P2.row(0);
    A.row(3) = p2.y * P2.row(2) - P2.row(1);

    cv::Mat u, w, vt;
    cv::SVD::compute(A, w, u, vt, cv::SVD::MODIFY_A | cv::SVD::FULL_UV);
    x3D = vt.row(3).t();
    x3D = x3D.rowRange(0, 3) / x3D.at<float>(3);
}

void F2FTransform::Normalize(const std::vector<cv::KeyPoint>& vKeys,
                            std::vector<cv::Point2f>&        vNormalizedPoints,
                            cv::Mat&                         T)
{
    float     meanX = 0;
    float     meanY = 0;
    const int N     = (int)vKeys.size();

    vNormalizedPoints.resize(N);

    for (int i = 0; i < N; i++)
    {
        meanX += vKeys[i].pt.x;
        meanY += vKeys[i].pt.y;
    }

    meanX = meanX / N;
    meanY = meanY / N;

    float meanDevX = 0;
    float meanDevY = 0;

    for (int i = 0; i < N; i++)
    {
        vNormalizedPoints[i].x = vKeys[i].pt.x - meanX;
        vNormalizedPoints[i].y = vKeys[i].pt.y - meanY;

        meanDevX += fabs(vNormalizedPoints[i].x);
        meanDevY += fabs(vNormalizedPoints[i].y);
    }

    meanDevX = meanDevX / N;
    meanDevY = meanDevY / N;

    float sX = 1.0f / meanDevX;
    float sY = 1.0f / meanDevY;

    for (int i = 0; i < N; i++)
    {
        vNormalizedPoints[i].x = vNormalizedPoints[i].x * sX;
        vNormalizedPoints[i].y = vNormalizedPoints[i].y * sY;
    }

    T                 = cv::Mat::eye(3, 3, CV_32F);
    T.at<float>(0, 0) = sX;
    T.at<float>(1, 1) = sY;
    T.at<float>(0, 2) = -meanX * sX;
    T.at<float>(1, 2) = -meanY * sY;
}

int F2FTransform::CheckRT(const cv::Mat&                  R,
                         const cv::Mat&                   t,
                         const std::vector<cv::KeyPoint>& vKeys1,
                         const std::vector<cv::KeyPoint>& vKeys2,
                         const std::vector<Match>&        vMatches12,
                         std::vector<bool>&               vbMatchesInliers,
                         const cv::Mat&                   K,
                         std::vector<cv::Point3f>&        vP3D,
                         float                            th2,
                         std::vector<bool>&               vbGood,
                         float&                           parallax)
{
    // Calibration parameters
    const float fx = K.at<float>(0, 0);
    const float fy = K.at<float>(1, 1);
    const float cx = K.at<float>(0, 2);
    const float cy = K.at<float>(1, 2);

    vbGood = std::vector<bool>(vKeys1.size(), false);
    vP3D.resize(vKeys1.size());

    std::vector<float> vCosParallax;
    vCosParallax.reserve(vKeys1.size());

    // Camera 1 Projection Matrix K[I|0]
    cv::Mat P1(3, 4, CV_32F, cv::Scalar(0));
    K.copyTo(P1.rowRange(0, 3).colRange(0, 3));

    cv::Mat O1 = cv::Mat::zeros(3, 1, CV_32F);

    // Camera 2 Projection Matrix K[R|t]
    cv::Mat P2(3, 4, CV_32F);
    R.copyTo(P2.rowRange(0, 3).colRange(0, 3));
    t.copyTo(P2.rowRange(0, 3).col(3));
    P2 = K * P2;

    cv::Mat O2 = -R.t() * t;

    int nGood = 0;

    for (size_t i = 0, iend = vMatches12.size(); i < iend; i++)
    {
        if (!vbMatchesInliers[i])
            continue;

        const cv::KeyPoint& kp1 = vKeys1[vMatches12[i].first];
        const cv::KeyPoint& kp2 = vKeys2[vMatches12[i].second];
        cv::Mat             p3dC1;

        Triangulate(kp1.pt, kp2.pt, P1, P2, p3dC1);

        if (!isfinite(p3dC1.at<float>(0)) || !isfinite(p3dC1.at<float>(1)) || !isfinite(p3dC1.at<float>(2)))
        {
            vbGood[vMatches12[i].first] = false;
            continue;
        }

        // Check parallax
        cv::Mat normal1 = p3dC1 - O1;
        float   dist1   = (float)cv::norm(normal1);

        cv::Mat normal2 = p3dC1 - O2;
        float   dist2   = (float)cv::norm(normal2);

        float cosParallax = (float)(normal1.dot(normal2) / (dist1 * dist2));

        // Check depth in front of first camera (only if enough parallax, as "infinite" points can easily go to negative depth)
        if (p3dC1.at<float>(2) <= 0 && cosParallax < 0.99998)
            continue;

        // Check depth in front of second camera (only if enough parallax, as "infinite" points can easily go to negative depth)
        cv::Mat p3dC2 = R * p3dC1 + t;

        if (p3dC2.at<float>(2) <= 0 && cosParallax < 0.99998)
            continue;

        // Check reprojection error in first image
        float im1x, im1y;
        float invZ1 = 1.0f / p3dC1.at<float>(2);
        im1x        = fx * p3dC1.at<float>(0) * invZ1 + cx;
        im1y        = fy * p3dC1.at<float>(1) * invZ1 + cy;

        float squareError1 = (im1x - kp1.pt.x) * (im1x - kp1.pt.x) + (im1y - kp1.pt.y) * (im1y - kp1.pt.y);

        if (squareError1 > th2)
            continue;

        // Check reprojection error in second image
        float im2x, im2y;
        float invZ2 = 1.0f / p3dC2.at<float>(2);
        im2x        = fx * p3dC2.at<float>(0) * invZ2 + cx;
        im2y        = fy * p3dC2.at<float>(1) * invZ2 + cy;

        float squareError2 = (im2x - kp2.pt.x) * (im2x - kp2.pt.x) + (im2y - kp2.pt.y) * (im2y - kp2.pt.y);

        if (squareError2 > th2)
            continue;

        vCosParallax.push_back(cosParallax);
        vP3D[vMatches12[i].first] = cv::Point3f(p3dC1.at<float>(0), p3dC1.at<float>(1), p3dC1.at<float>(2));
        nGood++;

        if (cosParallax < 0.99998)
            vbGood[vMatches12[i].first] = true;
    }

    if (nGood > 0)
    {
        sort(vCosParallax.begin(), vCosParallax.end());

        size_t idx = min(50, int(vCosParallax.size() - 1));
        parallax   = (float)(acos(vCosParallax[idx]) * 180.0f / CV_PI);
    }
    else
        parallax = 0;

    return nGood;
}

void F2FTransform::DecomposeE(const cv::Mat& E, cv::Mat& R1, cv::Mat& R2, cv::Mat& t)
{
    cv::Mat u, w, vt;
    cv::SVD::compute(E, w, u, vt);

    u.col(2).copyTo(t);
    t = t / cv::norm(t);

    cv::Mat W(3, 3, CV_32F, cv::Scalar(0));
    W.at<float>(0, 1) = -1;
    W.at<float>(1, 0) = 1;
    W.at<float>(2, 2) = 1;

    R1 = u * W * vt;
    if (cv::determinant(R1) < 0)
        R1 = -R1;

    R2 = u * W.t() * vt;
    if (cv::determinant(R2) < 0)
        R2 = -R2;
}


#define HISTO_LENGTH 30
#define TH_HIGH 100
#define TH_LOW 50

std::vector<F2FTransform::Match> F2FTransform::featureMatch(const WAIFrame& F1, const WAIFrame& F2, int windowSize)
{
    int nmatches = 0;
    vector<int> vnMatches12 = vector<int>(F1.mvKeysUn.size(), -1);

    vector<int> rotHist[HISTO_LENGTH];
    //IF ORB
    for (int i = 0; i < HISTO_LENGTH; i++)
        rotHist[i].reserve(500);

    const float factor = 1.0f / HISTO_LENGTH;

    vector<int> vMatchedDistance(F2.mvKeysUn.size(), INT_MAX);
    vector<int> vnMatches21(F2.mvKeysUn.size(), -1);

    for (size_t i1 = 0, iend1 = F1.mvKeysUn.size(); i1 < iend1; i1++)
    {
        cv::KeyPoint kp1    = F1.mvKeysUn[i1];
        int          level1 = kp1.octave;
        if (level1 > 0)
            continue;

        vector<size_t> vIndices2 = F2.GetFeaturesInArea(F1.mvKeysUn[i1].pt.x, F1.mvKeysUn[i1].pt.y, (float)windowSize, level1, level1);

        if (vIndices2.empty())
            continue;

        cv::Mat d1 = F1.mDescriptors.row((int)i1);

        int bestDist  = INT_MAX;
        int bestDist2 = INT_MAX;
        int bestIdx2  = -1;

        for (vector<size_t>::iterator vit = vIndices2.begin(); vit != vIndices2.end(); vit++)
        {
            size_t i2 = *vit;

            cv::Mat d2 = F2.mDescriptors.row((int)i2);

            int dist = ORBmatcher::DescriptorDistance(d1, d2);

            if (vMatchedDistance[i2] <= dist)
                continue;

            if (dist < bestDist)
            {
                bestDist2 = bestDist;
                bestDist  = dist;
                bestIdx2  = (int)i2;
            }
            else if (dist < bestDist2)
            {
                bestDist2 = dist;
            }
        }

        if (bestDist <= TH_LOW)
        {
            if (bestDist < 0.8 * (float)bestDist2)
            {
                if (vnMatches21[bestIdx2] >= 0)
                {
                    vnMatches12[vnMatches21[bestIdx2]] = -1;
                    nmatches--;
                }
                vnMatches12[i1]            = bestIdx2;
                vnMatches21[bestIdx2]      = (int)i1;
                vMatchedDistance[bestIdx2] = bestDist;
                nmatches++;

                    float rot = F1.mvKeysUn[i1].angle - F2.mvKeysUn[bestIdx2].angle;
                    if (rot < 0.0)
                        rot += 360.0f;
                    int bin = (int)round(rot * factor);
                    if (bin == HISTO_LENGTH)
                        bin = 0;
                    assert(bin >= 0 && bin < HISTO_LENGTH);
                    rotHist[bin].push_back((int)i1);
            }
        }
    }

//IF ORB
        int ind1 = -1;
        int ind2 = -1;
        int ind3 = -1;

        ComputeThreeMaxima(rotHist, HISTO_LENGTH, ind1, ind2, ind3);

        for (int i = 0; i < HISTO_LENGTH; i++)
        {
            if (i == ind1 || i == ind2 || i == ind3)
                continue;
            for (size_t j = 0, jend = rotHist[i].size(); j < jend; j++)
            {
                int idx1 = rotHist[i][j];
                if (vnMatches12[idx1] >= 0)
                {
                    vnMatches12[idx1] = -1;
                    nmatches--;
                }
            }
        }

        std::vector<Match> matching;
        for (size_t i = 0, iend = vnMatches12.size(); i < iend; i++)
        {
            if (vnMatches12[i] >= 0)
            {
                matching.push_back(make_pair(i, vnMatches12[i]));
            }
        }

        return matching;
}

void F2FTransform::ComputeThreeMaxima(vector<int>* histo, const int L, int& ind1, int& ind2, int& ind3)
{
    int max1 = 0;
    int max2 = 0;
    int max3 = 0;

    for (int i = 0; i < L; i++)
    {
        const int s = (int)histo[i].size();
        if (s > max1)
        {
            max3 = max2;
            max2 = max1;
            max1 = s;
            ind3 = ind2;
            ind2 = ind1;
            ind1 = i;
        }
        else if (s > max2)
        {
            max3 = max2;
            max2 = s;
            ind3 = ind2;
            ind2 = i;
        }
        else if (s > max3)
        {
            max3 = s;
            ind3 = i;
        }
    }

    if (max2 < 0.1f * (float)max1)
    {
        ind2 = -1;
        ind3 = -1;
    }
    else if (max3 < 0.1f * (float)max1)
    {
        ind3 = -1;
    }
}
