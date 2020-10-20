
#ifndef F2FTRANSFORM 
#define F2FTRANSFORM

#include <opencv2/opencv.hpp>

#include <WAIHelper.h>
#include <WAIFrame.h>
#include <Eigen/Core>

class WAI_API F2FTransform
{
    typedef std::pair<int, int> Match;

public:
    // Computes in parallel a fundamental matrix and a homography
    // Selects a model and tries to recover the motion and the structure from motion
    static bool FindTransform(const WAIFrame&           f1,
                              const WAIFrame&           f2,
                              int                       matchWindow,
                              cv::Mat&                  tcw,
                              std::vector<cv::Point3f>& vP3D,
                              std::vector<bool>&        vbTriangulated,
                              int                       nbIterations,
                              float                     sigma);

    static bool FindTransform(const WAIFrame& f1,
                              const WAIFrame& f2,
                              int             matchWindow,
                              cv::Mat&        tcw,
                              int             nbIterations,
                              float           sigma);

    static bool FindTransform(const cv::Mat            K,
                              std::vector<cv::Point2f> p1,
                              std::vector<cv::Point2f> p2,
                              cv::Mat&                 tcw);

    static float OpticalFlowMatch(const cv::Mat&            f1Gray,
                                  const cv::Mat&            f2Gray,
                                  std::vector<cv::KeyPoint>& kp1,
                                  std::vector<cv::Point2f>&  p1,
                                  std::vector<cv::Point2f>&  p2);

    static bool EstimateRot(const cv::Mat            K,
                            std::vector<cv::Point2f> p1,
                            std::vector<cv::Point2f> p2,
                            cv::Mat&                 tcw);

    /*
    static bool EstimateRot(const cv::Mat K,
                            std::vector<cv::Point2f> p1,
                            std::vector<cv::Point2f> p2,
                            cv::Mat&                 tcw);
                            */

    static void Triangulate(const cv::Point& p1, const cv::Point& p2, const cv::Mat& P1, const cv::Mat& P2, cv::Mat& x3D);

private:
    static void FindHomography(std::vector<Match>&               m,
                               std::vector<std::vector<size_t>>& vSets,
                               const std::vector<cv::KeyPoint>&  vKeysUn1,
                               const std::vector<cv::KeyPoint>&  vKeysUn2,
                               std::vector<bool>&                inliers,
                               float&                            score,
                               cv::Mat&                          H,
                               int                               iter,
                               float                             sigma);

    static void FindFundamental(std::vector<Match>&               m,
                                std::vector<std::vector<size_t>>& vSets,
                                const std::vector<cv::KeyPoint>&  vKeysUn1,
                                const std::vector<cv::KeyPoint>&  vKeysUn2,
                                std::vector<bool>&                inliers,
                                float&                            score,
                                cv::Mat&                          F21,
                                int                               iter,
                                float                             sigma);

    static void FindFundamental(const std::vector<cv::KeyPoint>& vKeysUn1,
                                const std::vector<cv::KeyPoint>& vKeysUn2,
                                std::vector<bool>&               inliers,
                                float&                           score,
                                cv::Mat&                         F21,
                                float                            sigma);

    static cv::Mat ComputeH21(const std::vector<cv::Point2f>& vP1, const std::vector<cv::Point2f>& vP2);
    static cv::Mat ComputeF21(const std::vector<cv::Point2f>& vP1, const std::vector<cv::Point2f>& vP2);

    static float CheckHomography(std::vector<Match>&              m,
                                 const std::vector<cv::KeyPoint>& vKeysUn1,
                                 const std::vector<cv::KeyPoint>& vKeysUn2,
                                 const cv::Mat&                   H21,
                                 const cv::Mat&                   H12,
                                 std::vector<bool>&               inliers,
                                 float                            sigma);

    static float CheckFundamental(std::vector<Match>&              m,
                                  const std::vector<cv::KeyPoint>& vKeysUn1,
                                  const std::vector<cv::KeyPoint>& vKeysUn2,
                                  const cv::Mat&                   F21,
                                  std::vector<bool>&               inliers,
                                  float                            sigma);

    static float CheckFundamental(const std::vector<cv::KeyPoint>& vKeysUn1,
                                  const std::vector<cv::KeyPoint>& vKeysUn2,
                                  const cv::Mat&                   F21,
                                  std::vector<bool>&               inliers,
                                  float                            sigma);

    static bool  ReconstructF(std::vector<Match>&              m,
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
                              float                            sigma);

    static bool ReconstructH(std::vector<Match>&              m,
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
                             float                            sigma);

    static void  Normalize(const std::vector<cv::KeyPoint>& vKeys, std::vector<cv::Point2f>& vNormalizedPoints, cv::Mat& T);
    static int   CheckRT(const cv::Mat& R, const cv::Mat& t, const std::vector<cv::KeyPoint>& vKeys1, const std::vector<cv::KeyPoint>& vKeys2, const std::vector<Match>& vMatches12, std::vector<bool>& vbInliers, const cv::Mat& K, std::vector<cv::Point3f>& vP3D, float th2, std::vector<bool>& vbGood, float& parallax);
    static void  DecomposeE(const cv::Mat& E, cv::Mat& R1, cv::Mat& R2, cv::Mat& t);

    static std::vector<Match> featureMatch(const WAIFrame& F1, const WAIFrame& F2, int windowSize);
    static void               ComputeThreeMaxima(std::vector<int>* histo, const int L, int& ind1, int& ind2, int& ind3);
};

#endif
