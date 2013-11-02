#ifndef PROJECTION_HOMOGRAPHY_HPP__
#define PROJECTION_HOMOGRAPHY_HPP__

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <opencv2/imgproc/imgproc.hpp>

namespace projection
{

/**
 * Holds the view of a virtual camera, to a plane, for which images from other
 * viewpoints can be added. These images will be projected onto the plane. This
 * can be used e.g. for Birds-eye views.
 */
class Homography
{
    cv::Mat vcam;
    Eigen::Matrix3f vcamMat;
    Eigen::Isometry3f vcam2plane;

public:
    /**
     * @return cv::Mat object of the virtual camera view
     */
    cv::Mat getVirtualImage();

    /**
     * @brief clear the virtual image
     */
    void clearVirtualImage();

    /** 
     * Initialize virtual camera 
     *
     * @param rows in the target image
     * @param cols in the target image
     * @param focal length of the virtual camera 
     * @param vcam2plane tranformation from the vcam frame to the projection plane
     */
    void init( int rows, int cols, float focal, const Eigen::Isometry3f& vcam2plane );

    void addImage( cv::Mat image, const::Eigen::Isometry3f& cam2plane, const Eigen::Matrix3f& camMat );

    Eigen::Matrix3f calcHomography( const Eigen::Matrix3f &R, const Eigen::Vector3f& t, const Eigen::Vector3f& n, float d ) const;

    Eigen::Matrix3f calcHomography( const Eigen::Isometry3f &trans, const Eigen::Vector3f &n, float dist ) const;

    /**
     * Calculate the homography transform between two cameras, for which the
     * transform to a common reference is given. The xy plane of the reference
     * frame is used as the projection surface.
     *
     * @param camA2plane - current frame
     * @param camB2plane - desired frame
     * @return homography which transforms current to desired frame 
     */
    Eigen::Matrix3f calcHomography( const Eigen::Isometry3f &camA2plane, const Eigen::Isometry3f &camB2plane ) const;

};

}

#endif
