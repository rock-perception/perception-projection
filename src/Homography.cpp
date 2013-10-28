#include "Homography.hpp"

#include <opencv2/core/eigen.hpp>

using namespace projection;

void Homography::init( int cols, int rows, float focal, const Eigen::Isometry3f& vcam2plane )
{
    // create the target image
    vcam.create( rows, cols, CV_8UC4 );

    // and the camera matrix
    vcamMat <<
	focal, 0, cols/2,
	0, focal, cols/2,
	0, 0, 1;
    this->vcam2plane = vcam2plane;
}

void Homography::addImage( cv::Mat image, const::Eigen::Isometry3f& cam2plane, const Eigen::Matrix3f& camMat )
{
    // calculate the homography matrix
    Eigen::Matrix3f H = calcHomography( cam2plane, vcam2plane ); 
    Eigen::Matrix3f hom = camMat * H * vcamMat.inverse();

    cv::Rect bounds( cv::Point(), image.size() );
    for(int i=0; i<vcam.rows; i++)
    {
	for(int j=0; j<vcam.cols; j++) 
	{
	    // apply homography
	    Eigen::Vector3f p = hom * Eigen::Vector3f(j,i,1.0f);

	    // only use the coordinates on the positive z side, 
	    // to prevent mirror images from the rear of the camera
	    if( p.z() > 0 )
	    {
		// see of screen coordinate is within bounds
		cv::Point pt( p.x() / p.z(), p.y() / p.z() );
		if( bounds.contains( pt ) )
		{
		    cv::Vec3b &s = image.at<cv::Vec3b>( pt.y, pt.x );
		    cv::Vec4b &t = vcam.at<cv::Vec4b>( i, j );
		    
		    // blend the two pixels using the alpha channel
		    // as an indicator for how many values have already
		    // been stored there
		    t[0] = (s[0] + t[0] * t[3]) / (t[3] + 1);
		    t[1] = (s[1] + t[1] * t[3]) / (t[3] + 1);
		    t[2] = (s[2] + t[2] * t[3]) / (t[3] + 1);
		    t[3] += 1;
		}
	    }
	}
    }
}

Eigen::Matrix3f Homography::calcHomography( const Eigen::Matrix3f &R, const Eigen::Vector3f& t, const Eigen::Vector3f& n, float d ) const
{
    Eigen::Matrix3f A = R + (t*n.transpose()) / d;
    return A;
}

Eigen::Matrix3f Homography::calcHomography( const Eigen::Isometry3f &trans, const Eigen::Vector3f &n, float dist ) const
{
    return calcHomography( trans.rotation(), trans.translation(), n, dist );
}

Eigen::Matrix3f Homography::calcHomography( const Eigen::Isometry3f &camA2plane, const Eigen::Isometry3f &camB2plane ) const
{
    // calculate the plane normal (z-vector) from the point of camB
    Eigen::Vector3f n = camB2plane.rotation().transpose() * -Eigen::Vector3f::UnitZ();

    // calculate the distance from camB to plane
    float d = camB2plane.inverse().translation().dot( n );

    // get transform from camB to camA
    Eigen::Isometry3f camBtoCamA = camA2plane.inverse() * camB2plane;

    // get the homography
    return calcHomography( camBtoCamA, n, d );
}

cv::Mat Homography::getVirtualImage()
{
    return vcam;
}


