#include <boost/test/unit_test.hpp>

#include <projection/Homography.hpp>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/core/eigen.hpp"
#include <iostream>
#include <stdio.h>

using namespace std;
using namespace projection;

/**
 * Comes from:
 * http://hal.inria.fr/docs/00/17/47/39/PDF/RR-6303.pdf
 * Deeper understanding of the homography decomposition for vision-based control
 */
Eigen::Matrix3f calcHomography( const Eigen::Matrix3f &R, const Eigen::Vector3f& t, const Eigen::Vector3f& n, float d )
{
    Eigen::Matrix3f A = R + (t*n.transpose()) / d;
    return A;
}

Eigen::Matrix3f calcHomography( const Eigen::Isometry3f &trans, const Eigen::Vector3f &n, float dist )
{
    return calcHomography( trans.rotation(), trans.translation(), n, dist );
}

/**
 * camA - current frame
 * camB - desired frame
 */
Eigen::Matrix3f calcHomography( const Eigen::Isometry3f &camA2plane, const Eigen::Isometry3f &camB2plane  )
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

Eigen::Matrix3f getCameraMatrix( float fx, float fy, float cx, float cy )
{
    Eigen::Matrix3f p;
    p << fx, 0, cx,
	0, fy, cy,
	0, 0, 1;

    return p;
}


BOOST_AUTO_TEST_CASE( image_projection )
{   
    cv::Mat src = cv::imread( "test/image1.png" );
    // Set the dst image the same type and size as src
    cv::Mat dst = cv::Mat::zeros( src.rows, src.cols, src.type() );

    // calculate Homography matrix
    cv::Mat hom_cv( 3, 3, CV_32FC1 );

    Eigen::Isometry3f camA2plane = 
        Eigen::Isometry3f( Eigen::Translation3f( Eigen::Vector3f( 0, 0, -1 ) ) ) *
        Eigen::AngleAxisf( M_PI*0.1, Eigen::Vector3f::UnitZ() ) * 
        Eigen::AngleAxisf( M_PI*0.435, Eigen::Vector3f::UnitX() ) * 
        Eigen::AngleAxisf( M_PI, Eigen::Vector3f::UnitX() ) ;

    Eigen::Isometry3f camB2plane = 
        Eigen::Isometry3f( Eigen::Translation3f( Eigen::Vector3f( 0, 0, -10 ) ) ) *
        Eigen::AngleAxisf( M_PI, Eigen::Vector3f::UnitX() );

    Eigen::Matrix3f cam1 = getCameraMatrix( 350, 350, src.cols / 2, src.rows / 2 );
    Eigen::Matrix3f H = calcHomography( camA2plane, camB2plane ); 

    Eigen::Matrix3f hom = cam1 * H * cam1.inverse();

    eigen2cv( hom, hom_cv );

    // apply perspective Transform
    cv::warpPerspective( src, dst, hom_cv, dst.size(), cv::WARP_INVERSE_MAP | cv::INTER_LINEAR ); 
    
    // draw line at infinity
    cv::Point p1( .0f, (.0f * hom(2,0) - hom(2,2)) / hom(2,1) );
    cv::Point p2( src.cols, (-src.cols * hom(2,0) - hom(2,2)) / hom(2,1) );
    cv::line( dst, p1, p2, CV_RGB( 0, 255, 0 ) );

    // show output
    cv::imshow( "projection", dst );
    cv::waitKey(0);
}

BOOST_AUTO_TEST_CASE( homography )
{   
    Eigen::Isometry3f vcam2plane = 
        Eigen::Isometry3f( Eigen::Translation3f( Eigen::Vector3f( 0, 0, -10 ) ) ) *
        Eigen::AngleAxisf( M_PI, Eigen::Vector3f::UnitX() );

    Homography hom;
    hom.init( 500, 500, 200, vcam2plane ); 

    // first image
    {
    cv::Mat img = cv::imread( "test/image1.png" );
    Eigen::Isometry3f cam2plane = 
        Eigen::Isometry3f( Eigen::Translation3f( Eigen::Vector3f( 0, 0, -1 ) ) ) *
        Eigen::AngleAxisf( M_PI*0.1, Eigen::Vector3f::UnitZ() ) * 
        Eigen::AngleAxisf( M_PI*0.435, Eigen::Vector3f::UnitX() ) * 
        Eigen::AngleAxisf( M_PI, Eigen::Vector3f::UnitX() ) ;
    Eigen::Matrix3f camMat = getCameraMatrix( 350, 350, img.cols / 2, img.rows / 2 );

    hom.addImage( img, cam2plane, camMat );
    }

    // second image
    {
    cv::Mat img = cv::imread( "test/image1.png" );
    Eigen::Isometry3f cam2plane = 
        Eigen::Isometry3f( Eigen::Translation3f( Eigen::Vector3f( 0, 0, -1 ) ) ) *
        Eigen::AngleAxisf( -M_PI*0.05, Eigen::Vector3f::UnitZ() ) * 
        Eigen::AngleAxisf( M_PI*0.435, Eigen::Vector3f::UnitX() ) * 
        Eigen::AngleAxisf( M_PI, Eigen::Vector3f::UnitX() ) ;
    Eigen::Matrix3f camMat = getCameraMatrix( 350, 350, img.cols / 2, img.rows / 2 );

    hom.addImage( img, cam2plane, camMat );
    }

    // show output
    cv::imshow( "projection", hom.getVirtualImage() );
    cv::waitKey(0);
}
