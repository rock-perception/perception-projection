#include <boost/test/unit_test.hpp>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/core/eigen.hpp"
#include <iostream>
#include <stdio.h>

using namespace std;
//using namespace projection;

Eigen::Matrix3f calcHomography( const Eigen::Matrix3f &R, const Eigen::Vector3f& t, float d )
{
    Eigen::Vector3f p = R.transpose() * Eigen::Vector3f::UnitZ() / d;
    float k = pow( (1.0+p.transpose()*t), 1.0/3.0 );
    Eigen::Matrix3f A = 1.0/k * (Eigen::Matrix3f::Identity() + p*t.transpose()) * R;
    return A;
}

Eigen::Matrix3f calcHomography( const Eigen::Isometry3f &trans, float dist )
{
    return calcHomography( trans.rotation(), trans.translation(), dist );
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
    Eigen::Isometry3f trans = Eigen::Isometry3f::Identity();
    //trans.rotate( Eigen::AngleAxisf( M_PI/8.0, Eigen::Vector3f::UnitX() ) );
    trans.translate( Eigen::Vector3f( 0, 0, 3 ) );
    trans.rotate( Eigen::AngleAxisf( M_PI/2.0, Eigen::Vector3f::UnitX() ) );

    std::cout << trans.matrix() << std::endl;

    Eigen::Matrix3f cam1 = getCameraMatrix( 200, 200, src.cols / 2, src.rows / 2 );
    Eigen::Matrix3f hom = cam1 * calcHomography( trans, 1.0 ) * cam1.inverse();
    eigen2cv( hom, hom_cv );

    // apply perspective Transform
    cv::warpPerspective( src, dst, hom_cv, dst.size() ); 
    
    // show output
    cv::imshow( "projection", dst );
    cv::waitKey(0);
}
