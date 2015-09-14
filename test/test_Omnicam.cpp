#include <boost/test/unit_test.hpp>

#include <projection/Omnidirectional.hpp>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

BOOST_AUTO_TEST_CASE( omnicam )
{   
    cv::Mat img = cv::imread( "test/omni.png" );
    projection::omnicam::Model model;
    model.loadFromFile( "test/omni_calibration.txt" );
    model.setAngleRange( -43.0 / 180.0 * M_PI, 35 / 180.0 * M_PI );

    // Equirectangular
    projection::omnicam::EquirectangularProjection ep;
    ep.init( 1200, model );

    ep.process( img );

    cv::imshow( "projection", ep.getView() );
    cv::waitKey(0);

    // Planar
    projection::omnicam::PlanarProjection pp;
    pp.init( 1024, 480, model );

    pp.process( img );

    cv::imshow( "projection", pp.getView() );
    cv::waitKey(0);
}
