#include <boost/test/unit_test.hpp>

#include <projection/Omnidirectional.hpp>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

BOOST_AUTO_TEST_CASE( omnicam )
{   
    cv::Mat img = cv::imread( "test/omni.png" );
    projection::omnicam::Model model;
    model.loadFromFile( "test/omni_calibration.txt" );

    projection::omnicam::EquirectangularProjection ep;
    ep.init( 600, model );

    ep.process( img );

    cv::imshow( "projection", ep.getView() );
    cv::waitKey(0);
}
