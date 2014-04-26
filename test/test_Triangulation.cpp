#include <boost/test/unit_test.hpp>

#include <projection/Triangulation.hpp>

using namespace projection;

BOOST_AUTO_TEST_CASE( stereo_triangulation )
{   
   StereoTriangulation tr; 
   Eigen::Isometry3d trans( Eigen::Translation3d( 1.0, 0.0, 0.0 ) );
   tr.setTransform( trans );

   Eigen::Vector2d p1( 0.25, 0.0 );
   Eigen::Vector2d p2( -0.25, 0.0 );

   tr.calcScenePoint( p1, p2 );

   std::cout << tr.getScenePoint() << std::endl;
   std::cout << tr.getError() << std::endl; 
}
