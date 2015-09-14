#ifndef PROJECTION_OMNIDIRECTIONAL_HPP__
#define PROJECTION_OMNIDIRECTIONAL_HPP__

#include <opencv2/imgproc/imgproc.hpp>
#include <Eigen/Core>
#include <Eigen/Geometry>

#include <projection/OmnidirectionalConfig.hpp>

namespace projection
{
namespace omnicam
{

class Projection
{
protected:
    cv::Mat mapx, mapy;
    cv::Mat view;

    cv::Size size;

public:
    /**
     * @brief initialize the projection
     * 
     * @param width of the resulting view
     * @param height of the resulting view
     */
    void init( size_t width, size_t height );

    /**
     * @brief reproject an input image
     *
     * @param frame the frame to project
     */
    void process( cv::Mat frame );

    /**
     * @brief the reprojected image
     */
    cv::Mat getView();
};

class EquirectangularProjection : public Projection
{
public:
    void init( size_t width, const Model &model );
};

class PlanarProjection : public Projection
{
    Eigen::Quaterniond rot;
    double f;
    Model model;

public:
    void init( size_t width, size_t height, const Model &model );

    /**
    * @brief set the view of virtual planar projection
    * @param azimuth of the virtual pan tilt unit in rad
    * @param elevation of the virtual pan til unit in rad
    * @param fov diagonal field of view in rad 
    */
    void setView( double azimuth, double elevation, double fov );

    void setView( const PlanarViewConfiguration& conf );

    void setView( Eigen::Quaterniond rot, double fov);
};

}
}

#endif

