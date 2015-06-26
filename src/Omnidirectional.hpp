#ifndef PROJECTION_OMNIDIRECTIONAL_HPP__
#define PROJECTION_OMNIDIRECTIONAL_HPP__

#include <opencv2/imgproc/imgproc.hpp>
#include <Eigen/Core>
#include <Eigen/Geometry>

namespace projection
{
namespace omnicam
{

class Model
{
    std::vector<double> pol;    // the polynomial coefficients: pol[0] + x"pol[1] + x^2*pol[2] + ... + x^(N-1)*pol[N-1]
    std::vector<double> invpol; // the coefficients of the inverse polynomial
    double xc;         // row coordinate of the center
    double yc;         // column coordinate of the center
    double c;          // affine parameter
    double d;          // affine parameter
    double e;          // affine parameter
    size_t width;         // image width
    size_t height;        // image height

public:
    void loadFromFile( std::string filename );
    Eigen::Vector2d world2cam( const Eigen::Vector3d& p ) const;
    Eigen::Vector3d cam2world( const Eigen::Vector2d& p ) const;

    size_t getWidth() const;
    size_t getHeight() const;
};


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

}
}

#endif

