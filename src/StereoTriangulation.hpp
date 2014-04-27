#ifndef PROJECTION_TRIANGULATION_HPP__
#define PROJECTION_TRIANGULATION_HPP__

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <cmath>

namespace projection
{
struct StereoTriangulation
{
    Eigen::Isometry3d trans;
    Eigen::Vector3d scenePoint;
    double error;

    Eigen::Vector3d direction;

    Eigen::Vector3d xl, xr;
    Eigen::Vector3d vl, vr;

    StereoTriangulation()
        : trans( Eigen::Isometry3d::Identity() ),
        scenePoint( Eigen::Vector3d::Zero() ),
        error( 0.0 )
    {}

    void setTransform( const Eigen::Isometry3d& camr2caml )
    {
        trans = camr2caml;
    }

    void calcScenePoint( const Eigen::Vector2d& p1, const Eigen::Vector2d& p2 )
    {
        // get homogenous coordinates
        xl << p1, 1;
        xr << p2, 1; 

        // get the transform from a to b
        Eigen::Quaterniond R(trans.rotation());
        Eigen::Vector3d T(trans.translation());

        // now calculate the triangulation by solving a linear system
        Eigen::Matrix<double,3,3> A;
        A << xl, -(R.inverse()*xr), (xl.cross(R.inverse()*xr-T)-T);
        Eigen::Vector3d b;
        b = -(R.inverse()*T);
        Eigen::Vector3d param = A.colPivHouseholderQr().solve(b);

        Eigen::Vector3d Vp = param[2]*(xl.cross(R.inverse()*xr-T)-T);
        Eigen::Vector3d Xl = xl * param[0];

        scenePoint = Xl + 0.5 * Vp;
        error = 0.5 * Vp.norm();
    }

    void calcDirection( double a1, double a2 )
    {
        // calc angle vectors
        vl << std::cos(a1), std::sin(a1), 0;
        vr << std::cos(a2), std::sin(a2), 0;
        
        Eigen::Quaterniond R(trans.rotation());
        Eigen::Matrix<double,3,3> A;
        A << xl, vl, -(R.inverse()*(xr+vr));
        Eigen::Vector3d b = Eigen::Vector3d::Zero();
        Eigen::Vector3d param = A.colPivHouseholderQr().solve(b);
        
        Eigen::Vector3d v = param[0] * xl + param[1] * vl;
        direction = v.normalized();
    }

    Eigen::Vector3d getScenePoint() const
    {
        return scenePoint;
    }

    Eigen::Vector3d getDirection() const
    {
        return direction;
    }

    double getError() const
    {
        return error;
    }

};
}

#endif
