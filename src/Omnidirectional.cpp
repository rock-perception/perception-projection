#include "Omnidirectional.hpp"
#include <stdlib.h>
#include <stdio.h>
#include <float.h>
#include <math.h>
#include <stdexcept>

using namespace projection::omnicam;

void Model::loadFromFile( std::string filename )
{
    const int CMV_MAX_BUF = 1024;
    int length_pol;
    int length_invpol;

    FILE *f;
    char buf[CMV_MAX_BUF];
    int i;

    //Open file
    if(!(f=fopen(filename.c_str(),"r")))
    {
        printf("File %s cannot be opened\n", filename.c_str());				  
        throw std::runtime_error("Could not load file " + filename);
    }

    //Read polynomial coefficients
    fgets(buf,CMV_MAX_BUF,f);
    fscanf(f,"\n");
    fscanf(f,"%d", &length_pol);
    pol.resize( length_pol );
    for (i = 0; i < length_pol; i++)
    {
        fscanf(f," %lf",&pol[i]);
    }

    //Read inverse polynomial coefficients
    fscanf(f,"\n");
    fgets(buf,CMV_MAX_BUF,f);
    fscanf(f,"\n");
    fscanf(f,"%d", &length_invpol);
    invpol.resize( length_invpol );
    for (i = 0; i < length_invpol; i++)
    {
        fscanf(f," %lf",&invpol[i]);
    }

    //Read center coordinates
    fscanf(f,"\n");
    fgets(buf,CMV_MAX_BUF,f);
    fscanf(f,"\n");
    fscanf(f,"%lf %lf\n", &xc, &yc);

    //Read affine coefficients
    fgets(buf,CMV_MAX_BUF,f);
    fscanf(f,"\n");
    fscanf(f,"%lf %lf %lf\n", &c,&d,&e);

    //Read image size
    fgets(buf,CMV_MAX_BUF,f);
    fscanf(f,"\n");
    fscanf(f,"%lu %lu", &height, &width);

    fclose(f);
}

Eigen::Vector2d Model::world2cam( const Eigen::Vector3d& point3D ) const
{
    double norm        = sqrt(point3D[0]*point3D[0] + point3D[1]*point3D[1]);
    double theta       = atan(point3D[2]/norm);
    double t, t_i;
    double rho, x, y;
    double invnorm;
    size_t i;

    Eigen::Vector2d point2D;

    if (norm != 0) 
    {
        invnorm = 1/norm;
        t  = theta;
        rho = invpol[0];
        t_i = 1;

        for (i = 1; i < invpol.size(); i++)
        {
            t_i *= t;
            rho += t_i*invpol[i];
        }

        x = point3D[0]*invnorm*rho;
        y = point3D[1]*invnorm*rho;

        point2D[0] = x*c + y*d + xc;
        point2D[1] = x*e + y   + yc;
    }
    else
    {
        point2D[0] = xc;
        point2D[1] = yc;
    }

    return point2D;
}

Eigen::Vector3d Model::cam2world( const Eigen::Vector2d& point2D ) const
{
    double invdet  = 1/(c-d*e); // 1/det(A), where A = [c,d;e,1] as in the Matlab file

    double xp = invdet*(    (point2D[0] - xc) - d*(point2D[1] - yc) );
    double yp = invdet*( -e*(point2D[0] - xc) + c*(point2D[1] - yc) );

    double r   = sqrt(  xp*xp + yp*yp ); //distance [pixels] of  the point from the image center
    double zp  = pol[0];
    double r_i = 1;
    size_t i;

    for (i = 1; i < pol.size(); i++)
    {
        r_i *= r;
        zp  += r_i*pol[i];
    }

    //normalize to unit norm
    double invnorm = 1/sqrt( xp*xp + yp*yp + zp*zp );

    Eigen::Vector3d point3D;
    point3D[0] = invnorm*xp;
    point3D[1] = invnorm*yp; 
    point3D[2] = invnorm*zp;

    return point3D;
}

size_t Model::getWidth() const
{
    return width;
}

size_t Model::getHeight() const
{
    return height;
}

void Projection::init( size_t width, size_t height )
{
    // initialize helper mats to target size
    size = cv::Size( width, height );
    mapx.create( size, CV_32FC1 );
    mapy.create( size, CV_32FC1 );
    view.create( size, CV_8UC3 );
}

void Projection::process( cv::Mat frame )
{
    cv::remap( frame, view, mapx, mapy, cv::INTER_LINEAR, cv::BORDER_CONSTANT, cv::Scalar(0,0,0) );
}

cv::Mat Projection::getView()
{
    return view;
}

void EquirectangularProjection::init( size_t width, const Model &model )
{

    // equirectangular projections always have a 2:1 aspect ratio
    size_t height = width / 2;
    Projection::init( width, height );

    // initialize the maps for remapping
    for( size_t x = 0; x < width; ++x )
    {
        for( size_t y = 0; y < height; ++y )
        {
            float xAng = 2.0 * x * M_PI / width;
            float yAng = y * M_PI / height;

            Eigen::Vector3d p = 
                Eigen::AngleAxisd( xAng, Eigen::Vector3d::UnitX() )
                * Eigen::AngleAxisd( yAng, Eigen::Vector3d::UnitZ() )
                * Eigen::Vector3d::UnitX();

            Eigen::Vector2d c = model.world2cam( p );
            mapx.at<float>( y, x ) = c.x();
            mapy.at<float>( y, x ) = c.y();
        }
    }
}


