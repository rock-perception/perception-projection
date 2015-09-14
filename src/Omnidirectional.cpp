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

bool Model::world2cam( const Eigen::Vector3d& point3D, Eigen::Vector2d &point2D ) const
{
    double norm        = sqrt(point3D[0]*point3D[0] + point3D[1]*point3D[1]);
    double theta       = atan(point3D[2]/norm);

    if( theta < min_angle || theta > max_angle )
        return false;

    double t, t_i;
    double rho, x, y;
    double invnorm;
    size_t i;

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

    return true;
}

bool Model::cam2world( const Eigen::Vector2d& point2D, Eigen::Vector3d &point3D ) const
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

    double theta = atan( r / zp );
    if( theta < min_angle || theta > max_angle )
        return false;

    //normalize to unit norm
    double invnorm = 1/sqrt( xp*xp + yp*yp + zp*zp );

    point3D[0] = invnorm*xp;
    point3D[1] = invnorm*yp; 
    point3D[2] = invnorm*zp;

    return true;
}

void Model::setAngleRange( float min, float max )
{
    min_angle = min;
    max_angle = max;
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
                Eigen::AngleAxisd( xAng, Eigen::Vector3d::UnitZ() )
                * Eigen::AngleAxisd( yAng, Eigen::Vector3d::UnitY() )
                * Eigen::Vector3d::UnitZ();

            Eigen::Vector2d c;
            if( model.world2cam( p, c ) )
            {
                mapx.at<float>( y, x ) = c.y();
                mapy.at<float>( y, x ) = c.x();
            }
        }
    }
}

void PlanarProjection::init( size_t width, size_t height, const Model &model )
{
    Projection::init( width, height );
    this->model = model;
    // init with 90 deg fov
    setView(0, 0, M_PI / 2.0);
}

void PlanarProjection::setView( double azimuth, double elevation, double fov )
{
    Eigen::Quaterniond rot =  
	Eigen::AngleAxisd( elevation, Eigen::Vector3d::UnitZ() )
	* Eigen::AngleAxisd( azimuth, Eigen::Vector3d::UnitY() );

    setView( rot, fov );
}

void PlanarProjection::setView( const PlanarViewConfiguration& conf )
{
    setView( conf.azimuth, conf.elevation, conf.fov );
}

void PlanarProjection::setView( Eigen::Quaterniond rot, double fov )
{
    // aspect = w / h
    // diag**2 = w**2 + h**2
    // w/h = aspect
    // h**2 = w**2/aspect**2
    // diag**2 = w**2 + w**2/aspect**2
    // diag**2 = (1 + 1/aspect**2) * w**2 
    // w**2 = diag**2 - (1+1/aspect**2)

    double aspect = (double)size.width / (double)size.height;
    double diag = asin( fov/2.0 ) * 2.0;
    double width = sqrt( diag*diag - (1.0+1.0/(aspect*aspect)) );
    double height = width / aspect;

    for( int x = 0; x < size.width; ++x )
    {
        for( int y = 0; y < size.height; ++y )
        {
	    Eigen::Vector3d p( 
                    width * (-1.0*x / size.width + 0.5), 
                    1.0,  
                    height * (-1.0*y / size.height + 0.5) );
	    
            Eigen::Vector2d c;
            if( model.world2cam( rot * p, c ) )
            {
                mapx.at<float>( y, x ) = c.y();
                mapy.at<float>( y, x ) = c.x();
            }
	}
    }
}
