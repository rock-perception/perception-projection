#include <projection/Omnidirectional.hpp>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

#include <boost/program_options.hpp>
#include <boost/filesystem.hpp>

#include <iostream>

using namespace std;
namespace po = boost::program_options;
namespace fs = boost::filesystem;

int main( int argc, char* argv[] )
{
    // Program Options
    string calib;
    float min_angle;
    float max_angle;
    int output_size;
    vector<string> input;
    string out_dir;

    po::positional_options_description p;
    p.add("input", -1);

    po::options_description desc("reprojection parameters");
    desc.add_options()
        ("help", "produce help message")
        ("calib", po::value<string>(&calib), "Path to calibration file")
        ("min-angle", po::value<float>(&min_angle)->default_value(-90.0f), "Minimum view angle (deg)")
        ("max-angle", po::value<float>(&max_angle)->default_value(90.0f), "Maximum view angle (deg)")
        ("output-size", po::value<int>(&output_size)->default_value(800), "Horizontal output size (px)")
        ("input", po::value< vector<string> >(&input), "Input files")
        ("output-dir", po::value<string>(&out_dir), "Output directory") 
        ;

    po::variables_map vm;
    po::store(po::command_line_parser(argc, argv).
            options(desc).positional(p).run(), vm);
    po::notify(vm);    

    if (vm.count("help")) {
        std::cout << desc << "\n";
        return 1;
    }

    fs::path out_path( out_dir );
    // TODO check path

    // initialize projection model
    projection::omnicam::Model model;
    model.loadFromFile( calib );
    model.setAngleRange( min_angle / 180.0 * M_PI, max_angle / 180.0 * M_PI );

    projection::omnicam::EquirectangularProjection ep;
    ep.init( output_size, model );
    for( size_t i=0; i<input.size(); i++ )
    {
        fs::path input_path( input[i] );
        cv::Mat img = cv::imread( input_path.string() );
        ep.process( img );
        cv::imwrite( (out_path / input_path.filename()).string(), ep.getView() );
        std::cout << i << "\r" << flush;
    }

    return 0;
}
