#include <boost/gil/image_view.hpp>
#include <boost/gil/image.hpp>
#include <boost/gil/rgb.hpp>
#include <boost/mpl/vector.hpp>

namespace gil = boost::gil;
namespace mpl = boost::mpl;

namespace shino {

    void apply_gaussian_blur(gil::rgb8_image_t& input_image, gil::rgb8_image_t& output_image);
}