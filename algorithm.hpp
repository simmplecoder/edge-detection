#include <boost/gil/image_view.hpp>
#include <boost/gil/image.hpp>
#include <boost/gil/rgb.hpp>
#include <boost/mpl/vector.hpp>

namespace gil = boost::gil;
namespace mpl = boost::mpl;

namespace shino {
    constexpr auto red_channel = mpl::int_<0>{};
    constexpr auto green_channel = mpl::int_<1>{};
    constexpr auto blue_channel = mpl::int_<2>{};    
    using image_view = decltype(gil::view(std::declval<gil::rgb8_image_t&>()));
    void apply_gaussian_blur(shino::image_view input_image, shino::image_view output_image);
    void rgb_to_grayscale(shino::image_view view);
    void find_edges(shino::image_view input, shino::image_view output);
}