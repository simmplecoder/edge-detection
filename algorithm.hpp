#include <boost/gil/image_view.hpp>
#include <boost/gil/image.hpp>
#include <boost/gil/rgb.hpp>
#include <boost/mpl/vector.hpp>
#include <limits>
#include <cstdint>

namespace gil = boost::gil;
namespace mpl = boost::mpl;

namespace shino {
    constexpr auto red_channel = mpl::int_<0>{};
    constexpr auto green_channel = mpl::int_<1>{};
    constexpr auto blue_channel = mpl::int_<2>{};    

    const static gil::gray8_pixel_t black_gray_pixel = gil::gray8_pixel_t(std::numeric_limits<std::uint8_t>::min());
    const static gil::gray8_pixel_t white_gray_pixel = gil::gray8_pixel_t(std::numeric_limits<std::uint8_t>::max());

    using image_view = decltype(gil::view(std::declval<gil::rgb8_image_t&>()));
    using gray_image_view = decltype(gil::view(std::declval<gil::gray8_image_t&>()));
    using gray_image = gil::gray8_image_t;
    void apply_gaussian_blur(shino::image_view input_image, shino::image_view output_image);
    void rgb_to_grayscale(shino::image_view view);
    void find_edges(shino::image_view input, shino::gray_image_view output, double upper_threshold, double lower_threshold);
}