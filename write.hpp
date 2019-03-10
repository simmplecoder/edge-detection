#include <string>
#include <boost/gil/extension/io/png.hpp>
#include <boost/gil/image.hpp>
#include <functional>

namespace gil = boost::gil;

namespace shino {
    using const_image_view = decltype(gil::const_view(std::declval<gil::rgb8_image_t&>()));
    void write_image(const std::string& filename, const_image_view image);
}