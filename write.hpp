#include <string>
#include <boost/gil/extension/io/png.hpp>
#include <boost/gil/image.hpp>

namespace gil = boost::gil;

namespace shino {
    void write_image(const std::string& filename, const gil::rgb8_image_t& image);
}