#include <string>
#include <boost/gil/image.hpp>

namespace gil = boost::gil;

namespace shino {
    void read_image(const std::string& filename, gil::rgb8_image_t& image);
}