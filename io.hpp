#include <string>
#include <boost/gil/image.hpp>

namespace gil = boost::gil;

namespace shino {
    using const_image_view = decltype(gil::const_view(std::declval<gil::rgb8_image_t&>()));
    void write_image(const std::string& filename, const_image_view image);
    void read_image(const std::string& filename, gil::rgb8_image_t& image);
    
}