#ifndef EDGE_DETECTION_IO
#define EDGE_DETECTION_IO

#include <string>
#include <boost/gil/image.hpp>

namespace gil = boost::gil;

namespace shino {
    using image_view = decltype(gil::view(std::declval<gil::rgb8_image_t&>()));
    using gray_image_view = decltype(gil::view(std::declval<gil::gray8_image_t&>()));
    void write_image(const std::string& filename, image_view image);
    void write_image(const std::string& filename, gray_image_view image_view);
    void read_image(const std::string& filename, gil::rgb8_image_t& image);
    
}
#endif