#include "write.hpp"

#include <boost/gil/image.hpp>
#include <boost/gil/extension/io/png.hpp>
#include <boost/gil/image_view.hpp>
#include <boost/gil/io/write_view.hpp>

void shino::write_image(const std::string& filename, const gil::rgb8_image_t& image) {
    gil::write_view(std::string("output.png"), gil::const_view(image), gil::image_write_info<gil::png_tag>());
}