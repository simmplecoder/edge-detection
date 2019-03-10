#include "read.hpp"
#include <boost/gil/io/read_image.hpp>
#include <boost/gil/extension/io/png.hpp>
#include <boost/gil/image.hpp>

void shino::read_image(const std::string& filename, gil::rgb8_image_t& image) {
    gil::read_image(filename, image, gil::png_tag{});
}