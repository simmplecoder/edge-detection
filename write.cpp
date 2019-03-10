#include "write.hpp"

#include <boost/gil/image.hpp>
#include <boost/gil/extension/io/png.hpp>
#include <boost/gil/image_view.hpp>
#include <boost/gil/io/write_view.hpp>

void shino::write_image(const std::string& filename, shino::const_image_view image_view) {
    gil::write_view(filename, image_view, gil::image_write_info<gil::png_tag>());
}