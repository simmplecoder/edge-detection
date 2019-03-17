#include "io.hpp"
#include <boost/gil/io/read_image.hpp>
#include <boost/gil/extension/io/png.hpp>
#include <boost/gil/image.hpp>
#include <boost/gil/extension/io/png.hpp>
#include <boost/gil/extension/io/jpeg.hpp>
#include <boost/gil/image_view.hpp>
#include <boost/gil/io/write_view.hpp>
#include <string>
#include <stdexcept>

namespace internal {
    enum class format {
        png,
        jpeg
    };

    internal::format get_format(const std::string& filename) {
        auto length = filename.size();
        if (length < 4) {
            throw std::invalid_argument("file name is too short to match any predefined formats");
        }

        auto dot_position = filename.find_last_of('.');
        auto format = filename.substr(dot_position);
        if (format == ".png") {
            return internal::format::png;
        } else if (format == ".jpg" || format == ".jpeg") {
            return internal::format::jpeg;
        }

        throw std::invalid_argument("unknown file format. Please read/write file manually");
    }
}

void shino::write_image(const std::string& filename, shino::const_image_view image_view) {
    auto format = internal::get_format(filename);
    switch (format) {
    case internal::format::png:
        gil::write_view(filename, image_view, gil::image_write_info<gil::png_tag>());
        break;
    case internal::format::jpeg:
        gil::write_view(filename, image_view, gil::image_write_info<gil::jpeg_tag>());
        break;
    default:
        throw std::logic_error("unhandled internal image format encountered");
    }
}

void shino::read_image(const std::string& filename, gil::rgb8_image_t& image) {
    gil::read_image(filename, image, gil::png_tag{});
    auto format = internal::get_format(filename);
    switch (format) {
    case internal::format::png:
        gil::read_image(filename, image, gil::png_tag{});
        break;
    case internal::format::jpeg:
        gil::read_image(filename, image, gil::jpeg_tag{});
        break;
    default:
        throw std::logic_error("unhandled internal image format encountered");
    }
}