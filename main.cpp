#include <string>
#include <iostream>
#include "read.hpp"
#include "write.hpp"
#include "algorithm.hpp"
#include <boost/gil/image.hpp>

int main(int argc, char* argv[]) {
    if (argc != 2) {
        std::cerr << "usage: program filter-application-count\n";
        return -1;
    }
    gil::rgb8_image_t image;
    shino::read_image("input.png", image);
    auto view = gil::view(image);

    auto repetition_count = std::stoul(argv[1]);
    for (std::size_t i = 0; i < repetition_count; ++i) {
        auto output_image = gil::rgb8_image_t(image);
        shino::apply_gaussian_blur(view, gil::view(output_image));
        image = output_image;
    }

    shino::rgb_to_grayscale(view);

    auto output_image = gil::rgb8_image_t(gil::point_t(image.width(), image.height()));
    shino::find_edges(view, gil::view(output_image));
    image = output_image;
    shino::write_image("output.png", gil::const_view(image));
    // shino::write_image("output.ong", image);
}