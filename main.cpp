#include <string>
#include <iostream>
#include "read.hpp"
#include "write.hpp"
#include "algorithm.hpp"
#include <boost/gil/image.hpp>

int main() {
    gil::rgb8_image_t image;
    shino::read_image("input.png", image);
    auto output_image = gil::rgb8_image_t(image);

    shino::apply_gaussian_blur(image, output_image);

    shino::write_image("output.png", output_image);
    // shino::write_image("output.ong", image);
}