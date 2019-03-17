#include <string>
#include <string_view>
#include <iostream>
#include "io.hpp"
#include "algorithm.hpp"
#include <boost/gil/image.hpp>

int main(int argc, char* argv[]) {
    if (argc != 5) {
        std::cerr << "usage: program input_image.png output_image.png upper_threshold lower_threshold\n";
        return -1;
    }
    std::string input_fname(argv[1]);
    std::string output_fname(argv[2]);

    double upper_threshold = std::stod(argv[3]);
    double lower_threshold = std::stod(argv[4]);
 
    gil::rgb8_image_t image;
    shino::read_image(input_fname, image);
    auto view = gil::view(image);

    {
    auto output_image = gil::rgb8_image_t(image);
    shino::apply_gaussian_blur(view, gil::view(output_image));
    image = output_image;
    }

    auto output_image = gil::gray8_image_t(gil::point_t(image.width(), image.height()));
    shino::find_edges(view, gil::view(output_image), upper_threshold, lower_threshold);
    shino::write_image(output_fname, gil::view(output_image));
}