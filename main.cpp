#include <string>
#include <string_view>
#include <iostream>
#include "read.hpp"
#include "write.hpp"
#include "algorithm.hpp"
#include <boost/gil/image.hpp>

bool ends_with_png(const std::string& fname) {
    const auto length = fname.size();
    return fname.substr(length - 4, length) == ".png";
}

int main(int argc, char* argv[]) {
    if (argc != 5) {
        std::cerr << "usage: program input_image.png output_image.png gauss-blur-application-count\n";
        return -1;
    }
    std::string input_fname(argv[1]);
    const auto error_text = "input and output can only be png images\n";
    if (input_fname.size() < 5 || !ends_with_png(input_fname)) {
        std::cerr << error_text;
        return -1;
    }

    std::string output_fname(argv[2]);
    if (output_fname.size() < 5 || !ends_with_png(output_fname)) {
        std::cerr << error_text;
        return -1;
    }

    double upper_threshold = std::stod(argv[3]);
    double lower_threshold = std::stod(argv[4]);

    std::cout << "upper threshold is " << upper_threshold << " and lower threshold is " << lower_threshold << '\n';
 
    gil::rgb8_image_t image;
    shino::read_image(input_fname, image);
    auto view = gil::view(image);

    {
    auto output_image = gil::rgb8_image_t(image);
    shino::apply_gaussian_blur(view, gil::view(output_image));
    image = output_image;
    }

    shino::rgb_to_grayscale(view);

    auto output_image = gil::rgb8_image_t(gil::point_t(image.width(), image.height()));
    shino::find_edges(view, gil::view(output_image), upper_threshold, lower_threshold);
    image = output_image;
    shino::write_image(output_fname, gil::const_view(image));
    // shino::write_image("output.ong", image);
}