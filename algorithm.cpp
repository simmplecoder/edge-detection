#include "algorithm.hpp"

void shino::apply_gaussian_blur(gil::rgb8_image_t& input_image, gil::rgb8_image_t& output_image) {
    constexpr static auto filterHeight = 5ull;
    constexpr static auto filterWidth = 5ull;

    constexpr static double filter[filterHeight][filterWidth] =
    {
        1,  4,  6,  4,  1,
        4, 16, 24, 16,  4,
        6, 24, 36, 24,  6,
        4, 16, 24, 16,  4,
        1,  4,  6,  4,  1,
    };


    constexpr double factor = 1.0 / 256.0;
    constexpr double bias = 0.0;

    auto input_view = gil::view(input_image);
    auto output_view = gil::view(output_image);
    auto red_channel = mpl::int_<0>{};
    auto green_channel = mpl::int_<1>{};
    auto blue_channel = mpl::int_<2>{};
    const auto height = input_view.height();
    const auto width = input_view.width();
    for (long x = 0; x < width; ++x) {
        for (long y = 0; y < height; ++y) {
            double red = 0.0; 
            double green = 0.0; 
            double blue = 0.0;
            for (size_t filterY = 0; filterY < filterHeight; ++filterY) {
                for (size_t filterX = 0; filterX < filterWidth; ++filterX) {
                    int imageX = (x - filterWidth / 2 + filterX + width) % width;
                    int imageY = (y - filterHeight / 2 + filterY + height) % height;
                    auto& pixel = input_view(gil::point_t(imageX, imageY));
                    red += pixel.at(red_channel) * filter[filterY][filterX];
                    green += pixel.at(green_channel) * filter[filterY][filterX];
                    blue += pixel.at(blue_channel) * filter[filterY][filterX];
                }
            }
            auto& pixel = output_view(gil::point_t(x, y));
            pixel.at(red_channel) = std::min(std::max(int(factor * red + bias), 0), 255);
            pixel.at(green_channel) = std::min(std::max(int(factor * green + bias), 0), 255);
            pixel.at(blue_channel) = std::min(std::max(int(factor * blue + bias), 0), 255);
        }
        
    }
}