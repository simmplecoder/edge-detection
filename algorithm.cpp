#include "algorithm.hpp"
#include <boost/gil/algorithm.hpp>

void shino::apply_gaussian_blur(gil::rgb8_image_t& input_image, gil::rgb8_image_t& output_image) {
    constexpr static auto filterHeight = 5ull;
    constexpr static auto filterWidth = 5ull;

    constexpr static double filter[filterHeight][filterWidth] =
    {
        2,  4,  6,  4,  2,
        4, 9, 12, 9,  4,
        5, 12, 15, 12,  5,
        4, 9, 12, 9,  4,
        2,  4,  5,  4,  2,
    };


    constexpr double factor = 1.0 / 159;
    constexpr double bias = 0.0;

    auto input_view = gil::view(input_image);
    auto output_view = gil::view(output_image);

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

void shino::rgb_to_grayscale(shino::image_view view) {
    gil::transform_pixels(view, view, [](gil::rgb8_pixel_t pixel) {
        const auto linear_intensity = (unsigned char)(0.2126f * pixel.at(red_channel) 
                                      + 0.7512f * pixel.at(green_channel) 
                                      + 0.0722f * pixel.at(blue_channel));
        return gil::rgb8_pixel_t(
            linear_intensity,
            linear_intensity,
            linear_intensity
        );
    });
}