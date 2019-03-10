#include "algorithm.hpp"
#include <boost/gil/algorithm.hpp>

namespace internal {
template <typename T, std::size_t filterHeight, std::size_t filterWidth>
void apply_filter(shino::image_view input_view, shino::image_view output_view, 
                  const T (&filter)[filterHeight][filterWidth],
                  double factor, double bias) {
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
                    red += pixel.at(shino::red_channel) * filter[filterY][filterX];
                    green += pixel.at(shino::green_channel) * filter[filterY][filterX];
                    blue += pixel.at(shino::blue_channel) * filter[filterY][filterX];
                }
            }
            auto& pixel = output_view(gil::point_t(x, y));
            pixel.at(shino::red_channel) = std::min(std::max(int(factor * red + bias), 0), 255);
            pixel.at(shino::green_channel) = std::min(std::max(int(factor * green + bias), 0), 255);
            pixel.at(shino::blue_channel) = std::min(std::max(int(factor * blue + bias), 0), 255);
        }
        
    }
}
}

void shino::apply_gaussian_blur(shino::image_view input_view, shino::image_view output_view) {
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

    internal::apply_filter(input_view, output_view, filter, factor, bias);
}

void shino::rgb_to_grayscale(shino::image_view view) {
    gil::transform_pixels(view, view, [](gil::rgb8_pixel_t pixel) {
        const auto linear_intensity = (char)(0.2126f * pixel.at(red_channel) 
                                      + 0.7512f * pixel.at(green_channel) 
                                      + 0.0722f * pixel.at(blue_channel));
        return gil::rgb8_pixel_t(
            linear_intensity,
            linear_intensity,
            linear_intensity
        );
    });
}

void shino::find_edges(shino::image_view input, shino::image_view output) {
    constexpr static std::size_t gradient_width = 3;
    constexpr static std::size_t gradient_height = 3;
    constexpr static char x_gradient[gradient_height][gradient_width] = {
        -1, 0, 1,
        -2, 0, 2,
        -1, 0, 1
    };

    constexpr static char y_gradient[gradient_height][gradient_width] = {
        -1, -2, -1,
        0, 0, 0,
        1, 2, 1
    };


}