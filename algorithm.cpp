#include "algorithm.hpp"
#ifndef LEAN_AND_MEAN
#include "io.hpp"
#endif
#include <boost/gil/algorithm.hpp>
#include <cmath>
#include <boost/math/constants/constants.hpp>
#include <utility>
#include <queue>
#include <cstdint>
#include <iostream>

namespace internal {
enum class gradient_direction {
    horizontal,
    vertical,
    slash, // the comments slash '/'
    backslash // backward slash used to escape, e.g. '\'
};

enum class edge_category {
    definitely_edge,
    definitely_not_edge,
    uncertainly_edge
};

using coordinate_pair = gil::point_t;

gradient_direction classify_angle(double perpendicularAngle) {
    double degrees_angle = perpendicularAngle * 180 / boost::math::constants::pi<double>();

    //angle is one of -45, 0, 45, and (-)90 due to layout of pixels
    //90 degrees offset should be taken, as degrees_angle is angle of perpendicular
    if (degrees_angle >= 67.5 || degrees_angle < -67.5) {
        return gradient_direction::horizontal;
    }  else if (degrees_angle >= 22.5 && degrees_angle < 67.5) {
        return gradient_direction::backslash;
    } else if (degrees_angle >= -22.5 && degrees_angle < 22.5) {
        return gradient_direction::vertical;
    } else if (degrees_angle >= -67.5 && degrees_angle < -22.5) {
        return gradient_direction::slash;
    }

    throw std::logic_error("not all cases of gradient angle were covered");
}

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
                    int imageX = x - filterWidth / 2 + filterX;
                    int imageY = y - filterHeight / 2 + filterY;
                    if (imageX >= input_view.width() || imageX < 0 
                        || imageY >= input_view.height() || imageY < 0) {
                        continue;
                    }
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

bool is_local_maximum(shino::gray_image_view view, long int x, long int y, gradient_direction direction) {
    static const auto black_pixel = gil::rgb8_pixel_t(0, 0, 0);
    gil::gray8_pixel_t left_pixel;
    gil::gray8_pixel_t right_pixel;
    auto target_pixel = view(x, y);
    switch (direction) {
    case gradient_direction::horizontal:
        if (x == 0) {
            left_pixel = black_pixel;
        } else {
            left_pixel = view(x - 1, y);
        }

        if (x == view.width() - 1) {
            right_pixel = black_pixel;
        } else {
            right_pixel = view(x + 1, y);
        }
        break;
    case gradient_direction::vertical:
        if (y == 0) {
            left_pixel = black_pixel;
        } else {
            left_pixel = view(x, y - 1);
        }

        if (y == view.height() - 1) {
            right_pixel = black_pixel;
        } else {
            right_pixel = view(x, y + 1);
        }
        break;
    case gradient_direction::slash:
        if (x == 0 || y == view.height() - 1) {
            left_pixel = black_pixel;
        } else {
            left_pixel = view(x - 1, y + 1);
        }

        if (x == view.width() - 1 || y == 0) {
            right_pixel = black_pixel;
        } else {
            right_pixel = view(x + 1, y - 1);
        }
        break;
    case gradient_direction::backslash:
        if (x == 0 || y == 0) {
            left_pixel = black_pixel;
        } else {
            left_pixel = view(x - 1, y - 1);
        }

        if (x == view.width() - 1 || y == view.height() - 1) {
            right_pixel = black_pixel;
        } else {
            right_pixel = view(x + 1, y + 1);
        }
        break;
    default:
        throw std::logic_error("unhandled gradient direction encountered");
    }

    return target_pixel.at(shino::red_channel) == std::max({
        target_pixel.at(shino::red_channel),
        left_pixel.at(shino::red_channel),
        right_pixel.at(shino::red_channel)
    });
}

std::size_t index_from_xy(int x, int y, long int width) {
    return y * width + x;
}

void perform_hysteresis(shino::gray_image_view view, 
                        double exact_upper_threshold, 
                        double exact_lower_threshold) {
    const std::uint8_t upper_threshold = std::numeric_limits<std::uint8_t>::max() * exact_upper_threshold;
    const std::uint8_t lower_threshold = std::numeric_limits<std::uint8_t>::max() * exact_lower_threshold;
    std::vector<internal::coordinate_pair> proved_pixels;
    std::vector<internal::edge_category> edge_labels(view.width() * view.height());

    for (long int y = 0; y < view.height(); ++y) {
        for (long int x = 0; x < view.width(); ++x) {
            auto intensity = view(x, y).at(shino::red_channel);
            auto index = internal::index_from_xy(x, y, view.width());
            if (intensity >= upper_threshold) {
                edge_labels[index] = internal::edge_category::definitely_edge;
            } else if (intensity <= lower_threshold) {
                edge_labels[index] = internal::edge_category::definitely_not_edge;
            }
        }
    }

    constexpr auto max_intensity = std::numeric_limits<std::uint8_t>::max();
    const auto white = gil::rgb8_pixel_t(max_intensity, max_intensity, max_intensity);
    const auto black = gil::rgb8_pixel_t(0, 0, 0);
    for (long int y = 0; y < view.height(); ++y) {
        for (long int x = 0; x < view.width(); ++x) {
            if (edge_labels[internal::index_from_xy(x, y, view.width())] != edge_category::uncertainly_edge) {
                continue;
            }

            internal::coordinate_pair neighbors[] = {
                {x - 1, y - 1},
                {x, y - 1},
                {x + 1, y - 1},
                {x + 1, y},
                {x + 1, y + 1},
                {x, y + 1},
                {x - 1, y + 1},
                {x - 1, y}
            };
            bool is_connected_to_strong_edge = std::count_if(std::begin(neighbors), std::end(neighbors), [&view, &edge_labels](internal::coordinate_pair coordinate) {
                if (coordinate.x == -1 || coordinate.y == view.width()
                    || coordinate.y == -1 || coordinate.y == view.height()) {
                    return false;
                }
                return edge_labels[internal::index_from_xy(coordinate.x, coordinate.y, view.width())] == edge_category::definitely_edge;
            });

            if (is_connected_to_strong_edge) {
                proved_pixels.push_back(internal::coordinate_pair{x, y});
            }
        }
    }

    for (const auto coordinate : proved_pixels) {
        edge_labels[internal::index_from_xy(coordinate.x, coordinate.y, view.width())] = edge_category::definitely_edge;
    }

    for (long int y = 0; y < view.height(); ++y) {
        for (long int x = 0; x < view.width(); ++x) {
            auto label = edge_labels[internal::index_from_xy(x, y, view.width())];
            view(x, y) = label == edge_category::definitely_edge ? white : black;
        }
    }
}

void perform_non_max_suppression(shino::image_view view) {
    
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

void shino::find_edges(shino::image_view input_view, shino::gray_image_view output, double upper_threshold, double lower_threshold) {
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

    gil::rgb8_image_t x_gradient_image(input_view.width(), input_view.height());
    auto x_gradient_view = gil::view(x_gradient_image);
    internal::apply_filter(input_view, x_gradient_view, x_gradient, 1, 0);
#ifndef LEAN_AND_MEAN
    shino::write_image("x_gradient_image.png", gil::view(x_gradient_image));
#endif

    gil::rgb8_image_t y_gradient_image(input_view.width(), input_view.height());
    auto y_gradient_view = gil::view(y_gradient_image);
    internal::apply_filter(input_view, y_gradient_view, y_gradient, 1, 0);
#ifndef LEAN_AND_MEAN
    shino::write_image("y_gradient_image.png", gil::view(y_gradient_image));
#endif
    
    std::vector<internal::gradient_direction> gradient_directions(input_view.height() * input_view.width());

    for (long y = 0; y < input_view.height(); ++y) {
        for (long x = 0; x < input_view.width(); ++x) {
            auto& pixel = output(gil::point_t(x, y));
            auto x_pixel = x_gradient_view(gil::point_t(x, y));
            auto y_pixel = y_gradient_view(gil::point_t(x, y));

            // since all channels have the same intensity, there is no need to compute for others
            auto intensity = (std::uint8_t)std::sqrt(x_pixel.at(shino::red_channel) * x_pixel.at(shino::red_channel) + 
                                                     y_pixel.at(shino::red_channel) * y_pixel.at(shino::red_channel));

            pixel = gil::gray8_pixel_t(intensity);

            auto direction_index = y * input_view.width() + x;
            if (y_pixel.at(shino::red_channel) == shino::black_gray_pixel.at(shino::red_channel)) {
                gradient_directions[direction_index] = internal::classify_angle(boost::math::constants::pi<double>() / 2);
            } else {
                gradient_directions[direction_index] = internal::classify_angle(std::atan(y_pixel.at(shino::red_channel) 
                                                                                          / (double)x_pixel.at(shino::red_channel)));
            }
        }
    }
#ifndef LEAN_AND_MEAN
    shino::write_image("after-xy-gradient.png", output);
#endif

    std::vector<bool> maximums(input_view.size());
    for (long y = 0; y < output.height(); ++y) {
        for (long x = 0; x < output.width(); ++x) {
            maximums[internal::index_from_xy(x, y, input_view.width())] = 
                internal::is_local_maximum(output, x, y, gradient_directions[y * output.width() + x]);
        }
    }

    for (long y = 0; y < output.height(); ++y) {
        for (long x = 0; x < output.width(); ++x) {
            if (!maximums[internal::index_from_xy(x, y, input_view.width())]) {
                output(x, y) = gil::rgb8_pixel_t(0,0,0);
            }
        }
    }
#ifndef LEAN_AND_MEAN
    shino::write_image("non-max-suppression.png", output);
#endif

    internal::perform_hysteresis(output, upper_threshold, lower_threshold);
}