#include "algorithm.hpp"
#include <boost/gil/algorithm.hpp>
#include <cmath>
#include <boost/math/constants/constants.hpp>
#include <utility>
#include <queue>
#include <limits>
#include <cstdint>
#include <iostream>
#include "write.hpp"

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
    if (degrees_angle >= 67.5) {
        return gradient_direction::horizontal;
    }
    else if (degrees_angle >= -22.5 && degrees_angle < 22.5) {
        return gradient_direction::vertical;
    } else if (degrees_angle >= 22.5 && degrees_angle < 67.5) {
        return gradient_direction::backslash;
    } else if (degrees_angle >= -67.5 && degrees_angle < -22.5) {
        return gradient_direction::slash;
    } else if (degrees_angle < -67.5) {
        return gradient_direction::horizontal;
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

bool is_local_maximum(shino::image_view view, long int x, long int y, gradient_direction direction) {
    static const auto black_pixel = gil::rgb8_pixel_t(0, 0, 0);
    gil::rgb8_pixel_t left_pixel;
    gil::rgb8_pixel_t right_pixel;
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
        
        
        return target_pixel.at(shino::red_channel) == std::max({
            target_pixel.at(shino::red_channel),
            left_pixel.at(shino::red_channel),
            right_pixel.at(shino::red_channel)
        });
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

        return target_pixel.at(shino::red_channel) == std::max({
            target_pixel.at(shino::red_channel),
            left_pixel.at(shino::red_channel),
            right_pixel.at(shino::red_channel)
        });
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

        return target_pixel.at(shino::red_channel) == std::max({
            target_pixel.at(shino::red_channel),
            left_pixel.at(shino::red_channel),
            right_pixel.at(shino::red_channel)
        });
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

        return target_pixel.at(shino::red_channel) == std::max({
            target_pixel.at(shino::red_channel),
            left_pixel.at(shino::red_channel),
            right_pixel.at(shino::red_channel)
        });
    }

    throw std::logic_error("unhandled gradient direction encountered");
}

std::size_t index_from_xy(int x, int y, long int width) {
    return y * width + x;
}

void perform_bfs(shino::image_view view, long int x, long int y, 
                 std::vector<bool>& visited,
                 std::vector<internal::edge_category>& edge_labels,
                 std::vector<internal::coordinate_pair>& bucket) {
    if (visited[index_from_xy(x, y, view.width())])
        return;

    std::cout << "width is " << view.width() << "and height is " << view.height() << '\n';
    std::queue<internal::coordinate_pair> to_visit;
    to_visit.push({x, y});
    visited[internal::index_from_xy(x, y, view.width())] = true;
    std::cout << "performing bfs with x " << x << " and y " << y << '\n';
    while (!to_visit.empty()) {
        auto current = to_visit.front();
        to_visit.pop();
                std::cout << "entered visitation loop with x " << current.x << " and y " << current.y << "\n";
        
        //clockwise, from left upper
        internal::coordinate_pair neighbors[] = {
            {current.x - 1, current.y - 1},
            {current.x, current.y - 1},
            {current.x + 1, current.y - 1},
            {current.x + 1, current.y},
            {current.x + 1, current.y + 1},
            {current.x, current.y + 1},
            {current.x - 1, current.y + 1},
            {current.x - 1, current.y}
        };

        bucket.push_back(current);
        //visited[internal::index_from_xy(current.x, current.y, view.width())] = true;
        std::cout << "entering neighbors loop\n";
        for (auto coordinate: neighbors) {
            auto index = index_from_xy(coordinate.x, coordinate.y, view.width());
            std::cout << "performing neighbor check with x " << coordinate.x << " and y " << coordinate.y << "\n";
            if (coordinate.x == -1 || coordinate.y == view.width()
                || coordinate.y == -1 || coordinate.y == view.height()
                || visited[index] 
                || edge_labels[index] == edge_category::definitely_not_edge) {
                continue;
            }
            std::cout << "accepted\n";
            to_visit.push(coordinate);
            visited[internal::index_from_xy(coordinate.x, coordinate.y, view.width())] = true;
        }
    }
}

void perform_hysteresis(shino::image_view view, 
                        double exact_upper_threshold, 
                        double exact_lower_threshold) {
    const std::uint8_t upper_threshold = std::numeric_limits<std::uint8_t>::max() * exact_upper_threshold;
    const std::uint8_t lower_threshold = std::numeric_limits<std::uint8_t>::max() * exact_lower_threshold;
    std::vector<bool> visited(view.width() * view.height());
    std::vector<internal::edge_category> edge_labels(view.width() * view.height());

    for (long int y = 0; y < view.height(); ++y) {
        for (long int x = 0; x < view.width(); ++x) {
            auto intensity = view(x, y).at(shino::red_channel);
            auto index = internal::index_from_xy(x, y, view.width());
            if (intensity >= upper_threshold) {
                edge_labels[index] = internal::edge_category::definitely_edge;
            } else if (intensity <= lower_threshold) {
                edge_labels[index] = internal::edge_category::definitely_not_edge;
                visited[index] = true;
            }
        }
    }

    constexpr auto max_intensity = std::numeric_limits<std::uint8_t>::max();
    const auto white = gil::rgb8_pixel_t(max_intensity, max_intensity, max_intensity);
    const auto black = gil::rgb8_pixel_t(0, 0, 0);
    // for (long int y = 0; y < view.height(); ++y) {
    //     for (long int x = 0; x < view.width(); ++x) {
    //         std::vector<internal::coordinate_pair> new_bucket;
    //         perform_bfs(view, x, y, visited, edge_labels, new_bucket);
    //         auto has_definitely_edge = std::any_of(new_bucket.cbegin(), new_bucket.cend(), [&edge_labels, width = view.width()](internal::coordinate_pair coordinate) {
    //             auto index = internal::index_from_xy(coordinate.x, coordinate.y, width);
    //             return edge_labels[index] == edge_category::definitely_edge;
    //         });
    //         auto label = has_definitely_edge ? edge_category::definitely_edge : edge_category::definitely_not_edge;
            
    //         for (auto coordinate: new_bucket) {
    //             edge_labels[internal::index_from_xy(coordinate.x, coordinate.y, view.width())] = label;
    //         }
    //     }
    // }

    for (long int y = 0; y < view.height(); ++y) {
        for (long int x = 0; x < view.width(); ++x) {
            auto label = edge_labels[internal::index_from_xy(x, y, view.width())];
            view(x, y) = label == edge_category::definitely_edge ? white : black;
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
        const auto linear_intensity = (std::uint8_t)(0.2126f * pixel.at(red_channel) 
                                      + 0.7512f * pixel.at(green_channel) 
                                      + 0.0722f * pixel.at(blue_channel));
        return gil::rgb8_pixel_t(
            linear_intensity,
            linear_intensity,
            linear_intensity
        );
    });
}

void shino::find_edges(shino::image_view input_view, shino::image_view output) {
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
    shino::write_image("x_gradient_image.png", gil::const_view(x_gradient_image));

    gil::rgb8_image_t y_gradient_image(input_view.width(), input_view.height());
    auto y_gradient_view = gil::view(y_gradient_image);
    internal::apply_filter(input_view, y_gradient_view, y_gradient, 1, 0);
    shino::write_image("y_gradient_image.png", gil::const_view(y_gradient_image));
    
    std::vector<internal::gradient_direction> gradient_directions(input_view.height() * input_view.width());

    for (long y = 0; y < input_view.height(); ++y) {
        for (long x = 0; x < input_view.width(); ++x) {
            auto& pixel = output(gil::point_t(x, y));
            auto x_pixel = x_gradient_view(gil::point_t(x, y));
            auto y_pixel = y_gradient_view(gil::point_t(x, y));

            // since all channels have the same intensity, there is no need to compute for others
            auto intensity = std::sqrt(x_pixel.at(shino::red_channel) * x_pixel.at(shino::red_channel) + 
                                       y_pixel.at(shino::red_channel) * y_pixel.at(shino::red_channel));

            pixel = gil::rgb8_pixel_t(intensity, intensity, intensity);

            auto direction_index = y * input_view.width() + x;
            if (y_pixel.at(shino::red_channel) == 0) {
                gradient_directions[direction_index] = internal::classify_angle(boost::math::constants::pi<double>() / 2);
            } else {
                gradient_directions[direction_index] = internal::classify_angle(std::atan(y_pixel.at(shino::red_channel) 
                                                                                / (double)x_pixel.at(shino::red_channel)));
            }
        }
    }

    shino::write_image("after-xy-gradient.png", output);

    for (long y = 0; y < output.height(); ++y) {
        for (long x = 0; x < output.width(); ++x) {
            if (!internal::is_local_maximum(output, x, y, gradient_directions[y * output.width() + x])) {
                output(x, y) = gil::rgb8_pixel_t(0, 0, 0);
            }
        }
    }

    shino::write_image("non-max-suppression.png", output);

    internal::perform_hysteresis(output, 0.7, 0.3);
}