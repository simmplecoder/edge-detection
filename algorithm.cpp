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

std::size_t index_from_xy(int x, int y, long int width) {
    return y * width + x;
}

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

template <typename InputView, typename OutputView, typename T, std::size_t filterHeight, std::size_t filterWidth>
void apply_filter(InputView input_view, OutputView output_view, 
                  const T (&filter)[filterHeight][filterWidth],
                  double factor, double bias) {
    const auto height = input_view.height();
    const auto width = input_view.width();
    for (long x = 0; x < width; ++x) {
        for (long y = 0; y < height; ++y) {
            double intensity = 0.0; 
            for (size_t filterY = 0; filterY < filterHeight; ++filterY) {
                for (size_t filterX = 0; filterX < filterWidth; ++filterX) {
                    int imageX = x - filterWidth / 2 + filterX;
                    int imageY = y - filterHeight / 2 + filterY;
                    if (imageX >= input_view.width() || imageX < 0 
                        || imageY >= input_view.height() || imageY < 0) {
                        continue;
                    }
                    auto& pixel = input_view(gil::point_t(imageX, imageY));
                    intensity += pixel.at(shino::red_channel) * filter[filterY][filterX];
                }
            }
            auto& pixel = output_view(gil::point_t(x, y));
            pixel = std::min(std::max(int(factor * intensity + bias), 0), 255);
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

std::vector<internal::gradient_direction> apply_sobel_operator(shino::gray_image_view input_view, shino::gray_image_view output) {
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
    std::vector<internal::gradient_direction> gradient_directions(input_view.size());
    shino::gray_image x_gradient_image(input_view.dimensions());
    auto x_gradient_view = gil::view(x_gradient_image);
    internal::apply_filter(input_view, x_gradient_view, x_gradient, 1, 0);
#ifndef LEAN_AND_MEAN
    shino::write_image("x_gradient_image.png", gil::view(x_gradient_image));
#endif

    shino::gray_image y_gradient_image(input_view.dimensions());
    auto y_gradient_view = gil::view(y_gradient_image);
    internal::apply_filter(input_view, y_gradient_view, y_gradient, 1, 0);
#ifndef LEAN_AND_MEAN
    shino::write_image("y_gradient_image.png", gil::view(y_gradient_image));
#endif

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

    return gradient_directions;
}

void perform_hysteresis(shino::gray_image_view view, 
                        double scaled_upper_threshold, 
                        double scaled_lower_threshold) {
    const std::uint8_t upper_threshold = std::numeric_limits<std::uint8_t>::max() * scaled_upper_threshold;
    const std::uint8_t lower_threshold = std::numeric_limits<std::uint8_t>::max() * scaled_lower_threshold;
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

std::vector<internal::coordinate_pair> perform_bfs(shino::gray_image_view view, long int x, long int y, 
                 std::vector<char>& visited,
                 std::vector<internal::edge_category>& edge_labels) {
    std::vector<internal::coordinate_pair> bucket;
    if (visited[index_from_xy(x, y, view.width())])
        return {};

    std::queue<internal::coordinate_pair> to_visit;
    to_visit.push({x, y});
    visited[internal::index_from_xy(x, y, view.width())] = true;
    while (!to_visit.empty()) {
        auto current = to_visit.front();
        to_visit.pop();
        
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
        for (auto coordinate: neighbors) {
            auto index = index_from_xy(coordinate.x, coordinate.y, view.width());
            if (coordinate.x == -1 || coordinate.y == view.width()
                || coordinate.y == -1 || coordinate.y == view.height()
                || visited[index] 
                || edge_labels[index] == edge_category::definitely_not_edge) {
                continue;
            }
            to_visit.push(coordinate);
            visited[internal::index_from_xy(coordinate.x, coordinate.y, view.width())] = true;
        }
    }

    return bucket;
}

bool is_connected_to_strong_edge(const std::vector<internal::coordinate_pair>& bucket,
                                 const std::vector<internal::edge_category>& edge_labels,
                                 long int view_width) {
    for (const auto coordinate: bucket) {
        if (edge_labels[internal::index_from_xy(coordinate.x, coordinate.y, view_width)] == edge_category::definitely_edge) {
            return true;
        }
    }

    return false;
}

void perform_bfs_hysteresis(shino::gray_image_view view, double scaled_upper_threshold, double scaled_lower_threshold) {
    std::vector<internal::edge_category> edge_labels(view.size());
    const std::uint8_t upper_threshold = std::numeric_limits<std::uint8_t>::max() * scaled_upper_threshold;
    const std::uint8_t lower_threshold = std::numeric_limits<std::uint8_t>::max() * scaled_lower_threshold;
    const auto white_pixel = std::numeric_limits<std::uint8_t>::max();
    const auto black_pixel = (std::uint8_t)0;
    for (long int y = 0; y < view.height(); ++y) {
        for (long int x = 0; x < view.width(); ++x) {
            auto value = internal::edge_category::uncertainly_edge;
            const auto pixel = view(x, y);
            if (pixel >= upper_threshold) {
                value = internal::edge_category::definitely_edge;
            } else if (pixel <= lower_threshold) {
                value = internal::edge_category::definitely_not_edge;
            }
            edge_labels[internal::index_from_xy(x, y, view.width())] = value;
        }
    }

    std::vector<char> visited(view.size());
    for (long int y = 0; y < view.height(); ++y) {
        for (long int x = 0; x < view.width(); ++x) {
            auto bucket = internal::perform_bfs(view, x, y, visited, edge_labels);
            //strong edges are unreachable from (x, y)
            auto connected_to_strong_edge = internal::is_connected_to_strong_edge(bucket, edge_labels, view.width());
            auto value_to_fill = connected_to_strong_edge ? white_pixel : black_pixel;
            for (const auto coordinate : bucket) {
                view(coordinate.x, coordinate.y) = value_to_fill;
            }
        }
    }
#ifndef LEAN_AND_MEAN
    shino::write_image("bfs hysteresis.png", view);
#endif
}

void perform_non_max_suppression(shino::gray_image_view view, 
                                 const std::vector<gradient_direction>& gradient_directions) {
        std::vector<bool> maximums(view.size());
    for (long y = 0; y < view.height(); ++y) {
        for (long x = 0; x < view.width(); ++x) {
            maximums[internal::index_from_xy(x, y, view.width())] = 
                internal::is_local_maximum(view, x, y, gradient_directions[y * view.width() + x]);
        }
    }

    for (long y = 0; y < view.height(); ++y) {
        for (long x = 0; x < view.width(); ++x) {
            if (!maximums[internal::index_from_xy(x, y, view.width())]) {
                view(x, y) = gil::rgb8_pixel_t(0,0,0);
            }
        }
    }
#ifndef LEAN_AND_MEAN
    shino::write_image("non-max-suppression.png", view);
#endif

}
}

void shino::rgb_to_grayscale(shino::image_view original, gray_image_view output) {
    constexpr double max_channel_intensity = std::numeric_limits<std::uint8_t>::max();
    for (long int y = 0; y < original.height(); ++y) {
        for (long int x = 0; x < original.width(); ++x) {
            // scale the values into range [0, 1] and calculate linear intensity
            double red_intensity = original(x, y).at(shino::red_channel) / max_channel_intensity;
            double green_intensity = original(x, y).at(shino::green_channel) / max_channel_intensity;
            double blue_intensity = original(x, y).at(shino::blue_channel) / max_channel_intensity;
            auto linear_luminosity = 0.2126 * red_intensity 
                                    + 0.7152 * green_intensity 
                                    + 0.0722 * blue_intensity;

            // perform gamma adjustment
            double gamma_compressed_luminosity = 0;
            if (linear_luminosity < 0.0031308) {
                gamma_compressed_luminosity = linear_luminosity * 12.92;
            } else {
                gamma_compressed_luminosity = 1.055 * std::pow(linear_luminosity, 1 / 2.4) - 0.055;
            }

            // since now it is scaled, descale it back
            output(x, y) = gamma_compressed_luminosity * max_channel_intensity;
        }
    }
}

void shino::apply_gaussian_blur(shino::gray_image_view input_view, shino::gray_image_view output_view) {
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

#ifndef LEAN_AND_MEAN
    shino::write_image("after-gaussian-blur.png", output_view);
#endif
}

void shino::find_edges(shino::gray_image_view input_view, shino::gray_image_view output, double upper_threshold, double lower_threshold) {
    shino::gray_image temporary_image(output.dimensions());
    shino::apply_gaussian_blur(input_view, gil::view(temporary_image));

    auto gradient_directions = internal::apply_sobel_operator(gil::view(temporary_image), output);
    temporary_image.recreate(0, 0); //minimize size for the rest of the function

    internal::perform_non_max_suppression(output, gradient_directions);
    internal::perform_bfs_hysteresis(output, upper_threshold, lower_threshold);
}