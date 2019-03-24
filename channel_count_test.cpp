#include "utility.hpp"
#include <boost/gil/pixel.hpp>

//the following code just needs to compile
int main() {
    namespace gil = boost::gil;
    static_assert(channel_count<gil::rgb8_pixel_t>::value == 3, "channel count detection doesn't work");
    static_assert(channel_count<gil::gray8_pixel_t>::value == 1, "channel count detection doesn't work");
}