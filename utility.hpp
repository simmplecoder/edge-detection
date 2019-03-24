#include <cstddef>
#include <functional>
#include <boost/mpl/int.hpp>

template <typename Pixel>
struct channel_count {
private:
    template <std::size_t Index>
    using channel_call_t = decltype(std::declval<Pixel>().at(boost::mpl::int_<Index>{}));

    template <std::size_t Index, typename = void>
    struct channel_count_impl {
        constexpr static long int value = Index - 1;
    };

    template <std::size_t Index>
    struct channel_count_impl<Index, std::void_t<channel_call_t<Index>>> {
        constexpr static long int value = channel_count_impl<Index + 1>::value;
    };
public:
    // +1 because zero based
    constexpr static long int value = channel_count_impl<0>::value + 1;
};