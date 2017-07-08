#ifndef UTILITY_MATH_STATISTICS_RUNNING_STATS_H
#define UTILITY_MATH_STATISTICS_RUNNING_STATS_H

#include <iostream>
#include <limits>

namespace utility {
namespace math {
    namespace statistics {

        template <typename T>
        class running_stat_counter {
        public:
            inline running_stat_counter() : d_count(static_cast<T>(0)), i_count(static_cast<size_t>(0)) {}

            inline const running_stat_counter& operator++() {
                if (i_count < std::numeric_limits<size_t>::max()) {
                    i_count++;
                }
                else {
                    d_count += static_cast<T>(std::numeric_limits<size_t>::max());
                    i_count = 1;
                }

                return *this;
            }

            inline void operator++(int) {
                operator++();
            }

            inline void reset() {
                d_count = static_cast<T>(0);
                i_count = static_cast<size_t>(0);
            }

            inline T value() const {
                return d_count + static_cast<T>(i_count);
            }

            inline T value_plus_1() const {
                if (i_count < std::numeric_limits<size_t>::max()) {
                    return d_count + static_cast<T>(i_count + 1);
                }
                else {
                    return d_count + static_cast<T>(std::numeric_limits<size_t>::max()) + static_cast<T>(1);
                }
            }

            inline T value_minus_1() const {
                if (i_count > 0) {
                    return d_count + static_cast<T>(i_count - 1);
                }
                else {
                    return d_count - static_cast<T>(1);
                }
            }

        private:
            T d_count;
            size_t i_count;
        };

        //! Class for keeping statistics of a continuously sampled process / signal.
        //! Useful if the storage of individual samples is not necessary or desired.
        //! Also useful if the number of samples is not known beforehand or exceeds
        //! available memory.
        template <typename T>
        class running_stat {
        public:
            inline running_stat()
                : r_mean(static_cast<T>(0))
                , r_var(static_cast<T>(0))
                , min_val(static_cast<T>(0))
                , max_val(static_cast<T>(0))
                , min_val_norm(static_cast<T>(0))
                , max_val_norm(static_cast<T>(0)) {}

            //! update statistics to reflect new sample
            inline void operator()(const T& sample) {
                if (!std::isfinite(sample)) {
                    std::cout
                        << "utility::math::statistics::running_stat: WARNING infinite sample given. Sample ignored."
                        << std::endl;
                    return;
                }

                update_stats(sample);
            }

            //! set all statistics to zero
            inline void reset() {
                counter.reset();

                r_mean = static_cast<T>(0);
                r_var  = static_cast<T>(0);

                min_val = static_cast<T>(0);
                max_val = static_cast<T>(0);

                min_val_norm = static_cast<T>(0);
                max_val_norm = static_cast<T>(0);
            }

            //! mean or average value
            inline T mean() const {
                return r_mean;
            }

            //! variance
            inline T var(size_t norm_type = 0) const {
                const T N = counter.value();

                if (N > static_cast<T>(1)) {
                    if (norm_type == 0) {
                        return r_var;
                    }
                    else {
                        const T N_minus_1 = counter.value_minus_1();
                        return (N_minus_1 / N) * r_var;
                    }
                }
                else {
                    return static_cast<T>(0);
                }
            }

            //! standard deviation
            inline T stddev(size_t norm_type = 0) const {
                return std::sqrt(this->var(norm_type));
            }

            //! minimum value
            inline T min() const {
                return min_val;
            }

            //! maximum value
            inline T max() const {
                return max_val;
            }

            //! range
            inline T range() const {
                return max_val - min_val;
            }

            //! number of samples so far
            inline T count() const {
                return counter.value();
            }

        private:
            running_stat_counter<T> counter;

            T r_mean;
            T r_var;

            T min_val;
            T max_val;

            T min_val_norm;
            T max_val_norm;

            inline void update_stats(const T& sample) {
                const T N = counter.value();

                if (N > static_cast<T>(0)) {
                    if (sample < min_val) {
                        min_val = sample;
                    }

                    if (sample > max_val) {
                        max_val = sample;
                    }

                    const T N_plus_1  = counter.value_plus_1();
                    const T N_minus_1 = counter.value_minus_1();

                    // note: variance has to be updated before the mean
                    const T tmp = sample - r_mean;

                    r_var  = N_minus_1 / N * r_var + (tmp * tmp) / N_plus_1;
                    r_mean = r_mean + (sample - r_mean) / N_plus_1;
                }

                else {
                    r_mean  = sample;
                    min_val = sample;
                    max_val = sample;
                }

                counter++;
            }
        };
    }
}
}

#endif  // UTILITY_MATH_STATISTICS_RUNNING_STATS_H
