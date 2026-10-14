#pragma once

#include <algorithm>
#include <array>
#include <cstddef>
#include <limits>
#include <optional>
#include <string>

#include <charqui/profile.hpp>

namespace charqui::detail {

//! The set of durations a single DoF can be synchronized to: a minimum, plus up to two blocked intervals.
//! Owns the storage for the extremal profiles of step 1, which writes them in place; the minimum profile
//! and the interval end profiles are referenced by index, so building a block never copies a Profile.
class Block {
public:
    static constexpr std::size_t kMaxProfiles{6};  // at most 5 valid profiles + 1 for probing

    struct Interval {
        double left, right;        // [s]
        std::size_t profile{0};    // Index of the profile corresponding to the right (end) time
    };

    std::array<Profile, kMaxProfiles> profiles;
    std::size_t min_index{0};  // Index of the fastest profile, reused by step 2
    double t_min{0.0};         // [s]

    // At most two blocked intervals; order does not matter
    std::optional<Interval> a, b;

    [[nodiscard]] Profile& p_min() noexcept { return profiles[min_index]; }
    [[nodiscard]] const Profile& p_min() const noexcept { return profiles[min_index]; }
    [[nodiscard]] const Profile& profile_of(const Interval& interval) const noexcept { return profiles[interval.profile]; }

    void set_min_profile(std::size_t index) {
        min_index = index;
        t_min = profiles[index].total_duration();
        a = std::nullopt;
        b = std::nullopt;
    }

    [[nodiscard]] Interval make_interval(std::size_t index_left, std::size_t index_right) const {
        const double left_duration = profiles[index_left].total_duration();
        const double right_duration = profiles[index_right].total_duration();
        if (left_duration < right_duration) {
            return Interval{left_duration, right_duration, index_right};
        }
        return Interval{right_duration, left_duration, index_left};
    }

    template <bool numerical_robust = true>
    static bool calculate_block(Block& block, std::size_t valid_profile_counter) {
        constexpr double eps = std::numeric_limits<double>::epsilon();
        auto& valid_profiles = block.profiles;

        if (valid_profile_counter == 1) {
            block.set_min_profile(0);
            return true;

        } else if (valid_profile_counter == 2) {
            if (std::abs(valid_profiles[0].t_sum.back() - valid_profiles[1].t_sum.back()) < 8 * eps) {
                block.set_min_profile(0);
                return true;
            }

            if constexpr (numerical_robust) {
                const std::size_t idx_min = (valid_profiles[0].t_sum.back() < valid_profiles[1].t_sum.back()) ? 0 : 1;
                const std::size_t idx_else_1 = (idx_min + 1) % 2;

                block.set_min_profile(idx_min);
                block.a = block.make_interval(idx_min, idx_else_1);
                return true;
            }

        // Only happens due to numerical issues
        } else if (valid_profile_counter == 4) {
            // Find "identical" profiles
            if (std::abs(valid_profiles[0].t_sum.back() - valid_profiles[1].t_sum.back()) < 32 * eps && valid_profiles[0].direction != valid_profiles[1].direction) {
                block.remove_profile(valid_profile_counter, 1);
            } else if (std::abs(valid_profiles[2].t_sum.back() - valid_profiles[3].t_sum.back()) < 256 * eps && valid_profiles[2].direction != valid_profiles[3].direction) {
                block.remove_profile(valid_profile_counter, 3);
            } else if (std::abs(valid_profiles[0].t_sum.back() - valid_profiles[3].t_sum.back()) < 256 * eps && valid_profiles[0].direction != valid_profiles[3].direction) {
                block.remove_profile(valid_profile_counter, 3);
            } else {
                return false;
            }

        } else if (valid_profile_counter % 2 == 0) {
            return false;
        }

        // Index of the fastest profile
        const auto idx_min_it = std::min_element(valid_profiles.cbegin(), valid_profiles.cbegin() + static_cast<std::ptrdiff_t>(valid_profile_counter),
                                                 [](const Profile& lhs, const Profile& rhs) { return lhs.t_sum.back() < rhs.t_sum.back(); });
        const std::size_t idx_min = static_cast<std::size_t>(std::distance(valid_profiles.cbegin(), idx_min_it));

        block.set_min_profile(idx_min);

        if (valid_profile_counter == 3) {
            const std::size_t idx_else_1 = (idx_min + 1) % 3;
            const std::size_t idx_else_2 = (idx_min + 2) % 3;

            block.a = block.make_interval(idx_else_1, idx_else_2);
            return true;

        } else if (valid_profile_counter == 5) {
            const std::size_t idx_else_1 = (idx_min + 1) % 5;
            const std::size_t idx_else_2 = (idx_min + 2) % 5;
            const std::size_t idx_else_3 = (idx_min + 3) % 5;
            const std::size_t idx_else_4 = (idx_min + 4) % 5;

            if (valid_profiles[idx_else_1].direction == valid_profiles[idx_else_2].direction) {
                block.a = block.make_interval(idx_else_1, idx_else_2);
                block.b = block.make_interval(idx_else_3, idx_else_4);
            } else {
                block.a = block.make_interval(idx_else_1, idx_else_4);
                block.b = block.make_interval(idx_else_2, idx_else_3);
            }
            return true;
        }

        return false;
    }

    [[nodiscard]] bool is_blocked(double t) const {
        return (t < t_min) || (a && a->left < t && t < a->right) || (b && b->left < t && t < b->right);
    }

    [[nodiscard]] const Profile& get_profile(double t) const {
        if (b && t >= b->right) {
            return profile_of(*b);
        }
        if (a && t >= a->right) {
            return profile_of(*a);
        }
        return p_min();
    }

    [[nodiscard]] std::string to_string() const {
        std::string result = "[" + std::to_string(t_min) + " ";
        if (a) {
            result += std::to_string(a->left) + "] [" + std::to_string(a->right) + " ";
        }
        if (b) {
            result += std::to_string(b->left) + "] [" + std::to_string(b->right) + " ";
        }
        return result + "-";
    }

private:
    void remove_profile(std::size_t& valid_profile_counter, std::size_t index) {
        for (std::size_t i = index; i < valid_profile_counter - 1; ++i) {
            profiles[i] = profiles[i + 1];
        }
        valid_profile_counter -= 1;
    }
};

}  // namespace charqui::detail
