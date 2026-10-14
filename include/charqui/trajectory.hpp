#pragma once

#include <cstddef>
#include <optional>
#include <span>
#include <vector>

#include <charqui/profile.hpp>
#include <charqui/types.hpp>

namespace charqui {

namespace detail {
class TargetSolver;
class WaypointSolver;
}  // namespace detail

//! A time-parameterized, jerk-limited trajectory for all degrees of freedom.
//! It consists of one or more sections (one per waypoint interval); each section holds one Profile per DoF.
class Trajectory {
public:
    Trajectory() = default;
    explicit Trajectory(std::size_t dofs, std::size_t max_sections = 1);

    [[nodiscard]] std::size_t dofs() const noexcept { return dofs_; }
    [[nodiscard]] std::size_t sections() const noexcept { return sections_; }
    [[nodiscard]] double duration() const noexcept { return duration_; }

    //! The time at which each section ends; the last entry equals duration()
    [[nodiscard]] std::span<const double> section_end_times() const noexcept { return {section_end_times_.data(), sections_}; }

    //! The minimum duration of every DoF on its own, before synchronization (first section only)
    [[nodiscard]] std::span<const double> independent_min_durations() const noexcept { return independent_min_durations_; }

    //! Kinematic state at a given time. Past the end, the acceleration is held constant and the section index equals sections().
    [[nodiscard]] Sample at(double time) const;

    //! Allocation-free variant; `jerk` may be empty. Spans must have dofs() elements.
    void at(double time, std::span<double> position, std::span<double> velocity, std::span<double> acceleration,
            std::span<double> jerk = {}, std::size_t* section = nullptr) const;

    //! Allocation-free variant writing into an existing Sample (its vectors are resized to dofs())
    void at(double time, Sample& sample) const;

    //! Kinematic states at several times
    [[nodiscard]] std::vector<Sample> sample(std::span<const double> times) const;

    //! Kinematic states every `dt` seconds from 0 to duration(), always including the final time
    [[nodiscard]] std::vector<Sample> sample(double dt) const;

    //! Minimum and maximum position of every DoF over the whole trajectory
    [[nodiscard]] std::vector<Bound> position_extrema() const;

    //! The first time (>= after) at which the given DoF passes the given position
    [[nodiscard]] std::optional<double> first_time_at_position(std::size_t dof, double position, double after = 0.0) const;

    //! The underlying single-DoF profile of a section
    [[nodiscard]] const Profile& profile(std::size_t section, std::size_t dof) const { return profiles_[section * dofs_ + dof]; }

    //! All profiles, flat, ordered by section then DoF
    [[nodiscard]] std::span<const Profile> profiles() const noexcept { return {profiles_.data(), sections_ * dofs_}; }

private:
    friend class detail::TargetSolver;
    friend class detail::WaypointSolver;
    friend class Planner;

    std::size_t dofs_{0};
    std::size_t sections_{0};
    double duration_{0.0};

    std::vector<Profile> profiles_;
    std::vector<double> section_end_times_;
    std::vector<double> independent_min_durations_;

    //! Set the active number of sections without releasing capacity
    void resize(std::size_t dofs, std::size_t sections);

    [[nodiscard]] Profile& profile_ref(std::size_t section, std::size_t dof) { return profiles_[section * dofs_ + dof]; }
    [[nodiscard]] std::span<Profile> section_profiles(std::size_t section) { return {profiles_.data() + section * dofs_, dofs_}; }

    template <class Func>
    void state_to_integrate_from(double time, std::size_t& section, Func&& set_integrate) const;
};

}  // namespace charqui
