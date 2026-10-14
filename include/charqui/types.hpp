#pragma once

#include <cstddef>
#include <expected>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace charqui {

//! Which kinematic quantity the goal state constrains
enum class Interface { Position, Velocity };

//! How the degrees of freedom are synchronized in time
enum class Sync {
    Time,             //!< All DoFs finish at the same time (default)
    TimeIfNecessary,  //!< Like Time, but DoFs with a zero goal velocity/acceleration are not slowed down
    Phase,            //!< Time synchronization with a straight-line path (only if the inputs allow it, otherwise Time)
    None,             //!< Every DoF is time-optimal on its own
};

enum class ErrorCode {
    InvalidInput,
    TrajectoryDuration,  //!< The resulting duration exceeds the supported range
    PositionalLimits,
    ZeroLimits,          //!< A zero limit conflicts with the problem
    ExecutionTime,       //!< No extremal profile could be found (step 1)
    Synchronization,     //!< No synchronized profile could be found (step 2)
};

struct Error {
    ErrorCode code;
    std::string message;
};

template <class T>
using Result = std::expected<T, Error>;

[[nodiscard]] constexpr std::string_view to_string(ErrorCode code) noexcept {
    switch (code) {
        case ErrorCode::InvalidInput: return "invalid input";
        case ErrorCode::TrajectoryDuration: return "trajectory duration";
        case ErrorCode::PositionalLimits: return "positional limits";
        case ErrorCode::ZeroLimits: return "zero limits";
        case ErrorCode::ExecutionTime: return "execution time calculation";
        case ErrorCode::Synchronization: return "synchronization calculation";
    }
    return "unknown";
}

//! Exception form of an Error, used by the throwing conveniences and the Python bindings
class CharquiError : public std::runtime_error {
    ErrorCode code_;

public:
    explicit CharquiError(Error error) : std::runtime_error("[charqui] " + error.message), code_(error.code) {}
    [[nodiscard]] ErrorCode code() const noexcept { return code_; }
};

//! Unwrap a Result or throw the contained Error as CharquiError
template <class T>
T value_or_throw(Result<T>&& result) {
    if (!result) {
        throw CharquiError(std::move(result.error()));
    }
    if constexpr (!std::is_void_v<T>) {
        return std::move(*result);
    }
}

//! A kinematic state. Velocity and acceleration may be left empty, meaning zero.
struct State {
    std::vector<double> position;
    std::vector<double> velocity;
    std::vector<double> acceleration;

    bool operator==(const State&) const = default;
};

//! Kinematic limits per degree of freedom.
//! Empty max_acceleration or max_jerk means unlimited (second-/first-order profiles).
//! Unset min_velocity / min_acceleration default to the negated maxima.
struct Limits {
    std::vector<double> max_velocity;
    std::vector<double> max_acceleration;
    std::vector<double> max_jerk;
    std::optional<std::vector<double>> min_velocity;
    std::optional<std::vector<double>> min_acceleration;

    bool operator==(const Limits&) const = default;
};

struct Options {
    Interface interface{Interface::Position};
    Sync sync{Sync::Time};

    //! Lower bound on the trajectory duration
    std::optional<double> minimum_duration;

    //! If set, the duration is rounded up to a multiple of this step (e.g. the control cycle)
    std::optional<double> duration_step;

    //! Per-DoF overrides of interface / synchronization, and the ability to freeze single DoFs
    std::optional<std::vector<Interface>> per_dof_interface;
    std::optional<std::vector<Sync>> per_dof_sync;
    std::optional<std::vector<bool>> enabled;

    bool operator==(const Options&) const = default;
};

//! Intermediate positions the trajectory passes through, in order, before reaching the goal.
//! Per-section vectors have one entry per section, i.e. positions.size() + 1 entries.
struct Waypoints {
    std::vector<std::vector<double>> positions;
    std::optional<std::vector<Limits>> per_section_limits;
    std::optional<std::vector<double>> per_section_minimum_duration;

    [[nodiscard]] bool empty() const noexcept { return positions.empty(); }
    [[nodiscard]] std::size_t sections() const noexcept { return positions.size() + 1; }

    bool operator==(const Waypoints&) const = default;
};

//! A complete trajectory generation problem
struct Problem {
    State start;
    State goal;
    Limits limits;
    Options options{};
    Waypoints waypoints{};

    [[nodiscard]] std::size_t dofs() const noexcept { return start.position.size(); }

    bool operator==(const Problem&) const = default;
};

//! The kinematic state of all DoFs at a point in time, plus the section of the trajectory it belongs to
struct Sample {
    std::vector<double> position;
    std::vector<double> velocity;
    std::vector<double> acceleration;
    std::vector<double> jerk;
    double time{0.0};
    std::size_t section{0};

    [[nodiscard]] State state() const { return State{position, velocity, acceleration}; }
};

}  // namespace charqui
