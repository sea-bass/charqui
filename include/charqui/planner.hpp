#pragma once

#include <cstddef>
#include <memory>
#include <span>
#include <vector>

#include <charqui/trajectory.hpp>
#include <charqui/types.hpp>

namespace charqui {

namespace detail {
class TargetSolver;
class WaypointSolver;
struct ResolvedProblem;
}  // namespace detail

//! Offline / one-shot trajectory generation. Workspace is allocated in the constructor; plan() into an existing Trajectory does not allocate.
//! A Planner is not thread-safe, but independent instances can be used concurrently.
class Planner {
public:
    //! `max_waypoints` only pre-allocates; problems with more waypoints still work (and allocate).
    explicit Planner(std::size_t dofs, std::size_t max_waypoints = 0);
    ~Planner();
    Planner(Planner&&) noexcept;
    Planner& operator=(Planner&&) noexcept;

    [[nodiscard]] std::size_t dofs() const noexcept { return dofs_; }

    //! Plan a new trajectory
    [[nodiscard]] Result<Trajectory> plan(const Problem& problem);

    //! Plan into an existing trajectory, reusing its storage
    [[nodiscard]] Result<void> plan(const Problem& problem, Trajectory& trajectory);

    //! Check a problem for consistency (sizes, NaNs, signs of limits, reachability of the goal state)
    [[nodiscard]] static Result<void> validate(const Problem& problem, bool check_start_within_limits = false, bool check_goal_within_limits = true);

    //! Drop waypoints that lie within `threshold` (per DoF) of the straight line between their neighbours
    [[nodiscard]] static std::vector<std::vector<double>> filter_waypoints(const Problem& problem, std::span<const double> threshold);

private:
    std::size_t dofs_;
    std::unique_ptr<detail::ResolvedProblem> resolved_;
    std::unique_ptr<detail::TargetSolver> target_;
    std::unique_ptr<detail::WaypointSolver> waypoints_;
};

//! Cyclic (online) trajectory generation: owns the current kinematic state and advances it every control cycle.
//! A new trajectory is computed only when the goal, limits, options or waypoints change.
class Online {
public:
    //! One control-cycle result
    struct Step : Sample {
        bool finished{false};   //!< The trajectory end has been passed
        bool replanned{false};  //!< A new trajectory was computed in this cycle
    };

    Online(std::size_t dofs, double delta_time, std::size_t max_waypoints = 0);

    [[nodiscard]] std::size_t dofs() const noexcept { return planner_.dofs(); }
    [[nodiscard]] double delta_time() const noexcept { return delta_time_; }

    //! Advance one control cycle towards `problem`. `problem.start` is used only for the very first update (or after reset());
    //! afterwards the internally tracked state is the start. Use set_state() to inject a measured state.
    [[nodiscard]] Result<Step> update(const Problem& problem);

    //! Convenience loop: run update() until finished, throwing CharquiError on error
    class Run;
    [[nodiscard]] Run run(const Problem& problem);

    //! Override the tracked state (forces a new trajectory on the next update)
    void set_state(const State& state);

    //! Forget the tracked state and trajectory
    void reset();

    [[nodiscard]] bool finished() const noexcept { return has_trajectory_ && time_ >= trajectory_.duration(); }
    [[nodiscard]] double time() const noexcept { return time_; }
    [[nodiscard]] const Trajectory& trajectory() const noexcept { return trajectory_; }
    [[nodiscard]] const State& state() const noexcept { return state_; }
    [[nodiscard]] double calculation_duration_us() const noexcept { return calculation_duration_us_; }

private:
    Planner planner_;
    double delta_time_;
    Problem current_;
    State state_;
    Trajectory trajectory_;
    Step step_;
    bool has_state_{false};
    bool has_trajectory_{false};
    double time_{0.0};
    std::size_t cycle_{0};  // control cycles since the last (re)plan; time_ = cycle_ * delta_time_ avoids drift
    std::size_t section_{0};
    std::size_t consumed_waypoints_{0};
    double calculation_duration_us_{0.0};

    [[nodiscard]] bool goal_changed(const Problem& problem) const;
};

class Online::Run {
    Online* online_;
    const Problem* problem_;

public:
    Run(Online& online, const Problem& problem) : online_(&online), problem_(&problem) {}

    //! Yields every step up to and including the one that reaches the goal
    class Iterator {
        Online* online_{nullptr};
        const Problem* problem_{nullptr};
        bool at_end_{true};
        bool yielded_final_{false};

        void advance() {
            if (yielded_final_) {
                at_end_ = true;
                return;
            }
            auto step = online_->update(*problem_);
            if (!step) {
                throw CharquiError(std::move(step.error()));
            }
            yielded_final_ = step->finished;
        }

    public:
        using value_type = Step;
        using difference_type = std::ptrdiff_t;

        Iterator() = default;
        Iterator(Online& online, const Problem& problem) : online_(&online), problem_(&problem), at_end_(false) { advance(); }

        const Step& operator*() const { return online_->step_; }
        const Step* operator->() const { return &online_->step_; }
        Iterator& operator++() {
            advance();
            return *this;
        }
        void operator++(int) { advance(); }
        bool operator==(const Iterator& other) const { return at_end_ == other.at_end_; }
    };

    Iterator begin() { return Iterator(*online_, *problem_); }
    Iterator end() { return Iterator(); }
};

inline Online::Run Online::run(const Problem& problem) {
    return Run(*this, problem);
}

}  // namespace charqui
