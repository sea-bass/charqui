#pragma once

#include <cstddef>
#include <optional>
#include <span>
#include <vector>

#include <charqui/detail/block.hpp>
#include <charqui/detail/resolved.hpp>
#include <charqui/profile.hpp>
#include <charqui/types.hpp>

namespace charqui::detail {

//! Solves a state-to-state problem for all DoFs: per-DoF extremal profiles, then synchronization.
//! Workspace is allocated once per instance; solve() itself does not allocate.
class TargetSolver {
public:
    explicit TargetSolver(std::size_t dofs);

    //! Solve `rp` into `profiles` (one per DoF). Writes each DoF's independent minimum duration and the synchronized duration.
    [[nodiscard]] Result<void> solve(const ResolvedProblem& rp, std::span<Profile> profiles, std::span<double> independent_min_durations, double& duration);

    //! Access to the per-DoF blocks of the last solve (step-1 results)
    [[nodiscard]] std::span<const Block> blocks() const noexcept { return blocks_; }

private:
    std::size_t dofs_;
    std::vector<Block> blocks_;
    std::vector<double> new_phase_control_, pd_;
    std::vector<double> possible_t_syncs_;
    std::vector<std::size_t> idx_;

    [[nodiscard]] bool is_input_collinear(const ResolvedProblem& rp, Profile::Direction limiting_direction, std::size_t limiting_dof);
    [[nodiscard]] bool synchronize(const ResolvedProblem& rp, double& t_sync, std::optional<std::size_t>& limiting_dof, std::span<Profile> profiles);
};

}  // namespace charqui::detail
