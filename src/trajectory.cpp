#include <charqui/trajectory.hpp>

#include <algorithm>
#include <cmath>
#include <iterator>

namespace charqui {

Trajectory::Trajectory(std::size_t dofs, std::size_t max_sections) {
    profiles_.reserve(dofs * std::max<std::size_t>(max_sections, 1));
    section_end_times_.reserve(std::max<std::size_t>(max_sections, 1));
    resize(dofs, 1);
}

void Trajectory::resize(std::size_t dofs, std::size_t sections) {
    dofs_ = dofs;
    sections_ = sections;
    if (profiles_.size() < dofs * sections) {
        profiles_.resize(dofs * sections);
    }
    if (section_end_times_.size() < sections) {
        section_end_times_.resize(sections);
    }
    independent_min_durations_.assign(dofs, 0.0);
}

template <class Func>
void Trajectory::state_to_integrate_from(double time, std::size_t& section, Func&& set_integrate) const {
    if (time >= duration_) {
        // At the end: the exact goal state; past it: constant acceleration. The section index is sections() only beyond the end.
        section = (time > duration_) ? sections_ : sections_ - 1;
        const Profile* last = profiles_.data() + (sections_ - 1) * dofs_;
        for (std::size_t dof = 0; dof < dofs_; ++dof) {
            const Profile& p = last[dof];
            const double t_pre = (sections_ > 1) ? section_end_times_[sections_ - 2] : p.brake.duration;
            const double t_diff = time - (t_pre + p.t_sum.back());
            set_integrate(dof, t_diff, p.p.back(), p.v.back(), p.a.back(), 0.0);
        }
        return;
    }

    const auto end_times = section_end_times();
    const auto section_it = std::upper_bound(end_times.begin(), end_times.end(), time);
    section = std::min(static_cast<std::size_t>(std::distance(end_times.begin(), section_it)), sections_ - 1);
    double t_diff = time;
    if (section > 0) {
        t_diff -= end_times[section - 1];
    }

    for (std::size_t dof = 0; dof < dofs_; ++dof) {
        const Profile& p = profile(section, dof);
        double t_diff_dof = t_diff;

        // Brake pre-trajectory (only in the first section)
        if (section == 0 && p.brake.duration > 0) {
            if (t_diff_dof < p.brake.duration) {
                const std::size_t index = (t_diff_dof < p.brake.t[0]) ? 0 : 1;
                if (index > 0) {
                    t_diff_dof -= p.brake.t[index - 1];
                }
                set_integrate(dof, t_diff_dof, p.brake.p[index], p.brake.v[index], p.brake.a[index], p.brake.j[index]);
                continue;
            }
            t_diff_dof -= p.brake.duration;
        }

        // A DoF without time synchronization may have finished earlier: hold constant acceleration
        if (t_diff_dof >= p.t_sum.back()) {
            set_integrate(dof, t_diff_dof - p.t_sum.back(), p.p.back(), p.v.back(), p.a.back(), 0.0);
            continue;
        }

        const auto phase_it = std::upper_bound(p.t_sum.begin(), p.t_sum.end(), t_diff_dof);
        const std::size_t phase = static_cast<std::size_t>(std::distance(p.t_sum.begin(), phase_it));
        if (phase > 0) {
            t_diff_dof -= p.t_sum[phase - 1];
        }
        set_integrate(dof, t_diff_dof, p.p[phase], p.v[phase], p.a[phase], p.j[phase]);
    }
}

void Trajectory::at(double time, std::span<double> position, std::span<double> velocity, std::span<double> acceleration,
                    std::span<double> jerk, std::size_t* section) const {
    std::size_t section_;
    state_to_integrate_from(time, section_, [&](std::size_t dof, double t, double p, double v, double a, double j) {
        std::tie(position[dof], velocity[dof], acceleration[dof]) = detail::integrate(t, p, v, a, j);
        if (!jerk.empty()) {
            jerk[dof] = j;
        }
    });
    if (section) {
        *section = section_;
    }
}

void Trajectory::at(double time, Sample& sample) const {
    sample.position.resize(dofs_);
    sample.velocity.resize(dofs_);
    sample.acceleration.resize(dofs_);
    sample.jerk.resize(dofs_);
    sample.time = time;
    at(time, sample.position, sample.velocity, sample.acceleration, sample.jerk, &sample.section);
}

Sample Trajectory::at(double time) const {
    Sample sample;
    at(time, sample);
    return sample;
}

std::vector<Sample> Trajectory::sample(std::span<const double> times) const {
    std::vector<Sample> samples(times.size());
    for (std::size_t i = 0; i < times.size(); ++i) {
        at(times[i], samples[i]);
    }
    return samples;
}

std::vector<Sample> Trajectory::sample(double dt) const {
    std::vector<Sample> samples;
    if (dt <= 0.0) {
        return samples;
    }
    const auto steps = static_cast<std::size_t>(std::floor(duration_ / dt));
    samples.reserve(steps + 2);
    for (std::size_t i = 0; i <= steps; ++i) {
        samples.emplace_back();
        at(static_cast<double>(i) * dt, samples.back());
    }
    if (samples.back().time < duration_) {
        samples.emplace_back();
        at(duration_, samples.back());
    }
    return samples;
}

std::vector<Bound> Trajectory::position_extrema() const {
    std::vector<Bound> extrema(dofs_);
    for (std::size_t dof = 0; dof < dofs_; ++dof) {
        extrema[dof] = profile(0, dof).get_position_extrema();
    }

    for (std::size_t section = 1; section < sections_; ++section) {
        const double offset = section_end_times_[section - 1];
        for (std::size_t dof = 0; dof < dofs_; ++dof) {
            const Bound b = profile(section, dof).get_position_extrema();
            if (b.max > extrema[dof].max) {
                extrema[dof].max = b.max;
                extrema[dof].t_max = offset + b.t_max;
            }
            if (b.min < extrema[dof].min) {
                extrema[dof].min = b.min;
                extrema[dof].t_min = offset + b.t_min;
            }
        }
    }
    return extrema;
}

std::optional<double> Trajectory::first_time_at_position(std::size_t dof, double position, double after) const {
    if (dof >= dofs_) {
        return std::nullopt;
    }

    for (std::size_t section = 0; section < sections_; ++section) {
        const double offset = (section > 0) ? section_end_times_[section - 1] : 0.0;
        double time;
        if (profile(section, dof).get_first_state_at_position(position, time, std::max(after - offset, 0.0))) {
            return offset + time;
        }
    }
    return std::nullopt;
}

}  // namespace charqui
