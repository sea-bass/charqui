#pragma once

#include <array>
#include <cstddef>

#include <charqui/detail/block.hpp>
#include <charqui/profile.hpp>

// Closed-form solvers for a single degree of freedom.
// Step 1 computes all extremal (minimum-time) profiles and the resulting Block.
// Step 2 computes a profile of a given, longer duration tf (time synchronization).

namespace charqui::detail {

//! Step 1, third-order position interface: extremal profiles
class PositionThirdOrderStep1 {
    using ReachedLimits = Profile::ReachedLimits;
    using ControlSigns = Profile::ControlSigns;

    const double v0, a0;
    const double vf, af;
    const double _vMax, _vMin, _aMax, _aMin, _jMax;

    // Pre-calculated expressions
    double pd;
    double v0_v0, vf_vf;
    double a0_a0, a0_p3, a0_p4;
    double af_af, af_p3, af_p4;
    double jMax_jMax;

    // Extremal profiles are written into the Block's storage (max 5 valid + 1 for probing)
    using ProfileIter = Profile*;

    void time_all_vel(ProfileIter& profile, double vMax, double vMin, double aMax, double aMin, double jMax, bool return_after_found) const;
    void time_acc0_acc1(ProfileIter& profile, double vMax, double vMin, double aMax, double aMin, double jMax, bool return_after_found) const;
    void time_all_none_acc0_acc1(ProfileIter& profile, double vMax, double vMin, double aMax, double aMin, double jMax, bool return_after_found) const;

    // Only for numerical issues, always return_after_found
    void time_acc1_vel_two_step(ProfileIter& profile, double vMax, double vMin, double aMax, double aMin, double jMax) const;
    void time_acc0_two_step(ProfileIter& profile, double vMax, double vMin, double aMax, double aMin, double jMax) const;
    void time_vel_two_step(ProfileIter& profile, double vMax, double vMin, double aMax, double aMin, double jMax) const;
    void time_none_two_step(ProfileIter& profile, double vMax, double vMin, double aMax, double aMin, double jMax) const;

    // Only for the zero-limits case
    bool time_all_single_step(Profile* profile, double vMax, double vMin, double aMax, double aMin, double jMax) const;

    void add_profile(ProfileIter& profile) const {
        const auto prev_profile = profile;
        ++profile;
        profile->set_boundary(*prev_profile);
    }

public:
    explicit PositionThirdOrderStep1(double p0, double v0, double a0, double pf, double vf, double af, double vMax, double vMin, double aMax, double aMin, double jMax);

    bool get_profile(const Profile& input, Block& block);
};

//! Step 2, third-order position interface: time synchronization
class PositionThirdOrderStep2 {
    using ReachedLimits = Profile::ReachedLimits;
    using ControlSigns = Profile::ControlSigns;

    const double v0, a0;
    const double tf, vf, af;
    const double _vMax, _vMin, _aMax, _aMin, _jMax;

    // Pre-calculated expressions
    double pd;
    double tf_tf, tf_p3, tf_p4;
    double vd, vd_vd;
    double ad, ad_ad;
    double v0_v0, vf_vf;
    double a0_a0, a0_p3, a0_p4, a0_p5, a0_p6;
    double af_af, af_p3, af_p4, af_p5, af_p6;
    double jMax_jMax;
    double g1, g2;

    bool time_acc0_acc1_vel(Profile& profile, double vMax, double vMin, double aMax, double aMin, double jMax);
    bool time_acc1_vel(Profile& profile, double vMax, double vMin, double aMax, double aMin, double jMax);
    bool time_acc0_vel(Profile& profile, double vMax, double vMin, double aMax, double aMin, double jMax);
    bool time_vel(Profile& profile, double vMax, double vMin, double aMax, double aMin, double jMax);
    bool time_acc0_acc1(Profile& profile, double vMax, double vMin, double aMax, double aMin, double jMax);
    bool time_acc1(Profile& profile, double vMax, double vMin, double aMax, double aMin, double jMax);
    bool time_acc0(Profile& profile, double vMax, double vMin, double aMax, double aMin, double jMax);
    bool time_none(Profile& profile, double vMax, double vMin, double aMax, double aMin, double jMax);

public:
    explicit PositionThirdOrderStep2(double tf, double p0, double v0, double a0, double pf, double vf, double af, double vMax, double vMin, double aMax, double aMin, double jMax);

    bool get_profile(Profile& profile);
};

//! Step 1, second-order position interface (infinite jerk): extremal profiles
class PositionSecondOrderStep1 {
    using ReachedLimits = Profile::ReachedLimits;
    using ControlSigns = Profile::ControlSigns;

    const double v0, vf;
    const double _vMax, _vMin, _aMax, _aMin;

    double pd;

    // Extremal profiles are written into the Block's storage (max 3 valid + 1 for probing)
    using ProfileIter = Profile*;

    void time_acc0(ProfileIter& profile, double vMax, double vMin, double aMax, double aMin, bool return_after_found) const;
    void time_none(ProfileIter& profile, double vMax, double vMin, double aMax, double aMin, bool return_after_found) const;

    bool time_all_single_step(Profile* profile, double vMax, double vMin, double aMax, double aMin) const;

    void add_profile(ProfileIter& profile) const {
        const auto prev_profile = profile;
        ++profile;
        profile->set_boundary(*prev_profile);
    }

public:
    explicit PositionSecondOrderStep1(double p0, double v0, double pf, double vf, double vMax, double vMin, double aMax, double aMin);

    bool get_profile(const Profile& input, Block& block);
};

//! Step 2, second-order position interface: time synchronization
class PositionSecondOrderStep2 {
    using ReachedLimits = Profile::ReachedLimits;
    using ControlSigns = Profile::ControlSigns;

    const double v0, tf, vf;
    const double _vMax, _vMin, _aMax, _aMin;

    double pd, vd;

    bool time_acc0(Profile& profile, double vMax, double vMin, double aMax, double aMin);
    bool time_none(Profile& profile, double vMax, double vMin, double aMax, double aMin);

    bool check_all(Profile& profile, double vMax, double vMin, double aMax, double aMin) {
        return time_acc0(profile, vMax, vMin, aMax, aMin) || time_none(profile, vMax, vMin, aMax, aMin);
    }

public:
    explicit PositionSecondOrderStep2(double tf, double p0, double v0, double pf, double vf, double vMax, double vMin, double aMax, double aMin);

    bool get_profile(Profile& profile);
};

//! Step 1, first-order position interface (infinite acceleration): extremal profiles
class PositionFirstOrderStep1 {
    const double _vMax, _vMin;
    double pd;

public:
    explicit PositionFirstOrderStep1(double p0, double pf, double vMax, double vMin);

    bool get_profile(const Profile& input, Block& block);
};

//! Step 2, first-order position interface: time synchronization
class PositionFirstOrderStep2 {
    const double tf, _vMax, _vMin;
    double pd;

public:
    explicit PositionFirstOrderStep2(double tf, double p0, double pf, double vMax, double vMin);

    bool get_profile(Profile& profile);
};

//! Step 1, third-order velocity interface: extremal profiles
class VelocityThirdOrderStep1 {
    using ReachedLimits = Profile::ReachedLimits;
    using ControlSigns = Profile::ControlSigns;

    const double a0, af;
    const double _aMax, _aMin, _jMax;

    double vd;

    // Extremal profiles are written into the Block's storage (max 3 valid + 1 for probing)
    using ProfileIter = Profile*;

    void time_acc0(ProfileIter& profile, double aMax, double aMin, double jMax, bool return_after_found) const;
    void time_none(ProfileIter& profile, double aMax, double aMin, double jMax, bool return_after_found) const;

    bool time_all_single_step(Profile* profile, double aMax, double aMin, double jMax) const;

    void add_profile(ProfileIter& profile) const {
        const auto prev_profile = profile;
        ++profile;
        profile->set_boundary(*prev_profile);
    }

public:
    explicit VelocityThirdOrderStep1(double v0, double a0, double vf, double af, double aMax, double aMin, double jMax);

    bool get_profile(const Profile& input, Block& block);
};

//! Step 2, third-order velocity interface: time synchronization
class VelocityThirdOrderStep2 {
    using ReachedLimits = Profile::ReachedLimits;
    using ControlSigns = Profile::ControlSigns;

    const double a0, tf, af;
    const double _aMax, _aMin, _jMax;

    double vd, ad;

    bool time_acc0(Profile& profile, double aMax, double aMin, double jMax);
    bool time_none(Profile& profile, double aMax, double aMin, double jMax);

    bool check_all(Profile& profile, double aMax, double aMin, double jMax) {
        return time_acc0(profile, aMax, aMin, jMax) || time_none(profile, aMax, aMin, jMax);
    }

public:
    explicit VelocityThirdOrderStep2(double tf, double v0, double a0, double vf, double af, double aMax, double aMin, double jMax);

    bool get_profile(Profile& profile);
};

//! Step 1, second-order velocity interface: extremal profiles
class VelocitySecondOrderStep1 {
    const double _aMax, _aMin;
    double vd;

public:
    explicit VelocitySecondOrderStep1(double v0, double vf, double aMax, double aMin);

    bool get_profile(const Profile& input, Block& block);
};

//! Step 2, second-order velocity interface: time synchronization
class VelocitySecondOrderStep2 {
    const double tf, _aMax, _aMin;
    double vd;

public:
    explicit VelocitySecondOrderStep2(double tf, double v0, double vf, double aMax, double aMin);

    bool get_profile(Profile& profile);
};

}  // namespace charqui::detail
