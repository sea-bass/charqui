#include <charqui/detail/steps.hpp>

namespace charqui::detail {

PositionFirstOrderStep1::PositionFirstOrderStep1(double p0, double pf, double vMax, double vMin): _vMax(vMax), _vMin(vMin) {
    pd = pf - p0;
}

bool PositionFirstOrderStep1::get_profile(const Profile& input, Block& block) {
    block.min_index = 0;
    Profile& p = block.p_min();
    p.set_boundary(input);

    const double vf = (pd > 0) ? _vMax : _vMin;
    p.t[0] = 0;
    p.t[1] = 0;
    p.t[2] = 0;
    p.t[3] = pd / vf;
    p.t[4] = 0;
    p.t[5] = 0;
    p.t[6] = 0;

    if (p.check_for_first_order<Profile::ControlSigns::UDDU, Profile::ReachedLimits::VEL>(vf)) {
        block.t_min = p.t_sum.back() + p.brake.duration;
        return true;
    }
    return false;
}

}  // namespace charqui::detail
