#pragma once
/* bicycle submodule imports */
#include "bicycle.h"

/*
 * The haptic namespace includes classes used to calculate feedback torques at
 * the following parts of the bicycle:
 *  - handlebar
 *  - rear wheel (not yet implemented)
 */
namespace haptic {

class HandlebarBase {
    public:
        virtual model::real_t feedback_torque(const model::Bicycle::state_t& x, const model::Bicycle::input_t& u) const = 0;

    protected:
        model::Bicycle& m_bicycle;
        ~HandlebarBase() { }
};

/*
 * This class calculates handlebar feedback torque using a simplified static
 * equation of motion for the physical handlebars and ignoring the torque sensor
 * measurement.
 */
class HandlebarStatic final : public HandlebarBase {
    public:
        HandlebarStatic(model::Bicycle& bicycle);
        virtual model::real_t feedback_torque(const model::Bicycle::state_t& x, const model::Bicycle::input_t& u) const override;
};

} // namespace haptic
