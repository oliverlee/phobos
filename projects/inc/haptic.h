#pragma once
/* bicycle submodule imports */
#include "bicycle/bicycle.h"

/*
 * The haptic namespace includes classes used to calculate feedback torques at
 * the following parts of the bicycle:
 *  - handlebar
 *  - rear wheel (not yet implemented)
 */
namespace haptic {

using real_t = model::real_t;
using model_t = model::Bicycle;

class HandlebarBase {
    public:
        virtual real_t torque(
                const model_t::state_t& x, const model_t::input_t& u) const = 0;

    protected:
        ~HandlebarBase() { }
};

/*
 * This class calculates handlebar feedback torque using a simplified static
 * equation of motion for the physical handlebars and ignoring the torque sensor
 * measurement.
 *
 * The class feedback torque calculation ignores input torque,
 * roll acceleration, steer acceleration, roll rate, steer rate.
 */
class Handlebar0 final : public HandlebarBase {
    public:
        Handlebar0(model_t& bicycle);
        virtual real_t torque(
                const model_t::state_t& x,
                const model_t::input_t& u = model_t::input_t::Zero()) const override;

    private:
        model_t& m_bicycle;
};

/*
 * This class calculates handlebar feedback torque using a simplified
 * equation of motion for the physical handlebars and ignoring the torque sensor
 * measurement.
 *
 * The class feedback torque calculation ignores input torque,
 * roll acceleration, steer acceleration.
 */
class Handlebar1 final : public HandlebarBase {
    public:
        Handlebar1(model_t& bicycle);
        virtual real_t torque(
                const model_t::state_t& x,
                const model_t::input_t& u = model_t::input_t::Zero()) const override;

    private:
        model_t& m_bicycle;
};

/*
 * This class calculates handlebar feedback torque using the bicycle state
 * transition equation and physical moment of inertia of the steering assembly.
 */
class Handlebar2 final : public HandlebarBase {
    public:
        Handlebar2(model_t& bicycle, real_t moment_of_inertia);
        virtual real_t torque(
                const model_t::state_t& x,
                const model_t::input_t& u = model_t::input_t::Zero()) const override;
        real_t moment_of_inertia() const;

    private:
        model_t& m_bicycle;
        real_t m_I_delta;
};

} // namespace  // namespace haptic
