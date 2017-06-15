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

class HandlebarBase {
    public:
        virtual model::real_t torque(
                const model::Bicycle::state_t& x, const model::Bicycle::input_t& u) const = 0;

    protected:
        ~HandlebarBase() { }
};

class null_t final : public HandlebarBase {
    public:
        null_t(model::Bicycle& bicycle);
        null_t(model::Bicycle& bicycle, model::real_t moment_of_inertia);
        virtual model::real_t torque(
                const model::Bicycle::state_t& x,
                const model::Bicycle::input_t& u = model::Bicycle::input_t::Zero()) const override;
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
        Handlebar0(model::Bicycle& bicycle);
        Handlebar0(model::Bicycle& bicycle, model::real_t moment_of_inertia);
        virtual model::real_t torque(
                const model::Bicycle::state_t& x,
                const model::Bicycle::input_t& u = model::Bicycle::input_t::Zero()) const override;

    private:
        model::Bicycle& m_bicycle;
};

/*
 * This class calculates handlebar feedback torque using the bicycle state
 * transition equation and physical moment of inertia of the steering assembly.
 */
class Handlebar2 final : public HandlebarBase {
    public:
        Handlebar2(model::Bicycle& bicycle, model::real_t moment_of_inertia);
        virtual model::real_t torque(
                const model::Bicycle::state_t& x,
                const model::Bicycle::input_t& u = model::Bicycle::input_t::Zero()) const override;
        model::real_t moment_of_inertia() const;

    private:
        model::Bicycle& m_bicycle;
        model::real_t m_I_delta;
};

} // namespace  // namespace haptic
