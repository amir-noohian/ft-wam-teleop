#pragma once

#include <barrett/detail/ca_macro.h>
#include <barrett/systems.h>
#include <barrett/units.h>

template <size_t DOF>
class BaseFTToJointTorque : public barrett::systems::System {
    BARRETT_UNITS_TEMPLATE_TYPEDEFS(DOF);

public:
    typedef typename barrett::math::Matrix<6, DOF>::type jacobian_type;
    typedef typename barrett::math::Vector<6>::type bf_type;

public:
    Input<jacobian_type> jacobianInput;
    Input<bf_type> baseFTInput;

    Output<jt_type> jointTorqueOutput;

protected:
    typename Output<jt_type>::Value* jointTorqueOutputValue;

public:
    explicit BaseFTToJointTorque(
        barrett::systems::ExecutionManager* em,
        const std::string& sysName = "BaseFTToJointTorque"
    ) :
        System(sysName),
        jacobianInput(this),
        baseFTInput(this),
        jointTorqueOutput(this, &jointTorqueOutputValue)
    {
        if (em != NULL) {
            em->startManaging(*this);
        }
    }

    virtual ~BaseFTToJointTorque()
    {
        this->mandatoryCleanUp();
    }

protected:
    jt_type jointTorque_;

    virtual void operate()
    {
        const jacobian_type& J = jacobianInput.getValue();
        const bf_type& F_base = baseFTInput.getValue();

        bf_type wrenchForce = F_base;
        wrenchForce[3] = 0.0;
        wrenchForce[4] = 0.0;
        wrenchForce[5] = 0.0;

        // tau = J^T * F
        // jointTorque_ = 1.0 * J.transpose() * F_base;
        jointTorque_ = 1.0 * J.transpose() * wrenchForce;

        jointTorqueOutputValue->setData(&jointTorque_);
    }

private:
    DISALLOW_COPY_AND_ASSIGN(BaseFTToJointTorque);
};

