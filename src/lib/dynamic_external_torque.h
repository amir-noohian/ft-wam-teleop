#pragma once

#include <barrett/detail/ca_macro.h>
#include <barrett/systems.h>
#include <barrett/units.h>

template <size_t DOF>
class DynamicExternalTorque : public barrett::systems::System {
    BARRETT_UNITS_TEMPLATE_TYPEDEFS(DOF);

  public:
    Input<jt_type> wamTorqueSumIn;
    Input<jt_type> wamGravityIn;
    Input<jt_type> wamDynamicsIn;
    Output<jt_type> wamExternalTorqueOut;

    explicit DynamicExternalTorque(barrett::systems::ExecutionManager* em, const std::string& sysName = "DynamicExternalTorque")
        : System(sysName)
        , wamTorqueSumIn(this)
        , wamGravityIn(this)
        , wamDynamicsIn(this)
        , wamExternalTorqueOut(this, &jtOutputValue) {

        if (em != NULL) {
            em->startManaging(*this);
        }
    }

    virtual ~DynamicExternalTorque() {
        this->mandatoryCleanUp();
    }

  protected:
    typename Output<jt_type>::Value* jtOutputValue;
    jt_type jtSum;
    jt_type gravity;
    jt_type dynamics;
    jt_type externalTorque;

    virtual void operate() {
        jtSum = wamTorqueSumIn.getValue();
        gravity = wamGravityIn.getValue();
        dynamics = wamDynamicsIn.getValue();
        externalTorque = jtSum - gravity - dynamics;
        jtOutputValue->setData(&externalTorque);
    }

  private:
    DISALLOW_COPY_AND_ASSIGN(DynamicExternalTorque);
};
