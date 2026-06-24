#pragma once

#include <barrett/detail/ca_macro.h>
#include <barrett/systems.h>
#include <barrett/units.h>

template <size_t DOF>
class ToolJacobianOutput : public barrett::systems::System {
    BARRETT_UNITS_TEMPLATE_TYPEDEFS(DOF);

  public:
    typedef barrett::math::Matrix<6, DOF> jacobian_type;

    Output<jacobian_type> output;

    explicit ToolJacobianOutput(barrett::systems::Wam<DOF>& wam_,
                                barrett::systems::ExecutionManager* em,
                                const std::string& sysName = "ToolJacobianOutput")
        : System(sysName)
        , wam(wam_)
        , output(this, &outputValue) {
        if (em != NULL) {
            em->startManaging(*this);
        }
    }

    virtual ~ToolJacobianOutput() {
        this->mandatoryCleanUp();
    }

  protected:
    barrett::systems::Wam<DOF>& wam;
    typename Output<jacobian_type>::Value* outputValue;
    jacobian_type J;

    virtual void operate() {
        J = wam.getToolJacobian();
        outputValue->setData(&J);
    }

  private:
    DISALLOW_COPY_AND_ASSIGN(ToolJacobianOutput);
};