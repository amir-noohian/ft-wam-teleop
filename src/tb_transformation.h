#ifndef TOOL_FRAME_SYSTEMS_HPP
#define TOOL_FRAME_SYSTEMS_HPP

#include <barrett/systems.h>
#include <barrett/math.h>
#include <barrett/detail/ca_macro.h>

#include <string>

template <size_t DOF>
class ToolOrientationOutput : public barrett::systems::System {
public:
    typedef typename barrett::math::Matrix<3, 3>::type rot_type;

public:
    Output<rot_type> rotationOut;

protected:
    typename Output<rot_type>::Value* rotationOutValue;

public:
    ToolOrientationOutput(
        barrett::systems::ExecutionManager* em,
        barrett::systems::Wam<DOF>& wam,
        const std::string& sysName = "ToolOrientationOutput"
    ) :
        barrett::systems::System(sysName),
        rotationOut(this, &rotationOutValue),
        wam_(wam)
    {
        if (em != NULL) {
            em->startManaging(*this);
        }
    }

    virtual ~ToolOrientationOutput()
    {
        this->mandatoryCleanUp();
    }

protected:
    rot_type rotation_;

    virtual void operate()
    {
        rotation_ = wam_.getToolOrientation().toRotationMatrix();

        rotationOutValue->setData(&rotation_);
    }

private:
    barrett::systems::Wam<DOF>& wam_;

    DISALLOW_COPY_AND_ASSIGN(ToolOrientationOutput);
};



class ToolFTToBaseFT : public barrett::systems::System {
public:
    typedef typename barrett::math::Vector<6>::type sf_type;
    typedef typename barrett::math::Vector<6>::type bf_type;
    typedef typename barrett::math::Matrix<3, 3>::type rot_type;

public:
    Input<sf_type> toolFTInput;
    Input<rot_type> rotationInput;

    Output<bf_type> baseFTOutput;

protected:
    typename Output<bf_type>::Value* baseFTOutputValue;

public:
    explicit ToolFTToBaseFT(
        barrett::systems::ExecutionManager* em,
        const std::string& sysName = "ToolFTToBaseFT"
    ) :
        barrett::systems::System(sysName),
        toolFTInput(this),
        rotationInput(this),
        baseFTOutput(this, &baseFTOutputValue)
    {
        if (em != NULL) {
            em->startManaging(*this);
        }
    }

    virtual ~ToolFTToBaseFT()
    {
        this->mandatoryCleanUp();
    }

protected:
    bf_type baseFT_;

    virtual void operate()
    {
        const sf_type& ft_tool = toolFTInput.getValue();
        const rot_type& R_BE = rotationInput.getValue();

        // Force part
        baseFT_.template segment<3>(0) =
            R_BE.transpose() * ft_tool.template segment<3>(0);

        // Torque/moment part
        baseFT_.template segment<3>(3) =
            R_BE.transpose() * ft_tool.template segment<3>(3);

        baseFTOutputValue->setData(&baseFT_);
    }

private:
    DISALLOW_COPY_AND_ASSIGN(ToolFTToBaseFT);
};

#endif