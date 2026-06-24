#ifndef ATI_FT_SYSTEM_HPP
#define ATI_FT_SYSTEM_HPP

#include <barrett/systems.h>
#include <barrett/detail/ca_macro.h>

#include <Eigen/Core>
#include <string>
#include <thread>
#include <mutex>
#include <atomic>
#include <chrono>
#include <iostream>

#include "atift_reader.h"

class ATIFTSystem : public barrett::systems::System {
    typedef typename barrett::math::Vector<6>::type sf_type;

public:
    Output<sf_type> ftOutput;

protected:
    typename Output<sf_type>::Value* ftOutputValue;

public:
    explicit ATIFTSystem(
        barrett::systems::ExecutionManager* em,
        const std::string& calPath,
        const std::string& channelString = "Dev2/ai16:21",
        const std::string& sysName = "ATIFTSystem"
    )
        : barrett::systems::System(sysName),
          ftOutput(this, &ftOutputValue),
          reader_(calPath, channelString),
          running_(true),
          activeIndex_(0),
          valid_(false)
    {
        ft.setZero();
        buffers_[0].setZero();
        buffers_[1].setZero();

        readerThread_ = std::thread(&ATIFTSystem::readerLoop, this);

        if (em != NULL) {
            em->startManaging(*this);
        }
    }

    virtual ~ATIFTSystem()
    {
        running_.store(false);

        if (readerThread_.joinable()) {
            readerThread_.join();
        }

        this->mandatoryCleanUp();
    }

protected:
    sf_type ft;

    virtual void operate()
    {
        if (valid_.load(std::memory_order_acquire)) {
            int index = activeIndex_.load(std::memory_order_acquire);
            ft = buffers_[index];
        } else {
            ft.setZero();
        }

        ftOutputValue->setData(&ft);
    }

private:
    ATIFTReader reader_;

    std::thread readerThread_;
    std::atomic<bool> running_;

    sf_type buffers_[2];
    std::atomic<int> activeIndex_;
    std::atomic<bool> valid_;

    void readerLoop()
    {
        while (running_.load()) {
            try {
                ATIFTReader::Vector6 ftArray = reader_.read();

                sf_type temp;
                temp.setZero();

                for (int i = 0; i < 6; ++i) {
                    temp[i] = ftArray[i];
                }

                int current = activeIndex_.load(std::memory_order_relaxed);
                int inactive = 1 - current;

                buffers_[inactive] = temp;

                activeIndex_.store(inactive, std::memory_order_release);
                valid_.store(true, std::memory_order_release);

            } catch (const std::exception& e) {
                std::cerr << "ATI F/T read error: " << e.what() << std::endl;
            }

            // std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
    }

    DISALLOW_COPY_AND_ASSIGN(ATIFTSystem);
};



class FTSaturationSystem : public barrett::systems::System {
public:
    typedef typename barrett::math::Vector<6>::type sf_type;

public:
    Input<sf_type> ftInput;
    Output<sf_type> ftOutput;

protected:
    typename Output<sf_type>::Value* ftOutputValue;

public:
    explicit FTSaturationSystem(
        barrett::systems::ExecutionManager* em,
        const sf_type& ftLimit,
        const std::string& sysName = "FTSaturationSystem"
    )
        : barrett::systems::System(sysName),
          ftInput(this),
          ftOutput(this, &ftOutputValue),
          ftLimit_(ftLimit)
    {
        ftLimited_.setZero();

        if (em != NULL) {
            em->startManaging(*this);
        }
    }

    virtual ~FTSaturationSystem()
    {
        this->mandatoryCleanUp();
    }

protected:
    virtual void operate()
    {
        const sf_type& ftRaw = ftInput.getValue();

        ftLimited_ = barrett::math::saturate(ftRaw, ftLimit_);

        ftOutputValue->setData(&ftLimited_);
    }

private:
    sf_type ftLimit_;
    sf_type ftLimited_;

    DISALLOW_COPY_AND_ASSIGN(FTSaturationSystem);
};


template <size_t DOF>
class ATIFTSplitterSystem : public barrett::systems::System {
    BARRETT_UNITS_TEMPLATE_TYPEDEFS(DOF);
    typedef typename barrett::math::Vector<6>::type sf_type;

public:
    Input<sf_type> ftInput;

    Output<cf_type> forceOutput;
    Output<ct_type> torqueOutput;

protected:
    typename Output<cf_type>::Value* forceOutputValue;
    typename Output<ct_type>::Value* torqueOutputValue;

public:
    explicit ATIFTSplitterSystem(
        barrett::systems::ExecutionManager* em,
        const std::string& sysName = "ATIFTSplitterSystem"
    )
        : barrett::systems::System(sysName),
          ftInput(this),
          forceOutput(this, &forceOutputValue),
          torqueOutput(this, &torqueOutputValue)
    {
        if (em != NULL) {
            em->startManaging(*this);
        }
    }

    virtual ~ATIFTSplitterSystem()
    {
        this->mandatoryCleanUp();
    }

protected:
    cf_type force;
    ct_type torque;

    virtual void operate()
    {
        const sf_type& ft = ftInput.getValue();

        force[0] = ft[0];
        force[1] = ft[1];
        force[2] = ft[2];

        torque[0] = ft[3];
        torque[1] = ft[4];
        torque[2] = ft[5];

        forceOutputValue->setData(&force);
        torqueOutputValue->setData(&torque);
    }

private:
    DISALLOW_COPY_AND_ASSIGN(ATIFTSplitterSystem);
};

#endif