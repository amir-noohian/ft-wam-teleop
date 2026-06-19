#pragma once

#include <barrett/detail/ca_macro.h>
#include <barrett/systems/abstract/system.h>
#include <barrett/systems/abstract/single_io.h>
#include <barrett/math.h>

#include <boost/tuple/tuple.hpp>

#include <fstream>
#include <iomanip>
#include <stdexcept>
#include <string>

template <typename RecordTuple>
class FTCsvLogger : public barrett::systems::System,
                    public barrett::systems::SingleInput<RecordTuple> {
public:
    FTCsvLogger(barrett::systems::ExecutionManager* em,
                const std::string& path,
                size_t periodMultiplier = 1,
                const std::string& sysName = "FTCsvLogger")
        : barrett::systems::System(sysName),
          barrett::systems::SingleInput<RecordTuple>(this),
          logging_(true),
          ecCount_(0),
          ecCountRollover_(periodMultiplier == 0 ? 1 : periodMultiplier)
    {
        out_.open(path.c_str());
        if (!out_) {
            throw std::runtime_error("FTCsvLogger: failed to open " + path);
        }

        out_ << std::fixed << std::setprecision(8);
        out_ << "time,Fx,Fy,Fz,Tx,Ty,Tz\n";
        out_.flush();

        if (em != NULL) {
            em->startManaging(*this);
        }
    }

    virtual ~FTCsvLogger() {
        this->mandatoryCleanUp();
        closeFile();
    }

    void closeFile() {
        if (out_.is_open()) {
            out_.flush();
            out_.close();
        }
        logging_ = false;
    }

protected:
    virtual bool inputsValid() {
        ecCount_ = (ecCount_ + 1) % ecCountRollover_;
        return logging_ && ecCount_ == 0 && this->input.valueDefined();
    }

    virtual void operate() {
        const RecordTuple& rec = this->input.getValue();

        const double t = rec.template get<0>();
        const typename barrett::math::Vector<6>::type& ft = rec.template get<1>();

        out_ << t << ','
             << ft[0] << ',' << ft[1] << ',' << ft[2] << ','
             << ft[3] << ',' << ft[4] << ',' << ft[5] << '\n';
    }

    virtual void invalidateOutputs() {}

private:
    std::ofstream out_;
    bool logging_;
    size_t ecCount_;
    size_t ecCountRollover_;

    DISALLOW_COPY_AND_ASSIGN(FTCsvLogger);
};