#ifndef ATI_FT_READER_HPP
#define ATI_FT_READER_HPP

#include <NIDAQmx.h>
#include <tinyxml2.h>

#include <array>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

class ATIFTReader {
public:
    using Matrix6x6 = std::array<std::array<double, 6>, 6>;
    using Vector6 = std::array<double, 6>;

    ATIFTReader(
        const std::string& calPath,
        const std::string& channelString = "Dev2/ai16:21",
        double samplingFrequency = 1000.0,
        int tareSamples = 500
    )
        : taskHandle_(nullptr)
    {
        calibrationMatrix_ = loadAtiCalibration(calPath);

        checkDaqError(DAQmxCreateTask("", &taskHandle_));

        checkDaqError(DAQmxCreateAIVoltageChan(
            taskHandle_,
            channelString.c_str(),
            "",
            DAQmx_Val_Diff,
            -10.0,
            10.0,
            DAQmx_Val_Volts,
            nullptr
        ));

        checkDaqError(DAQmxCfgSampClkTiming(
            taskHandle_,
            "",
            samplingFrequency,
            DAQmx_Val_Rising,
            DAQmx_Val_ContSamps,
            5000
        ));

        checkDaqError(DAQmxStartTask(taskHandle_));

        bias_ = computeBias(tareSamples);
    }

    ~ATIFTReader()
    {
        if (taskHandle_ != nullptr) {
            DAQmxStopTask(taskHandle_);
            DAQmxClearTask(taskHandle_);
            taskHandle_ = nullptr;
        }
    }

    Vector6 read()
    {
        std::vector<double> rawData(6);
        int32 samplesRead = 0;

        int32 error = DAQmxReadAnalogF64(
            taskHandle_,
            1,
            0.001,   // very short timeout
            DAQmx_Val_GroupByChannel,
            rawData.data(),
            static_cast<uInt32>(rawData.size()),
            &samplesRead,
            nullptr
        );

        if (error < 0 || samplesRead <= 0) {
            return lastFT_;
        }

        Vector6 netVoltage{};

        for (int i = 0; i < 6; ++i) {
            netVoltage[i] = rawData[i] - bias_[i];
        }

        lastFT_ = multiplyMatrixVector(calibrationMatrix_, netVoltage);
        return lastFT_;
    }

private:
    TaskHandle taskHandle_;
    Matrix6x6 calibrationMatrix_{};
    Vector6 bias_{};
    Vector6 lastFT_{};

    void checkDaqError(int32 error)
    {
        if (error < 0) {
            char errBuff[2048] = {'\0'};
            DAQmxGetExtendedErrorInfo(errBuff, sizeof(errBuff));
            throw std::runtime_error(errBuff);
        }
    }

    Matrix6x6 loadAtiCalibration(const std::string& filepath)
    {
        tinyxml2::XMLDocument doc;

        if (doc.LoadFile(filepath.c_str()) != tinyxml2::XML_SUCCESS) {
            throw std::runtime_error("Could not load calibration file: " + filepath);
        }

        auto* root = doc.FirstChildElement("FTSensor");
        if (!root) {
            throw std::runtime_error("Could not find <FTSensor> tag.");
        }

        auto* calibration = root->FirstChildElement("Calibration");
        if (!calibration) {
            throw std::runtime_error("Could not find <Calibration> tag.");
        }

        Matrix6x6 matrix{};
        int row = 0;

        for (auto* axis = calibration->FirstChildElement("Axis");
             axis != nullptr;
             axis = axis->NextSiblingElement("Axis")) {

            if (row >= 6) {
                break;
            }

            const char* valuesText = axis->Attribute("values");
            if (!valuesText) {
                throw std::runtime_error("Axis has no values attribute.");
            }

            double scale = 1.0;
            axis->QueryDoubleAttribute("scale", &scale);

            std::stringstream ss(valuesText);

            for (int col = 0; col < 6; ++col) {
                double value = 0.0;
                ss >> value;
                matrix[row][col] = value / scale;
            }

            row++;
        }

        if (row != 6) {
            throw std::runtime_error("Calibration matrix does not have 6 rows.");
        }

        return matrix;
    }

    Vector6 multiplyMatrixVector(const Matrix6x6& M, const Vector6& v)
    {
        Vector6 result{};

        for (int i = 0; i < 6; ++i) {
            result[i] = 0.0;
            for (int j = 0; j < 6; ++j) {
                result[i] += M[i][j] * v[j];
            }
        }

        return result;
    }

    Vector6 computeBias(int tareSamples)
    {
        std::vector<double> tareData(6 * tareSamples);
        int32 samplesRead = 0;

        int32 error = DAQmxReadAnalogF64(
            taskHandle_,
            tareSamples,
            5.0,
            DAQmx_Val_GroupByChannel,
            tareData.data(),
            static_cast<uInt32>(tareData.size()),
            &samplesRead,
            nullptr
        );

        checkDaqError(error);

        if (samplesRead <= 0) {
            throw std::runtime_error("No samples read during ATI F/T tare.");
        }

        Vector6 bias{};

        for (int ch = 0; ch < 6; ++ch) {
            double sum = 0.0;

            for (int s = 0; s < samplesRead; ++s) {
                sum += tareData[ch * samplesRead + s];
            }

            bias[ch] = sum / samplesRead;
        }

        return bias;
    }
};

#endif