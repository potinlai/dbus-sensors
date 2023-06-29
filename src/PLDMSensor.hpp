#pragma once

#include "sensor.hpp"

#include <sdbusplus/asio/object_server.hpp>

#include <memory>
#include <string>

class PLDMSensor : public Sensor
{
public:
    static constexpr const char* sensorType = "PLDMSensor";

    PLDMSensor(
      sdbusplus::asio::object_server& objectServer,
      std::shared_ptr<sdbusplus::asio::connection>& conn,
      const std::string& sensorName,
      std::vector<thresholds::Threshold>&& thresholdsIn,
      const std::string& sensorConfiguration,
      uint16_t eid, uint16_t snrId);
    ~PLDMSensor();

    void checkThresholds(void);
    int getSensorReading(void);

    uint16_t getMctpEid(void) {return mctpEid;}
    uint16_t getSensorId(void) {return sensorId;}

private:
    sdbusplus::asio::object_server& objServer;
    uint8_t mctpEid;
    uint16_t sensorId;
};
