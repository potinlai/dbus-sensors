/*
// Copyright (c) 2018 Intel Corporation
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//      http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
*/

#include <unistd.h>

#include <CPUSensor.hpp>
#include <Utils.hpp>
#include <boost/algorithm/string/predicate.hpp>
#include <boost/algorithm/string/replace.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <iostream>
#include <limits>
#include <sdbusplus/asio/connection.hpp>
#include <sdbusplus/asio/object_server.hpp>
#include <string>

static constexpr size_t warnAfterErrorCount = 10;

CPUSensor::CPUSensor(const std::string &path, const std::string &objectType,
                     sdbusplus::asio::object_server &objectServer,
                     std::shared_ptr<sdbusplus::asio::connection> &conn,
                     boost::asio::io_service &io, const std::string &sensorName,
                     std::vector<thresholds::Threshold> &&_thresholds,
                     const std::string &sensorConfiguration) :
    Sensor(),
    path(path), objectType(objectType), objServer(objectServer),
    name(boost::replace_all_copy(sensorName, " ", "_")), dbusConnection(conn),
    configuration(sensorConfiguration),

    inputDev(io, open(path.c_str(), O_RDONLY)), waitTimer(io), errCount(0),
    // todo, get these from config
    maxValue(127), minValue(-128)
{
    thresholds = std::move(_thresholds);
    sensorInterface = objectServer.add_interface(
        "/xyz/openbmc_project/sensors/temperature/" + name,
        "xyz.openbmc_project.Sensor.Value");
    if (thresholds::hasWarningInterface(thresholds))
    {
        thresholdInterfaceWarning = objectServer.add_interface(
            "/xyz/openbmc_project/sensors/temperature/" + name,
            "xyz.openbmc_project.Sensor.Threshold.Warning");
    }
    if (thresholds::hasCriticalInterface(thresholds))
    {
        thresholdInterfaceCritical = objectServer.add_interface(
            "/xyz/openbmc_project/sensors/temperature/" + name,
            "xyz.openbmc_project.Sensor.Threshold.Critical");
    }
    setInitialProperties(conn);
    isPowerOn(dbusConnection); // first call initializes
    setupRead();
}

CPUSensor::~CPUSensor()
{
    // close the input dev to cancel async operations
    inputDev.close();
    waitTimer.cancel();
    objServer.remove_interface(thresholdInterfaceWarning);
    objServer.remove_interface(thresholdInterfaceCritical);
    objServer.remove_interface(sensorInterface);
}

void CPUSensor::setupRead(void)
{
    boost::asio::async_read_until(
        inputDev, readBuf, '\n',
        [&](const boost::system::error_code &ec,
            std::size_t /*bytes_transfered*/) { handleResponse(ec); });
}

void CPUSensor::handleResponse(const boost::system::error_code &err)
{
    if (err == boost::system::errc::bad_file_descriptor)
    {
        return; // we're being destroyed
    }
    std::istream responseStream(&readBuf);
    if (!err)
    {
        std::string response;
        try
        {
            std::getline(responseStream, response);
            float nvalue = std::stof(response);
            responseStream.clear();
            nvalue /= CPUSensor::sensorScaleFactor;
            if (nvalue != value)
            {
                updateValue(nvalue);
            }
            errCount = 0;
        }
        catch (const std::invalid_argument &)
        {
            errCount++;
        }
    }
    else
    {
        errCount++;
    }

    // only send value update once
    if (errCount == warnAfterErrorCount)
    {
        // only an error if power is on
        if (isPowerOn(dbusConnection))
        {
            std::cerr << "Failure to read sensor " << name << " at " << path
                      << "\n";
            updateValue(0);
            errCount++;
        }
        else
        {
            errCount = 0; // check power again in 10 cycles
            sensorInterface->set_property(
                "Value", std::numeric_limits<double>::quiet_NaN());
        }
    }

    responseStream.clear();
    inputDev.close();
    int fd = open(path.c_str(), O_RDONLY);
    if (fd <= 0)
    {
        return; // we're no longer valid
    }
    inputDev.assign(fd);
    waitTimer.expires_from_now(
        boost::posix_time::milliseconds(CPUSensor::sensorPollMs));
    waitTimer.async_wait([&](const boost::system::error_code &ec) {
        if (ec == boost::asio::error::operation_aborted)
        {
            return; // we're being canceled
        }
        setupRead();
    });
}

void CPUSensor::checkThresholds(void)
{
    thresholds::checkThresholds(this);
}

void CPUSensor::updateValue(const double &newValue)
{
    sensorInterface->set_property("Value", newValue);
    value = newValue;
    checkThresholds();
}

void CPUSensor::setInitialProperties(
    std::shared_ptr<sdbusplus::asio::connection> &conn)
{
    // todo, get max and min from configuration
    sensorInterface->register_property("MaxValue", maxValue);
    sensorInterface->register_property("MinValue", minValue);
    sensorInterface->register_property("Value", value);

    for (auto &threshold : thresholds)
    {
        std::shared_ptr<sdbusplus::asio::dbus_interface> iface;
        std::string level;
        std::string alarm;
        if (threshold.level == thresholds::Level::CRITICAL)
        {
            iface = thresholdInterfaceCritical;
            if (threshold.direction == thresholds::Direction::HIGH)
            {
                level = "CriticalHigh";
                alarm = "CriticalAlarmHigh";
            }
            else
            {
                level = "CriticalLow";
                alarm = "CriticalAlarmLow";
            }
        }
        else if (threshold.level == thresholds::Level::WARNING)
        {
            iface = thresholdInterfaceWarning;
            if (threshold.direction == thresholds::Direction::HIGH)
            {
                level = "WarningHigh";
                alarm = "WarningAlarmHigh";
            }
            else
            {
                level = "WarningLow";
                alarm = "WarningAlarmLow";
            }
        }
        else
        {
            std::cerr << "Unknown threshold level" << threshold.level << "\n";
            continue;
        }
        if (!iface)
        {
            std::cout << "trying to set uninitialized interface\n";
            continue;
        }
        if (threshold.writeable)
        {
            iface->register_property(
                level, threshold.value,
                [&](const double &request, double &oldValue) {
                    oldValue = request; // todo, just let the config do this?
                    threshold.value = request;
                    thresholds::persistThreshold(configuration, objectType,
                                                 threshold, conn);
                    return 1;
                });
        }
        else
        {
            iface->register_property(level, threshold.value);
        }
        iface->register_property(alarm, false);
    }
    if (!sensorInterface->initialize())
    {
        std::cerr << "error initializing value interface\n";
    }
    if (thresholdInterfaceWarning && !thresholdInterfaceWarning->initialize())
    {
        std::cerr << "error initializing warning threshold interface\n";
    }

    if (thresholdInterfaceCritical && !thresholdInterfaceCritical->initialize())
    {
        std::cerr << "error initializing critical threshold interface\n";
    }
}