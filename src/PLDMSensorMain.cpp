#include "PLDMSensor.hpp"

#include <boost/asio/steady_timer.hpp>
#include <boost/container/flat_map.hpp>

#include <sdbusplus/asio/connection.hpp>
#include <sdbusplus/asio/object_server.hpp>
#include <sdbusplus/bus/match.hpp>

#include <array>

class PollPLDMSensors;
std::shared_ptr<PollPLDMSensors> sensorsPtr = nullptr;

class PollPLDMSensors
{
public:
    PollPLDMSensors(boost::asio::io_context& io) : waitTimer(io)
    {

    }

    ~PollPLDMSensors() = default;

    void init(void)
    {
        read();
    }

    void read(void)
    {
        static constexpr size_t pollTime = 1; // in seconds

        waitTimer.expires_after(std::chrono::seconds(pollTime));
        waitTimer.async_wait([this](const boost::system::error_code& ec) {
            if (ec == boost::asio::error::operation_aborted)
            {
                return; // we're being cancelled
            }
            // read timer error
            if (ec)
            {
                std::cerr << "timer error\n";
                return;
            }

            for (auto sensorPtr : sensors)
            {
                // sensorPtr->updateValue(sensorPtr->value + 1);
                sensorPtr->getSensorReading();
            }

            read();
        });
    }

    std::shared_ptr<PLDMSensor> findSensorbyName(const std::string& name)
    {
        for (auto sensorPtr : sensors)
        {
            if (name == sensorPtr->name)
            {
                return sensorPtr;
            }
        }
        return nullptr;
    }

    void addSensor(const std::shared_ptr<PLDMSensor>& sensorPtr)
    {
        sensorPtr->setInitialProperties(sensor_paths::unitDegreesC);
        sensorPtr->updateValue(0);
        sensors.emplace_back(sensorPtr);
    }

    boost::asio::steady_timer waitTimer;
    std::vector<std::shared_ptr<PLDMSensor>> sensors;
};

static void handleSensorConfigurations(
    sdbusplus::asio::object_server& objectServer,
    std::shared_ptr<sdbusplus::asio::connection>& dbusConnection,
    const ManagedObjectType& sensorConfigurations)
{
    // iterate through all found configurations
    for (const auto& [interfacePath, sensorData] : sensorConfigurations)
    {
        std::cout << "path: " << std::string(interfacePath) << std::endl;
        // find base configuration
        auto sensorBase =
            sensorData.find(configInterfaceName(PLDMSensor::sensorType));
        if (sensorBase == sensorData.end())
        {
            continue;
        }

        const SensorBaseConfigMap& sensorConfig = sensorBase->second;
        auto findMctpEid = sensorConfig.find("MctpEid");
        auto findSensorId = sensorConfig.find("SensorId");
        auto findName = sensorConfig.find("Name");
        if (findMctpEid == sensorConfig.end() || findSensorId == sensorConfig.end() || findName == sensorConfig.end())
        {
            continue;
        }

        uint16_t mctpEid = std::visit(VariantToUnsignedIntVisitor(), findMctpEid->second);
        uint16_t sensorId = std::visit(VariantToUnsignedIntVisitor(), findSensorId->second);
        std::string sensorName = std::visit(VariantToStringVisitor(), findName->second);
        std::vector<thresholds::Threshold> sensorThresholds;
        if (!parseThresholdsFromConfig(sensorData, sensorThresholds))
        {
            std::cerr << "error populating thresholds for " << sensorName << "\n";
        }

        try
        {
            std::shared_ptr<PLDMSensor> sensorPtr =
                std::make_shared<PLDMSensor>(
                    objectServer, dbusConnection, sensorName,
                    std::move(sensorThresholds), interfacePath, mctpEid, sensorId);

            sensorsPtr->addSensor(sensorPtr);
        }
        catch (const std::invalid_argument& ex)
        {
            std::cerr << "Failed to add sensor for "
                      << std::string(interfacePath) << ": " << ex.what()
                      << "\n";
        }
    }

    sensorsPtr->init();
}

void createSensors(sdbusplus::asio::object_server& objectServer,
                   std::shared_ptr<sdbusplus::asio::connection>& dbusConnection)
{
    auto getter = std::make_shared<GetSensorConfiguration>(
        dbusConnection, [&objectServer, &dbusConnection](
                            const ManagedObjectType& sensorConfigurations) {
            handleSensorConfigurations(objectServer, dbusConnection,
                                       sensorConfigurations);
        });
    getter->getConfiguration(std::vector<std::string>{PLDMSensor::sensorType});
}

int main()
{
    boost::asio::io_context io;
    auto systemBus = std::make_shared<sdbusplus::asio::connection>(io);
    sdbusplus::asio::object_server objectServer(systemBus, true);
    objectServer.add_manager("/xyz/openbmc_project/sensors");
    systemBus->request_name("xyz.openbmc_project.PLDMSensor");
    sensorsPtr = std::make_shared<PollPLDMSensors>(io);

    boost::asio::post(io,
                    [&]() { createSensors(objectServer, systemBus); });

    boost::asio::steady_timer filterTimer(io);
    std::function<void(sdbusplus::message_t&)> eventHandler =
        [&filterTimer, &io, &objectServer, &systemBus](sdbusplus::message_t&) {
        // this implicitly cancels the timer
        filterTimer.expires_after(std::chrono::seconds(1));

        filterTimer.async_wait([&](const boost::system::error_code& ec) {
            if (ec == boost::asio::error::operation_aborted)
            {
                return; // we're being canceled
            }

            if (ec)
            {
                std::cerr << "Error: " << ec.message() << "\n";
                return;
            }

            createSensors(objectServer, systemBus);
        });
    };

    std::vector<std::unique_ptr<sdbusplus::bus::match_t>> matches =
        setupPropertiesChangedMatches(*systemBus, std::to_array<const char*>({PLDMSensor::sensorType}), eventHandler);

    setupManufacturingModeMatch(*systemBus);
    io.run();
    return 0;
}
