#include "PLDMSensor.hpp"

#include <iostream>

#include <linux/mctp.h>
#include <libpldm/pldm.h>
#include <libpldm/base.h>
#include <libpldm/platform.h>
#include <sdbusplus/server.hpp>

static constexpr double maxReading = 127;
static constexpr double minReading = 0;

static constexpr uint8_t MCTP_MSG_TYPE_PLDM = 1;

struct CustomFD
{
    CustomFD(const CustomFD&) = delete;
    CustomFD& operator=(const CustomFD&) = delete;
    CustomFD(CustomFD&&) = delete;
    CustomFD& operator=(CustomFD&&) = delete;

    CustomFD(int fd) : fd(fd) {}

    ~CustomFD()
    {
        if (fd >= 0)
        {
            close(fd);
        }
    }

    int operator()() const
    {
        return fd;
    }

  private:
    int fd = -1;
};

int mctpSockSendRecv(uint16_t mctpEid,
                     const std::vector<uint8_t>& requestMsg,
                     std::vector<uint8_t>& responseMsg,
                     [[maybe_unused]] bool pldmVerbose)
{
    int returnCode = 0;
    pldm_header_info responseHdrFields{};
    pldm_header_info requestHdrFields{};

    int sockFd = socket(AF_MCTP, SOCK_DGRAM, 0);
    if (-1 == sockFd)
    {
        returnCode = -errno;
        std::cerr << "Failed to create the socket : RC = " << sockFd << "\n";
        return returnCode;
    }
    // Logger(pldmVerbose, "Success in creating the socket : RC = ", sockFd);

    struct sockaddr_mctp addr = {0, 0, 0, 0, 0, 0, 0};
    addr.smctp_family = AF_MCTP;
    addr.smctp_addr.s_addr = mctpEid; //PLDM_ENTITY_ID;
    addr.smctp_type = MCTP_MSG_TYPE_PLDM;
    addr.smctp_tag = MCTP_TAG_OWNER;
    addr.smctp_network = MCTP_NET_ANY;

    CustomFD socketFd(sockFd);

    int result =
        sendto(socketFd(), requestMsg.data(), requestMsg.size(), 0,
               reinterpret_cast<struct sockaddr*>(&addr), sizeof(addr));
    if (-1 == result)
    {
        returnCode = -errno;
        std::cerr << "Write to socket failure : RC = " << returnCode << "\n";
        return returnCode;
    }
    // Logger(pldmVerbose, "Write to socket successful : RC = ", result);
    auto hdr = reinterpret_cast<const pldm_msg_hdr*>(requestMsg.data());

    if (PLDM_SUCCESS != unpack_pldm_header(hdr, &requestHdrFields))
    {
        std::cerr << "Empty PLDM request header \n";
    }

    /* We need to make sure the buffer can hold the whole message or we could
     * lose bits */
    /* TODO? Add timeout here? */
    socklen_t addrlen = sizeof(struct sockaddr_mctp);
    ssize_t peekedLength =
        recvfrom(socketFd(), responseMsg.data(), responseMsg.size(),
                 MSG_PEEK | MSG_TRUNC,
                 reinterpret_cast<struct sockaddr*>(&addr), &addrlen);

    responseMsg.resize(peekedLength);
    do
    {
        auto recvDataLength =
            recv(socketFd(), reinterpret_cast<void*>(responseMsg.data()),
                 peekedLength, 0);
        hdr = reinterpret_cast<const pldm_msg_hdr*>(responseMsg.data());
        if (PLDM_SUCCESS != unpack_pldm_header(hdr, &responseHdrFields))
        {
            std::cerr << "Empty PLDM response header \n";
        }

        if (recvDataLength == peekedLength &&
            responseHdrFields.instance == requestHdrFields.instance &&
            responseHdrFields.msg_type != PLDM_REQUEST)
        {
            // Logger(pldmVerbose, "Total length:", recvDataLength);
            break;
        }
        else if (recvDataLength != peekedLength)
        {
            std::cerr << "Failure to read response length packet: length = "
                      << recvDataLength << "and result " << result << "\n";
            return returnCode;
        }
    } while (1);

    return PLDM_SUCCESS;
}



PLDMSensor::PLDMSensor(
    sdbusplus::asio::object_server& objectServer,
    std::shared_ptr<sdbusplus::asio::connection>& conn,
    const std::string& sensorName,
    std::vector<thresholds::Threshold>&& thresholdsIn,
    const std::string& sensorConfiguration,
    uint16_t eid, uint16_t snrId) :
    Sensor(escapeName(sensorName), std::move(thresholdsIn), sensorConfiguration,
           "PLDMSensor", false, false, maxReading, minReading, conn),
    objServer(objectServer), mctpEid(eid), sensorId(snrId)
{
    sensorInterface = objectServer.add_interface(
        "/xyz/openbmc_project/sensors/temperature/" + name,
        "xyz.openbmc_project.Sensor.Value");

    for (const auto& threshold : thresholds)
    {
        std::string interface = thresholds::getInterface(threshold.level);
        thresholdInterfaces[static_cast<size_t>(threshold.level)] =
            objectServer.add_interface(
                "/xyz/openbmc_project/sensors/temperature/" + name, interface);
    }
    association = objectServer.add_interface(
        "/xyz/openbmc_project/sensors/temperature/" + name,
        association::interface);

    setInitialProperties(sensor_paths::unitDegreesC);
}

PLDMSensor::~PLDMSensor()
{
    // close the input dev to cancel async operations
    for (const auto& iface : thresholdInterfaces)
    {
        objServer.remove_interface(iface);
    }
    objServer.remove_interface(sensorInterface);
    objServer.remove_interface(association);
}

void PLDMSensor::checkThresholds(void)
{
    thresholds::checkThresholds(this);
}

int PLDMSensor::getSensorReading(void)
{
    int rc;

    // require Instance Id
    uint8_t instanceId;
    static constexpr auto pldmBusName = "xyz.openbmc_project.PLDM";
    static constexpr auto pldmObjPath = "/xyz/openbmc_project/pldm";
    static constexpr auto pldmRequester = "xyz.openbmc_project.PLDM.Requester";
    try
    {
        auto bus = sdbusplus::bus::new_default();
        auto method = bus.new_method_call(pldmBusName, pldmObjPath, pldmRequester, "GetInstanceId");
        method.append(mctpEid);
        auto reply = bus.call(method, std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::seconds(5)).count());
        reply.read(instanceId);
    }
    catch (const std::exception& e)
    {
        std::cerr << "GetInstanceId D-Bus call failed, MCTP id = "
                  << (unsigned)mctpEid << ", error = " << e.what() << "\n";
        return PLDM_ERROR;
    }

    // prepare request message
    std::vector<uint8_t> requestMsg(sizeof(pldm_msg_hdr) + PLDM_GET_SENSOR_READING_REQ_BYTES);
    auto request = reinterpret_cast<pldm_msg*>(requestMsg.data());
    rc = encode_get_sensor_reading_req(instanceId, sensorId, false, request);
    if (rc != PLDM_SUCCESS)
    {
        std::cerr << "encode_get_sensor_reading_req failed: rc = " << rc << std::endl;
        return rc;
    }

    // send and recv message
    bool mctpVerbose = false;
    std::vector<uint8_t> responseMsg;
    rc = mctpSockSendRecv(mctpEid, requestMsg, responseMsg, mctpVerbose);
    if (rc != PLDM_SUCCESS)
    {
        std::cerr << "mctpSockSendRecv failed: rc = " << rc << std::endl;
        return rc;
    }

    // parsing response message
    uint8_t completionCode;
    uint8_t sensorDataSize;
    uint8_t sensorOperationalState;
    uint8_t sensorEventMessageEnable;
    uint8_t presentState;
    uint8_t previousState;
    uint8_t eventState;
    union_sensor_data_size presentReading;
    auto responsePtr = reinterpret_cast<struct pldm_msg*>(responseMsg.data());
    auto payloadLength = responseMsg.size() - sizeof(pldm_msg_hdr);
    rc = decode_get_sensor_reading_resp(
        responsePtr, payloadLength, &completionCode, &sensorDataSize,
        &sensorOperationalState, &sensorEventMessageEnable, &presentState,
        &previousState, &eventState, &(presentReading.value_u8));

    if (rc != PLDM_SUCCESS || completionCode != PLDM_SUCCESS)
    {
        std::cerr << "decode_get_sensor_reading_resp failed: "
                    << "rc=" << rc << ",cc=" << (int)completionCode
                    << std::endl;
        return rc;
    }

    switch (sensorDataSize)
    {
        case PLDM_SENSOR_DATA_SIZE_UINT8:
            updateValue(presentReading.value_u8);
            break;
        case PLDM_SENSOR_DATA_SIZE_SINT8:
            updateValue(presentReading.value_s8);
            break;
        case PLDM_SENSOR_DATA_SIZE_UINT16:
            updateValue(presentReading.value_u16);
            break;
        case PLDM_SENSOR_DATA_SIZE_SINT16:
            updateValue(presentReading.value_s16);
            break;
        case PLDM_SENSOR_DATA_SIZE_UINT32:
            updateValue(presentReading.value_u32);
            break;
        case PLDM_SENSOR_DATA_SIZE_SINT32:
            updateValue(presentReading.value_s32);
            break;
        default:
            updateValue(std::numeric_limits<double>::quiet_NaN());
            break;
    }

    return PLDM_SUCCESS;
}
