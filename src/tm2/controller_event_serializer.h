// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2017 Intel Corporation. All Rights Reserved.
#pragma once

#include <string>
#include "TrackingManager.h"
#include "../../third-party/json.hpp"

namespace librealsense
{
    template<size_t SIZE>
    inline std::string buffer_to_string(const uint8_t(&buff)[SIZE], char separator = ',', bool as_hex = false)
    {
        std::ostringstream oss;
        if (as_hex)
            oss << std::hex;

        for (size_t i = 0; i < SIZE; i++)
        {
            if (i != 0)
                oss << separator;
            oss << (int)buff[i];
        }
        return oss.str();
    }

    std::string get_string(perc::Status value)
    {

#define CASE_RETURN_STR(X) case perc::Status::##X: {\
        static std::string s##X##_str = make_less_screamy(#X);\
        return s##X##_str; }

        switch (value)
        {
            CASE_RETURN_STR(SUCCESS)
            CASE_RETURN_STR(COMMON_ERROR)
            CASE_RETURN_STR(FEATURE_UNSUPPORTED)
            CASE_RETURN_STR(ERROR_PARAMETER_INVALID)
            CASE_RETURN_STR(INIT_FAILED)
            CASE_RETURN_STR(ALLOC_FAILED)
            CASE_RETURN_STR(ERROR_USB_TRANSFER)
            CASE_RETURN_STR(ERROR_EEPROM_VERIFY_FAIL)
            CASE_RETURN_STR(ERROR_FW_INTERNAL)
            CASE_RETURN_STR(BUFFER_TOO_SMALL)
            CASE_RETURN_STR(NOT_SUPPORTED_BY_FW)
            CASE_RETURN_STR(DEVICE_BUSY)
            CASE_RETURN_STR(TIMEOUT)
            CASE_RETURN_STR(TABLE_NOT_EXIST)
            CASE_RETURN_STR(TABLE_LOCKED)
            CASE_RETURN_STR(DEVICE_STOPPED)
            CASE_RETURN_STR(TEMPERATURE_WARNING)
            CASE_RETURN_STR(TEMPERATURE_STOP)
            CASE_RETURN_STR(CRC_ERROR)
            CASE_RETURN_STR(INCOMPATIBLE)
            CASE_RETURN_STR(SLAM_NO_DICTIONARY)
        default: return to_string() << "Unknown (" << (int)value << ")";
        }
#undef CASE_RETURN_STR
    }

    inline std::ostream& operator<<(std::ostream& os, const perc::TrackingData::Version& v)
    {
        return os << v.major << "." << v.minor << "." << v.patch << "." << v.build;
    }

    class controller_event_serializer 
    {
    public:
        static std::string serialized_data(const perc::TrackingData::ControllerDiscoveryEventFrame& frame)
        {
            nlohmann::json frame_obj;
            frame_obj["MAC"] = buffer_to_string(frame.macAddress);
            return to_json(event_type_discovery(), event_type_discovery());
        }

        static std::string serialized_data(const perc::TrackingData::ControllerDisconnectedEventFrame& frame)
        {
            nlohmann::json frame_obj;
            frame_obj["ID"] = (int)frame.controllerId;
            return to_json(frame_obj, event_type_disconnection());
        }

        static std::string serialized_data(const perc::TrackingData::ControllerFrame& frame)
        {
            nlohmann::json frame_obj;
            frame_obj["sensorIndex"] = (int)frame.sensorIndex;
            frame_obj["frameId"] = (int)frame.frameId;
            frame_obj["eventId"] = (int)frame.eventId;
            frame_obj["instanceId"] = (int)frame.instanceId;
            frame_obj["sensorData"] = buffer_to_string(frame.sensorData);
            return to_json(frame_obj, event_type_frame());
        }
        
        static std::string serialized_data(const perc::TrackingData::ControllerConnectedEventFrame& frame, const uint8_t(&mac_address)[6])
        {
            nlohmann::json frame_obj;
            frame_obj["status"] = get_string(frame.status);
            frame_obj["controllerId"] = (int)frame.controllerId;
            frame_obj["manufacturerId"] = (int)frame.manufacturerId;
            frame_obj["protocol"] = to_string() << frame.protocol;
            frame_obj["app"] = to_string() << frame.app;
            frame_obj["softDevice"] = to_string() << frame.softDevice;
            frame_obj["bootLoader"] = to_string() << frame.bootLoader;
            frame_obj["MAC"] = mac_address;
            return to_json(frame_obj, event_type_connection());
        }

    private:
        static constexpr const char* prefix() { return R"JSON({"Event Type":"Controller Event", "Data" : {)JSON"; }
        static constexpr const char* suffix() { return "}}"; }
        static constexpr const char* event_type_frame() { return "Frame"; }
        static constexpr const char* event_type_connection() { return "Connection"; }
        static constexpr const char* event_type_disconnection() { return "Disconnection"; }
        static constexpr const char* event_type_discovery() { return "Discovery"; }

        static std::string to_json(const nlohmann::json& frame_obj, const std::string& subtype)
        {
            nlohmann::json event_obj;
            event_obj["Event Type"] = "Controller Event";
            nlohmann::json subtype_obj;
            subtype_obj["Sub Type"] = subtype;
            subtype_obj["Data"] = frame_obj;
            event_obj["Data"] = subtype_obj;
            return event_obj.dump();
        }
    };
}