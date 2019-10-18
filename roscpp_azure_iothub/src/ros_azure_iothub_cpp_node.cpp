// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#include <stdlib.h>
#include <inttypes.h>
#include <stdio.h>
#include <map>
#include <vector>

// Azure IoT Hub
#ifdef _WIN32
#include <azure_c_shared_utility/macro_utils.h>
#else
#include <azure_macro_utils/macro_utils.h>
#endif

#include <azure_c_shared_utility/threadapi.h>
#include <azure_c_shared_utility/platform.h>
#include <iothub_device_client.h>
#include <iothub_client_options.h>
#include <iothub.h>
#include <iothub_message.h>
#include <parson.h>
#include <iothubtransportmqtt.h>

#include "rclcpp/rclcpp.hpp"

int g_interval = 10000;  // 10 sec send interval initially, currently not used
static size_t g_message_count_send_confirmations = 0;

struct AzureIoTHubNode 
{
    IOTHUB_DEVICE_CLIENT_HANDLE deviceHandle;
    //Parser parser;
    std::shared_ptr<rclcpp::Node> nh;
    //std::vector<std::shared_ptr<rclcpp::Subscription> subscribers;
    std::vector<std::string> topicsToSubscribe;
};

static IOTHUBMESSAGE_DISPOSITION_RESULT receive_msg_callback(IOTHUB_MESSAGE_HANDLE message, void* user_context)
{
    AzureIoTHubNode* iotHub = (AzureIoTHubNode*)user_context;
    const char* messageId;
    const char* correlationId;
    // Message properties
    if ((messageId = IoTHubMessage_GetMessageId(message)) == NULL)
    {
        messageId = "<unavailable>";
    }

    if ((correlationId = IoTHubMessage_GetCorrelationId(message)) == NULL)
    {
        correlationId = "<unavailable>";
    }

    IOTHUBMESSAGE_CONTENT_TYPE content_type = IoTHubMessage_GetContentType(message);
    if (content_type == IOTHUBMESSAGE_BYTEARRAY)
    {
        const unsigned char* buff_msg;
        size_t buff_len;

        if (IoTHubMessage_GetByteArray(message, &buff_msg, &buff_len) != IOTHUB_MESSAGE_OK)
        {
            RCLCPP_ERROR(iotHub->nh->get_logger(), "Failure retrieving byte array message");
        }
        else
        {
            RCLCPP_INFO(iotHub->nh->get_logger(), "Received Binary message\r\n Message ID: %s\r\n Correlation ID: %s\r\n Data: <<<%.*s>>> & Size=%d", messageId, correlationId, (int)buff_len, buff_msg, (int)buff_len);
        }
    }
    else
    {
        const char* string_msg = IoTHubMessage_GetString(message);
        if (string_msg == NULL)
        {
            RCLCPP_ERROR(iotHub->nh->get_logger(), "Failure retrieving byte array message");
        }
        else
        {
            RCLCPP_INFO(iotHub->nh->get_logger(), "Received String Message\r\n Message ID: %s\r\n Correlation ID: %s\r\n Data: <<<%s>>>", messageId, correlationId, string_msg);
        }
    }
    return IOTHUBMESSAGE_ACCEPTED;
}

static int device_method_callback(const char* method_name, const unsigned char* payload, size_t size, unsigned char** response, size_t* resp_size, void* user_context)
{
    const char* SetTelemetryIntervalMethod = "SetTelemetryInterval";
    AzureIoTHubNode* iotHub = (AzureIoTHubNode*)user_context;
    char* end = NULL;
    int newInterval;

    int status = 501;
    const char* RESPONSE_STRING = "{ \"Response\": \"Unknown method requested.\" }";

    // odd - this should be null RCLCPP_INFO(iotHub->nh->get_logger(), "Device Method called for device %s", device_id);
    RCLCPP_INFO(iotHub->nh->get_logger(), "Device Method name:    %s", method_name);
    RCLCPP_INFO(iotHub->nh->get_logger(), "Device Method payload: %.*s", (int)size, (const char*)payload);

    if (strcmp(method_name, SetTelemetryIntervalMethod) == 0)
    {
        if (payload)
        {
            newInterval = (int)strtol((char*)payload, &end, 10);

            // Interval must be greater than zero.
            if (newInterval > 0)
            {
                // Expect sec and covert to ms
                g_interval = 1000 * (int)strtol((char*)payload, &end, 10);
                status = 200;
                RESPONSE_STRING = "{ \"Response\": \"Telemetry reporting interval updated.\" }";
            }
            else
            {
                status = 500;
                RESPONSE_STRING = "{ \"Response\": \"Invalid telemetry reporting interval.\" }";
            }
        }
    }

    RCLCPP_INFO(iotHub->nh->get_logger(), "Response status: %d", status);
    RCLCPP_INFO(iotHub->nh->get_logger(), "Response payload: %s", RESPONSE_STRING);

    *resp_size = strlen(RESPONSE_STRING);
    *response = reinterpret_cast<unsigned char*>(malloc(*resp_size));
    if (*response)
    {
        memcpy(*response, RESPONSE_STRING, *resp_size);
    }
    else
    {
        status = -1;
    }
    return status;
}

static void connection_status_callback(IOTHUB_CLIENT_CONNECTION_STATUS result, IOTHUB_CLIENT_CONNECTION_STATUS_REASON reason, void* user_context)
{
    (void)reason;
    AzureIoTHubNode* iotHub = (AzureIoTHubNode*)user_context;

    // This sample DOES NOT take into consideration network outages.
    if (result == IOTHUB_CLIENT_CONNECTION_AUTHENTICATED)
    {
        RCLCPP_INFO(iotHub->nh->get_logger(), "The device client is connected to iothub");
    }
    else
    {
        RCLCPP_INFO(iotHub->nh->get_logger(), "The device client has been disconnected");
    }
}

static void send_confirm_callback(IOTHUB_CLIENT_CONFIRMATION_RESULT result, void* user_context)
{
    AzureIoTHubNode* iotHub = (AzureIoTHubNode*)user_context;
    // When a message is sent this callback will get envoked
    g_message_count_send_confirmations++;
    #ifdef _WIN32
    RCLCPP_INFO(iotHub->nh->get_logger(), "Confirmation callback received for message %lu with result %s", (unsigned long)g_message_count_send_confirmations, ENUM_TO_STRING(IOTHUB_CLIENT_CONFIRMATION_RESULT, result));
    #else
    RCLCPP_INFO(iotHub->nh->get_logger(), "Confirmation callback received for message %lu with result %s", (unsigned long)g_message_count_send_confirmations, MU_ENUM_TO_STRING(IOTHUB_CLIENT_CONFIRMATION_RESULT, result));
    #endif
}

static bool IsTopicAvailableForSubscribe(AzureIoTHubNode* iotHub, const char* topicName)
{
    std::string topic_name = topicName;

    auto topicsAndTypes = iotHub->nh->get_topic_names_and_types();

    bool found = false;
    for (const auto& publishedTopic: topicsAndTypes)
    {
        if( publishedTopic.first == topic_name)
        {
            found = true;
            break;
        }
    }

    if (!found)
    {
        return false;
    }
    return true;
}


/*
void sendMsgToAzureIoTHub(const char* msg, IOTHUB_DEVICE_CLIENT_HANDLE deviceHandle,
                   AzureIoTHubNode* iotHub)
{
    static int messagecount;
    
    // Construct the iothub message from a string or a byte array
    IOTHUB_MESSAGE_HANDLE message_handle = IoTHubMessage_CreateFromString(msg);

    // Set Message property
    (void)IoTHubMessage_SetMessageId(message_handle, "MSG_ID");
    (void)IoTHubMessage_SetCorrelationId(message_handle, "CORE_ID");
    (void)IoTHubMessage_SetContentTypeSystemProperty(message_handle, "application%2fjson");
    (void)IoTHubMessage_SetContentEncodingSystemProperty(message_handle, "utf-8");

    RCLCPP_INFO(iotHub->nh->get_logger(), "Sending message %d to IoTHub\r\nMessage: %s", (int)(messagecount + 1), msg);
    IoTHubDeviceClient_SendEventAsync(deviceHandle, message_handle, send_confirm_callback, NULL);

    // The message is copied to the sdk so the we can destroy it
    IoTHubMessage_Destroy(message_handle);
    messagecount++;
}

void topicCallback(const topic_tools::ShapeShifter::ConstPtr& msg,
                   const std::string &topic_name,
                   RosIntrospection::Parser& parser,
                   IOTHUB_DEVICE_CLIENT_HANDLE deviceHandle,
                   AzureIoTHubNode* iotHub)
{
    const std::string&  datatype   =  msg->getDataType();
    const std::string&  definition =  msg->getMessageDefinition();

    parser.registerMessageDefinition( topic_name,
                                      RosIntrospection::ROSType(datatype),
                                      definition );

    static std::vector<uint8_t> buffer;
    static std::map<std::string,FlatMessage>   flat_containers;
    static std::map<std::string,RenamedValues> renamed_vectors;

    FlatMessage&   flat_container = flat_containers[topic_name];
    RenamedValues& renamed_values = renamed_vectors[topic_name];

    // Copy raw memory into the buffer
    buffer.resize( msg->size() );
    ros::serialization::OStream stream(buffer.data(), buffer.size());
    msg->write(stream);

    // Deserialize and rename the vectors
    parser.deserializeIntoFlatContainer( topic_name, absl::Span<uint8_t>(buffer), &flat_container, 100);
    parser.applyNameTransform( topic_name, flat_container, &renamed_values );

    RCLCPP_INFO(iotHub->nh->get_logger(), "--------- %s ----------", topic_name.c_str() );
    sendMsgToAzureIoTHub(topic_name.c_str(), deviceHandle, iotHub);
    for (auto it: renamed_values)
    {
        const std::string& key = it.first;
        const Variant& value   = it.second;
        char buffer [256] = {0};
        snprintf(buffer, sizeof(buffer), " %s = %f", key.c_str(), value.convert<double>());
        RCLCPP_INFO(iotHub->nh->get_logger(), "%s",buffer);
        sendMsgToAzureIoTHub(buffer, deviceHandle, iotHub);
    }
    for (auto it: flat_container.name)
    {
        const std::string& key    = it.first.toStdString();
        const std::string& value  = it.second;
        char buffer [256] = {0};
        snprintf(buffer, sizeof(buffer), " %s = %s", key.c_str(), value.c_str());
        RCLCPP_INFO(iotHub->nh->get_logger(), "%s",buffer);
        sendMsgToAzureIoTHub(buffer, deviceHandle, iotHub);
    }
}
*/

static void subscribeTopic(const char* topicName, AzureIoTHubNode* iotHub)
{
    /*
    boost::function<void(const topic_tools::ShapeShifter::ConstPtr&) > callback;
    callback = [iotHub, topicName](const topic_tools::ShapeShifter::ConstPtr& msg) -> void
    {
        topicCallback(msg, topicName, iotHub->parser, iotHub->deviceHandle, iothub) ;
    };
    iotHub->subscribers.push_back( iotHub->nh.subscribe(topicName, 10, callback) );
    */
}

// Current code does not keep track of topics already subscribed
static void deviceTwinCallback(DEVICE_TWIN_UPDATE_STATE update_state, const unsigned char* payLoad, size_t size, void* userContextCallback)
{
    (void)update_state;
    (void)size;

    AzureIoTHubNode* iotHub = (AzureIoTHubNode*)userContextCallback;

    JSON_Value* root_value = json_parse_string((const char*)payLoad);
    JSON_Object* root_object = json_value_get_object(root_value);
    JSON_Object* desired_object = json_object_dotget_object(root_object, "desired");
    JSON_Object* ros_object = (desired_object != NULL) ? desired_object : root_object;

    // Get topics for subscription
    JSON_Object* arrayObject = json_object_dotget_object(ros_object, "ros_relays");
    size_t objectCount = json_object_get_count(arrayObject);

    for (size_t i = 0; i < objectCount; i++)
    {
        RCLCPP_INFO(iotHub->nh->get_logger(), "  %s", json_object_get_name(arrayObject, i));
        JSON_Value* value = json_object_get_value_at(arrayObject, i);
        const char* topicToSubscribe = json_value_get_string(value);
        RCLCPP_INFO(iotHub->nh->get_logger(), "  %s", json_value_get_string(value));

        // Only subscribe the topic that is avaiable but not subscribed before
        if (topicToSubscribe != NULL && 
            IsTopicAvailableForSubscribe(iotHub, topicToSubscribe) && 
            std::find(iotHub->topicsToSubscribe.begin(), iotHub->topicsToSubscribe.end(), topicToSubscribe) == iotHub->topicsToSubscribe.end())
        {
            RCLCPP_INFO(iotHub->nh->get_logger(), "Subscribe topic:  %s", topicToSubscribe);
            iotHub->topicsToSubscribe.push_back(topicToSubscribe);
            subscribeTopic(topicToSubscribe, iotHub);
        }
    }

    // Get dynamic reconfiguration settings
    arrayObject = json_object_dotget_object(ros_object, "ros_dynamic_configurations");
    objectCount = json_object_get_count(arrayObject);

    std::map<std::string, std::vector<rclcpp::Parameter>> NodeToParameterMap;

    for (size_t i = 0; i < objectCount; i++)
        {
        JSON_Value* congifure_value = json_object_get_value_at(arrayObject, i);
        JSON_Object* congifure_object = json_value_get_object(congifure_value);

        const char *node = NULL, *param = NULL, *type = NULL, *value = NULL;
        JSON_Value* node_value = json_object_get_value(congifure_object, "node");
        if (node_value != NULL)
        {
            node = json_value_get_string(node_value);
        }

        JSON_Value* param_value = json_object_get_value(congifure_object, "param");
        if (param_value != NULL)
        {
            param = json_value_get_string(param_value);
        }

        JSON_Value* type_value = json_object_get_value(congifure_object, "type");
        if (type_value != NULL)
        {
            type = json_value_get_string(type_value);
        }

        JSON_Value* val_value = json_object_get_value(congifure_object, "value");
        if (val_value != NULL)
        {
            value = json_value_get_string(val_value);
        }

        if (node != NULL && param != NULL && type != NULL && value != NULL)
        {
            RCLCPP_INFO(iotHub->nh->get_logger(), "Trying to send dynamic configuration command - node:%s, parameter:%s, data type:%s, value:%s", node, param, type, value);

            if (strcmp(type, "string") == 0)
            {
                rclcpp::Parameter param(param, value);
                NodeToParameterMap[node].push_back(param);

            }
            else if (strcmp(type, "int") == 0)
            {
                char* end = NULL;
                rclcpp::Parameter param(param, (int)strtol(value, &end, 10));
                NodeToParameterMap[node].push_back(param);
            }
            else if (strcmp(type, "double") == 0)
            {
                char* end = NULL;
                rclcpp::Parameter param(param, (double)strtod(value, &end));
                NodeToParameterMap[node].push_back(param);
            }
            else if (strcmp(type, "bool") == 0)
            {
                /*
                try
                {
                    rclcpp::Parameter param(param, boost::lexical_cast<bool>(value));
                    NodeToParameterMap[node].push_back(param);
                }
                catch (const boost::bad_lexical_cast &e)
                {
                    (void)e;
                    RCLCPP_ERROR(iotHub->nh->get_logger(), "Failure converting %s to bool type", value);
                }
                */
            }
        }

        for (const auto& nodeParam : NodeToParameterMap) 
        {
            rclcpp::SyncParametersClient::SharedPtr parameters_client = std::make_shared<rclcpp::SyncParametersClient>(iotHub->nh, nodeParam.first);
            auto set_parameters_results = parameters_client->set_parameters(nodeParam.second);

            for (auto & result : set_parameters_results) 
            {
                if (!result.successful) 
                {
                    //RCLCPP_ERROR(node->get_logger(), "Failed to set parameter: %s", result.reason.c_str());
                }
            }
        }
    }

    // Free resources
    json_value_free(root_value);
}

static bool InitializeAzureIoTHub(AzureIoTHubNode* iotHub)
{
    IOTHUB_CLIENT_TRANSPORT_PROVIDER protocol = MQTT_Protocol;

    (void)IoTHub_Init();

    iotHub->nh = rclcpp::Node::make_shared("azure_iot_hub");
    if (iotHub->nh == nullptr)
    {
        return false;
    }

    std::string connectionString;
    auto connectionStringParameter = iotHub->nh->declare_parameter("connection_string");
    if (connectionStringParameter.get_type() == rclcpp::PARAMETER_STRING)
    {
        connectionString = connectionStringParameter.get<std::string>(); 
    }
    else
    {
        RCLCPP_ERROR(iotHub->nh->get_logger(), "Connection_string is currently required");
        return false;
    }

    RCLCPP_INFO(iotHub->nh->get_logger(), "Creating IoTHub Device handle");

    RCLCPP_INFO(iotHub->nh->get_logger(), "connection_string: %s", connectionString.c_str());

    // Create the iothub handle here
    iotHub->deviceHandle = IoTHubDeviceClient_CreateFromConnectionString(connectionString.c_str(), protocol);
    if (iotHub->deviceHandle == NULL)
    {
        RCLCPP_ERROR(iotHub->nh->get_logger(), "Failure createing Iothub device.  Hint: Check you connection string.");
        return false;
    }
    // Setting message callback to get C2D messages
    (void)IoTHubDeviceClient_SetMessageCallback(iotHub->deviceHandle, receive_msg_callback, iotHub);
    // Setting method callback to handle a SetTelemetryInterval method to control
    //   how often telemetry messages are sent from the simulated device.
    (void)IoTHubDeviceClient_SetDeviceMethodCallback(iotHub->deviceHandle, device_method_callback, iotHub);
    // Setting connection status callback to get indication of connection to iothub
    (void)IoTHubDeviceClient_SetConnectionStatusCallback(iotHub->deviceHandle, connection_status_callback, iotHub);
    (void)IoTHubDeviceClient_SetDeviceTwinCallback(iotHub->deviceHandle, deviceTwinCallback, iotHub);  //Device Twin callback requires context to send message and device Handle

    return true;
}

static void DeinitializeAzureIoTHub(AzureIoTHubNode* iotHub)
{
    RCLCPP_INFO(iotHub->nh->get_logger(), "Deinitializing IoTHub Device client");
    IoTHubDeviceClient_Destroy(iotHub->deviceHandle);
    IoTHub_Deinit();
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    AzureIoTHubNode iotHub;

    if (!InitializeAzureIoTHub(&iotHub))
    {
        return -1;
    }

    rclcpp::spin(iotHub.nh);

    DeinitializeAzureIoTHub(&iotHub);
    rclcpp::shutdown();
    return 0;
}
