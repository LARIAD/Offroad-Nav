#include "binary_telemetry_client.h"

#include <iostream>
#include <asio.hpp>
#include <chrono>
#include <thread>
#include <signal.h>
#include <iomanip>
#include <atomic>
#include <ctime>
#include <sstream>
#include <map>
#include <string>
#include <optional>
#include <cstring>
#include <utility>
#include <nlohmann/json.hpp>
#include "httplib.h"

#include <ros/console.h>

// ============================================================================
// Binary Protocol Decoder (client-side only)
// 
// Protocol format (defined by server):
// - Header: MAGIC(4) + TIMESTAMP(4) + FIELD_COUNT(1) = 9 bytes
// - Per field: FIELD_ID(1) + VALUE(4 float) = 5 bytes
// ============================================================================
namespace binary_protocol
{
  /**
   * @brief Deserialize binary packet to field ID -> value map
   *
   * Field IDs are raw integers - their semantic meaning comes from
   * the schema fetched from the server at registration time.
   *
   * @param packet Binary packet received via UDP
   * @param data Output map of field IDs (raw uint8_t) to float values
   * @return true if deserialization successful, false otherwise
   */
  bool deserialize(const std::vector<uint8_t>& packet,
                   std::map<uint8_t, float>& data)
  {
    if (packet.size() < 9)
    {
      return false;
    }

    size_t offset = 0;

    // Check magic number (big-endian)
    uint32_t magic = (packet[0] << 24) | (packet[1] << 16) | (packet[2] << 8) | packet[3];
    if (magic != MAGIC)
    {
      return false;
    }
    offset += 4;

    // Skip timestamp (extracted separately before calling deserialize)
    offset += 4;

    // Get field count
    uint8_t field_count = packet[offset++];

    // Validate packet size
    if (packet.size() != static_cast<size_t>(9 + field_count * 5))
    {
      return false;
    }

    data.clear();

    // Parse fields - field IDs are raw bytes, meaning comes from schema
    for (uint8_t i = 0; i < field_count; ++i)
    {
      if (offset + 5 > packet.size())
      {
        return false;
      }

      uint8_t field_id = packet[offset++];

      // Extract float value (little-endian)
      float value;
      std::memcpy(&value, packet.data() + offset, 4);
      data[field_id] = value;
      offset += 4;
    }

    return true;
  }
} // namespace binary_protocol

// ============================================================================
// Telemetry Client Implementation
// ============================================================================

BinaryTelemetryClient::BinaryTelemetryClient(std::string host, uint16_t port,
                                             const std::function<void(const std::map<std::string, float>&,
                                                                      uint32_t)> callback,
                                             const bool shouldDisplay,
                                             std::string server_url)
  : io_context_(), socket_(io_context_), host_(std::move(host)), port_(port), server_url_(std::move(server_url)),
    running_(false), packet_count_(0), local_ip_(host_), callback_(callback), shouldDisplay_(shouldDisplay)
{
}

BinaryTelemetryClient::~BinaryTelemetryClient()
{
  stop();
}

bool BinaryTelemetryClient::register_with_server()
{
  try
  {
    // Get local IP address
    // Connect to a dummy address to determine local IP
    asio::ip::udp::socket temp_socket(io_context_);
    temp_socket.open(asio::ip::udp::v4());
    try
    {
      // Parse server URL (extract host and port)
      std::string server_host;
      int server_port = 8080;

      // Remove http:// or https:// prefix
      std::string url = server_url_;
      if (url.find("http://") == 0)
      {
        url = url.substr(7);
      }
      else if (url.find("https://") == 0)
      {
        url = url.substr(8);
      }

      // Extract host and port
      size_t colon_pos = url.find(':');
      if (colon_pos != std::string::npos)
      {
        server_host = url.substr(0, colon_pos);
        std::string port_str = url.substr(colon_pos + 1);
        if (!port_str.empty())
        {
          server_port = std::stoi(port_str);
        }
      }
      else
      {
        size_t slash_pos = url.find('/');
        if (slash_pos != std::string::npos)
        {
          server_host = url.substr(0, slash_pos);
        }
        else
        {
          server_host = url;
        }
      }

      // Create HTTP client
      httplib::Client cli(server_host, server_port);
      cli.set_connection_timeout(5, 0); // 5 second timeout
      cli.set_read_timeout(5, 0);

      // Register with server
      nlohmann::json payload;
      payload["client_ip"] = local_ip_;
      payload["udp_port"] = port_;

      ROS_DEBUG_STREAM_NAMED("BinaryTelemetryClient",
                             "Registering with server: " << server_url_ << "/telemetry/register");
      auto response = cli.Post("/telemetry/register", payload.dump(), "application/json");

      if (!response)
      {
        ROS_ERROR_STREAM_NAMED("BinaryTelemetryClient", "Failed to connect to server");
        return false;
      }

      if (response->status != 200)
      {
        ROS_ERROR_STREAM_NAMED("BinaryTelemetryClient", "Registration failed with status " << response->status);
        if (!response->body.empty())
        {
          ROS_ERROR_STREAM_NAMED("BinaryTelemetryClient", "Response: " << response->body);
        }
        return false;
      }

      nlohmann::json registration_data = nlohmann::json::parse(response->body);

      // Extract schema from registration response
      if (registration_data.contains("binary_frame_schema"))
      {
        nlohmann::json schema = registration_data["binary_frame_schema"];
        parse_schema(schema);
        ROS_DEBUG_STREAM_NAMED("BinaryTelemetryClient",
                               "Registration successful! Loaded schema with " << binary_id_to_semantic_id_.size() <<
                               " fields");
        return true;
      }
      else
      {
        // Fallback: fetch schema separately
        auto schema_response = cli.Get("/telemetry/schema");
        if (schema_response && schema_response->status == 200)
        {
          nlohmann::json schema = nlohmann::json::parse(schema_response->body);
          parse_schema(schema);
          ROS_DEBUG_STREAM_NAMED("BinaryTelemetryClient",
                                 "Registration successful! Fetched schema separately with "
                                 << binary_id_to_semantic_id_.size() << " fields");
          return true;
        }
        else
        {
          ROS_ERROR_STREAM_NAMED("BinaryTelemetryClient", "Failed to fetch schema");
          return false;
        }
      }
    }
    catch (const std::exception& e)
    {
      ROS_ERROR_STREAM_NAMED("BinaryTelemetryClient", "Error determining local IP: " << e.what());
      return false;
    }
  }
  catch (const std::exception& e)
  {
    ROS_ERROR_STREAM_NAMED("BinaryTelemetryClient", "Error registering with server: " << e.what());
    ROS_DEBUG_STREAM_NAMED("BinaryTelemetryClient",
                           "Continuing without schema (will use binary IDs as keys)" << e.what());
    return false;
  }
}

void BinaryTelemetryClient::parse_schema(const nlohmann::json& schema)
{
  if (!schema.contains("fields") || !schema["fields"].is_array())
  {
    ROS_WARN_STREAM_NAMED("BinaryTelemetryClient", "Schema missing 'fields' array");
    return;
  }

  binary_id_to_semantic_id_.clear();
  field_metadata_.clear();

  for (const auto& field : schema["fields"])
  {
    if (field.contains("binary_id") && field.contains("semantic_id"))
    {
      int binary_id = field["binary_id"].get<int>();
      std::string semantic_id = field["semantic_id"].get<std::string>();

      binary_id_to_semantic_id_[binary_id] = semantic_id;

      // Store metadata using semantic_id as key
      FieldMetadata metadata;
      metadata.name = field.contains("name") ? field["name"].get<std::string>() : "";
      metadata.unit = field.contains("unit") ? field["unit"].get<std::string>() : "";
      metadata.display_name = field.contains("display_name") ? field["display_name"].get<std::string>() : semantic_id;

      field_metadata_[semantic_id] = metadata;
    }
  }
}

void BinaryTelemetryClient::start()
{
  try
  {
    // Register with server and fetch schema
    if (!register_with_server())
    {
      ROS_WARN_STREAM_NAMED("BinaryTelemetryClient", "Continuing without schema");
    }

    // Bind to local address
    asio::ip::udp::endpoint local_endpoint(asio::ip::address::from_string(host_), port_);
    socket_.open(asio::ip::udp::v4());
    socket_.bind(local_endpoint);

    running_ = true;
    start_time_ = std::chrono::steady_clock::now();

    ROS_DEBUG_STREAM_NAMED("BinaryTelemetryClient", "Binary Telemetry Client - Shark Robotics Edition");
    ROS_INFO_STREAM_NAMED("BinaryTelemetryClient", "Listening on " << host_.c_str() << ":" << port_);
    ROS_DEBUG_STREAM_NAMED("BinaryTelemetryClient", "Protocol: Compact Binary (TELM)");
    ROS_DEBUG_STREAM_NAMED("BinaryTelemetryClient", "Expected rate: 20Hz");
    ROS_INFO_STREAM_NAMED("BinaryTelemetryClient", "Schema loaded: " << binary_id_to_semantic_id_.size() << " fields");

    // Start receiving
    start_receive();

    // Run I/O context
    io_context_.run();
  }
  catch (const std::exception& e)
  {
    ROS_FATAL_STREAM_NAMED("BinaryTelemetryClient", "Error starting client: " << e.what());
    exit(0);
  }
}

bool BinaryTelemetryClient::unregister_with_server()
{
  if (local_ip_.empty())
  {
    return false; // Never registered, nothing to unregister
  }

  try
  {
    // Parse server URL (extract host and port)
    std::string server_host;
    int server_port = 8080;

    // Remove http:// or https:// prefix
    std::string url = server_url_;
    if (url.find("http://") == 0)
    {
      url = url.substr(7);
    }
    else if (url.find("https://") == 0)
    {
      url = url.substr(8);
    }

    // Extract host and port
    size_t colon_pos = url.find(':');
    if (colon_pos != std::string::npos)
    {
      server_host = url.substr(0, colon_pos);
      std::string port_str = url.substr(colon_pos + 1);
      size_t slash_pos = port_str.find('/');
      if (slash_pos != std::string::npos)
      {
        port_str = port_str.substr(0, slash_pos);
      }
      if (!port_str.empty())
      {
        server_port = std::stoi(port_str);
      }
    }
    else
    {
      size_t slash_pos = url.find('/');
      if (slash_pos != std::string::npos)
      {
        server_host = url.substr(0, slash_pos);
      }
      else
      {
        server_host = url;
      }
    }

    // Create HTTP client
    httplib::Client cli(server_host, server_port);
    cli.set_connection_timeout(5, 0); // 5 second timeout
    cli.set_read_timeout(5, 0);

    // Unregister from server
    nlohmann::json payload;
    payload["client_ip"] = local_ip_;
    payload["udp_port"] = port_;

    auto response = cli.Delete("/telemetry/unregister", payload.dump(), "application/json");

    if (response && response->status == 200)
    {
      ROS_INFO_STREAM_NAMED("BinaryTelemetryClient", "Unregistered from server successfully");
      return true;
    }
    else
    {
      if (response)
      {
        ROS_WARN_STREAM_NAMED("BinaryTelemetryClient", "Unregistration returned status " << response->status);
      }
      else
      {
        ROS_WARN_STREAM_NAMED("BinaryTelemetryClient", "Failed to connect to server for unregistration");
      }
      return false;
    }
  }
  catch (const std::exception& e)
  {
    ROS_ERROR_STREAM_NAMED("BinaryTelemetryClient", "Failed to unregister from server: " << e.what());
    return false;
  }
}

void BinaryTelemetryClient::stop()
{
  running_ = false;

  // Unregister from server before closing socket
  if (!local_ip_.empty())
  {
    unregister_with_server();
  }

  if (socket_.is_open())
  {
    socket_.close();
  }
  io_context_.stop();

  if (start_time_ != std::chrono::steady_clock::time_point{})
  {
    auto duration = std::chrono::steady_clock::now() - start_time_;
    auto seconds = std::chrono::duration_cast<std::chrono::seconds>(duration).count();
    if (seconds > 0)
    {
      double rate = packet_count_ / static_cast<double>(seconds);

      ROS_DEBUG_STREAM_NAMED("BinaryTelemetryClient", "Session Summary:");
      ROS_DEBUG_STREAM_NAMED("BinaryTelemetryClient", "  Duration: " << seconds << " seconds");
      ROS_DEBUG_STREAM_NAMED("BinaryTelemetryClient", "  Packets received: " << packet_count_);
      ROS_DEBUG_STREAM_NAMED("BinaryTelemetryClient",
                             "  Average rate: " << std::fixed << std::setprecision(1) << rate << " Hz");

      if (rate >= 19.0)
      {
        ROS_DEBUG_STREAM_NAMED("BinaryTelemetryClient", "  Excellent! Your system is performing at its best!");
      }
      else if (rate >= 15.0)
      {
        ROS_INFO_STREAM_NAMED("BinaryTelemetryClient", " Good! Your telemetry is working well! Got "
                              << std::fixed << std::setprecision(1) << rate << "Hz");
      }
      else
      {
        ROS_WARN_STREAM_NAMED("BinaryTelemetryClient", " Low rate detected! Expected ~20Hz, got "
                              << std::fixed << std::setprecision(1) << rate << "Hz");
      }
    }
  }
}

void BinaryTelemetryClient::start_receive()
{
  socket_.async_receive_from(
    asio::buffer(recv_buffer_), remote_endpoint_,
    [this](const asio::error_code& error, std::size_t bytes_received)
    {
      handle_receive(error, bytes_received);
    }
  );
}

void BinaryTelemetryClient::handle_receive(const asio::error_code& error, std::size_t bytes_received)
{
  if (!error && running_)
  {
    // Process received packet
    std::vector<uint8_t> packet(recv_buffer_.data(), recv_buffer_.data() + bytes_received);
    process_packet(packet, remote_endpoint_);

    // Continue receiving
    start_receive();
  }
  else if (error != asio::error::operation_aborted)
  {
    ROS_ERROR_STREAM_NAMED("BinaryTelemetryClient", "Receive error: " << error.message());
  }
}

void BinaryTelemetryClient::process_packet(const std::vector<uint8_t>& packet, const asio::ip::udp::endpoint& endpoint)
{
  try
  {
    // Extract timestamp from packet header before deserialization
    // Timestamp is at bytes 4-7 (big-endian uint32_t, milliseconds since epoch)
    uint32_t packet_timestamp = 0;
    if (packet.size() >= 8)
    {
      packet_timestamp = (static_cast<uint32_t>(packet[4]) << 24) |
        (static_cast<uint32_t>(packet[5]) << 16) |
        (static_cast<uint32_t>(packet[6]) << 8) |
        static_cast<uint32_t>(packet[7]);
    }

    // Deserialize using raw field IDs (no static enum - fully dynamic)
    std::map<uint8_t, float> telemetry_raw;

    if (binary_protocol::deserialize(packet, telemetry_raw))
    {
      // Map raw field IDs to semantic IDs using schema from server
      std::map<std::string, float> telemetry;
      for (const auto& [field_id, value] : telemetry_raw)
      {
        // Map binary field ID to semantic_id from dynamic schema
        auto it = binary_id_to_semantic_id_.find(static_cast<int>(field_id));
        if (it != binary_id_to_semantic_id_.end())
        {
          telemetry[it->second] = value; // Use semantic_id as key
        }
        else
        {
          // Fallback: use binary ID as key if not in schema
          telemetry["field_" + std::to_string(field_id)] = value;
        }
      }

      packet_count_++;
      callback_(telemetry, packet_timestamp);
      if (shouldDisplay_)
      {
        display_telemetry(telemetry, endpoint, packet_timestamp);
      }
    }
    else
    {
      ROS_ERROR_STREAM_NAMED("BinaryTelemetryClient", "Failed to deserialize packet from " << endpoint);
    }
  }
  catch (const std::exception& e)
  {
    ROS_ERROR_STREAM_NAMED("BinaryTelemetryClient", "Error processing packet from " << endpoint << ": " << e.what());
  }
}

void BinaryTelemetryClient::display_telemetry(const std::map<std::string, float>& telemetry,
                                              const asio::ip::udp::endpoint& endpoint, uint32_t packet_timestamp)
{
  // Calculate rate
  double rate = 0.0;
  if (start_time_ != std::chrono::steady_clock::time_point{})
  {
    auto elapsed = std::chrono::steady_clock::now() - start_time_;
    auto seconds = std::chrono::duration_cast<std::chrono::milliseconds>(elapsed).count() / 1000.0;
    if (seconds > 0)
    {
      rate = packet_count_ / seconds;
    }
  }

  ROS_INFO_STREAM_NAMED("BinaryTelemetryClient", "Packet #" << packet_count_ << " from " << endpoint
                        << " (rate: " << std::fixed << std::setprecision(1) << rate << " Hz)");

  // Display packet timestamp (convert from milliseconds since epoch)
  if (packet_timestamp > 0)
  {
    auto packet_time_point = std::chrono::system_clock::time_point(
      std::chrono::milliseconds(packet_timestamp));
    auto packet_time_t = std::chrono::system_clock::to_time_t(packet_time_point);
    auto packet_ms = packet_timestamp % 1000;

    ROS_INFO_STREAM_NAMED("BinaryTelemetryClient", "  Packet timestamp "
                          << std::put_time(std::localtime(&packet_time_t), "%H:%M:%S")
                          << "." << std::setfill('0') << std::setw(3) << packet_ms
                          << " (" << packet_timestamp << " ms)");
  }

  // Display telemetry using semantic IDs and metadata from server schema
  for (const auto& [semantic_id, value] : telemetry)
  {
    auto it = field_metadata_.find(semantic_id);
    std::string display_name = semantic_id;
    std::string unit = "";

    if (it != field_metadata_.end())
    {
      display_name = it->second.display_name;
      unit = it->second.unit;
    }

    // Format value with unit
    if (!unit.empty())
    {
      ROS_INFO_STREAM_NAMED("BinaryTelemetryClient", "  " << display_name << ": "
                            << std::fixed << std::setprecision(2) << value << " " << unit);
    }
    else
    {
      ROS_INFO_STREAM_NAMED("BinaryTelemetryClient", "  " << display_name << ": "
                            << std::fixed << std::setprecision(2) << value);
    }
  }

  // Also output as JSON map for programmatic use
  if (!telemetry.empty())
  {
    nlohmann::json json_output;

    // Add all telemetry fields using semantic IDs as keys
    for (const auto& [semantic_id, value] : telemetry)
    {
      json_output[semantic_id] = value;
    }

    // Add timestamp fields
    json_output["timestamp"] = packet_timestamp; // Packet timestamp in milliseconds since epoch
    if (packet_timestamp > 0)
    {
      auto packet_time_point = std::chrono::system_clock::time_point(
        std::chrono::milliseconds(packet_timestamp));
      auto packet_time_t = std::chrono::system_clock::to_time_t(packet_time_point);
      auto packet_ms = packet_timestamp % 1000;

      std::ostringstream timestamp_str;
      timestamp_str << std::put_time(std::localtime(&packet_time_t), "%H:%M:%S");
      timestamp_str << "." << std::setfill('0') << std::setw(3) << packet_ms;
      json_output["timestamp_readable"] = timestamp_str.str();
    }
    ROS_INFO_STREAM_NAMED("BinaryTelemetryClient", "  JSON: " << json_output.dump(2));
  }
}
