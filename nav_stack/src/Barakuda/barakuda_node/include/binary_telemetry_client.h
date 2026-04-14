/*
 * Binary telemetry UDP client for Shark Robotics robots.
 * 
 * Built with love for the Barakuda and other wonderful Shark machines.
 * 
 * This is a STANDALONE client - no internal headers required.
 * Uses FULLY DYNAMIC schema decoding - fetches binary frame schema from server on registration.
 * No hardcoded field IDs - all field mappings come from the server's schema.
 * 
 * Dependencies (all publicly available):
 *   - asio (standalone, non-Boost): https://think-async.com/Asio/
 *   - nlohmann/json: https://github.com/nlohmann/json
 *   - cpp-httplib: https://github.com/yhirose/cpp-httplib
 * 
 * Compilation:
 *   g++ -std=c++17 -O2 -o binary_telemetry_client binary_telemetry_client.cpp \
 *       -I/path/to/asio/include -I/path/to/json/include -I/path/to/httplib \
 *       -DASIO_STANDALONE -lpthread
 * 
 * Usage:
 *   ./binary_telemetry_client --server http://robot-ip:8080 --port 12345
 */

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
#include <nlohmann/json.hpp>
#include "httplib.h"

// ============================================================================
// Binary Protocol Decoder (client-side only)
// 
// Protocol format (defined by server):
// - Header: MAGIC(4) + TIMESTAMP(4) + FIELD_COUNT(1) = 9 bytes
// - Per field: FIELD_ID(1) + VALUE(4 float) = 5 bytes
// ============================================================================
namespace binary_protocol
{
  // Protocol constant - magic number to identify valid packets
  static constexpr uint32_t MAGIC = 0x54454C4D; // "TELM"

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
  bool deserialize(const std::vector<uint8_t>& packet, std::map<uint8_t, float>& data);
} // namespace binary_protocol

// ============================================================================
// Telemetry Client Implementation
// ============================================================================

struct FieldMetadata
{
  std::string name;
  std::string unit;
  std::string display_name;
};

class BinaryTelemetryClient
{
public:
  /**
   * Create a client to the shark telemetry server
   *
   * @param host The ip of the host where the code runs
   * @param port The port on which the telemetry data is received. Can be customized (but should not conflict with previous ports).
   * @param callback Function is called when a packet is received and deserialized with the deserialized fields.
   * @param shouldDisplay If the telemetry should be printed in ROS messages
   * @param server_url The url where the client can register/unregister
   */
  BinaryTelemetryClient(std::string host, uint16_t port,
                        std::function<void(const std::map<std::string, float>&, uint32_t)> callback,
                        bool shouldDisplay = false,
                        std::string server_url = "http://localhost:8080");

  ~BinaryTelemetryClient();

  bool register_with_server();
  void parse_schema(const nlohmann::json& schema);
  void start();
  bool unregister_with_server();

  void stop();

private:
  void start_receive();

  void handle_receive(const asio::error_code& error, std::size_t bytes_received);

  void process_packet(const std::vector<uint8_t>& packet, const asio::ip::udp::endpoint& endpoint);

  void display_telemetry(const std::map<std::string, float>& telemetry,
                         const asio::ip::udp::endpoint& endpoint, uint32_t packet_timestamp);

  asio::io_context io_context_;
  asio::ip::udp::socket socket_;
  asio::ip::udp::endpoint remote_endpoint_;
  std::array<uint8_t, 1024> recv_buffer_;

  std::string host_;
  uint16_t port_;
  std::string server_url_;
  const std::function<void(const std::map<std::string, float>&, uint32_t)> callback_;
  const bool shouldDisplay_;
  bool running_;
  int packet_count_;
  std::chrono::steady_clock::time_point start_time_;
  std::string local_ip_; // Store local IP for unregistration

  // Schema data - ALL from server, NO hardcoded field IDs
  std::map<int, std::string> binary_id_to_semantic_id_; // binary_id -> semantic_id (from server)
  std::map<std::string, FieldMetadata> field_metadata_; // semantic_id -> metadata (from server)
};
