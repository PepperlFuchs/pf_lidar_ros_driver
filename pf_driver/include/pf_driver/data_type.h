#include <vector>
#include <cstdint>

constexpr std::uint32_t scan_datainfo = (1 << 0);
constexpr std::uint32_t new_settings = (1 << 1);
constexpr std::uint32_t invalid_data = (1 << 2);
constexpr std::uint32_t unstable_rotation = (1 << 3);
constexpr std::uint32_t skipped_packet = (1 << 4);

//warnings
constexpr std::uint32_t device_warning = (1 << 8);
constexpr std::uint32_t low_temperature_warning = (1 << 10);
constexpr std::uint32_t high_temperature_warning = (1 << 11);
constexpr std::uint32_t device_overload_warn = (1 << 12);

//errors
constexpr std::uint32_t device_error = (1 << 16);
constexpr std::uint32_t low_temperature_error = (1 << 18);
constexpr std::uint32_t high_temperature_error = (1 << 19);
constexpr std::uint32_t device_overload_error = (1 << 20);

//defects
std::uint32_t device_defect = (1 << 30);

struct PacketHeader {};

struct PacketType {};
