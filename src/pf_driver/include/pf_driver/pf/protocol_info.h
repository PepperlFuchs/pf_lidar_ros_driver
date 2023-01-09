#pragma once

#include <string>
#include <vector>

struct ProtocolInfo
{
  bool isError = false;
  std::string protocol_name;          // protocol name, defaults to "pfsdp"
  int version_major;                  // major version of protocol
  int version_minor;                  // minor version of protocol
  std::vector<std::string> commands;  // list of available commands
                                      // Since R2300 may not give correct error reports
                                      // it is safer to keep the list of commands
  uint16_t device_family;
};
