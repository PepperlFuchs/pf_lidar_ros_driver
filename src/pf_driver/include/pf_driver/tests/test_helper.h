#ifndef TESTHELPER
#define TESTHELPER

#include <fstream>
#include <iostream>
#include <vector>

#include <ament_index_cpp/get_package_share_directory.hpp>

inline std::vector<uint8_t> hex_to_bytes(const std::string& hex)
{
  std::vector<uint8_t> bytes;
  for (unsigned int i = 0; i < hex.length(); i += 2)
  {
    std::string byteString = hex.substr(i, 2);
    uint8_t byte = (uint8_t)strtol(byteString.c_str(), nullptr, 16);
    bytes.push_back(byte);
  }
  return bytes;
}

inline std::string get_dump_path()
{
  std::string dump_dir = ament_index_cpp::get_package_share_directory("pf_driver") + "/dumps/";
  return dump_dir;
}

inline std::vector<uint8_t> read_dump(std::string& FILENAME)
{
  std::string filepath = get_dump_path() + FILENAME;
  std::ifstream file(filepath, std::fstream::in);
  std::string line = "";
  std::vector<uint8_t> vec;
  if (file.is_open())
  {
    std::getline(file, line);
    std::cout << line << std::endl;
    vec = hex_to_bytes(line);

    file.close();
  }
  return vec;
}

#endif  // TESTHELPER
