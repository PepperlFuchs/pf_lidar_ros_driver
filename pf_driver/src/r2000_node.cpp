#include "pf_driver/pf_interface.hpp"

int main(int argc, char *argv[])
{
  return PF_Interface<TCPConnection, PFSDP_2000, PacketHeaderR2000>::main("r2000", argc, argv);
}
