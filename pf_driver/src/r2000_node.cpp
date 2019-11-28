#include "pf_driver/pf_interface.hpp"

int main(int argc, char *argv[])
{
  return PF_Interface<TCPConnection, PFSDPBase, PacketHeaderR2000>::main("r2000", argc, argv);
}
