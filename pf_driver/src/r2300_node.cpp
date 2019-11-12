#include "pf_driver/pf_interface.hpp"

int main(int argc, char *argv[])
{
  return PF_Interface<UDPConnection, PFSDPBase, PacketHeaderR2300>::main("r2300", argc, argv);
}
