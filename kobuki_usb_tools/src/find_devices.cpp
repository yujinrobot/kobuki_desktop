#include <iostream>
#include <usb.h>

static struct usb_device *find_devices(uint16_t vendor, uint16_t product)
{
  struct usb_bus *bus;
  struct usb_device *dev;
  struct usb_bus *busses;

  usb_init();
  usb_find_busses();
  usb_find_devices();
  busses = usb_get_busses();

  for (bus = busses; bus; bus = bus->next)
    for (dev = bus->devices; dev; dev = dev->next)
      if ((dev->descriptor.idVendor == vendor) && (dev->descriptor.idProduct == product))
        return dev;

  return NULL;
}

int main(int argc, char** argv)
{
  std::cout << "hello world." << std::endl;
  if( find_devices(0x0403,0x6001) == NULL ) {
    std::cout << "failed to find." << std::endl;
  } else {
    std::cout << "found!!!" << std::endl;
  }
  return 0;
}
