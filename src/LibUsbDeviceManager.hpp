#pragma once

#ifdef __GNUC__
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpedantic"
#endif

#include <libusb.h>

#ifdef __GNUC__
#pragma GCC diagnostic pop
#endif

#include <functional>
#include <map>
#include <memory>
#include <mutex>
#include <thread>
#include <vector>

template<typename Device>
struct LibUsbDeviceManager {
	private:
		libusb_context*                                   usb_context;
		std::map<libusb_device*, std::shared_ptr<Device>> devices;

		std::mutex   mutex;
		std::jthread libUsbThread;
		std::jthread deviceThread;

		std::shared_ptr<Device> registerDevice(libusb_device* device) {
			std::lock_guard<std::mutex> lock{mutex};
			try {
				auto dev = std::make_shared<Device>(device);
				devices.emplace(device, dev);
				return dev;
			} catch(std::exception const& e) {
				/* TODO(rkta): handle exception */
			}
			return nullptr;
		}

		void deregisterDevice(libusb_device* device) {
			std::lock_guard<std::mutex> lock{mutex};
			devices.erase(device);
		}

		void runLibUsb(std::stop_token stoken) {
			while(!stoken.stop_requested()) {
				timeval tv{};
				tv.tv_sec  = 0;
				tv.tv_usec = 1000 * 10;
				libusb_handle_events_timeout_completed(usb_context, std::addressof(tv), nullptr);
			}
		}
		void runDevice(std::stop_token stoken) {
			while(!stoken.stop_requested()) {
				{
					std::lock_guard<std::mutex> lock{mutex};
					for(auto& dev : devices) {
						dev.second->periodicTask();
					}
				}
				std::this_thread::sleep_for(std::chrono::milliseconds{10});
			}
		}

	public:
		LibUsbDeviceManager() {
			if(0 != libusb_init(std::addressof(usb_context))) {
				throw std::runtime_error{"failed to init libusb"};
			}

			libusb_set_option(usb_context, LIBUSB_OPTION_LOG_LEVEL, LIBUSB_LOG_LEVEL_INFO);

			libUsbThread = std::jthread{std::bind_front(&LibUsbDeviceManager::runLibUsb, this)};
			deviceThread = std::jthread{std::bind_front(&LibUsbDeviceManager::runDevice, this)};
		}

		LibUsbDeviceManager(LibUsbDeviceManager&)            = delete;
		LibUsbDeviceManager& operator=(LibUsbDeviceManager&) = delete;

		LibUsbDeviceManager(LibUsbDeviceManager&& other)            = delete;
		LibUsbDeviceManager& operator=(LibUsbDeviceManager&& other) = delete;

		~LibUsbDeviceManager() {
			clearDevices();

			deviceThread.request_stop();

			libUsbThread.request_stop();
			libusb_interrupt_event_handler(usb_context);
			if(libUsbThread.joinable()) {
				libUsbThread.join();
			}
			libusb_exit(usb_context);
		}

		std::vector<std::shared_ptr<Device>> getDevices(int vid, int pid) {
			clearDevices();

			bool                                 hadDevicesWithRightVIDandPID = false;
			std::vector<std::shared_ptr<Device>> ret;
			libusb_device**                      device_list;
			if(0 < libusb_get_device_list(usb_context, std::addressof(device_list))) {
				{
					libusb_device* dev;
					std::size_t    i{};
					while((dev = device_list[i++])) {
						libusb_device_descriptor desc;
						if(0 == libusb_get_device_descriptor(dev, &desc)) {
							if(!(desc.idVendor == vid && desc.idProduct == pid)) {
								continue;
							}
							hadDevicesWithRightVIDandPID = true;
							auto newDev                  = registerDevice(dev);
							if(newDev) {
								ret.push_back(std::move(newDev));
							}
						}
					}
				}
				libusb_free_device_list(device_list, 1);

				auto const deadline = std::chrono::steady_clock::now() + std::chrono::seconds{2};

				while(deadline > std::chrono::steady_clock::now()) {
					if(std::all_of(ret.begin(), ret.end(), [](auto& device) {
						       return device->getId().has_value();
						       }))
					{
						break;
					}
				}

				std::erase_if(ret, [&](auto& device) {
					      auto hasId = device->getId().has_value();
					      if(!hasId) {
					      deregisterDevice(libusb_get_device(device->device_handle));
					      }
					      return !hasId;
					      });

				for(auto& dev : ret) {
					dev->enable_callback();
				};
			} else {
				return ret;
			}

			if(ret.empty()) {
				if(hadDevicesWithRightVIDandPID) {
					/* TODO(rkta): handle error */
				}
			}

			return ret;
		}

		void clearDevices() {
			std::lock_guard<std::mutex> lock{mutex};
			for(auto& dev : devices) {
				if(dev.second.use_count() != 1) {
					/* TODO(rkta): handle error */
				}
			}
			devices.clear();
		}
};
