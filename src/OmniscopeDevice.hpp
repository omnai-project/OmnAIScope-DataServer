#ifndef OMNISCOPEDEVICE_HPP__
#define OMNISCOPEDEVICE_HPP__

#include "OmniscopeCommands.hpp"

#include <aglio/packager.hpp>

#ifdef __GNUC__
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpedantic"
#endif

#include <atomic>
#include <boost/crc.hpp>
#include <chrono>
#include <libusb.h>
#include <optional>
#include <span>

#ifdef __GNUC__
#pragma GCC diagnostic pop
#endif

struct OmniscopeDevice {
	public:
		OmniscopeDevice(libusb_device* device);
		~OmniscopeDevice();

#if 0
		OmniscopeDevice(OmniscopeDevice&)            = delete;
		OmniscopeDevice& operator=(OmniscopeDevice&) = delete;

		OmniscopeDevice(OmniscopeDevice&&)            = delete;
		OmniscopeDevice& operator=(OmniscopeDevice&&) = delete;
#endif

		bool isRunning();

		template<typename T>
		void send(T&& v) {
			if (error) return;
			std::unique_lock<std::mutex> lock{mutex};
			sendPackages.emplace_back(std::forward<T>(v));
			if(!sending) {
				lock.unlock();
				transmit();
			}
		}

		std::optional<Omniscope::Id> getId();

		void parse();
		void setScale(double i);
		void setOffset(double i);
		void setTimeScale(double i);
		void setEgu(std::string s);
		void transmit();
		std::optional<double> getScale();
		std::optional<double> getOffset();
		std::optional<double> getTimeScale();
		std::optional<std::string> getEgu();
		void sendCallback(libusb_transfer* transfer);
		void recvCallback(libusb_transfer* transfer);

		std::mutex mutex;
		std::function<void(Omniscope::UCToHostTypes)> msgCallback;
		template<typename Callback>
			void setMessageCallback(Callback&& cb) {
				std::lock_guard<std::mutex> lock{mutex};
				msgCallback     = std::forward<Callback>(cb);
				callbackEnabled = true;
			}
		void clearMessageCallback();
		void enable_callback();
		void periodicTask();

		static void staticSendCallback(libusb_transfer* transfer) {
			reinterpret_cast<OmniscopeDevice*>(transfer->user_data)->sendCallback(transfer);
		}

		static void staticRecvCallback(libusb_transfer* transfer) {
			reinterpret_cast<OmniscopeDevice*>(transfer->user_data)->recvCallback(transfer);
		}



	private:
		friend LibUsbDeviceManager<OmniscopeDevice>;

		struct PacketCrc {
			using type = std::uint16_t;
			static type calc(std::span<std::byte const> s) {
				boost::crc_ccitt_type crc;
				crc.process_bytes(s.data(), s.size());
				return crc.checksum();
			}
		};
		struct PackagerConfig {
			static constexpr bool isChannelSave         = false;
			using Crc                                   = PacketCrc;
			using Size_t                                = std::uint16_t;
			static constexpr std::uint16_t PackageStart = 0x55AA;
			static constexpr std::size_t   MaxSize      = 8192;
		};
		using Packager = aglio::Packager<PackagerConfig>;

		static constexpr int EndPoint  = 1;
		static constexpr int Interface = 0;

		libusb_device_handle*      device_handle{nullptr};
		libusb_transfer*           sendTransfer{nullptr};
		libusb_transfer*           recvTransfer1{nullptr};
		libusb_transfer*           recvTransfer2{nullptr};
		std::vector<unsigned char> recvBuffer1;
		std::vector<unsigned char> recvBuffer2;
		std::vector<unsigned char> sendBuffer;
		std::atomic<bool>          sendSubmitted{false};
		std::atomic<bool>          recv1Submitted{false};
		std::atomic<bool>          recv2Submitted{false};
		std::atomic<bool>          callbackEnabled{false};

		std::vector<unsigned char> parseBuffer;

		std::chrono::steady_clock::time_point lastPing{std::chrono::steady_clock::now()};

		std::atomic_bool                      error{false};
		std::chrono::steady_clock::time_point lastMsg{lastPing};
		std::atomic<bool>                     sending{};
		std::vector<Omniscope::HostToUCTypes> sendPackages;
		std::optional<Omniscope::Id>          deviceId;
		std::optional<double>                 scale;
		std::optional<double>                 offset;
		std::optional<double>                 timeScale;
		std::optional<std::string>            egu;
};

using OmniscopeDeviceManager = LibUsbDeviceManager<OmniscopeDevice>;

#endif
