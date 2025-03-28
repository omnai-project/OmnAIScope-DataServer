#ifndef OMNISCOPESAMPLER_HPP__
#define OMNISCOPESAMPLER_HPP__

#include "LibUsbDeviceManager.hpp"
#include "OmniscopeDevice.hpp"
#include "OmniscopeCommands.hpp"

#include <map>
#include <thread>

struct OmniscopeSampler {
	struct SampleDevice {
		std::array<std::vector<Omniscope::MeasureData>, 2>  packages;
		std::size_t                                         currentBuffer{};
		std::vector<std::pair<std::uint16_t, std::int16_t>> samples;
		std::mutex                                          mutex;
		OmniscopeDevice&                                    device;
		SampleDevice(OmniscopeDevice& device_);
		~SampleDevice();

		void unpackPackages();

		void swapBuffer();
	};

	LibUsbDeviceManager<OmniscopeDevice>&                                     deviceManager;
	std::map<std::shared_ptr<OmniscopeDevice>, std::unique_ptr<SampleDevice>> sampleDevices;
	std::jthread                                                              alignThread;
	std::mutex                                                                copyOutMutex;
	bool                                                                      firstAlign{true};
	std::map<Omniscope::Id, std::vector<std::pair<double, double>>>           alignedSamples;

	OmniscopeSampler(LibUsbDeviceManager<OmniscopeDevice>& deviceManager_,
			 std::vector<std::shared_ptr<OmniscopeDevice>> devices);

	~OmniscopeSampler();

	void alignSamples();

	void alignRunner(std::stop_token stoken);

	void copyOut(std::map<Omniscope::Id, std::vector<std::pair<double, double>>>& outMap);

	void copyOut(std::map<Omniscope::Id, std::vector<std::pair<double, double>>>& outMap, double scale, double offset, double sampleRate);
};

#endif
