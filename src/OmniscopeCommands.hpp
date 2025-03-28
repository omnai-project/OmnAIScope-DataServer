#ifndef OMNISCOPECOMMANDS_HPP__
#define OMNISCOPECOMMANDS_HPP__

#include <aglio/type_descriptor.hpp>
#include <array>
#include <variant>
#include <string>

namespace Omniscope {

	struct Version {
		std::uint8_t major{};
		std::uint8_t minor{};
		std::uint8_t patch{};
		auto operator<=>(Version const&) const=default;
	};

	struct Id {
		std::string   serial;
		std::string   type;
		std::uint32_t sampleRate;
		Version       hwVersion;
		Version       swVersion;
		std::string   swGitHash;
		auto operator<=>(Id const&) const=default;
	};

	struct StartOfFrame {
		std::uint16_t sof_and_change_index;
	};

	struct MeasureData {
		std::uint8_t              packageCounter;
		StartOfFrame              sof;
		std::vector<std::int16_t> data;
	};

	struct SetRgb {
		std::uint8_t r;
		std::uint8_t g;
		std::uint8_t b;
	};

	struct MetaData{
		std::string data;

		constexpr bool operator==(MetaData const& rhs) const = default;
		constexpr bool operator!=(MetaData const& rhs) const = default;
	};

	struct SetMetaData{
		std::string data;
	};

	struct Ping {};
	struct GetId {};
	struct Start {};
	struct Stop {};
	struct GetMetaData{};

	using UCToHostTypes = std::variant<Id, MeasureData, MetaData>;
	using HostToUCTypes = std::variant<GetId, Ping, Start, Stop, SetRgb, SetMetaData, GetMetaData>;
}   // namespace Omniscope

#include "TypeDescriptor_OmniscopeCommands.hpp"

#endif
