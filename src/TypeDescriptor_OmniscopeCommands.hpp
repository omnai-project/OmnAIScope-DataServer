#ifndef TYPEDESCRIPTOR_OMNISCOPECOMMANDS_HPP__
#define TYPEDESCRIPTOR_OMNISCOPECOMMANDS_HPP__

namespace aglio {
	template <>
		struct TypeDescriptorGen<Omniscope::Version>
		: MemberList<MemberDescriptor<&Omniscope::Version::major, "major">,
		MemberDescriptor<&Omniscope::Version::minor, "minor">,
		MemberDescriptor<&Omniscope::Version::patch, "patch">> {
			static constexpr std::string_view Name{"Version"};
			static constexpr std::string_view QualifiedName{"Omniscope::Version"};
		};
}  // namespace aglio
namespace aglio {
	template <>
		struct TypeDescriptorGen<Omniscope::Id>
		: MemberList<MemberDescriptor<&Omniscope::Id::serial, "serial">,
		MemberDescriptor<&Omniscope::Id::type, "type">,
		MemberDescriptor<&Omniscope::Id::sampleRate, "sampleRate">,
		MemberDescriptor<&Omniscope::Id::hwVersion, "hwVersion">,
		MemberDescriptor<&Omniscope::Id::swVersion, "swVersion">,
		MemberDescriptor<&Omniscope::Id::swGitHash, "swGitHash">> {
			static constexpr std::string_view Name{"Id"};
			static constexpr std::string_view QualifiedName{"Omniscope::Id"};
		};
}  // namespace aglio
namespace aglio {
	template <>
		struct TypeDescriptorGen<Omniscope::StartOfFrame>
		: MemberList<
		MemberDescriptor<&Omniscope::StartOfFrame::sof_and_change_index,
		"sof_and_change_index">> {
			static constexpr std::string_view Name{"StartOfFrame"};
			static constexpr std::string_view QualifiedName{"Omniscope::StartOfFrame"};
		};
}  // namespace aglio
namespace aglio {
	template <>
		struct TypeDescriptorGen<Omniscope::MeasureData>
		: MemberList<MemberDescriptor<&Omniscope::MeasureData::packageCounter,
		"packageCounter">,
		MemberDescriptor<&Omniscope::MeasureData::sof, "sof">,
		MemberDescriptor<&Omniscope::MeasureData::data, "data">> {
			static constexpr std::string_view Name{"MeasureData"};
			static constexpr std::string_view QualifiedName{"Omniscope::MeasureData"};
		};
}  // namespace aglio
namespace aglio {
	template <>
		struct TypeDescriptorGen<Omniscope::SetRgb>
		: MemberList<MemberDescriptor<&Omniscope::SetRgb::r, "r">,
		MemberDescriptor<&Omniscope::SetRgb::g, "g">,
		MemberDescriptor<&Omniscope::SetRgb::b, "b">> {
			static constexpr std::string_view Name{"SetRgb"};
			static constexpr std::string_view QualifiedName{"Omniscope::SetRgb"};
		};
}  // namespace aglio
namespace aglio {
	template <>
		struct TypeDescriptorGen<Omniscope::Ping> {
			static constexpr std::string_view Name{"Ping"};
			static constexpr std::string_view QualifiedName{"Omniscope::Ping"};
		};
}  // namespace aglio
namespace aglio {
	template <>
		struct TypeDescriptorGen<Omniscope::GetId> {
			static constexpr std::string_view Name{"GetId"};
			static constexpr std::string_view QualifiedName{"Omniscope::GetId"};
		};
}  // namespace aglio
namespace aglio {
	template <>
		struct TypeDescriptorGen<Omniscope::Start> {
			static constexpr std::string_view Name{"Start"};
			static constexpr std::string_view QualifiedName{"Omniscope::Start"};
		};
}  // namespace aglio
namespace aglio {
	template <>
		struct TypeDescriptorGen<Omniscope::Stop> {
			static constexpr std::string_view Name{"Stop"};
			static constexpr std::string_view QualifiedName{"Omniscope::Stop"};
		};
}  // namespace aglio
namespace aglio {
	template <>
		struct TypeDescriptorGen<Omniscope::MetaData>
		: MemberList<MemberDescriptor<&Omniscope::MetaData::data, "data">> {
			static constexpr std::string_view Name{"MetaData"};
			static constexpr std::string_view QualifiedName{"Omniscope::MetaData"};
		};
}  // namespace aglio
namespace aglio {
	template <>
		struct TypeDescriptorGen<Omniscope::SetMetaData>
		: MemberList<MemberDescriptor<&Omniscope::SetMetaData::data, "data">> {
			static constexpr std::string_view Name{"SetMetaData"};
			static constexpr std::string_view QualifiedName{"Omniscope::SetMetaData"};
		};
}  // namespace aglio
namespace aglio {
	template <>
		struct TypeDescriptorGen<Omniscope::GetMetaData> {
			static constexpr std::string_view Name{"GetMetaData"};
			static constexpr std::string_view QualifiedName{"Omniscope::GetMetaData"};
		};
}  // namespace aglio

#endif
