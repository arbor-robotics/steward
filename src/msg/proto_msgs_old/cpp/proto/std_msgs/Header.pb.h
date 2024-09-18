// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: proto/std_msgs/Header.proto

#ifndef GOOGLE_PROTOBUF_INCLUDED_proto_2fstd_5fmsgs_2fHeader_2eproto
#define GOOGLE_PROTOBUF_INCLUDED_proto_2fstd_5fmsgs_2fHeader_2eproto

#include <limits>
#include <string>

#include <google/protobuf/port_def.inc>
#if PROTOBUF_VERSION < 3012000
#error This file was generated by a newer version of protoc which is
#error incompatible with your Protocol Buffer headers. Please update
#error your headers.
#endif
#if 3012004 < PROTOBUF_MIN_PROTOC_VERSION
#error This file was generated by an older version of protoc which is
#error incompatible with your Protocol Buffer headers. Please
#error regenerate this file with a newer version of protoc.
#endif

#include <google/protobuf/port_undef.inc>
#include <google/protobuf/io/coded_stream.h>
#include <google/protobuf/arena.h>
#include <google/protobuf/arenastring.h>
#include <google/protobuf/generated_message_table_driven.h>
#include <google/protobuf/generated_message_util.h>
#include <google/protobuf/inlined_string_field.h>
#include <google/protobuf/metadata_lite.h>
#include <google/protobuf/generated_message_reflection.h>
#include <google/protobuf/message.h>
#include <google/protobuf/repeated_field.h>  // IWYU pragma: export
#include <google/protobuf/extension_set.h>  // IWYU pragma: export
#include <google/protobuf/unknown_field_set.h>
#include <google/protobuf/timestamp.pb.h>
// @@protoc_insertion_point(includes)
#include <google/protobuf/port_def.inc>
#define PROTOBUF_INTERNAL_EXPORT_proto_2fstd_5fmsgs_2fHeader_2eproto
PROTOBUF_NAMESPACE_OPEN
namespace internal {
class AnyMetadata;
}  // namespace internal
PROTOBUF_NAMESPACE_CLOSE

// Internal implementation detail -- do not use these members.
struct TableStruct_proto_2fstd_5fmsgs_2fHeader_2eproto {
  static const ::PROTOBUF_NAMESPACE_ID::internal::ParseTableField entries[]
    PROTOBUF_SECTION_VARIABLE(protodesc_cold);
  static const ::PROTOBUF_NAMESPACE_ID::internal::AuxillaryParseTableField aux[]
    PROTOBUF_SECTION_VARIABLE(protodesc_cold);
  static const ::PROTOBUF_NAMESPACE_ID::internal::ParseTable schema[1]
    PROTOBUF_SECTION_VARIABLE(protodesc_cold);
  static const ::PROTOBUF_NAMESPACE_ID::internal::FieldMetadata field_metadata[];
  static const ::PROTOBUF_NAMESPACE_ID::internal::SerializationTable serialization_table[];
  static const ::PROTOBUF_NAMESPACE_ID::uint32 offsets[];
};
extern const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable descriptor_table_proto_2fstd_5fmsgs_2fHeader_2eproto;
namespace steward {
class Header;
class HeaderDefaultTypeInternal;
extern HeaderDefaultTypeInternal _Header_default_instance_;
}  // namespace steward
PROTOBUF_NAMESPACE_OPEN
template<> ::steward::Header* Arena::CreateMaybeMessage<::steward::Header>(Arena*);
PROTOBUF_NAMESPACE_CLOSE
namespace steward {

// ===================================================================

class Header PROTOBUF_FINAL :
    public ::PROTOBUF_NAMESPACE_ID::Message /* @@protoc_insertion_point(class_definition:steward.Header) */ {
 public:
  inline Header() : Header(nullptr) {};
  virtual ~Header();

  Header(const Header& from);
  Header(Header&& from) noexcept
    : Header() {
    *this = ::std::move(from);
  }

  inline Header& operator=(const Header& from) {
    CopyFrom(from);
    return *this;
  }
  inline Header& operator=(Header&& from) noexcept {
    if (GetArena() == from.GetArena()) {
      if (this != &from) InternalSwap(&from);
    } else {
      CopyFrom(from);
    }
    return *this;
  }

  static const ::PROTOBUF_NAMESPACE_ID::Descriptor* descriptor() {
    return GetDescriptor();
  }
  static const ::PROTOBUF_NAMESPACE_ID::Descriptor* GetDescriptor() {
    return GetMetadataStatic().descriptor;
  }
  static const ::PROTOBUF_NAMESPACE_ID::Reflection* GetReflection() {
    return GetMetadataStatic().reflection;
  }
  static const Header& default_instance();

  static void InitAsDefaultInstance();  // FOR INTERNAL USE ONLY
  static inline const Header* internal_default_instance() {
    return reinterpret_cast<const Header*>(
               &_Header_default_instance_);
  }
  static constexpr int kIndexInFileMessages =
    0;

  friend void swap(Header& a, Header& b) {
    a.Swap(&b);
  }
  inline void Swap(Header* other) {
    if (other == this) return;
    if (GetArena() == other->GetArena()) {
      InternalSwap(other);
    } else {
      ::PROTOBUF_NAMESPACE_ID::internal::GenericSwap(this, other);
    }
  }
  void UnsafeArenaSwap(Header* other) {
    if (other == this) return;
    GOOGLE_DCHECK(GetArena() == other->GetArena());
    InternalSwap(other);
  }

  // implements Message ----------------------------------------------

  inline Header* New() const final {
    return CreateMaybeMessage<Header>(nullptr);
  }

  Header* New(::PROTOBUF_NAMESPACE_ID::Arena* arena) const final {
    return CreateMaybeMessage<Header>(arena);
  }
  void CopyFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) final;
  void MergeFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) final;
  void CopyFrom(const Header& from);
  void MergeFrom(const Header& from);
  PROTOBUF_ATTRIBUTE_REINITIALIZES void Clear() final;
  bool IsInitialized() const final;

  size_t ByteSizeLong() const final;
  const char* _InternalParse(const char* ptr, ::PROTOBUF_NAMESPACE_ID::internal::ParseContext* ctx) final;
  ::PROTOBUF_NAMESPACE_ID::uint8* _InternalSerialize(
      ::PROTOBUF_NAMESPACE_ID::uint8* target, ::PROTOBUF_NAMESPACE_ID::io::EpsCopyOutputStream* stream) const final;
  int GetCachedSize() const final { return _cached_size_.Get(); }

  private:
  inline void SharedCtor();
  inline void SharedDtor();
  void SetCachedSize(int size) const final;
  void InternalSwap(Header* other);
  friend class ::PROTOBUF_NAMESPACE_ID::internal::AnyMetadata;
  static ::PROTOBUF_NAMESPACE_ID::StringPiece FullMessageName() {
    return "steward.Header";
  }
  protected:
  explicit Header(::PROTOBUF_NAMESPACE_ID::Arena* arena);
  private:
  static void ArenaDtor(void* object);
  inline void RegisterArenaDtor(::PROTOBUF_NAMESPACE_ID::Arena* arena);
  public:

  ::PROTOBUF_NAMESPACE_ID::Metadata GetMetadata() const final;
  private:
  static ::PROTOBUF_NAMESPACE_ID::Metadata GetMetadataStatic() {
    ::PROTOBUF_NAMESPACE_ID::internal::AssignDescriptors(&::descriptor_table_proto_2fstd_5fmsgs_2fHeader_2eproto);
    return ::descriptor_table_proto_2fstd_5fmsgs_2fHeader_2eproto.file_level_metadata[kIndexInFileMessages];
  }

  public:

  // nested types ----------------------------------------------------

  // accessors -------------------------------------------------------

  enum : int {
    kFrameIdFieldNumber = 2,
    kStampFieldNumber = 1,
  };
  // string frame_id = 2;
  void clear_frame_id();
  const std::string& frame_id() const;
  void set_frame_id(const std::string& value);
  void set_frame_id(std::string&& value);
  void set_frame_id(const char* value);
  void set_frame_id(const char* value, size_t size);
  std::string* mutable_frame_id();
  std::string* release_frame_id();
  void set_allocated_frame_id(std::string* frame_id);
  GOOGLE_PROTOBUF_RUNTIME_DEPRECATED("The unsafe_arena_ accessors for"
  "    string fields are deprecated and will be removed in a"
  "    future release.")
  std::string* unsafe_arena_release_frame_id();
  GOOGLE_PROTOBUF_RUNTIME_DEPRECATED("The unsafe_arena_ accessors for"
  "    string fields are deprecated and will be removed in a"
  "    future release.")
  void unsafe_arena_set_allocated_frame_id(
      std::string* frame_id);
  private:
  const std::string& _internal_frame_id() const;
  void _internal_set_frame_id(const std::string& value);
  std::string* _internal_mutable_frame_id();
  public:

  // .google.protobuf.Timestamp stamp = 1;
  bool has_stamp() const;
  private:
  bool _internal_has_stamp() const;
  public:
  void clear_stamp();
  const PROTOBUF_NAMESPACE_ID::Timestamp& stamp() const;
  PROTOBUF_NAMESPACE_ID::Timestamp* release_stamp();
  PROTOBUF_NAMESPACE_ID::Timestamp* mutable_stamp();
  void set_allocated_stamp(PROTOBUF_NAMESPACE_ID::Timestamp* stamp);
  private:
  const PROTOBUF_NAMESPACE_ID::Timestamp& _internal_stamp() const;
  PROTOBUF_NAMESPACE_ID::Timestamp* _internal_mutable_stamp();
  public:
  void unsafe_arena_set_allocated_stamp(
      PROTOBUF_NAMESPACE_ID::Timestamp* stamp);
  PROTOBUF_NAMESPACE_ID::Timestamp* unsafe_arena_release_stamp();

  // @@protoc_insertion_point(class_scope:steward.Header)
 private:
  class _Internal;

  template <typename T> friend class ::PROTOBUF_NAMESPACE_ID::Arena::InternalHelper;
  typedef void InternalArenaConstructable_;
  typedef void DestructorSkippable_;
  ::PROTOBUF_NAMESPACE_ID::internal::ArenaStringPtr frame_id_;
  PROTOBUF_NAMESPACE_ID::Timestamp* stamp_;
  mutable ::PROTOBUF_NAMESPACE_ID::internal::CachedSize _cached_size_;
  friend struct ::TableStruct_proto_2fstd_5fmsgs_2fHeader_2eproto;
};
// ===================================================================


// ===================================================================

#ifdef __GNUC__
  #pragma GCC diagnostic push
  #pragma GCC diagnostic ignored "-Wstrict-aliasing"
#endif  // __GNUC__
// Header

// .google.protobuf.Timestamp stamp = 1;
inline bool Header::_internal_has_stamp() const {
  return this != internal_default_instance() && stamp_ != nullptr;
}
inline bool Header::has_stamp() const {
  return _internal_has_stamp();
}
inline const PROTOBUF_NAMESPACE_ID::Timestamp& Header::_internal_stamp() const {
  const PROTOBUF_NAMESPACE_ID::Timestamp* p = stamp_;
  return p != nullptr ? *p : *reinterpret_cast<const PROTOBUF_NAMESPACE_ID::Timestamp*>(
      &PROTOBUF_NAMESPACE_ID::_Timestamp_default_instance_);
}
inline const PROTOBUF_NAMESPACE_ID::Timestamp& Header::stamp() const {
  // @@protoc_insertion_point(field_get:steward.Header.stamp)
  return _internal_stamp();
}
inline void Header::unsafe_arena_set_allocated_stamp(
    PROTOBUF_NAMESPACE_ID::Timestamp* stamp) {
  if (GetArena() == nullptr) {
    delete reinterpret_cast<::PROTOBUF_NAMESPACE_ID::MessageLite*>(stamp_);
  }
  stamp_ = stamp;
  if (stamp) {

  } else {

  }
  // @@protoc_insertion_point(field_unsafe_arena_set_allocated:steward.Header.stamp)
}
inline PROTOBUF_NAMESPACE_ID::Timestamp* Header::release_stamp() {
  auto temp = unsafe_arena_release_stamp();
  if (GetArena() != nullptr) {
    temp = ::PROTOBUF_NAMESPACE_ID::internal::DuplicateIfNonNull(temp);
  }
  return temp;
}
inline PROTOBUF_NAMESPACE_ID::Timestamp* Header::unsafe_arena_release_stamp() {
  // @@protoc_insertion_point(field_release:steward.Header.stamp)

  PROTOBUF_NAMESPACE_ID::Timestamp* temp = stamp_;
  stamp_ = nullptr;
  return temp;
}
inline PROTOBUF_NAMESPACE_ID::Timestamp* Header::_internal_mutable_stamp() {

  if (stamp_ == nullptr) {
    auto* p = CreateMaybeMessage<PROTOBUF_NAMESPACE_ID::Timestamp>(GetArena());
    stamp_ = p;
  }
  return stamp_;
}
inline PROTOBUF_NAMESPACE_ID::Timestamp* Header::mutable_stamp() {
  // @@protoc_insertion_point(field_mutable:steward.Header.stamp)
  return _internal_mutable_stamp();
}
inline void Header::set_allocated_stamp(PROTOBUF_NAMESPACE_ID::Timestamp* stamp) {
  ::PROTOBUF_NAMESPACE_ID::Arena* message_arena = GetArena();
  if (message_arena == nullptr) {
    delete reinterpret_cast< ::PROTOBUF_NAMESPACE_ID::MessageLite*>(stamp_);
  }
  if (stamp) {
    ::PROTOBUF_NAMESPACE_ID::Arena* submessage_arena =
      reinterpret_cast<::PROTOBUF_NAMESPACE_ID::MessageLite*>(stamp)->GetArena();
    if (message_arena != submessage_arena) {
      stamp = ::PROTOBUF_NAMESPACE_ID::internal::GetOwnedMessage(
          message_arena, stamp, submessage_arena);
    }

  } else {

  }
  stamp_ = stamp;
  // @@protoc_insertion_point(field_set_allocated:steward.Header.stamp)
}

// string frame_id = 2;
inline void Header::clear_frame_id() {
  frame_id_.ClearToEmpty(&::PROTOBUF_NAMESPACE_ID::internal::GetEmptyStringAlreadyInited(), GetArena());
}
inline const std::string& Header::frame_id() const {
  // @@protoc_insertion_point(field_get:steward.Header.frame_id)
  return _internal_frame_id();
}
inline void Header::set_frame_id(const std::string& value) {
  _internal_set_frame_id(value);
  // @@protoc_insertion_point(field_set:steward.Header.frame_id)
}
inline std::string* Header::mutable_frame_id() {
  // @@protoc_insertion_point(field_mutable:steward.Header.frame_id)
  return _internal_mutable_frame_id();
}
inline const std::string& Header::_internal_frame_id() const {
  return frame_id_.Get();
}
inline void Header::_internal_set_frame_id(const std::string& value) {

  frame_id_.Set(&::PROTOBUF_NAMESPACE_ID::internal::GetEmptyStringAlreadyInited(), value, GetArena());
}
inline void Header::set_frame_id(std::string&& value) {

  frame_id_.Set(
    &::PROTOBUF_NAMESPACE_ID::internal::GetEmptyStringAlreadyInited(), ::std::move(value), GetArena());
  // @@protoc_insertion_point(field_set_rvalue:steward.Header.frame_id)
}
inline void Header::set_frame_id(const char* value) {
  GOOGLE_DCHECK(value != nullptr);

  frame_id_.Set(&::PROTOBUF_NAMESPACE_ID::internal::GetEmptyStringAlreadyInited(), ::std::string(value),
              GetArena());
  // @@protoc_insertion_point(field_set_char:steward.Header.frame_id)
}
inline void Header::set_frame_id(const char* value,
    size_t size) {

  frame_id_.Set(&::PROTOBUF_NAMESPACE_ID::internal::GetEmptyStringAlreadyInited(), ::std::string(
      reinterpret_cast<const char*>(value), size), GetArena());
  // @@protoc_insertion_point(field_set_pointer:steward.Header.frame_id)
}
inline std::string* Header::_internal_mutable_frame_id() {

  return frame_id_.Mutable(&::PROTOBUF_NAMESPACE_ID::internal::GetEmptyStringAlreadyInited(), GetArena());
}
inline std::string* Header::release_frame_id() {
  // @@protoc_insertion_point(field_release:steward.Header.frame_id)
  return frame_id_.Release(&::PROTOBUF_NAMESPACE_ID::internal::GetEmptyStringAlreadyInited(), GetArena());
}
inline void Header::set_allocated_frame_id(std::string* frame_id) {
  if (frame_id != nullptr) {

  } else {

  }
  frame_id_.SetAllocated(&::PROTOBUF_NAMESPACE_ID::internal::GetEmptyStringAlreadyInited(), frame_id,
      GetArena());
  // @@protoc_insertion_point(field_set_allocated:steward.Header.frame_id)
}
inline std::string* Header::unsafe_arena_release_frame_id() {
  // @@protoc_insertion_point(field_unsafe_arena_release:steward.Header.frame_id)
  GOOGLE_DCHECK(GetArena() != nullptr);

  return frame_id_.UnsafeArenaRelease(&::PROTOBUF_NAMESPACE_ID::internal::GetEmptyStringAlreadyInited(),
      GetArena());
}
inline void Header::unsafe_arena_set_allocated_frame_id(
    std::string* frame_id) {
  GOOGLE_DCHECK(GetArena() != nullptr);
  if (frame_id != nullptr) {

  } else {

  }
  frame_id_.UnsafeArenaSetAllocated(&::PROTOBUF_NAMESPACE_ID::internal::GetEmptyStringAlreadyInited(),
      frame_id, GetArena());
  // @@protoc_insertion_point(field_unsafe_arena_set_allocated:steward.Header.frame_id)
}

#ifdef __GNUC__
  #pragma GCC diagnostic pop
#endif  // __GNUC__

// @@protoc_insertion_point(namespace_scope)

}  // namespace steward

// @@protoc_insertion_point(global_scope)

#include <google/protobuf/port_undef.inc>
#endif  // GOOGLE_PROTOBUF_INCLUDED_GOOGLE_PROTOBUF_INCLUDED_proto_2fstd_5fmsgs_2fHeader_2eproto
