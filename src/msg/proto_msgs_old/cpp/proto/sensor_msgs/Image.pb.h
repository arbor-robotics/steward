// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: proto/sensor_msgs/Image.proto

#ifndef GOOGLE_PROTOBUF_INCLUDED_proto_2fsensor_5fmsgs_2fImage_2eproto
#define GOOGLE_PROTOBUF_INCLUDED_proto_2fsensor_5fmsgs_2fImage_2eproto

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
#include "proto/std_msgs/Header.pb.h"
// @@protoc_insertion_point(includes)
#include <google/protobuf/port_def.inc>
#define PROTOBUF_INTERNAL_EXPORT_proto_2fsensor_5fmsgs_2fImage_2eproto
PROTOBUF_NAMESPACE_OPEN
namespace internal {
class AnyMetadata;
}  // namespace internal
PROTOBUF_NAMESPACE_CLOSE

// Internal implementation detail -- do not use these members.
struct TableStruct_proto_2fsensor_5fmsgs_2fImage_2eproto {
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
extern const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable descriptor_table_proto_2fsensor_5fmsgs_2fImage_2eproto;
namespace steward {
class Image;
class ImageDefaultTypeInternal;
extern ImageDefaultTypeInternal _Image_default_instance_;
}  // namespace steward
PROTOBUF_NAMESPACE_OPEN
template<> ::steward::Image* Arena::CreateMaybeMessage<::steward::Image>(Arena*);
PROTOBUF_NAMESPACE_CLOSE
namespace steward {

// ===================================================================

class Image PROTOBUF_FINAL :
    public ::PROTOBUF_NAMESPACE_ID::Message /* @@protoc_insertion_point(class_definition:steward.Image) */ {
 public:
  inline Image() : Image(nullptr) {};
  virtual ~Image();

  Image(const Image& from);
  Image(Image&& from) noexcept
    : Image() {
    *this = ::std::move(from);
  }

  inline Image& operator=(const Image& from) {
    CopyFrom(from);
    return *this;
  }
  inline Image& operator=(Image&& from) noexcept {
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
  static const Image& default_instance();

  static void InitAsDefaultInstance();  // FOR INTERNAL USE ONLY
  static inline const Image* internal_default_instance() {
    return reinterpret_cast<const Image*>(
               &_Image_default_instance_);
  }
  static constexpr int kIndexInFileMessages =
    0;

  friend void swap(Image& a, Image& b) {
    a.Swap(&b);
  }
  inline void Swap(Image* other) {
    if (other == this) return;
    if (GetArena() == other->GetArena()) {
      InternalSwap(other);
    } else {
      ::PROTOBUF_NAMESPACE_ID::internal::GenericSwap(this, other);
    }
  }
  void UnsafeArenaSwap(Image* other) {
    if (other == this) return;
    GOOGLE_DCHECK(GetArena() == other->GetArena());
    InternalSwap(other);
  }

  // implements Message ----------------------------------------------

  inline Image* New() const final {
    return CreateMaybeMessage<Image>(nullptr);
  }

  Image* New(::PROTOBUF_NAMESPACE_ID::Arena* arena) const final {
    return CreateMaybeMessage<Image>(arena);
  }
  void CopyFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) final;
  void MergeFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) final;
  void CopyFrom(const Image& from);
  void MergeFrom(const Image& from);
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
  void InternalSwap(Image* other);
  friend class ::PROTOBUF_NAMESPACE_ID::internal::AnyMetadata;
  static ::PROTOBUF_NAMESPACE_ID::StringPiece FullMessageName() {
    return "steward.Image";
  }
  protected:
  explicit Image(::PROTOBUF_NAMESPACE_ID::Arena* arena);
  private:
  static void ArenaDtor(void* object);
  inline void RegisterArenaDtor(::PROTOBUF_NAMESPACE_ID::Arena* arena);
  public:

  ::PROTOBUF_NAMESPACE_ID::Metadata GetMetadata() const final;
  private:
  static ::PROTOBUF_NAMESPACE_ID::Metadata GetMetadataStatic() {
    ::PROTOBUF_NAMESPACE_ID::internal::AssignDescriptors(&::descriptor_table_proto_2fsensor_5fmsgs_2fImage_2eproto);
    return ::descriptor_table_proto_2fsensor_5fmsgs_2fImage_2eproto.file_level_metadata[kIndexInFileMessages];
  }

  public:

  // nested types ----------------------------------------------------

  // accessors -------------------------------------------------------

  enum : int {
    kEncodingFieldNumber = 4,
    kDataFieldNumber = 7,
    kHeaderFieldNumber = 1,
    kHeightFieldNumber = 2,
    kWidthFieldNumber = 3,
    kIsBigendianFieldNumber = 5,
    kStepFieldNumber = 6,
  };
  // string encoding = 4;
  void clear_encoding();
  const std::string& encoding() const;
  void set_encoding(const std::string& value);
  void set_encoding(std::string&& value);
  void set_encoding(const char* value);
  void set_encoding(const char* value, size_t size);
  std::string* mutable_encoding();
  std::string* release_encoding();
  void set_allocated_encoding(std::string* encoding);
  GOOGLE_PROTOBUF_RUNTIME_DEPRECATED("The unsafe_arena_ accessors for"
  "    string fields are deprecated and will be removed in a"
  "    future release.")
  std::string* unsafe_arena_release_encoding();
  GOOGLE_PROTOBUF_RUNTIME_DEPRECATED("The unsafe_arena_ accessors for"
  "    string fields are deprecated and will be removed in a"
  "    future release.")
  void unsafe_arena_set_allocated_encoding(
      std::string* encoding);
  private:
  const std::string& _internal_encoding() const;
  void _internal_set_encoding(const std::string& value);
  std::string* _internal_mutable_encoding();
  public:

  // bytes data = 7;
  void clear_data();
  const std::string& data() const;
  void set_data(const std::string& value);
  void set_data(std::string&& value);
  void set_data(const char* value);
  void set_data(const void* value, size_t size);
  std::string* mutable_data();
  std::string* release_data();
  void set_allocated_data(std::string* data);
  GOOGLE_PROTOBUF_RUNTIME_DEPRECATED("The unsafe_arena_ accessors for"
  "    string fields are deprecated and will be removed in a"
  "    future release.")
  std::string* unsafe_arena_release_data();
  GOOGLE_PROTOBUF_RUNTIME_DEPRECATED("The unsafe_arena_ accessors for"
  "    string fields are deprecated and will be removed in a"
  "    future release.")
  void unsafe_arena_set_allocated_data(
      std::string* data);
  private:
  const std::string& _internal_data() const;
  void _internal_set_data(const std::string& value);
  std::string* _internal_mutable_data();
  public:

  // .steward.Header header = 1;
  bool has_header() const;
  private:
  bool _internal_has_header() const;
  public:
  void clear_header();
  const ::steward::Header& header() const;
  ::steward::Header* release_header();
  ::steward::Header* mutable_header();
  void set_allocated_header(::steward::Header* header);
  private:
  const ::steward::Header& _internal_header() const;
  ::steward::Header* _internal_mutable_header();
  public:
  void unsafe_arena_set_allocated_header(
      ::steward::Header* header);
  ::steward::Header* unsafe_arena_release_header();

  // uint32 height = 2;
  void clear_height();
  ::PROTOBUF_NAMESPACE_ID::uint32 height() const;
  void set_height(::PROTOBUF_NAMESPACE_ID::uint32 value);
  private:
  ::PROTOBUF_NAMESPACE_ID::uint32 _internal_height() const;
  void _internal_set_height(::PROTOBUF_NAMESPACE_ID::uint32 value);
  public:

  // uint32 width = 3;
  void clear_width();
  ::PROTOBUF_NAMESPACE_ID::uint32 width() const;
  void set_width(::PROTOBUF_NAMESPACE_ID::uint32 value);
  private:
  ::PROTOBUF_NAMESPACE_ID::uint32 _internal_width() const;
  void _internal_set_width(::PROTOBUF_NAMESPACE_ID::uint32 value);
  public:

  // bool is_bigendian = 5;
  void clear_is_bigendian();
  bool is_bigendian() const;
  void set_is_bigendian(bool value);
  private:
  bool _internal_is_bigendian() const;
  void _internal_set_is_bigendian(bool value);
  public:

  // uint32 step = 6;
  void clear_step();
  ::PROTOBUF_NAMESPACE_ID::uint32 step() const;
  void set_step(::PROTOBUF_NAMESPACE_ID::uint32 value);
  private:
  ::PROTOBUF_NAMESPACE_ID::uint32 _internal_step() const;
  void _internal_set_step(::PROTOBUF_NAMESPACE_ID::uint32 value);
  public:

  // @@protoc_insertion_point(class_scope:steward.Image)
 private:
  class _Internal;

  template <typename T> friend class ::PROTOBUF_NAMESPACE_ID::Arena::InternalHelper;
  typedef void InternalArenaConstructable_;
  typedef void DestructorSkippable_;
  ::PROTOBUF_NAMESPACE_ID::internal::ArenaStringPtr encoding_;
  ::PROTOBUF_NAMESPACE_ID::internal::ArenaStringPtr data_;
  ::steward::Header* header_;
  ::PROTOBUF_NAMESPACE_ID::uint32 height_;
  ::PROTOBUF_NAMESPACE_ID::uint32 width_;
  bool is_bigendian_;
  ::PROTOBUF_NAMESPACE_ID::uint32 step_;
  mutable ::PROTOBUF_NAMESPACE_ID::internal::CachedSize _cached_size_;
  friend struct ::TableStruct_proto_2fsensor_5fmsgs_2fImage_2eproto;
};
// ===================================================================


// ===================================================================

#ifdef __GNUC__
  #pragma GCC diagnostic push
  #pragma GCC diagnostic ignored "-Wstrict-aliasing"
#endif  // __GNUC__
// Image

// .steward.Header header = 1;
inline bool Image::_internal_has_header() const {
  return this != internal_default_instance() && header_ != nullptr;
}
inline bool Image::has_header() const {
  return _internal_has_header();
}
inline const ::steward::Header& Image::_internal_header() const {
  const ::steward::Header* p = header_;
  return p != nullptr ? *p : *reinterpret_cast<const ::steward::Header*>(
      &::steward::_Header_default_instance_);
}
inline const ::steward::Header& Image::header() const {
  // @@protoc_insertion_point(field_get:steward.Image.header)
  return _internal_header();
}
inline void Image::unsafe_arena_set_allocated_header(
    ::steward::Header* header) {
  if (GetArena() == nullptr) {
    delete reinterpret_cast<::PROTOBUF_NAMESPACE_ID::MessageLite*>(header_);
  }
  header_ = header;
  if (header) {

  } else {

  }
  // @@protoc_insertion_point(field_unsafe_arena_set_allocated:steward.Image.header)
}
inline ::steward::Header* Image::release_header() {
  auto temp = unsafe_arena_release_header();
  if (GetArena() != nullptr) {
    temp = ::PROTOBUF_NAMESPACE_ID::internal::DuplicateIfNonNull(temp);
  }
  return temp;
}
inline ::steward::Header* Image::unsafe_arena_release_header() {
  // @@protoc_insertion_point(field_release:steward.Image.header)

  ::steward::Header* temp = header_;
  header_ = nullptr;
  return temp;
}
inline ::steward::Header* Image::_internal_mutable_header() {

  if (header_ == nullptr) {
    auto* p = CreateMaybeMessage<::steward::Header>(GetArena());
    header_ = p;
  }
  return header_;
}
inline ::steward::Header* Image::mutable_header() {
  // @@protoc_insertion_point(field_mutable:steward.Image.header)
  return _internal_mutable_header();
}
inline void Image::set_allocated_header(::steward::Header* header) {
  ::PROTOBUF_NAMESPACE_ID::Arena* message_arena = GetArena();
  if (message_arena == nullptr) {
    delete reinterpret_cast< ::PROTOBUF_NAMESPACE_ID::MessageLite*>(header_);
  }
  if (header) {
    ::PROTOBUF_NAMESPACE_ID::Arena* submessage_arena =
      reinterpret_cast<::PROTOBUF_NAMESPACE_ID::MessageLite*>(header)->GetArena();
    if (message_arena != submessage_arena) {
      header = ::PROTOBUF_NAMESPACE_ID::internal::GetOwnedMessage(
          message_arena, header, submessage_arena);
    }

  } else {

  }
  header_ = header;
  // @@protoc_insertion_point(field_set_allocated:steward.Image.header)
}

// uint32 height = 2;
inline void Image::clear_height() {
  height_ = 0u;
}
inline ::PROTOBUF_NAMESPACE_ID::uint32 Image::_internal_height() const {
  return height_;
}
inline ::PROTOBUF_NAMESPACE_ID::uint32 Image::height() const {
  // @@protoc_insertion_point(field_get:steward.Image.height)
  return _internal_height();
}
inline void Image::_internal_set_height(::PROTOBUF_NAMESPACE_ID::uint32 value) {

  height_ = value;
}
inline void Image::set_height(::PROTOBUF_NAMESPACE_ID::uint32 value) {
  _internal_set_height(value);
  // @@protoc_insertion_point(field_set:steward.Image.height)
}

// uint32 width = 3;
inline void Image::clear_width() {
  width_ = 0u;
}
inline ::PROTOBUF_NAMESPACE_ID::uint32 Image::_internal_width() const {
  return width_;
}
inline ::PROTOBUF_NAMESPACE_ID::uint32 Image::width() const {
  // @@protoc_insertion_point(field_get:steward.Image.width)
  return _internal_width();
}
inline void Image::_internal_set_width(::PROTOBUF_NAMESPACE_ID::uint32 value) {

  width_ = value;
}
inline void Image::set_width(::PROTOBUF_NAMESPACE_ID::uint32 value) {
  _internal_set_width(value);
  // @@protoc_insertion_point(field_set:steward.Image.width)
}

// string encoding = 4;
inline void Image::clear_encoding() {
  encoding_.ClearToEmpty(&::PROTOBUF_NAMESPACE_ID::internal::GetEmptyStringAlreadyInited(), GetArena());
}
inline const std::string& Image::encoding() const {
  // @@protoc_insertion_point(field_get:steward.Image.encoding)
  return _internal_encoding();
}
inline void Image::set_encoding(const std::string& value) {
  _internal_set_encoding(value);
  // @@protoc_insertion_point(field_set:steward.Image.encoding)
}
inline std::string* Image::mutable_encoding() {
  // @@protoc_insertion_point(field_mutable:steward.Image.encoding)
  return _internal_mutable_encoding();
}
inline const std::string& Image::_internal_encoding() const {
  return encoding_.Get();
}
inline void Image::_internal_set_encoding(const std::string& value) {

  encoding_.Set(&::PROTOBUF_NAMESPACE_ID::internal::GetEmptyStringAlreadyInited(), value, GetArena());
}
inline void Image::set_encoding(std::string&& value) {

  encoding_.Set(
    &::PROTOBUF_NAMESPACE_ID::internal::GetEmptyStringAlreadyInited(), ::std::move(value), GetArena());
  // @@protoc_insertion_point(field_set_rvalue:steward.Image.encoding)
}
inline void Image::set_encoding(const char* value) {
  GOOGLE_DCHECK(value != nullptr);

  encoding_.Set(&::PROTOBUF_NAMESPACE_ID::internal::GetEmptyStringAlreadyInited(), ::std::string(value),
              GetArena());
  // @@protoc_insertion_point(field_set_char:steward.Image.encoding)
}
inline void Image::set_encoding(const char* value,
    size_t size) {

  encoding_.Set(&::PROTOBUF_NAMESPACE_ID::internal::GetEmptyStringAlreadyInited(), ::std::string(
      reinterpret_cast<const char*>(value), size), GetArena());
  // @@protoc_insertion_point(field_set_pointer:steward.Image.encoding)
}
inline std::string* Image::_internal_mutable_encoding() {

  return encoding_.Mutable(&::PROTOBUF_NAMESPACE_ID::internal::GetEmptyStringAlreadyInited(), GetArena());
}
inline std::string* Image::release_encoding() {
  // @@protoc_insertion_point(field_release:steward.Image.encoding)
  return encoding_.Release(&::PROTOBUF_NAMESPACE_ID::internal::GetEmptyStringAlreadyInited(), GetArena());
}
inline void Image::set_allocated_encoding(std::string* encoding) {
  if (encoding != nullptr) {

  } else {

  }
  encoding_.SetAllocated(&::PROTOBUF_NAMESPACE_ID::internal::GetEmptyStringAlreadyInited(), encoding,
      GetArena());
  // @@protoc_insertion_point(field_set_allocated:steward.Image.encoding)
}
inline std::string* Image::unsafe_arena_release_encoding() {
  // @@protoc_insertion_point(field_unsafe_arena_release:steward.Image.encoding)
  GOOGLE_DCHECK(GetArena() != nullptr);

  return encoding_.UnsafeArenaRelease(&::PROTOBUF_NAMESPACE_ID::internal::GetEmptyStringAlreadyInited(),
      GetArena());
}
inline void Image::unsafe_arena_set_allocated_encoding(
    std::string* encoding) {
  GOOGLE_DCHECK(GetArena() != nullptr);
  if (encoding != nullptr) {

  } else {

  }
  encoding_.UnsafeArenaSetAllocated(&::PROTOBUF_NAMESPACE_ID::internal::GetEmptyStringAlreadyInited(),
      encoding, GetArena());
  // @@protoc_insertion_point(field_unsafe_arena_set_allocated:steward.Image.encoding)
}

// bool is_bigendian = 5;
inline void Image::clear_is_bigendian() {
  is_bigendian_ = false;
}
inline bool Image::_internal_is_bigendian() const {
  return is_bigendian_;
}
inline bool Image::is_bigendian() const {
  // @@protoc_insertion_point(field_get:steward.Image.is_bigendian)
  return _internal_is_bigendian();
}
inline void Image::_internal_set_is_bigendian(bool value) {

  is_bigendian_ = value;
}
inline void Image::set_is_bigendian(bool value) {
  _internal_set_is_bigendian(value);
  // @@protoc_insertion_point(field_set:steward.Image.is_bigendian)
}

// uint32 step = 6;
inline void Image::clear_step() {
  step_ = 0u;
}
inline ::PROTOBUF_NAMESPACE_ID::uint32 Image::_internal_step() const {
  return step_;
}
inline ::PROTOBUF_NAMESPACE_ID::uint32 Image::step() const {
  // @@protoc_insertion_point(field_get:steward.Image.step)
  return _internal_step();
}
inline void Image::_internal_set_step(::PROTOBUF_NAMESPACE_ID::uint32 value) {

  step_ = value;
}
inline void Image::set_step(::PROTOBUF_NAMESPACE_ID::uint32 value) {
  _internal_set_step(value);
  // @@protoc_insertion_point(field_set:steward.Image.step)
}

// bytes data = 7;
inline void Image::clear_data() {
  data_.ClearToEmpty(&::PROTOBUF_NAMESPACE_ID::internal::GetEmptyStringAlreadyInited(), GetArena());
}
inline const std::string& Image::data() const {
  // @@protoc_insertion_point(field_get:steward.Image.data)
  return _internal_data();
}
inline void Image::set_data(const std::string& value) {
  _internal_set_data(value);
  // @@protoc_insertion_point(field_set:steward.Image.data)
}
inline std::string* Image::mutable_data() {
  // @@protoc_insertion_point(field_mutable:steward.Image.data)
  return _internal_mutable_data();
}
inline const std::string& Image::_internal_data() const {
  return data_.Get();
}
inline void Image::_internal_set_data(const std::string& value) {

  data_.Set(&::PROTOBUF_NAMESPACE_ID::internal::GetEmptyStringAlreadyInited(), value, GetArena());
}
inline void Image::set_data(std::string&& value) {

  data_.Set(
    &::PROTOBUF_NAMESPACE_ID::internal::GetEmptyStringAlreadyInited(), ::std::move(value), GetArena());
  // @@protoc_insertion_point(field_set_rvalue:steward.Image.data)
}
inline void Image::set_data(const char* value) {
  GOOGLE_DCHECK(value != nullptr);

  data_.Set(&::PROTOBUF_NAMESPACE_ID::internal::GetEmptyStringAlreadyInited(), ::std::string(value),
              GetArena());
  // @@protoc_insertion_point(field_set_char:steward.Image.data)
}
inline void Image::set_data(const void* value,
    size_t size) {

  data_.Set(&::PROTOBUF_NAMESPACE_ID::internal::GetEmptyStringAlreadyInited(), ::std::string(
      reinterpret_cast<const char*>(value), size), GetArena());
  // @@protoc_insertion_point(field_set_pointer:steward.Image.data)
}
inline std::string* Image::_internal_mutable_data() {

  return data_.Mutable(&::PROTOBUF_NAMESPACE_ID::internal::GetEmptyStringAlreadyInited(), GetArena());
}
inline std::string* Image::release_data() {
  // @@protoc_insertion_point(field_release:steward.Image.data)
  return data_.Release(&::PROTOBUF_NAMESPACE_ID::internal::GetEmptyStringAlreadyInited(), GetArena());
}
inline void Image::set_allocated_data(std::string* data) {
  if (data != nullptr) {

  } else {

  }
  data_.SetAllocated(&::PROTOBUF_NAMESPACE_ID::internal::GetEmptyStringAlreadyInited(), data,
      GetArena());
  // @@protoc_insertion_point(field_set_allocated:steward.Image.data)
}
inline std::string* Image::unsafe_arena_release_data() {
  // @@protoc_insertion_point(field_unsafe_arena_release:steward.Image.data)
  GOOGLE_DCHECK(GetArena() != nullptr);

  return data_.UnsafeArenaRelease(&::PROTOBUF_NAMESPACE_ID::internal::GetEmptyStringAlreadyInited(),
      GetArena());
}
inline void Image::unsafe_arena_set_allocated_data(
    std::string* data) {
  GOOGLE_DCHECK(GetArena() != nullptr);
  if (data != nullptr) {

  } else {

  }
  data_.UnsafeArenaSetAllocated(&::PROTOBUF_NAMESPACE_ID::internal::GetEmptyStringAlreadyInited(),
      data, GetArena());
  // @@protoc_insertion_point(field_unsafe_arena_set_allocated:steward.Image.data)
}

#ifdef __GNUC__
  #pragma GCC diagnostic pop
#endif  // __GNUC__

// @@protoc_insertion_point(namespace_scope)

}  // namespace steward

// @@protoc_insertion_point(global_scope)

#include <google/protobuf/port_undef.inc>
#endif  // GOOGLE_PROTOBUF_INCLUDED_GOOGLE_PROTOBUF_INCLUDED_proto_2fsensor_5fmsgs_2fImage_2eproto
