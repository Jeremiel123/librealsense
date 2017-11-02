// Generated by gencpp from file realsense_legacy_msgs/compressed_frame_info.msg
// DO NOT EDIT!


#ifndef realsense_legacy_msgs_MESSAGE_COMPRESSED_FRAME_INFO_H
#define realsense_legacy_msgs_MESSAGE_COMPRESSED_FRAME_INFO_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <realsense_legacy_msgs/metadata.h>

namespace realsense_legacy_msgs
{
template <class ContainerAllocator>
struct compressed_frame_info_
{
  typedef compressed_frame_info_<ContainerAllocator> Type;

  compressed_frame_info_()
    : system_time(0)
    , frame_metadata()
    , time_stamp_domain(0)
    , width(0)
    , height(0)
    , encoding()
    , step(0)  {
    }
  compressed_frame_info_(const ContainerAllocator& _alloc)
    : system_time(0)
    , frame_metadata(_alloc)
    , time_stamp_domain(0)
    , width(0)
    , height(0)
    , encoding(_alloc)
    , step(0)  {
  (void)_alloc;
    }



   typedef uint64_t _system_time_type;
  _system_time_type system_time;

   typedef std::vector< ::realsense_legacy_msgs::metadata_<ContainerAllocator> , typename ContainerAllocator::template rebind< ::realsense_legacy_msgs::metadata_<ContainerAllocator> >::other >  _frame_metadata_type;
  _frame_metadata_type frame_metadata;

   typedef uint32_t _time_stamp_domain_type;
  _time_stamp_domain_type time_stamp_domain;

   typedef uint32_t _width_type;
  _width_type width;

   typedef uint32_t _height_type;
  _height_type height;

   typedef std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  _encoding_type;
  _encoding_type encoding;

   typedef uint32_t _step_type;
  _step_type step;




  typedef std::shared_ptr< ::realsense_legacy_msgs::compressed_frame_info_<ContainerAllocator> > Ptr;
  typedef std::shared_ptr< ::realsense_legacy_msgs::compressed_frame_info_<ContainerAllocator> const> ConstPtr;

}; // struct compressed_frame_info_

typedef ::realsense_legacy_msgs::compressed_frame_info_<std::allocator<void> > compressed_frame_info;

typedef std::shared_ptr< ::realsense_legacy_msgs::compressed_frame_info > compressed_frame_infoPtr;
typedef std::shared_ptr< ::realsense_legacy_msgs::compressed_frame_info const> compressed_frame_infoConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::realsense_legacy_msgs::compressed_frame_info_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::realsense_legacy_msgs::compressed_frame_info_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace realsense_legacy_msgs

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': False, 'IsMessage': True, 'HasHeader': False}
// {'sensor_msgs': ['/opt/ros/kinetic/share/sensor_msgs/cmake/../msg'], 'geometry_msgs': ['/opt/ros/kinetic/share/geometry_msgs/cmake/../msg'], 'std_msgs': ['/opt/ros/kinetic/share/std_msgs/cmake/../msg'], 'realsense_legacy_msgs': ['/home/administrator/ros/realsense_sdk_internal/sdk_internal/tools/realsense_legacy_msgs_generator/msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::realsense_legacy_msgs::compressed_frame_info_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::realsense_legacy_msgs::compressed_frame_info_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::realsense_legacy_msgs::compressed_frame_info_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::realsense_legacy_msgs::compressed_frame_info_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::realsense_legacy_msgs::compressed_frame_info_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::realsense_legacy_msgs::compressed_frame_info_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::realsense_legacy_msgs::compressed_frame_info_<ContainerAllocator> >
{
  static const char* value()
  {
    return "64a56816b5cc611196b9f32528adbddc";
  }

  static const char* value(const ::realsense_legacy_msgs::compressed_frame_info_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x64a56816b5cc6111ULL;
  static const uint64_t static_value2 = 0x96b9f32528adbddcULL;
};

template<class ContainerAllocator>
struct DataType< ::realsense_legacy_msgs::compressed_frame_info_<ContainerAllocator> >
{
  static const char* value()
  {
    return "realsense_legacy_msgs/compressed_frame_info";
  }

  static const char* value(const ::realsense_legacy_msgs::compressed_frame_info_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::realsense_legacy_msgs::compressed_frame_info_<ContainerAllocator> >
{
  static const char* value()
  {
    return "uint64 system_time\n\
realsense_legacy_msgs/metadata[] frame_metadata\n\
uint32 time_stamp_domain\n\
uint32 width\n\
uint32 height	\n\
string encoding\n\
uint32 step\n\
\n\
================================================================================\n\
MSG: realsense_legacy_msgs/metadata\n\
uint32 type\n\
uint8[] data\n\
";
  }

  static const char* value(const ::realsense_legacy_msgs::compressed_frame_info_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::realsense_legacy_msgs::compressed_frame_info_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.system_time);
      stream.next(m.frame_metadata);
      stream.next(m.time_stamp_domain);
      stream.next(m.width);
      stream.next(m.height);
      stream.next(m.encoding);
      stream.next(m.step);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER;
  }; // struct compressed_frame_info_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::realsense_legacy_msgs::compressed_frame_info_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::realsense_legacy_msgs::compressed_frame_info_<ContainerAllocator>& v)
  {
    s << indent << "system_time: ";
    Printer<uint64_t>::stream(s, indent + "  ", v.system_time);
    s << indent << "frame_metadata[]" << std::endl;
    for (size_t i = 0; i < v.frame_metadata.size(); ++i)
    {
      s << indent << "  frame_metadata[" << i << "]: ";
      s << std::endl;
      s << indent;
      Printer< ::realsense_legacy_msgs::metadata_<ContainerAllocator> >::stream(s, indent + "    ", v.frame_metadata[i]);
    }
    s << indent << "time_stamp_domain: ";
    Printer<uint32_t>::stream(s, indent + "  ", v.time_stamp_domain);
    s << indent << "width: ";
    Printer<uint32_t>::stream(s, indent + "  ", v.width);
    s << indent << "height: ";
    Printer<uint32_t>::stream(s, indent + "  ", v.height);
    s << indent << "encoding: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::stream(s, indent + "  ", v.encoding);
    s << indent << "step: ";
    Printer<uint32_t>::stream(s, indent + "  ", v.step);
  }
};

} // namespace message_operations
} // namespace ros

#endif // realsense_legacy_msgs_MESSAGE_COMPRESSED_FRAME_INFO_H
