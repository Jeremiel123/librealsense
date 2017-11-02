// Generated by gencpp from file realsense_legacy_msgs/controller_event.msg
// DO NOT EDIT!


#ifndef realsense_legacy_msgs_MESSAGE_CONTROLLER_EVENT_H
#define realsense_legacy_msgs_MESSAGE_CONTROLLER_EVENT_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace realsense_legacy_msgs
{
template <class ContainerAllocator>
struct controller_event_
{
  typedef controller_event_<ContainerAllocator> Type;

  controller_event_()
    : controller_id(0)
    , type(0)
    , mac_address()
    , timestamp()  {
      mac_address.assign(0);
  }
  controller_event_(const ContainerAllocator& _alloc)
    : controller_id(0)
    , type(0)
    , mac_address()
    , timestamp()  {
  (void)_alloc;
      mac_address.assign(0);
  }



   typedef uint8_t _controller_id_type;
  _controller_id_type controller_id;

   typedef uint32_t _type_type;
  _type_type type;

   typedef std::array<uint8_t, 6>  _mac_address_type;
  _mac_address_type mac_address;

   typedef ros::Time _timestamp_type;
  _timestamp_type timestamp;




  typedef std::shared_ptr< ::realsense_legacy_msgs::controller_event_<ContainerAllocator> > Ptr;
  typedef std::shared_ptr< ::realsense_legacy_msgs::controller_event_<ContainerAllocator> const> ConstPtr;

}; // struct controller_event_

typedef ::realsense_legacy_msgs::controller_event_<std::allocator<void> > controller_event;

typedef std::shared_ptr< ::realsense_legacy_msgs::controller_event > controller_eventPtr;
typedef std::shared_ptr< ::realsense_legacy_msgs::controller_event const> controller_eventConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::realsense_legacy_msgs::controller_event_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::realsense_legacy_msgs::controller_event_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace realsense_legacy_msgs

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': True, 'IsMessage': True, 'HasHeader': False}
// {'sensor_msgs': ['/opt/ros/kinetic/share/sensor_msgs/cmake/../msg'], 'geometry_msgs': ['/opt/ros/kinetic/share/geometry_msgs/cmake/../msg'], 'std_msgs': ['/opt/ros/kinetic/share/std_msgs/cmake/../msg'], 'realsense_legacy_msgs': ['/home/administrator/intelrealsense/realsense_sdk_internal/sdk_internal/tools/realsense_legacy_msgs_generator/msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::realsense_legacy_msgs::controller_event_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::realsense_legacy_msgs::controller_event_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::realsense_legacy_msgs::controller_event_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::realsense_legacy_msgs::controller_event_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::realsense_legacy_msgs::controller_event_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::realsense_legacy_msgs::controller_event_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::realsense_legacy_msgs::controller_event_<ContainerAllocator> >
{
  static const char* value()
  {
    return "2cbcc2243504f0f4d13d886a9bafd85f";
  }

  static const char* value(const ::realsense_legacy_msgs::controller_event_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x2cbcc2243504f0f4ULL;
  static const uint64_t static_value2 = 0xd13d886a9bafd85fULL;
};

template<class ContainerAllocator>
struct DataType< ::realsense_legacy_msgs::controller_event_<ContainerAllocator> >
{
  static const char* value()
  {
    return "realsense_legacy_msgs/controller_event";
  }

  static const char* value(const ::realsense_legacy_msgs::controller_event_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::realsense_legacy_msgs::controller_event_<ContainerAllocator> >
{
  static const char* value()
  {
    return "uint8 controller_id\n\
uint32 type\n\
uint8[6] mac_address\n\
time timestamp\n\
";
  }

  static const char* value(const ::realsense_legacy_msgs::controller_event_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::realsense_legacy_msgs::controller_event_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.controller_id);
      stream.next(m.type);
      stream.next(m.mac_address);
      stream.next(m.timestamp);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER;
  }; // struct controller_event_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::realsense_legacy_msgs::controller_event_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::realsense_legacy_msgs::controller_event_<ContainerAllocator>& v)
  {
    s << indent << "controller_id: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.controller_id);
    s << indent << "type: ";
    Printer<uint32_t>::stream(s, indent + "  ", v.type);
    s << indent << "mac_address[]" << std::endl;
    for (size_t i = 0; i < v.mac_address.size(); ++i)
    {
      s << indent << "  mac_address[" << i << "]: ";
      Printer<uint8_t>::stream(s, indent + "  ", v.mac_address[i]);
    }
    s << indent << "timestamp: ";
    Printer<ros::Time>::stream(s, indent + "  ", v.timestamp);
  }
};

} // namespace message_operations
} // namespace ros

#endif // realsense_legacy_msgs_MESSAGE_CONTROLLER_EVENT_H
