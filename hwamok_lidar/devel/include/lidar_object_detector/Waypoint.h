// Generated by gencpp from file lidar_object_detector/Waypoint.msg
// DO NOT EDIT!


#ifndef LIDAR_OBJECT_DETECTOR_MESSAGE_WAYPOINT_H
#define LIDAR_OBJECT_DETECTOR_MESSAGE_WAYPOINT_H


#include <string>
#include <vector>
#include <memory>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace lidar_object_detector
{
template <class ContainerAllocator>
struct Waypoint_
{
  typedef Waypoint_<ContainerAllocator> Type;

  Waypoint_()
    : cnt(0)
    , x_arr()
    , y_arr()  {
      x_arr.assign(0.0);

      y_arr.assign(0.0);
  }
  Waypoint_(const ContainerAllocator& _alloc)
    : cnt(0)
    , x_arr()
    , y_arr()  {
  (void)_alloc;
      x_arr.assign(0.0);

      y_arr.assign(0.0);
  }



   typedef int32_t _cnt_type;
  _cnt_type cnt;

   typedef boost::array<float, 200>  _x_arr_type;
  _x_arr_type x_arr;

   typedef boost::array<float, 200>  _y_arr_type;
  _y_arr_type y_arr;





  typedef boost::shared_ptr< ::lidar_object_detector::Waypoint_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::lidar_object_detector::Waypoint_<ContainerAllocator> const> ConstPtr;

}; // struct Waypoint_

typedef ::lidar_object_detector::Waypoint_<std::allocator<void> > Waypoint;

typedef boost::shared_ptr< ::lidar_object_detector::Waypoint > WaypointPtr;
typedef boost::shared_ptr< ::lidar_object_detector::Waypoint const> WaypointConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::lidar_object_detector::Waypoint_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::lidar_object_detector::Waypoint_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::lidar_object_detector::Waypoint_<ContainerAllocator1> & lhs, const ::lidar_object_detector::Waypoint_<ContainerAllocator2> & rhs)
{
  return lhs.cnt == rhs.cnt &&
    lhs.x_arr == rhs.x_arr &&
    lhs.y_arr == rhs.y_arr;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::lidar_object_detector::Waypoint_<ContainerAllocator1> & lhs, const ::lidar_object_detector::Waypoint_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace lidar_object_detector

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::lidar_object_detector::Waypoint_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::lidar_object_detector::Waypoint_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::lidar_object_detector::Waypoint_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::lidar_object_detector::Waypoint_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::lidar_object_detector::Waypoint_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::lidar_object_detector::Waypoint_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::lidar_object_detector::Waypoint_<ContainerAllocator> >
{
  static const char* value()
  {
    return "ef3b913564c2c1d5123a607b0c2cca14";
  }

  static const char* value(const ::lidar_object_detector::Waypoint_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xef3b913564c2c1d5ULL;
  static const uint64_t static_value2 = 0x123a607b0c2cca14ULL;
};

template<class ContainerAllocator>
struct DataType< ::lidar_object_detector::Waypoint_<ContainerAllocator> >
{
  static const char* value()
  {
    return "lidar_object_detector/Waypoint";
  }

  static const char* value(const ::lidar_object_detector::Waypoint_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::lidar_object_detector::Waypoint_<ContainerAllocator> >
{
  static const char* value()
  {
    return "int32 cnt\n"
"float32[200] x_arr\n"
"float32[200] y_arr\n"
;
  }

  static const char* value(const ::lidar_object_detector::Waypoint_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::lidar_object_detector::Waypoint_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.cnt);
      stream.next(m.x_arr);
      stream.next(m.y_arr);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct Waypoint_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::lidar_object_detector::Waypoint_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::lidar_object_detector::Waypoint_<ContainerAllocator>& v)
  {
    s << indent << "cnt: ";
    Printer<int32_t>::stream(s, indent + "  ", v.cnt);
    s << indent << "x_arr[]" << std::endl;
    for (size_t i = 0; i < v.x_arr.size(); ++i)
    {
      s << indent << "  x_arr[" << i << "]: ";
      Printer<float>::stream(s, indent + "  ", v.x_arr[i]);
    }
    s << indent << "y_arr[]" << std::endl;
    for (size_t i = 0; i < v.y_arr.size(); ++i)
    {
      s << indent << "  y_arr[" << i << "]: ";
      Printer<float>::stream(s, indent + "  ", v.y_arr[i]);
    }
  }
};

} // namespace message_operations
} // namespace ros

#endif // LIDAR_OBJECT_DETECTOR_MESSAGE_WAYPOINT_H
