// Generated by gencpp from file beginner_tutorials/scan_range.msg
// DO NOT EDIT!


#ifndef BEGINNER_TUTORIALS_MESSAGE_SCAN_RANGE_H
#define BEGINNER_TUTORIALS_MESSAGE_SCAN_RANGE_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace beginner_tutorials
{
template <class ContainerAllocator>
struct scan_range_
{
  typedef scan_range_<ContainerAllocator> Type;

  scan_range_()
    : min(0.0)
    , max(0.0)  {
    }
  scan_range_(const ContainerAllocator& _alloc)
    : min(0.0)
    , max(0.0)  {
  (void)_alloc;
    }



   typedef double _min_type;
  _min_type min;

   typedef double _max_type;
  _max_type max;





  typedef boost::shared_ptr< ::beginner_tutorials::scan_range_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::beginner_tutorials::scan_range_<ContainerAllocator> const> ConstPtr;

}; // struct scan_range_

typedef ::beginner_tutorials::scan_range_<std::allocator<void> > scan_range;

typedef boost::shared_ptr< ::beginner_tutorials::scan_range > scan_rangePtr;
typedef boost::shared_ptr< ::beginner_tutorials::scan_range const> scan_rangeConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::beginner_tutorials::scan_range_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::beginner_tutorials::scan_range_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::beginner_tutorials::scan_range_<ContainerAllocator1> & lhs, const ::beginner_tutorials::scan_range_<ContainerAllocator2> & rhs)
{
  return lhs.min == rhs.min &&
    lhs.max == rhs.max;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::beginner_tutorials::scan_range_<ContainerAllocator1> & lhs, const ::beginner_tutorials::scan_range_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace beginner_tutorials

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::beginner_tutorials::scan_range_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::beginner_tutorials::scan_range_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::beginner_tutorials::scan_range_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::beginner_tutorials::scan_range_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::beginner_tutorials::scan_range_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::beginner_tutorials::scan_range_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::beginner_tutorials::scan_range_<ContainerAllocator> >
{
  static const char* value()
  {
    return "32e1c0b6f254bb48e963512143e9aa6f";
  }

  static const char* value(const ::beginner_tutorials::scan_range_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x32e1c0b6f254bb48ULL;
  static const uint64_t static_value2 = 0xe963512143e9aa6fULL;
};

template<class ContainerAllocator>
struct DataType< ::beginner_tutorials::scan_range_<ContainerAllocator> >
{
  static const char* value()
  {
    return "beginner_tutorials/scan_range";
  }

  static const char* value(const ::beginner_tutorials::scan_range_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::beginner_tutorials::scan_range_<ContainerAllocator> >
{
  static const char* value()
  {
    return "float64 min\n"
"float64 max\n"
;
  }

  static const char* value(const ::beginner_tutorials::scan_range_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::beginner_tutorials::scan_range_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.min);
      stream.next(m.max);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct scan_range_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::beginner_tutorials::scan_range_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::beginner_tutorials::scan_range_<ContainerAllocator>& v)
  {
    s << indent << "min: ";
    Printer<double>::stream(s, indent + "  ", v.min);
    s << indent << "max: ";
    Printer<double>::stream(s, indent + "  ", v.max);
  }
};

} // namespace message_operations
} // namespace ros

#endif // BEGINNER_TUTORIALS_MESSAGE_SCAN_RANGE_H
