// Generated by gencpp from file exploration_planner/String_cmd.msg
// DO NOT EDIT!


#ifndef EXPLORATION_PLANNER_MESSAGE_STRING_CMD_H
#define EXPLORATION_PLANNER_MESSAGE_STRING_CMD_H

#include <ros/service_traits.h>


#include <exploration_planner/String_cmdRequest.h>
#include <exploration_planner/String_cmdResponse.h>


namespace exploration_planner
{

struct String_cmd
{

typedef String_cmdRequest Request;
typedef String_cmdResponse Response;
Request request;
Response response;

typedef Request RequestType;
typedef Response ResponseType;

}; // struct String_cmd
} // namespace exploration_planner


namespace ros
{
namespace service_traits
{


template<>
struct MD5Sum< ::exploration_planner::String_cmd > {
  static const char* value()
  {
    return "d4463b49bd5bb77dbd8c4356f5dc1c28";
  }

  static const char* value(const ::exploration_planner::String_cmd&) { return value(); }
};

template<>
struct DataType< ::exploration_planner::String_cmd > {
  static const char* value()
  {
    return "exploration_planner/String_cmd";
  }

  static const char* value(const ::exploration_planner::String_cmd&) { return value(); }
};


// service_traits::MD5Sum< ::exploration_planner::String_cmdRequest> should match 
// service_traits::MD5Sum< ::exploration_planner::String_cmd > 
template<>
struct MD5Sum< ::exploration_planner::String_cmdRequest>
{
  static const char* value()
  {
    return MD5Sum< ::exploration_planner::String_cmd >::value();
  }
  static const char* value(const ::exploration_planner::String_cmdRequest&)
  {
    return value();
  }
};

// service_traits::DataType< ::exploration_planner::String_cmdRequest> should match 
// service_traits::DataType< ::exploration_planner::String_cmd > 
template<>
struct DataType< ::exploration_planner::String_cmdRequest>
{
  static const char* value()
  {
    return DataType< ::exploration_planner::String_cmd >::value();
  }
  static const char* value(const ::exploration_planner::String_cmdRequest&)
  {
    return value();
  }
};

// service_traits::MD5Sum< ::exploration_planner::String_cmdResponse> should match 
// service_traits::MD5Sum< ::exploration_planner::String_cmd > 
template<>
struct MD5Sum< ::exploration_planner::String_cmdResponse>
{
  static const char* value()
  {
    return MD5Sum< ::exploration_planner::String_cmd >::value();
  }
  static const char* value(const ::exploration_planner::String_cmdResponse&)
  {
    return value();
  }
};

// service_traits::DataType< ::exploration_planner::String_cmdResponse> should match 
// service_traits::DataType< ::exploration_planner::String_cmd > 
template<>
struct DataType< ::exploration_planner::String_cmdResponse>
{
  static const char* value()
  {
    return DataType< ::exploration_planner::String_cmd >::value();
  }
  static const char* value(const ::exploration_planner::String_cmdResponse&)
  {
    return value();
  }
};

} // namespace service_traits
} // namespace ros

#endif // EXPLORATION_PLANNER_MESSAGE_STRING_CMD_H
