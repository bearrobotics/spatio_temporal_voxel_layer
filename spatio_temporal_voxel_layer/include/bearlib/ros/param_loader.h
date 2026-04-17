/**
 * Copyright [2023] Bear Robotics
 * Licensed under the LGPL v2.1 License.
 *
 * File: param_loader.h
 * Helper functions for loading parameters from the ROS parameter server
 * and reporting errors when parameters are missing or have wrong types.
 */
#ifndef BEARLIB_ROS_PARAM_LOADER_H
#define BEARLIB_ROS_PARAM_LOADER_H
#include <ros/ros.h>
#include <xmlrpcpp/XmlRpcValue.h>

#include <boost/core/demangle.hpp>
#include <boost/optional.hpp>
#include <sstream>
#include <string>

#include "ros/node_handle.h"
namespace bear {
namespace lib {
namespace ros {
using NodeHandle = ::ros::NodeHandle;

namespace detail {

static std::string XmlRpcValueTypeToString(
    const XmlRpc::XmlRpcValue::Type &type) {
  switch (type) {
    case XmlRpc::XmlRpcValue::Type::TypeInvalid:
      return "invalid";
    case XmlRpc::XmlRpcValue::Type::TypeBoolean:
      return "boolean";
    case XmlRpc::XmlRpcValue::Type::TypeInt:
      return "int";
    case XmlRpc::XmlRpcValue::Type::TypeDouble:
      return "double";
    case XmlRpc::XmlRpcValue::Type::TypeString:
      return "string";
    case XmlRpc::XmlRpcValue::Type::TypeDateTime:
      return "dateTime";
    case XmlRpc::XmlRpcValue::Type::TypeBase64:
      return "base64";
    case XmlRpc::XmlRpcValue::Type::TypeArray:
      return "array";
    case XmlRpc::XmlRpcValue::Type::TypeStruct:
      return "struct";
    default:
      return "unknown";
  }
}

template <typename TReturn>
std::string GetFailureLog(const NodeHandle &param_nh,
                          const std::string &param_name, bool is_required) {
  std::string param_loader_type = is_required ? "Required" : "Optional";
  bool is_param_key_present = param_nh.hasParam(param_name);
  std::stringstream log;
  log << "In Path \"" << param_nh.getNamespace() << "\"";
  if (is_param_key_present) {
    XmlRpc::XmlRpcValue xml_value;
    param_nh.getParam(param_name, xml_value);
    auto type = xml_value.getType();
    log << " Wrong Type for " << param_loader_type << " Param: '" << param_name
        << "' Expected: '" << boost::core::demangle(typeid(TReturn).name())
        << "' Got: '" << detail::XmlRpcValueTypeToString(type) << "'";
  } else {
    log << " Missing " << param_loader_type << " Param: '" << param_name << "'";
  }
  return log.str();
}

}  // namespace detail

/**
 * @brief Helper function that loads a param from the ROS param server and
 * returns the param if successful. If the required
 * param is not loaded, it logs the error and shuts down the ROS node.
 * @tparam TReturn param type
 * @param param_nh node handle that is used for param retrieval
 * @param param_name key for the required param
 * @return param from server if succesfully
 */
template <typename TReturn>
TReturn LoadRequiredParam(const NodeHandle &param_nh,
                          const std::string &param_name) {
  TReturn param;
  if (!param_nh.getParam(param_name, param)) {
    std::string log =
        detail::GetFailureLog<TReturn>(param_nh, param_name, true);
    // TODO: Want this to be fatal, but the exit code is consumed by
    //.   bazel, breaking the test
    ROS_ERROR_STREAM(log);
    ::ros::shutdown();
  }
  return param;
}

/**
 * @brief Helper function that loads a param from the ROS param server and
 * returns the param into a function for handling if successful. If the optional
 * param is not loaded, it logs the missing param as INFO
 * @tparam TReturn param type
 * @param param_nh node handle that is used for param retrieval
 * @param param_name key for the optional param
 * @return boost::optional<TReturn> param, none if param is not present in
 * server
 *
 */
template <typename TReturn>
boost::optional<TReturn> LoadOptionalParam(const NodeHandle &param_nh,
                                           const std::string &param_name) {
  TReturn param;
  if (!param_nh.getParam(param_name, param)) {
    std::string log =
        detail::GetFailureLog<TReturn>(param_nh, param_name, false);
    ROS_INFO_STREAM(log);
    return boost::none;
  }
  return param;
}

/**
 * @brief Helper function that loads a param from the local cache after the
 * first request and returns the param if successful. If the required param is
 * not loaded, it logs the error and shuts down the ROS node.
 * @tparam TReturn param type
 * @param param_nh node handle that is used for param retrieval
 * @param param_name key for the required param
 * @return param from local cache if succesfully
 */
template <typename TReturn>
TReturn LoadRequiredCachedParam(const NodeHandle &param_nh,
                                const std::string &param_name) {
  TReturn param;
  if (!param_nh.getParamCached(param_name, param)) {
    std::string log =
        detail::GetFailureLog<TReturn>(param_nh, param_name, true);
    // TODO: Want this to be fatal, but the exit code is consumed by
    //.   bazel, breaking the test
    ROS_ERROR_STREAM(log);
    ::ros::shutdown();
  }
  return param;
}

}  // namespace ros
}  // namespace lib
}  // namespace bear
#endif  // BEARLIB_ROS_PARAM_LOADER_H
