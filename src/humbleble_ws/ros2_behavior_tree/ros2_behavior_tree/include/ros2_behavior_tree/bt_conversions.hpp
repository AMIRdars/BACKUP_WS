// Copyright (c) 2019 Intel Corporation
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef ROS2_BEHAVIOR_TREE__BT_CONVERSIONS_HPP_
#define ROS2_BEHAVIOR_TREE__BT_CONVERSIONS_HPP_

#include "behaviortree_cpp_v3/behavior_tree.h"

// The follow templates are required when using these types as parameters
// in our BT XML files. They parse the strings in the XML into their corresponding
// data types.

namespace BT
{

template<>
inline std::chrono::milliseconds convertFromString<std::chrono::milliseconds>(const StringView key)
{
  return std::chrono::milliseconds(std::stoul(key.data()));
}

template<>
inline int64_t convertFromString<int64_t>(const StringView key)
{
  return std::strtoll(key.data(), NULL, 10);
}


// We want to be able to use this custom type
struct Position2D 
{ 
  double x;
  double y; 
};

template <> 
inline Position2D convertFromString(StringView str)
{
    // The next line should be removed...
    printf("Converting string: \"%s\"\n", str.data() );

    // We expect real numbers separated by semicolons
    auto parts = splitString(str, ';');
    if (parts.size() != 2)
    {
        throw RuntimeError("invalid input)");
    }
    else{
        Position2D output;
        output.x     = convertFromString<double>(parts[0]);
        output.y     = convertFromString<double>(parts[1]);
        return output;
    }
}


struct RosParam 
{ 
  std::string param_name;
  bool parameter_bool;
  int parameter_integer;
  double parameter_double;
  std::string parameter_string;
};

template <> 
inline RosParam convertFromString(StringView str)
{
    // The next line should be removed...
    printf("Converting string: \"%s\"\n", str.data() );

    // We expect real numbers separated by semicolons
    auto parts = splitString(str, ';');
    if (parts.size() != 2)
    {
        throw RuntimeError("invalid input)");
    }
    else{
        RosParam output;
        output.param_name     = convertFromString<std::string>(parts[0]);
        std::cout << typeid(parts[1]).name() << std::endl;
        

        // switch (bt_root_status_) {
        //   case BT::NodeStatus::IDLE:
        //     bt_status_.root_status.status = behavior_tree_msgs::msg::NodeStatus::IDLE;
        //     break;
        //   case BT::NodeStatus::RUNNING:
        //     bt_status_.root_status.status = behavior_tree_msgs::msg::NodeStatus::RUNNING;
        //     break;

        //   case BT::NodeStatus::SUCCESS:
        //     bt_status_.root_status.status = behavior_tree_msgs::msg::NodeStatus::SUCCESS;
        //     break;

        //   case BT::NodeStatus::FAILURE:
        //     bt_status_.root_status.status = behavior_tree_msgs::msg::NodeStatus::FAILURE;
        //     break;

        //   default:
        //     throw std::logic_error("Invalid return value from the BT");
        // }
        return output;
    }
}

}  // namespace BT

#endif  // ROS2_BEHAVIOR_TREE__BT_CONVERSIONS_HPP_
