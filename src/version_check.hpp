#pragma once

#if __has_include(<rclcpp/version.h>)
#include <rclcpp/version.h>
#if (RCLCPP_VERSION_MAJOR >= 5)
#define CALLBACK_GROUP_SUPPORTED
#endif
#endif
