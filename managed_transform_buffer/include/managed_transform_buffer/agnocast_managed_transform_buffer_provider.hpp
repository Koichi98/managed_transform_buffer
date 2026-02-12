// Copyright 2024 The Autoware Foundation
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
//
// Co-developed by Tier IV, Inc.

#ifndef MANAGED_TRANSFORM_BUFFER__AGNOCAST_MANAGED_TRANSFORM_BUFFER_PROVIDER_HPP_
#define MANAGED_TRANSFORM_BUFFER__AGNOCAST_MANAGED_TRANSFORM_BUFFER_PROVIDER_HPP_

#include "managed_transform_buffer/managed_transform_buffer_provider.hpp"

#include <agnocast/agnocast.hpp>
#include <agnocast/node/agnocast_node.hpp>
#include <agnocast/node/agnocast_only_single_threaded_executor.hpp>
#include <agnocast/node/tf2/buffer.hpp>
#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_msgs/msg/tf_message.hpp>

#include <atomic>
#include <cstddef>
#include <memory>
#include <mutex>
#include <optional>
#include <random>
#include <shared_mutex>
#include <string>
#include <thread>
#include <unordered_map>
#include <utility>

namespace managed_transform_buffer
{

/**
 * @brief A managed TF buffer provider for agnocast::Node with use of singleton pattern.
 */
class AgnocastManagedTransformBufferProvider
{
public:
  /** @brief Get the instance of the AgnocastManagedTransformBufferProvider
   *
   * @return the instance of the AgnocastManagedTransformBufferProvider
   */
  static AgnocastManagedTransformBufferProvider & getInstance(
    rcl_clock_type_t clock_type, const bool force_dynamic, tf2::Duration discovery_timeout,
    tf2::Duration cache_time);

  AgnocastManagedTransformBufferProvider(const AgnocastManagedTransformBufferProvider &) = delete;
  AgnocastManagedTransformBufferProvider & operator=(const AgnocastManagedTransformBufferProvider &) =
    delete;

  /** @brief Destroy the Agnocast Managed Transform Buffer object */
  ~AgnocastManagedTransformBufferProvider();

  /**
   * @brief Get the transform between two frames by frame ID.
   *
   * @param[in] target_frame the frame to which data should be transformed
   * @param[in] source_frame the frame where the data originated
   * @param[in] time the time at which the value of the transform is desired
   * @param[in] timeout how long to block before failing
   * @param[in] logger logger, if not specified, default logger will be used
   * @return an optional containing the transform if successful, or empty if not
   */
  std::optional<TransformStamped> getTransform(
    const std::string & target_frame, const std::string & source_frame, const tf2::TimePoint & time,
    const tf2::Duration & timeout, const rclcpp::Logger & logger);

  /** @brief Check if all TFs requests have been for static TF so far.
   *
   * @return true if only static TFs have been requested
   */
  bool isStatic() const;

  /** @brief Get clock.
   *
   * @return the clock
   */
  rclcpp::Clock::SharedPtr getClock() const;

private:
  /**
   * @brief Construct a new Agnocast Managed Transform Buffer object
   *
   * @param[in] clock_type type of the clock
   * @param[in] discovery_timeout how long to wait for first TF discovery
   * @param[in] cache_time how long to keep a history of transforms
   */
  explicit AgnocastManagedTransformBufferProvider(
    rcl_clock_type_t clock_type, tf2::Duration discovery_timeout, tf2::Duration cache_time);

  /** @brief Initialize TF listener */
  void activateListener();

  /** @brief Deactivate TF listener */
  void deactivateListener();

  /** @brief Generate a unique node name
   *
   * @return a unique node name
   */
  std::string generateUniqueNodeName();

  /** @brief Callback for TF messages
   *
   * @param[in] msg the TF message
   * @param[in] is_static whether the TF topic refers to static transforms
   */
  void tfCallback(agnocast::ipc_shared_ptr<tf2_msgs::msg::TFMessage> && msg, const bool is_static);

  /** @brief Default ROS-ish lookupTransform trigger.
   *
   * @param[in] target_frame the frame to which data should be transformed
   * @param[in] source_frame the frame where the data originated
   * @param[in] time the time at which the value of the transform is desired (0 will get the latest)
   * @param[in] timeout how long to block before failing
   * @param[in] logger logger, if not specified, default logger will be used
   * @return an optional containing the transform if successful, or empty if not
   */
  std::optional<TransformStamped> lookupTransform(
    const std::string & target_frame, const std::string & source_frame, const tf2::TimePoint & time,
    const tf2::Duration & timeout, const rclcpp::Logger & logger) const;

  /** @brief Traverse TF tree built by local TF listener.
   *
   * @param[in] target_frame the frame to which data should be transformed
   * @param[in] source_frame the frame where the data originated
   * @param[in] logger logger, if not specified, default logger will be used
   * @return a traverse result indicating if the transform is possible and if it is static
   */
  TraverseResult traverseTree(
    const std::string & target_frame, const std::string & source_frame,
    const rclcpp::Logger & logger);

  /** @brief Get a dynamic transform from the TF buffer.
   *
   * @param[in] target_frame the frame to which data should be transformed
   * @param[in] source_frame the frame where the data originated
   * @param[in] time the time at which the value of the transform is desired (0 will get the latest)
   * @param[in] timeout how long to block before
   * @param[in] logger logger, if not specified, default logger will be used
   * @return an optional containing the transform if successful, or empty if not
   */
  std::optional<TransformStamped> getDynamicTransform(
    const std::string & target_frame, const std::string & source_frame, const tf2::TimePoint & time,
    const tf2::Duration & timeout, const rclcpp::Logger & logger) const;

  /** @brief Get a static transform from local TF buffer.
   *
   * @param[in] target_frame the frame to which data should be transformed
   * @param[in] source_frame the frame where the data originated
   * @param[in] time the time to be assigned to static transform
   * @return an optional containing the transform if successful, or empty if not
   */
  std::optional<TransformStamped> getStaticTransform(
    const std::string & target_frame, const std::string & source_frame,
    const tf2::TimePoint & time);

  /** @brief Get an unknown (static or dynamic) transform.
   *
   * @param[in] target_frame the frame to which data should be transformed
   * @param[in] source_frame the frame where the data originated
   * @param[in] time the time at which the value of the transform is desired (0 will get the latest)
   * @param[in] timeout how long to block before failing
   * @param[in] logger logger, if not specified, default logger will be used
   * @return an optional containing the transform if successful, or empty if not
   */
  std::optional<TransformStamped> getUnknownTransform(
    const std::string & target_frame, const std::string & source_frame, const tf2::TimePoint & time,
    const tf2::Duration & timeout, const rclcpp::Logger & logger);

  static std::unique_ptr<AgnocastManagedTransformBufferProvider> instance;
  std::shared_ptr<agnocast::Node> node_{nullptr};
  rclcpp::Clock::SharedPtr clock_{nullptr};
  rclcpp::CallbackGroup::SharedPtr callback_group_{nullptr};
  rclcpp::NodeOptions options_;
  agnocast::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr tf_static_sub_{nullptr};
  agnocast::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr tf_sub_{nullptr};
  agnocast::SubscriptionOptions tf_options_;
  agnocast::SubscriptionOptions tf_static_options_;
  std::shared_ptr<agnocast::AgnocastOnlySingleThreadedExecutor> executor_{nullptr};
  std::shared_ptr<std::thread> executor_thread_{nullptr};
  std::unique_ptr<agnocast::Buffer> tf_buffer_;
  std::unique_ptr<TFMap> static_tf_buffer_;
  std::unique_ptr<TreeMap> tf_tree_;
  std::mt19937 random_engine_;
  std::uniform_int_distribution<> dis_;
  std::shared_mutex buffer_mutex_;
  std::shared_mutex tree_mutex_;
  std::mutex listener_mutex_;
  std::atomic<bool> is_static_{true};
  std::atomic<std::size_t> operational_threads_{0};
  tf2::Duration discovery_timeout_;
  rclcpp::Logger logger_;
};

}  // namespace managed_transform_buffer

#endif  // MANAGED_TRANSFORM_BUFFER__AGNOCAST_MANAGED_TRANSFORM_BUFFER_PROVIDER_HPP_
