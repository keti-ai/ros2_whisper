#include "whisper_nodes/inference_node.hpp"

namespace whisper {
InferenceNode::InferenceNode(const rclcpp::Node::SharedPtr node_ptr)
    : node_ptr_(node_ptr), episodic_buffer_(node_ptr_->get_node_logging_interface()),
      language_("en") {
  declare_parameters_();

  auto cb_group = node_ptr_->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
  rclcpp::SubscriptionOptions options;
  options.callback_group = cb_group;

  // audio subscription
  audio_sub_ = node_ptr_->create_subscription<std_msgs::msg::Int16MultiArray>(
      "audio", 10, std::bind(&InferenceNode::on_audio_, this, std::placeholders::_1), options);

  // inference action server
  inference_action_server_ = rclcpp_action::create_server<Inference>(
      node_ptr_, "inference",
      std::bind(&InferenceNode::on_inference_, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&InferenceNode::on_cancel_inference_, this, std::placeholders::_1),
      std::bind(&InferenceNode::on_inference_accepted_, this, std::placeholders::_1));

  // parameter callback handle
  on_parameter_set_handle_ = node_ptr_->add_on_set_parameters_callback(
      std::bind(&InferenceNode::on_parameter_set_, this, std::placeholders::_1));

  // initialize model
  initialize_whisper_();
}

void InferenceNode::declare_parameters_() {
  node_ptr_->declare_parameter("model_name", "base.en");
  // consider other parameters:
  // https://github.com/ggerganov/whisper.cpp/blob/a4bb2df36aeb4e6cfb0c1ca9fbcf749ef39cc852/whisper.h#L351
  node_ptr_->declare_parameter("language", "en");
  node_ptr_->declare_parameter("n_threads", 1);
  node_ptr_->declare_parameter("print_progress", false);
}

void InferenceNode::initialize_whisper_() {
  std::string model_name = node_ptr_->get_parameter("model_name").as_string();
  RCLCPP_INFO(node_ptr_->get_logger(), "Checking if model %s is available...", model_name.c_str());
  if (!model_manager_.is_available(model_name)) {
    RCLCPP_INFO(node_ptr_->get_logger(), "Model %s is not available. Attempting download...",
                model_name.c_str());
    if (model_manager_.make_available(model_name) != 0) {
      std::string err_msg = "Failed to download model " + model_name + ".";
      RCLCPP_ERROR(node_ptr_->get_logger(), err_msg.c_str());
      throw std::runtime_error(err_msg);
    }
    RCLCPP_INFO(node_ptr_->get_logger(), "Model %s downloaded.", model_name.c_str());
  }
  RCLCPP_INFO(node_ptr_->get_logger(), "Model %s is available.", model_name.c_str());

  RCLCPP_INFO(node_ptr_->get_logger(), "Initializing model %s...", model_name.c_str());
  whisper_.initialize(model_manager_.get_model_path(model_name));
  RCLCPP_INFO(node_ptr_->get_logger(), "Model %s initialized.", model_name.c_str());

  language_ = node_ptr_->get_parameter("language").as_string();
  whisper_.params.language = language_.c_str();
  whisper_.params.n_threads = node_ptr_->get_parameter("n_threads").as_int();
  whisper_.params.print_progress = node_ptr_->get_parameter("print_progress").as_bool();
}

rcl_interfaces::msg::SetParametersResult
InferenceNode::on_parameter_set_(const std::vector<rclcpp::Parameter> &parameters) {
  rcl_interfaces::msg::SetParametersResult result;
  for (const auto &parameter : parameters) {
    if (parameter.get_name() == "n_threads") {
      whisper_.params.n_threads = parameter.as_int();
      RCLCPP_INFO(node_ptr_->get_logger(), "Parameter %s set to %d.", parameter.get_name().c_str(),
                  whisper_.params.n_threads);
      continue;
    }
    result.reason = "Parameter " + parameter.get_name() + " not handled.";
    result.successful = false;
    RCLCPP_WARN(node_ptr_->get_logger(), result.reason.c_str());
  }
  result.successful = true;
  return result;
}

void InferenceNode::on_audio_(const std_msgs::msg::Int16MultiArray::SharedPtr msg) {
  episodic_buffer_.insert_from_stream(msg->data);
}

rclcpp_action::GoalResponse
InferenceNode::on_inference_(const rclcpp_action::GoalUUID &uuid,
                             std::shared_ptr<const Inference::Goal> goal) {
  RCLCPP_INFO(node_ptr_->get_logger(), "Received inference request.");
  if (running_inference_) {
    RCLCPP_WARN(node_ptr_->get_logger(), "Inference already running.");
    return rclcpp_action::GoalResponse::REJECT;
  }

  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse
InferenceNode::on_cancel_inference_(const std::shared_ptr<GoalHandleInference> goal_handle) {
  RCLCPP_INFO(node_ptr_->get_logger(), "Canceling inference...");
  return rclcpp_action::CancelResponse::ACCEPT;
}

void InferenceNode::on_inference_accepted_(const std::shared_ptr<GoalHandleInference> goal_handle) {
  RCLCPP_INFO(node_ptr_->get_logger(), "Starting inference...");
  running_inference_ = true;
  auto result = std::make_shared<Inference::Result>();
  auto feedback = std::make_shared<Inference::Feedback>();

  inference_start_time_ = node_ptr_->now();

  std::size_t previous_batch_idx;
  while (node_ptr_->now() - inference_start_time_ < goal_handle->get_goal()->max_duration &&
         rclcpp::ok()) {

    auto text = whisper_.forward(episodic_buffer_.retrieve_audio_batch());
    auto batch_idx = episodic_buffer_.batch_idx();

    if (batch_idx != previous_batch_idx) {
      RCLCPP_INFO(node_ptr_->get_logger(), "Epoch %d", batch_idx);
      previous_batch_idx = batch_idx;
      // feedback->text[batch_idx] = text;
    }

    feedback->batch_idx = batch_idx;
    feedback->text[batch_idx] = text;
    goal_handle->publish_feedback(feedback);
  }
  // result->text = text;

  // goal_handle->canceled
  // goal_handle->is_canceling
  // goal_handle->publish_feedback

  goal_handle->succeed(result);
  running_inference_ = false;
}
} // end of namespace whisper
