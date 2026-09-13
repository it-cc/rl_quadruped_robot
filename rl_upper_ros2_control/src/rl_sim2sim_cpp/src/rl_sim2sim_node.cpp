#include "rl_sim2sim/rl_sim2sim_node.hpp"

#include <GLFW/glfw3.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <fcntl.h>
#include <iomanip>
#include <iostream>
#include <stdexcept>
#include <thread>
#include <termios.h>
#include <unistd.h>

namespace
{
class TerminalController
{
 public:
  TerminalController()
  {
    enabled_ = isatty(STDIN_FILENO) != 0 && tcgetattr(STDIN_FILENO, &original_) == 0;
    if (!enabled_) return;

    termios raw = original_;
    raw.c_lflag &= static_cast<unsigned>(~(ICANON | ECHO));
    raw.c_cc[VMIN] = 0;
    raw.c_cc[VTIME] = 0;
    if (tcsetattr(STDIN_FILENO, TCSANOW, &raw) != 0) {
      enabled_ = false;
      return;
    }
    original_flags_ = fcntl(STDIN_FILENO, F_GETFL, 0);
    if (original_flags_ >= 0) fcntl(STDIN_FILENO, F_SETFL, original_flags_ | O_NONBLOCK);
  }

  ~TerminalController()
  {
    if (!enabled_) return;
    tcsetattr(STDIN_FILENO, TCSANOW, &original_);
    if (original_flags_ >= 0) fcntl(STDIN_FILENO, F_SETFL, original_flags_);
    std::cout << '\n';
  }

  template <typename Callback>
  void poll(Callback&& callback)
  {
    if (!enabled_) return;
    char key = 0;
    while (read(STDIN_FILENO, &key, 1) == 1) callback(key);
  }

  bool enabled() const { return enabled_; }

 private:
  bool enabled_{false};
  termios original_{};
  int original_flags_{-1};
};

void print_controls()
{
  std::cout << "终端控制: w/s 前后, a/d 横向, e/c 旋转, x 停止, q 退出\n";
  std::cout << "视角控制: 左键旋转, 右键平移, 中键或滚轮缩放, Shift 改变拖动方向\n";
}
}  // namespace

Sim2SimNode::Sim2SimNode(const std::string& model_path,
                         const std::string& policy_path)
    : onnx_env_(ORT_LOGGING_LEVEL_WARNING, "rl_sim2sim_node"),
      onnx_session_(onnx_env_, policy_path.c_str(), onnx_options_)
{
  char error[1000]{};
  model_ = mj_loadXML(model_path.c_str(), nullptr, error, sizeof(error));
  if (!model_) throw std::runtime_error("MuJoCo XML 加载失败: " + std::string(error));

  data_ = mj_makeData(model_);
  if (!data_) throw std::runtime_error("MuJoCo 数据创建失败");
  model_->opt.timestep = sim_dt_;

  if (onnx_session_.GetInputCount() != 1 || onnx_session_.GetOutputCount() != 1)
    throw std::runtime_error("ONNX policy 必须有一个输入和一个输出");
  const auto input_shape = onnx_session_.GetInputTypeInfo(0)
                               .GetTensorTypeAndShapeInfo()
                               .GetShape();
  if (input_shape.size() != 2 || input_shape[0] != 1 ||
      input_shape[1] != kObservationDim)
    throw std::runtime_error("ONNX policy 输入维度必须是 [1, 26]");
  auto input = onnx_session_.GetInputNameAllocated(0, onnx_allocator_);
  auto output = onnx_session_.GetOutputNameAllocated(0, onnx_allocator_);
  input_name_ = input.get();
  output_name_ = output.get();

  bind_model();
  init_visualizer();
}

Sim2SimNode::~Sim2SimNode()
{
  if (window_) {
    mjr_freeContext(&context_);
    mjv_freeScene(&scene_);
    glfwDestroyWindow(window_);
    glfwTerminate();
  }
  if (data_) mj_deleteData(data_);
  if (model_) mj_deleteModel(model_);
}

void Sim2SimNode::bind_model()
{
  trunk_id_ = mj_name2id(model_, mjOBJ_BODY, "trunk");
  if (trunk_id_ < 0) throw std::runtime_error("找不到 MuJoCo body: trunk");

  for (const char* joint_name : kJointNames) {
    const int joint_id = mj_name2id(model_, mjOBJ_JOINT, joint_name);
    const int actuator_id = mj_name2id(model_, mjOBJ_ACTUATOR, joint_name);
    if (joint_id < 0 || actuator_id < 0)
      throw std::runtime_error(std::string("找不到关节或执行器: ") + joint_name);
    qpos_indices_.push_back(model_->jnt_qposadr[joint_id]);
    ctrl_indices_.push_back(actuator_id);
  }
}

void Sim2SimNode::init_visualizer()
{
  if (!glfwInit()) throw std::runtime_error("GLFW 初始化失败");
  window_ = glfwCreateWindow(1200, 900, "MuJoCo Sim2Sim", nullptr, nullptr);
  if (!window_) {
    glfwTerminate();
    throw std::runtime_error("创建 GLFW 窗口失败");
  }
  glfwSetWindowUserPointer(window_, this);
  glfwSetMouseButtonCallback(window_, &Sim2SimNode::mouse_button_callback);
  glfwSetCursorPosCallback(window_, &Sim2SimNode::mouse_move_callback);
  glfwSetScrollCallback(window_, &Sim2SimNode::scroll_callback);
  glfwMakeContextCurrent(window_);
  glfwSwapInterval(1);
  mjv_defaultCamera(&camera_);
  mjv_defaultOption(&visual_options_);
  mjv_defaultScene(&scene_);
  mjr_defaultContext(&context_);
  mjv_makeScene(model_, &scene_, 2000);
  mjr_makeContext(model_, &context_, mjFONTSCALE_150);
  camera_.type = mjCAMERA_TRACKING;
  camera_.trackbodyid = trunk_id_;
  camera_.distance = 3.0;
}

void Sim2SimNode::mouse_button_callback(GLFWwindow* window, int button, int action, int /*mods*/)
{
  auto* node = static_cast<Sim2SimNode*>(glfwGetWindowUserPointer(window));
  if (node) node->handle_mouse_button(button, action);
}

void Sim2SimNode::mouse_move_callback(GLFWwindow* window, double xpos, double ypos)
{
  auto* node = static_cast<Sim2SimNode*>(glfwGetWindowUserPointer(window));
  if (node) node->handle_mouse_move(xpos, ypos);
}

void Sim2SimNode::scroll_callback(GLFWwindow* window, double /*xoffset*/, double yoffset)
{
  auto* node = static_cast<Sim2SimNode*>(glfwGetWindowUserPointer(window));
  if (node) node->handle_scroll(yoffset);
}

void Sim2SimNode::handle_mouse_button(int button, int action)
{
  if (button == GLFW_MOUSE_BUTTON_LEFT) mouse_left_ = action == GLFW_PRESS;
  if (button == GLFW_MOUSE_BUTTON_MIDDLE) mouse_middle_ = action == GLFW_PRESS;
  if (button == GLFW_MOUSE_BUTTON_RIGHT) mouse_right_ = action == GLFW_PRESS;
  glfwGetCursorPos(window_, &last_mouse_x_, &last_mouse_y_);
}

void Sim2SimNode::handle_mouse_move(double xpos, double ypos)
{
  if (!mouse_left_ && !mouse_middle_ && !mouse_right_) return;

  const double dx = xpos - last_mouse_x_;
  const double dy = ypos - last_mouse_y_;
  last_mouse_x_ = xpos;
  last_mouse_y_ = ypos;

  int height = 0;
  glfwGetWindowSize(window_, nullptr, &height);
  if (height <= 0) return;

  const bool shift = glfwGetKey(window_, GLFW_KEY_LEFT_SHIFT) == GLFW_PRESS ||
                     glfwGetKey(window_, GLFW_KEY_RIGHT_SHIFT) == GLFW_PRESS;
  mjtMouse action = mjMOUSE_ZOOM;
  if (mouse_right_) {
    action = shift ? mjMOUSE_MOVE_H : mjMOUSE_MOVE_V;
  } else if (mouse_left_) {
    action = shift ? mjMOUSE_ROTATE_H : mjMOUSE_ROTATE_V;
  }
  mjv_moveCamera(model_, action, dx / height, dy / height, &scene_, &camera_);
}

void Sim2SimNode::handle_scroll(double yoffset)
{
  mjv_moveCamera(model_, mjMOUSE_ZOOM, 0.0, -0.05 * yoffset, &scene_, &camera_);
}

std::array<float, 3> Sim2SimNode::read_sensor(const mjModel* model,
                                               const mjData* data,
                                               const char* name)
{
  const int sensor_id = mj_name2id(model, mjOBJ_SENSOR, name);
  if (sensor_id < 0) throw std::runtime_error(std::string("找不到 MuJoCo sensor: ") + name);
  if (model->sensor_dim[sensor_id] != 3)
    throw std::runtime_error(std::string("sensor 维度错误: ") + name);
  const int address = model->sensor_adr[sensor_id];
  return {static_cast<float>(data->sensordata[address]),
          static_cast<float>(data->sensordata[address + 1]),
          static_cast<float>(data->sensordata[address + 2])};
}

std::array<float, 3> Sim2SimNode::quat_rotate_inverse(
    const std::array<float, 4>& q, const std::array<float, 3>& v)
{
  const float qw = q[3];
  const std::array<float, 3> qv{q[0], q[1], q[2]};
  const float dot = qv[0] * v[0] + qv[1] * v[1] + qv[2] * v[2];
  const std::array<float, 3> cross{qv[1] * v[2] - qv[2] * v[1],
                                   qv[2] * v[0] - qv[0] * v[2],
                                   qv[0] * v[1] - qv[1] * v[0]};
  std::array<float, 3> result{};
  for (int i = 0; i < 3; ++i)
    result[i] = v[i] * (2.0f * qw * qw - 1.0f) - 2.0f * qw * cross[i] + 2.0f * dot * qv[i];
  return result;
}

void Sim2SimNode::update_observation()
{
  const auto angular_velocity = read_sensor(model_, data_, "imu_ang_vel");
  const auto acceleration = read_sensor(model_, data_, "imu_accel");
  const mjtNum* quat = &data_->xquat[4 * trunk_id_];
  const auto gravity = quat_rotate_inverse(
      {static_cast<float>(quat[1]), static_cast<float>(quat[2]),
       static_cast<float>(quat[3]), static_cast<float>(quat[0])},
      {0.0f, 0.0f, -1.0f});

  const bool standing_command =
      std::sqrt(command_[0] * command_[0] + command_[1] * command_[1] +
                command_[2] * command_[2]) < kCommandDeadband;
  const float phase = static_cast<float>(
      std::fmod(phase_time_seconds_, static_cast<double>(kPhasePeriod)) /
      static_cast<double>(kPhasePeriod));

  // Keep this order identical to the actor observation terms in training.
  int offset = 0;
  for (float value : angular_velocity) observation_[offset++] = value;
  for (float value : gravity) observation_[offset++] = value;
  for (float value : command_) observation_[offset++] = value;
  if (standing_command) {
    observation_[offset++] = 0.0f;
    observation_[offset++] = 0.0f;
  } else {
    observation_[offset++] = std::sin(phase * 2.0f * 3.14159265358979323846f);
    observation_[offset++] = std::cos(phase * 2.0f * 3.14159265358979323846f);
  }
  for (float value : last_actions_) observation_[offset++] = value;
  for (float value : acceleration) observation_[offset++] = value;
}

std::array<float, Sim2SimNode::kActionDim> Sim2SimNode::forward_policy()
{
  std::array<int64_t, 2> shape{1, kObservationDim};
  auto memory = Ort::MemoryInfo("Cpu", OrtArenaAllocator, 0, OrtMemTypeDefault);
  auto input = Ort::Value::CreateTensor<float>(memory, observation_.data(), observation_.size(),
                                               shape.data(), shape.size());
  const char* input_names[] = {input_name_.c_str()};
  const char* output_names[] = {output_name_.c_str()};
  auto outputs = onnx_session_.Run(Ort::RunOptions{nullptr}, input_names, &input, 1,
                                   output_names, 1);
  if (outputs.empty() || !outputs[0].IsTensor())
    throw std::runtime_error("ONNX policy 输出不是 tensor");
  const auto output_shape = outputs[0].GetTensorTypeAndShapeInfo().GetShape();
  if (output_shape.size() != 2 || output_shape[0] != 1 || output_shape[1] != kActionDim)
    throw std::runtime_error("ONNX policy 输出维度必须是 [1, 12]");
  std::array<float, kActionDim> actions{};
  std::memcpy(actions.data(), outputs[0].GetTensorData<float>(), sizeof(actions));
  return actions;
}

void Sim2SimNode::set_initial_pose()
{
  const int home_key_id = mj_name2id(model_, mjOBJ_KEY, "home");
  if (home_key_id < 0)
    throw std::runtime_error("找不到 MuJoCo keyframe: home");

  const std::array<float, 3> position{0.0f, 0.0f, 0.23f};
  const std::array<float, 4> quaternion{1.0f, 0.0f, 0.0f, 0.0f};
  for (int i = 0; i < 3; ++i) data_->qpos[i] = position[i];
  for (int i = 0; i < 4; ++i) data_->qpos[3 + i] = quaternion[i];
  const mjtNum* home_qpos = model_->key_qpos + home_key_id * model_->nq;
  std::fill(data_->qvel, data_->qvel + model_->nv, 0.0);
  if (model_->na > 0) std::fill(data_->act, data_->act + model_->na, 0.0);
  for (int i = 0; i < kJointCount; ++i)
  {
    data_->qpos[qpos_indices_[i]] = home_qpos[qpos_indices_[i]];
    default_joint_angles_[i] = static_cast<float>(home_qpos[qpos_indices_[i]]);
  }
  if (model_->nu > 0)
  {
    const mjtNum* home_ctrl = model_->key_ctrl + home_key_id * model_->nu;
    std::copy(home_ctrl, home_ctrl + model_->nu, data_->ctrl);
  }
  phase_time_seconds_ = 0.0;
  last_actions_.fill(0.0f);
  observation_.fill(0.0f);
  mj_forward(model_, data_);
}

void Sim2SimNode::set_joint_targets(const std::array<float, kActionDim>& actions)
{
  for (int i = 0; i < kJointCount; ++i)
    data_->ctrl[ctrl_indices_[i]] = actions[i];
}

void Sim2SimNode::step() { mj_step(model_, data_); }

void Sim2SimNode::render()
{
  mjrRect viewport{0, 0, 0, 0};
  glfwGetFramebufferSize(window_, &viewport.width, &viewport.height);
  mjv_updateScene(model_, data_, &visual_options_, nullptr, &camera_, mjCAT_ALL, &scene_);
  mjr_render(viewport, &scene_, &context_);
  glfwSwapBuffers(window_);
  glfwPollEvents();
}

bool Sim2SimNode::window_closed() const { return glfwWindowShouldClose(window_); }

void Sim2SimNode::run(float vx, float vy, float wz)
{
  command_ = {vx, vy, wz};
  set_initial_pose();
  TerminalController terminal;
  print_controls();
  if (!terminal.enabled())
    std::cout << "标准输入不是终端，键盘控制不可用。\n";

  bool quit = false;
  while (!window_closed() && !quit) {
    const auto start = std::chrono::steady_clock::now();
    terminal.poll([&](char key) {
      constexpr float kLinearStep = 0.1f;
      constexpr float kAngularStep = 0.2f;
      constexpr float kMaxLinearSpeed = 1.0f;
      constexpr float kMaxAngularSpeed = 1.0f;
      switch (key) {
        case 'w': command_[0] = std::min(command_[0] + kLinearStep, kMaxLinearSpeed); break;
        case 's': command_[0] = std::max(command_[0] - kLinearStep, -kMaxLinearSpeed); break;
        case 'a': command_[1] = std::min(command_[1] + kLinearStep, kMaxLinearSpeed); break;
        case 'd': command_[1] = std::max(command_[1] - kLinearStep, -kMaxLinearSpeed); break;
        case 'e': command_[2] = std::min(command_[2] + kAngularStep, kMaxAngularSpeed); break;
        case 'c': command_[2] = std::max(command_[2] - kAngularStep, -kMaxAngularSpeed); break;
        case 'x': command_ = {0.0f, 0.0f, 0.0f}; break;
        case 'q': quit = true; break;
        default: break;
      }
    });
    if (quit) break;

    update_observation();
    const auto actions = forward_policy();
    std::array<float, kActionDim> joint_angles{};
    for (int i = 0; i < kActionDim; ++i)
      joint_angles[i] = actions[i] * action_scale_ + default_joint_angles_[i];

    for (int i = 0; i < control_decimation_; ++i) {
      set_joint_targets(joint_angles);
      step();
    }
    last_actions_ = actions;
    phase_time_seconds_ += control_dt_;
    render();
    const auto elapsed = std::chrono::steady_clock::now() - start;
    const double remaining = control_dt_ - std::chrono::duration<double>(elapsed).count();
    if (remaining > 0.0)
      std::this_thread::sleep_for(std::chrono::duration<double>(remaining));
  }
}

int main()
{
  try {
    Sim2SimNode node(
        "/home/cc/workspace/quadruped_robot/my-quadruped-robot/rl_quadruped_robot/rl_mjlab/src/assets/robots/my_quadruped/mjcf/scene.xml",
        "/home/cc/workspace/quadruped_robot/my-quadruped-robot/rl_quadruped_robot/rl_mjlab/logs/rsl_rl/my_quadruped_velocity/2026-09-11_21-57-47/policy.onnx");
    node.run(0.5f, 0.0f, 0.0f);
  } catch (const std::exception& error) {
    std::cerr << "sim2sim 失败: " << error.what() << '\n';
    return EXIT_FAILURE;
  }
  return EXIT_SUCCESS;
}
