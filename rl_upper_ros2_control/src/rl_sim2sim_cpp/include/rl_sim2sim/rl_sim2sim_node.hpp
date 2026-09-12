#pragma once

#include <GLFW/glfw3.h>
#include <mujoco/mujoco.h>
#include <onnxruntime_cxx_api.h>

#include <array>
#include <string>
#include <vector>

class Sim2SimNode
{
 public:
  Sim2SimNode(const std::string& model_path, const std::string& policy_path);
  ~Sim2SimNode();

  Sim2SimNode(const Sim2SimNode&) = delete;
  Sim2SimNode& operator=(const Sim2SimNode&) = delete;

  void run(float vx, float vy, float wz);

 private:
  static constexpr int kJointCount = 12;
  static constexpr int kObservationDim = 26;
  static constexpr int kActionDim = 12;
  static constexpr float kPhasePeriod = 0.4f;
  static constexpr float kCommandDeadband = 0.1f;

  void init_visualizer();
  void bind_model();
  void set_initial_pose();
  void update_observation();
  std::array<float, kActionDim> forward_policy();
  void set_joint_targets(const std::array<float, kActionDim>& targets);
  void step();
  void render();
  bool window_closed() const;

  static std::array<float, 3> read_sensor(const mjModel* model,
                                          const mjData* data, const char* name);
  static std::array<float, 3> quat_rotate_inverse(
      const std::array<float, 4>& quaternion,
      const std::array<float, 3>& vector);

  static constexpr std::array<const char*, kJointCount> kJointNames = {
      "FL_hip_joint",   "FL_thigh_joint", "FL_calf_joint",  "FR_hip_joint",
      "FR_thigh_joint", "FR_calf_joint",  "RL_hip_joint",   "RL_thigh_joint",
      "RL_calf_joint",  "RR_hip_joint",   "RR_thigh_joint", "RR_calf_joint"};

  static constexpr std::array<float, kJointCount> kDefaultJointAngles = {
      0.0, 0.6151, -0.9065, 0.0, 0.6151, -0.9065,
      0.0, 0.6519, -0.9709, 0.0, 0.6519, -0.9709};

  mjModel* model_{nullptr};
  mjData* data_{nullptr};
  GLFWwindow* window_{nullptr};
  mjvCamera camera_{};
  mjvOption visual_options_{};
  mjvScene scene_{};
  mjrContext context_{};

  Ort::Env onnx_env_;
  Ort::SessionOptions onnx_options_;
  Ort::Session onnx_session_{nullptr};
  Ort::AllocatorWithDefaultOptions onnx_allocator_;
  std::string input_name_;
  std::string output_name_;

  double sim_dt_{0.004};
  int control_decimation_{5};
  double control_dt_{0.02};
  float action_scale_{0.25f};
  int trunk_id_{-1};
  std::vector<int> qpos_indices_;
  std::vector<int> ctrl_indices_;
  std::array<float, 3> command_{};
  std::array<float, kActionDim> last_actions_{};
  std::array<float, kObservationDim> observation_{};
  double phase_time_seconds_{0.0};
};
