#ifndef API_TASK_H
#define API_TASK_H

#include "cmsis_os.h"
#include "main.h"

#ifdef __cplusplus

#define TASK_NAME(var) #var  // 函数名转字符串
#define MAX_TASK_NUM 15      // 最大可管理任务数量

namespace task
{
enum class TaskType
{
  TASK_DELAY,  // 延时任务
  TASK_PERIOD  // 周期任务
};

class TaskCreator
{
 public:
  TaskCreator(const char *name, uint8_t priority, uint16_t stack_size,
              osThreadFunc_t func, void *argument);
  ~TaskCreator() = default;

 protected:
  osThreadId_t TaskHandle;  // 任务句柄
};

class ManagedTask : public TaskCreator
{
 public:
  ManagedTask(const char *name, uint8_t priority, uint16_t stack_size,
              TaskType task_type_, uint8_t ticks_);
  ~ManagedTask() = default;

 private:
  uint8_t ticks;
  TaskType task_type;

  virtual void taskProcess() = 0;
  uint8_t task_list_dx;

  static void allTaskProcess(void *argument);
  static ManagedTask *task_list[MAX_TASK_NUM];  // 可管理任务列表
  static uint8_t task_num;                      // 可管理任务数量
};
}  // namespace task
#endif

#endif  // API_TASK_H
