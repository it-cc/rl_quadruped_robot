#include "api_task.h"

namespace task
{
TaskCreator::TaskCreator(const char *name_, uint8_t priority_,
                         uint16_t stack_size_, osThreadFunc_t func_,
                         void *argument_)
{
  // 限制任务优先级
  if (priority_ < 8)
    priority_ = 8;
  else if (priority_ > 55)
    priority_ = 55;

  osThreadAttr_t Task_attributes = {
      .name = name_,
      .stack_size = (uint32_t)stack_size_ * 4,
      .priority = (osPriority_t)priority_,
  };

  // 创建任务
  TaskHandle = osThreadNew(func_, argument_, &Task_attributes);

  if (TaskHandle == NULL)
  {
    Error_Handler();
  }
}

/*--------------------------------------------------------------------------*/

ManagedTask *ManagedTask::task_list[MAX_TASK_NUM] = {
    nullptr};                       // 可管理任务列表
uint8_t ManagedTask::task_num = 0;  // 可管理任务数量

ManagedTask::ManagedTask(const char *name_, uint8_t priority_,
                         uint16_t stack_size_, TaskType task_type_,
                         uint8_t ticks_)
    : TaskCreator(name_, priority_, stack_size_, allTaskProcess, &task_list_dx)
{
  task_num++;

  if (task_num > MAX_TASK_NUM) Error_Handler();

  task_list_dx = task_num - 1;
  task_list[task_list_dx] = this;

  if (ticks_ == 0) ticks_ = 1;
  ticks = ticks_;

  task_type = task_type_;
}

void ManagedTask::allTaskProcess(void *argument)
{
  uint8_t dx = *(uint8_t *)(argument);  // 获取任务索引

  if (dx > MAX_TASK_NUM - 1) Error_Handler();

  if (task_list[dx] == nullptr) Error_Handler();

  // 周期任务获取任务开始时间
  uint32_t nextWakeTime = osKernelGetTickCount();

  for (;;)
  {
    task_list[dx]->taskProcess();  // 任务处理函数

    if (task_list[dx]->task_type == TaskType::TASK_DELAY)
      osDelay(task_list[dx]->ticks);  // 延时任务
    else
    {
      nextWakeTime += task_list[dx]->ticks;
      osDelayUntil(nextWakeTime);  // 周期任务
    }
  }
}

}  // namespace task
