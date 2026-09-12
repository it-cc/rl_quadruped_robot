#include "api_serial.h"

namespace serial
{

// 初始化静态成员变量
uint8_t UartRx::rxCount_ = 0;
UartRx *UartRx::rxList_[MAX_SERIAL_COUNT] = {nullptr};
uint8_t UartTx::txCount_ = 0;
UartTx *UartTx::txList_[MAX_SERIAL_COUNT] = {nullptr};

UartRx::UartRx(UART_HandleTypeDef &huart, uint8_t *buffer, uint16_t bufferSize,
               bool useIdle, bool useDMA, bool useCircularDMA)
    : huart_(&huart),
      buffer_(buffer),
      bufferSize_(bufferSize),
      useIdle_(useIdle),
      useDMA_(useDMA),
      useCircularDMA_(useCircularDMA)
{
  isInit_ = false;

  if (huart_ != nullptr && buffer_ != nullptr && bufferSize_ != 0)
  {
    if (rxCount_ < MAX_SERIAL_COUNT)
    {
      rxList_[rxCount_] = this;
      rxCount_++;
      isInit_ = true;
    }
  }
}

UartRx::~UartRx()
{
  for (uint8_t i = 0; i < rxCount_; ++i)
  {
    if (rxList_[i] != this) continue;

    for (uint8_t j = i; j + 1U < rxCount_; ++j)
    {
      rxList_[j] = rxList_[j + 1U];
    }
    rxList_[--rxCount_] = nullptr;
    break;
  }
}

inline void UartRx::handleUartRxInterrupt(UART_HandleTypeDef *huart,
                                          uint16_t size)
{
  for (uint8_t i = 0; i < rxCount_; i++)
  {
    if (rxList_[i] != nullptr && rxList_[i]->huart_ == huart)
    {
      // 空闲中断
      if (rxList_[i]->useIdle_)
      {
        if (size > 0)
        {
          rxList_[i]->OnDataReceived(rxList_[i]->buffer_, size);
        }
        if (!rxList_[i]->useCircularDMA_)
        {
          rxList_[i]->start();
        }
      }
      // 普通固定长度中断或普通DMA接收
      else
      {
        rxList_[i]->OnDataReceived(rxList_[i]->buffer_,
                                   rxList_[i]->bufferSize_);
        // 处理完数据后必须重启中断接收
        if (rxList_[i]->useDMA_)
        {
          HAL_UART_Receive_DMA(rxList_[i]->huart_, rxList_[i]->buffer_,
                               rxList_[i]->bufferSize_);
        }
        else
        {
          HAL_UART_Receive_IT(rxList_[i]->huart_, rxList_[i]->buffer_,
                              rxList_[i]->bufferSize_);
        }
      }
      return;
    }
  }
}

void UartRx::handleUartErrorInterrupt(UART_HandleTypeDef *huart)
{
  for (uint8_t i = 0; i < rxCount_; i++)
  {
    if (rxList_[i] != nullptr && rxList_[i]->huart_ == huart)
    {
      rxList_[i]->start();
      break;
    }
  }
}

void UartRx::start()
{
  if (!isInit_) return;

  if (useIdle_)
  {
    if (useDMA_)
    {
      HAL_UARTEx_ReceiveToIdle_DMA(huart_, buffer_, bufferSize_);
    }
    else
    {
      HAL_UARTEx_ReceiveToIdle_IT(huart_, buffer_, bufferSize_);
    }
  }
  else
  {
    if (useDMA_)
    {
      HAL_UART_Receive_DMA(huart_, buffer_, bufferSize_);
    }
    else
    {
      HAL_UART_Receive_IT(huart_, buffer_, bufferSize_);
    }
  }
}

UartTx::UartTx(UART_HandleTypeDef &huart) : huart_(&huart)
{
  if (txCount_ < MAX_SERIAL_COUNT)
  {
    txList_[txCount_++] = this;
    registered_ = true;
  }
}

UartTx::~UartTx()
{
  if (!registered_) return;

  for (uint8_t index = 0; index < txCount_; ++index)
  {
    if (txList_[index] != this) continue;
    for (uint8_t next = index; next + 1U < txCount_; ++next)
      txList_[next] = txList_[next + 1U];
    txList_[--txCount_] = nullptr;
    break;
  }
}

bool UartTx::transmitDma(const uint8_t *data, uint16_t length)
{
  if (huart_ == nullptr || data == nullptr || length == 0U || txBusy_)
    return false;

  txBusy_ = true;
  const HAL_StatusTypeDef status =
      HAL_UART_Transmit_DMA(huart_, const_cast<uint8_t *>(data), length);
  if (status != HAL_OK) txBusy_ = false;
  return status == HAL_OK;
}

bool UartTx::isTxBusy() const { return txBusy_; }

void UartTx::handleUartTxCompleteInterrupt(UART_HandleTypeDef *huart)
{
  for (uint8_t index = 0; index < txCount_; ++index)
  {
    if (txList_[index] != nullptr && txList_[index]->huart_ == huart)
    {
      txList_[index]->txBusy_ = false;
      txList_[index]->OnTxComplete();
      return;
    }
  }
}

void UartTx::handleUartTxErrorInterrupt(UART_HandleTypeDef *huart)
{
  for (uint8_t index = 0; index < txCount_; ++index)
  {
    if (txList_[index] != nullptr && txList_[index]->huart_ == huart)
    {
      txList_[index]->txBusy_ = false;
      txList_[index]->OnTxError();
      return;
    }
  }
}

}  // namespace serial

void uart_puts(const char *str)
{
  if (str == nullptr) return;

  const size_t length = strlen(str);
  if (length == 0) return;

  const uint16_t transmitLength =
      (length > UINT16_MAX) ? UINT16_MAX : static_cast<uint16_t>(length);
  (void)HAL_UART_Transmit(&huart7, reinterpret_cast<const uint8_t *>(str),
                          transmitLength, 100);
}

// 自定义printf函数
int uart_printf(const char *format, ...)
{
  char buffer[256];
  va_list args;

  if (format == nullptr) return -1;

  va_start(args, format);
  const int ret = vsnprintf(buffer, sizeof(buffer), format, args);
  va_end(args);

  if (ret <= 0) return ret;

  // vsnprintf returns the required length, which can exceed buffer's size.
  const size_t sendLength = (static_cast<size_t>(ret) >= sizeof(buffer))
                                ? sizeof(buffer) - 1U
                                : static_cast<size_t>(ret);
  const HAL_StatusTypeDef status =
      HAL_UART_Transmit(&huart7, reinterpret_cast<const uint8_t *>(buffer),
                        static_cast<uint16_t>(sendLength), 100);

  return (status == HAL_OK) ? ret : -static_cast<int>(status);
}

extern "C" void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  serial::UartRx::handleUartRxInterrupt(huart, 0);
}

extern "C" void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart,
                                           uint16_t size)
{
  serial::UartRx::handleUartRxInterrupt(huart, size);
}

extern "C" void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
  serial::UartTx::handleUartTxCompleteInterrupt(huart);
}

extern "C" void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  // 清除所有可能的错误标志
  if (__HAL_UART_GET_FLAG(huart, UART_FLAG_PE))
  {
    __HAL_UART_CLEAR_FLAG(huart, UART_CLEAR_PEF);  // 清除奇偶校验错误标志
  }

  if (__HAL_UART_GET_FLAG(huart, UART_FLAG_FE))
  {
    __HAL_UART_CLEAR_FLAG(huart, UART_CLEAR_FEF);  // 清除帧错误标志
  }

  if (__HAL_UART_GET_FLAG(huart, UART_FLAG_NE))
  {
    __HAL_UART_CLEAR_FLAG(huart, UART_CLEAR_NEF);  // 清除噪声错误标志
  }

  if (__HAL_UART_GET_FLAG(huart, UART_CLEAR_OREF) ||
      __HAL_UART_GET_FLAG(huart, UART_FLAG_ORE))
  {
    __HAL_UART_CLEAR_FLAG(huart, UART_CLEAR_OREF);  // 清除溢出错误标志
  }

  if (__HAL_UART_GET_FLAG(huart, UART_FLAG_LBDF))
  {
    __HAL_UART_CLEAR_FLAG(huart, UART_CLEAR_LBDF);  // LIN断点检测标志处理
  }

  serial::UartRx::handleUartErrorInterrupt(huart);  // 重启中断
  serial::UartTx::handleUartTxErrorInterrupt(huart);
}
