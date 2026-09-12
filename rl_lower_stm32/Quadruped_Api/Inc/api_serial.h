#ifndef API_SERIAL_H
#define API_SERIAL_H

#include <stdio.h>
#include <string.h>

#include "usart.h"

#ifdef __cplusplus

#define MAX_SERIAL_COUNT 20

namespace serial
{
class UartRx
{
 public:
  UartRx(UART_HandleTypeDef &huart, uint8_t *buffer, uint16_t bufferSize,
         bool useIdle, bool useDMA, bool useCircularDMA);
  ~UartRx();

  static void handleUartRxInterrupt(UART_HandleTypeDef *huart, uint16_t size);
  static void handleUartErrorInterrupt(UART_HandleTypeDef *huart);

  void start();

 protected:
  virtual void OnDataReceived(uint8_t *data, uint16_t length) = 0;

 private:
  UART_HandleTypeDef *huart_{nullptr};
  uint8_t *buffer_{nullptr};
  uint16_t bufferSize_{0};
  bool useIdle_{false};
  bool useDMA_{false};
  bool useCircularDMA_{false};
  bool isInit_{false};

  static uint8_t rxCount_;
  static UartRx *rxList_[MAX_SERIAL_COUNT];
};

class UartTx
{
 public:
  explicit UartTx(UART_HandleTypeDef& huart);
  virtual ~UartTx();

  bool transmitDma(const uint8_t* data, uint16_t length);
  bool isTxBusy() const;

  static void handleUartTxCompleteInterrupt(UART_HandleTypeDef* huart);
  static void handleUartTxErrorInterrupt(UART_HandleTypeDef* huart);

 protected:
  virtual void OnTxComplete() {}
  virtual void OnTxError() {}

 private:
  UART_HandleTypeDef* huart_{nullptr};
  volatile bool txBusy_{false};
  bool registered_{false};

  static uint8_t txCount_;
  static UartTx* txList_[MAX_SERIAL_COUNT];
};

}  // namespace serial

#endif
#ifdef __cplusplus
extern "C"
{
#endif

#include <stdarg.h>

  int uart_printf(const char *format, ...);
  void uart_puts(const char *str);

#ifdef __cplusplus
}
#endif

#endif  // API_SERIAL_H
