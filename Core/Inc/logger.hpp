#ifndef CORE_INC_LOGGER_HPP_
#define CORE_INC_LOGGER_HPP_

#include <plog/Appenders/IAppender.h>
#include <plog/Formatters/TxtFormatter.h>
#include <plog/Init.h>
#include <plog/Log.h>

#include <cstddef>

extern "C" {
#include "usart.h"
}

namespace logging {

class UartAppender : public plog::IAppender {
 public:
  explicit UartAppender(UART_HandleTypeDef* uart) : uart_(uart) {}

  void write(const plog::Record& record) override {
    const auto msg = plog::TxtFormatter::format(record);
    const uint8_t* data = reinterpret_cast<const uint8_t*>(msg.c_str());
    const std::size_t len = msg.size();
    // Best-effort transmit; ignore errors in logging path.
    HAL_UART_Transmit(uart_, const_cast<uint8_t*>(data), static_cast<uint16_t>(len), 100);
  }

 private:
  UART_HandleTypeDef* uart_;
};

inline void InitLogger(plog::Severity level = plog::info) {
  static UartAppender uart_appender(&huart4);
  plog::init(level, &uart_appender);
}

}  // namespace logging

#endif  // CORE_INC_LOGGER_HPP_
