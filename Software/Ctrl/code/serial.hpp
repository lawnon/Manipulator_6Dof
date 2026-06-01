///
/// File: Serial.hpp
/// Date: 31.05.2026
/// Autor: Chukwunonso Bob-Anyeji
/// Description: Serial Communication Class
///

#ifndef SERIAL_H_
#define SERIAL_H_

#include <string>
#include <termios.h>

class SerialCom {
private:
  int _data = -1;
  termios _settings_{};

  void configure(speed_t baudRate, int timeOut);
  
public:  
  explicit SerialCom(const std::string &device, speed_t baudRate = B115200,
                     int timeout_ds = 10);
  ~SerialCom();

  bool IsOpen() const { return _data >= 0; }

  // Return bytes written or -1 on error
  int write(const std::string &data);
  int write(const uint8_t *buffer, size_t length);

  // Return bytes read into buffer, 0 on timeout, -1 on error
  int read(uint8_t *buffer, size_t maxLength);
  std::string ReadLine(char delim = '\n');
  

};  
#endif // SERIAL_H_
