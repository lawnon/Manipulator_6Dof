/*
 * Program: Logger.hpp
 * Description: header file containing  procedures for fromating and writing
 *               State Date to Serial port
 * Autor: Chukwunonso Bob-Anyeji
 * Date: 09.06.2024
 */

#ifndef LOGGER_H_
#define LOGGER_H_

#include <HardwareSerial.h>
#include <WString.h>
#include "Kinematic.hpp"
#include "Commands.hpp"

/*
 * Current Naming convention clashes with logarithmic
 * fuctions from <math.h> C-header files ie.
 * => log(double arg);
 * => logf(float arg);
 * => logl(long double arg);
 * Todo:
 * + Update Encapsulat the below logging functions is in
 *   seperate namespace.
*/

void log(String text);
void logft(float dec);
void logInt(int digit);
void log(String text, String title);
void logft(float dec, String title);
void log(int digit, String titel);
void log(Posture pt);
void logln(Posture pt);
void logln(Posture  pt, String title);
void logToSerial(Posture pt);
void logToSerial(String input);
void log(Position ps);
void logln(Position ps);
void log(Position ps, String title);
void logln(Position ps, String title);
void log(Command cmd);
void log(PosData pdata, PosState state);

#endif // LOGGER_H_
