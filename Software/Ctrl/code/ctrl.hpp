///
/// File: ctrl.hpp
/// Data: 03.01.2026
/// Autor: Chukwunonso Bob-Anyeji
/// Description: Header File for Programm Entry Point 
///

#ifndef CTRL_H_
#define CTRL_H_

#include <format>
#include <string>
#include <iostream>
#include <windows.h>
#include <tchar.h>
#include <assert.h>

#define BUFFERSIZE 32768

// Consolt Color Codes
const char* cReset   = "\033[00m";
const char* cBlack   = "\033[30m";
const char* cRed     = "\033[31m";
const char* cGreen   = "\033[32m";
const char* cYellow  = "\033[33m";
const char* cBlue    = "\033[34m";
const char* cMagenta = "\033[35m";
const char* cCyan    = "\033[36m";
const char* cWhite   = "\033[37m";

void clog(std::string text, std::string colour = cReset)
{
    std::cout << cMagenta  << "-> " << colour  << text << cReset << std::endl;   
}
void logInfo(std::string text)
{
    clog(text, cGreen);
}
void logWarn(std::string text)
{
    clog(text, cYellow);
}
void logError(std::string text)
{
    clog(text, cRed);
}

#endif // CTRL_H_
