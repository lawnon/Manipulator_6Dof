///
/// File: ctrl.hpp
/// Data: 03.01.2026
/// Autor: Chukwunonso Bob-Anyeji
/// Description: Header File for Programm Entry Point 
///

#ifndef CTRL_H_
#define CTRL_H_

#include <windows.h>
#include <winuser.h>
#include <tchar.h>
#include <assert.h>

#define BUFFERSIZE 32768
#define KeyDown    0x00000080
#define KeyEnabled 0x00000001

struct Terminal //
{
    // Console Color Codes
    const char* cReset   = "\033[00m\n";
    const char* cBlack   = "\033[30m";
    const char* cRed     = "\033[31m";
    const char* cGreen   = "\033[32m";
    const char* cYellow  = "\033[33m";
    const char* cBlue    = "\033[34m";
    const char* cMagenta = "\033[35m";
    const char* cCyan    = "\033[36m";
    const char* cWhite   = "\033[37m";

    const char* cInfo    = "\033[35m->\033[32m ";
    const char* cWarn    = "\033[35m->\033[33m ";
    const char* cError   = "\033[35m->\033[31m ";

    // Debug Info
    BOOL Debug = false;
};

/// State Steps
struct Step
{
    int Active = 0;
    int Previous = 0;
    int Next = 0;
};

/// Communication State Maschine
enum ComState
{
    /// Persisten StateMaschine Data
    // Initialization-SetUp 
    Init,
    Idel,

    // Read and Evaluate Input data from Embedded Controller
    Input,            
    InputPending,     
    InputFinished,        
    InputReadBuffer,  
    InputWriteConsole,
    InputDone,        
    InputError,       

    // Evaulate and Write to Embedded Controller
    Output,
    OutputGetConsole,
    OutputWriteBuffer,
    OutputDone,
    OutputError
};

#endif // CTRL_H_
