///
/// File: ctrl.hpp
/// Data: 03.01.2026
/// Autor: Chukwunonso Bob-Anyeji
/// Description: Header File for Programm Entry Point 
///

#ifndef CTRL_H_
#define CTRL_H_

#define BUFFERSIZE 32768
#define KeyDown    0x00000080
#define KeyEnabled 0x00000001

// #define cReset   "\033[00m";
// #define cBlack   "\033[30m";
// #define cRed     "\033[31m";
// #define cGreen   "\033[32m";
// #define cYellow  "\033[33m";
// #define cBlue    "\033[34m";
// #define cMagenta "\033[35m";
// #define cCyan    "\033[36m";
// #define cWhite   "\033[37m";

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
