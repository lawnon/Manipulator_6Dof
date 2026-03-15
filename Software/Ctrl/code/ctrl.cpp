/////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////
/// File: ctrl.hpp
/// Data: 03.01.2026
/// Autor: Chukwunonso Bob-Anyeji
/// Description: This file contains the 'main' function.
///              Program execution begins and ends there. 
/////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////

#include "ctrl.hpp"

#include <time.h>
#include <iostream>
#include <string>
#include <initializer_list>
#include <bitset>


DWORD ReadBytesTransferred = 0;
DWORD WriteBytesTransferred = 0;

std::time_t TimeStamp = std::time(NULL);
std::string ConsoleInput = "";
char TimeStampBuffer[32];
Terminal Term;

/////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////
/// TODO: 1) Add Refactor logging
///       2) Add timestamps with ctime or chrono
///       3) Refactor / Pull out code chunks in seprate functions
///       4) Auto set Baudrate
///       5) Consider using a state Maschine
///       x) Refactor, Refactor, Refactor
/////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////// 

template<typename T>
struct Message{
    std::string text = "";
    const T &code;
};

void clog(const std::string &text, std::string colour = Term.cReset)
{
    if (Term.Debug)
    {
        std::cout << Term.cMagenta  << "-> " << colour  << text << Term.cReset;           
    }
}
void logInfo(const std::string &text)
{
    clog(text, Term.cGreen);
}
// TODO: undate with templates
// in order to pas an Array of <string, var>
// NOTE: First Iteration
template <typename T>
void logInfos(std::initializer_list<Message<T>> infos)
{
    if (Term.Debug)
    {        
        if (infos.size() < 0)
        {
            return;        
        }
        std::cout << Term.cMagenta  << "-> "  << Term.cGreen;
        for (auto &info : infos)
        {
            std::cout << info.text << static_cast<T>(info.code);
        }
        std::cout << Term.cReset;
    }
}
void logWarn(const std::string &text)
{
    clog(text, Term.cYellow);
}
void logError(const std::string &text)
{
    clog(text, Term.cRed);
}
void logGetLastError(const char* text, const char* msgType = Term.cError)
{
    std::cout << msgType
              << text << GetLastError()
              << Term.cReset;
}
std::string  logStep(Step stp)
{
    switch (stp.Active) {
        case ComState::Init: return "Initialization";
        case ComState::Idel: return "Idel";
        case ComState::Input: return "Input";            
        case ComState::InputPending: return "InputPending";     
        case ComState::InputFinished: return "InputFinished";        
        case ComState::InputReadBuffer: return "InputeReadBuffer";  
        case ComState::InputWriteConsole: return "InputeWriteConsole";
        case ComState::InputDone: return "InputDone";        
        case ComState::InputError: return "InputError";       
        case ComState::Output: return "Output";
        case ComState::OutputGetConsole: return "OutputGetConsole";
        case ComState::OutputWriteBuffer: return "OutputWriteBuffer";
        case ComState::OutputDone: return "OutputDone";
        case ComState::OutputError: return "OutpurtError";
        default: return "NA";
    }
}

void EventReset(OVERLAPPED& overlap)
{
    if(!ResetEvent(overlap.hEvent))
    {
        logError("Event reset failed with Error {} "
                 + std::to_string(GetLastError()));
    }
    overlap.Internal = 0;
    overlap.InternalHigh = 0;
}

void logDeviceState(DCB deviceCtrlBlk)
{
    std::cout << Term.cInfo
              << "DCB["      << (int)deviceCtrlBlk.DCBlength << "]:= "
              << "BaudRate=" << (int)deviceCtrlBlk.BaudRate << " |"
              << "ByteSize=" << (int)deviceCtrlBlk.ByteSize << " |"
              << "Parity="   << (int)deviceCtrlBlk.Parity   << " |" 
              << "StopBits=" << (int)deviceCtrlBlk.StopBits
              << Term.cReset;
}

void logIOCompletion(DWORD errorCode, DWORD nrOfBytes, LPOVERLAPPED overlaped)
{
    std::cout << Term.cError
              << "ErrorCode: " << std::bitset<32>(errorCode)
              << " | Number of bytes: " << nrOfBytes
              << " | Overlap: " << static_cast<void*>(overlaped)
              << Term.cReset;
}

void CALLBACK onReadIOCompletion(
    __in DWORD errorCode,
    __in DWORD nrOfBytes,
    __in LPOVERLAPPED overlaped)
{
    if (errorCode > 0)
    {
        logIOCompletion(errorCode, nrOfBytes, overlaped);
    }
    ReadBytesTransferred = nrOfBytes;
    EventReset(*overlaped);
}

void CALLBACK onWriteIOCompletion(
    __in DWORD errorCode,
    __in DWORD nrOfBytes,
    __in LPOVERLAPPED overlaped)
{
    if(errorCode > 0)
    {
        logIOCompletion(errorCode, nrOfBytes, overlaped);
    }
    WriteBytesTransferred = nrOfBytes;
    EventReset(*overlaped);
}

int main(int argc, const char* argv[], const char* envp[])
{
    // NOTE: Refractor, Refractor, Refractor
     
    // Print Debug Info
    std::cout << Term.cInfo
              <<"(* programm Started *)"
              << Term.cReset;
    std::initializer_list<Message<int>> infc = {{"Argument Count: ", argc}};
    logInfos<int>(infc);

    // Activate verborse logging
    if(!strcmp(argv[argc-1], "-v") || !strcmp(argv[argc-1], "--verbose"))
    {
        Term.Debug = true;            
    }
    
    std::string devicePort = "COM1";
    for (int i = 0; i < argc; i++)
    {       
        if (!strcmp(argv[i], "-c") || !strcmp(argv[i],"--com"))
        {
            if ((i+1 < argc) && (strlen(argv[i+1]) > 0))
            {
                devicePort = "COM" + static_cast<std::string>(argv[i+1]);
            }            
        }

        std::initializer_list<Message<const char*>> infv = {{"Argument Value: ", argv[i]}};
        logInfos<const char*>(infv);
    } 

    char deviceBuffer[BUFFERSIZE] = {0};
    DCB deviceCtrlBlk; 
    HANDLE deviceHandle;
    
    //OVERLAPPED write
    DWORD bytesRead = 0;
    DWORD eventMask = 0;
    DWORD eventError = 0;
    BOOL eventResult;
    BOOL eventOverlapResult = 0;

    BOOL result;
    
    //  Open a handle to the specified com port.
    deviceHandle = CreateFile(devicePort.data(),
                              GENERIC_READ | GENERIC_WRITE,
                              0,      //  must be opened with exclusive-access
                              NULL,   //  default security attributes
                              OPEN_EXISTING, //  must use OPEN_EXISTING
                              FILE_FLAG_OVERLAPPED,      //  not overlapped I/O
                              NULL); //  hTemplate must be NULL for comm devices  
    if (deviceHandle == INVALID_HANDLE_VALUE)
    {
        //  Handle the error.
        logGetLastError("CreateFile/ Opening COM-PORT failed with error ");
        CloseHandle(deviceHandle);
        return (1);
    }

    //  Initialize the DCB structure.
    logInfo("Initializing DCB Structure");
    SecureZeroMemory(&deviceCtrlBlk, sizeof(DCB));
    deviceCtrlBlk.DCBlength = sizeof(DCB);

    result = GetCommState(deviceHandle, &deviceCtrlBlk);
    if (!result)
    {
        std::cout << Term.cError
                  << "GetCommState failed with error " << GetLastError()
                  << Term.cReset;
        CloseHandle(deviceHandle);
        return (2);
    }

    deviceCtrlBlk.BaudRate = CBR_115200; //  baud rate
    deviceCtrlBlk.ByteSize = 8;          //  data size, xmit and rcv
    deviceCtrlBlk.Parity   = NOPARITY;     //  parity bit
    deviceCtrlBlk.StopBits = ONESTOPBIT; //  stop bit

    result = SetCommState(deviceHandle, &deviceCtrlBlk);
    if (!result)
    {
        std::cout << Term.cError
                  << "SetCommState failed with error " << GetLastError()
                  << Term.cReset;
        CloseHandle(deviceHandle);
        return (3);
    }
    //std::initializer_list<Info<string>> infs = { }
    std::cout << Term.cInfo
              << "Serial port " << devicePort  << " successfully reconfigured."
              << Term.cReset;
    logDeviceState(deviceCtrlBlk);

    result = SetCommMask(deviceHandle,
                         // EV_BREAK  | // A break was detected on input
                         // EV_CTS    | // clear to send
                         // EV_DSR    | // data set ready
                         // EV_RXFLAG | // Received certain character ie. XonChar
                         EV_RXCHAR // | // Recieved a character
                         // EV_RLSD     // The RLSD (receive-line-signal-detect) signal changed state.
    );
    if (!result)
    {
        std::cout << Term.cError
                  << "SetCommMask failed with error " << GetLastError()
                  << Term.cReset;
        CloseHandle(deviceHandle);
        return (4);
    }

    OVERLAPPED eventOverlap;
    eventOverlap.hEvent = CreateEvent(NULL,TRUE,FALSE,"EventOverlap");
    eventOverlap.Internal = 0;
    eventOverlap.InternalHigh = 0;
    eventOverlap.Offset = 0;
    eventOverlap.OffsetHigh = 0;
    assert(eventOverlap.hEvent);
    
    OVERLAPPED readOverlap;
    readOverlap.hEvent = CreateEvent(NULL, TRUE, FALSE, "ReadOverlap");
    readOverlap.Internal = 0;
    readOverlap.InternalHigh = 0;
    readOverlap.Offset = 0;
    readOverlap.OffsetHigh = 0;
    assert(readOverlap.hEvent);

    OVERLAPPED writeOverlap;
    writeOverlap.hEvent = CreateEvent(NULL, TRUE, FALSE, "WriteOverlap");
    writeOverlap.Internal = 0;
    writeOverlap.InternalHigh = 0;
    writeOverlap.Offset = 0;
    writeOverlap.OffsetHigh = 0;
    assert(writeOverlap.hEvent);
    
    // TODO: Refactor While Loops to State Maschine

    COMSTAT comState;
    DWORD bufferSize = BUFFERSIZE;
    BYTE vKeyBoardState[256] = {};
    short vKeyStatus = 0;
    Step cStep = {};
    long pendingCycleCount = 0;
    
    while(true){
        /// NOTE: Here we use the IPO/EVA Model to structure/Sequence of
        ///       of our function calls ie.
        ///       IPO: (I) Read Input Signals/Data.
        ///            (P) Process->Evaluate Stati,
        ///                         Process Logic/Statemachine,
        ///                         do Function calls etc.
        ///            (O) Write Output Signals/Data.
        ///

        switch(cStep.Active)
        {
            case ComState::Init:
            {
                cStep.Next = ComState::Idel;
                break;
            }
            
            case ComState::Idel:
            {
                // Init Keyboard State
                vKeyStatus = GetKeyState(0);
                
                // Check for Incoming data
                eventError = GetLastError();
                if(eventError != ERROR_IO_PENDING)
                {                    
                    // logInfo("Jesus Is Lord");
                    eventResult = WaitCommEvent(deviceHandle,
                                                &eventMask,
                                                &eventOverlap);
                }
                if (eventError == ERROR_IO_PENDING)
                { 
                    eventOverlapResult = GetOverlappedResult(
                        deviceHandle,  // Handle Hfile
                        &eventOverlap, // Overlapped
                        &bytesRead,    // NumberOfBytesTransferred EventMask
                        false);        // Alertable

                    
                }
                if(eventOverlapResult)
                {
                    if(eventMask)
                    {                        
                        eventOverlapResult = false;
                        cStep.Next = ComState::InputFinished;
                        pendingCycleCount = 0;
                        logInfo("Event Error: " + std::to_string(eventError) +
                                " Event Mask: " + std::to_string(eventMask));
                    
                        break; 
                    }
                }
                else
                {
                    eventError = GetLastError();                
                    switch (eventError)
                    {
                        case ERROR_IO_INCOMPLETE:
                        {            
                            if (pendingCycleCount <= 0)
                            {
                                std::cout << Term.cInfo
                                          << "IO Completion ... "
                                          << pendingCycleCount
                                          << Term.cReset;
                            }
                            else
                            {
                                std::cout << "\x1b[1A"  // Move cursor up one
                                          << "\x1b[2K" // Delete the entire line
                                          << Term.cInfo
                                          << "IO Completion ... "
                                          << pendingCycleCount
                                          << Term.cReset;
                            }
                            pendingCycleCount += 1;
                            break;
                        }
                    }
                }
                
                // Get keyboard Input
                if (!GetKeyboardState(vKeyBoardState))
                {
                    logGetLastError("Get Key Board State failed with error:  ", Term.cWarn);
                }
                for (int i = 48; i <= 90; i++)
                {                    
                    if (vKeyBoardState[i] & KeyDown)
                    {
                        cStep.Next = ComState::Output;
                        break;
                    }
                }
                break;
            }

            case ComState::Input:
            {
                cStep.Next = ComState::Idel;
                eventError = GetLastError();
                if (eventError == ERROR_IO_PENDING)
                {
                    cStep.Next = ComState::InputPending;
                    pendingCycleCount = 0;
                }
                else
                {
                    std::cout << Term.cRed << "I\\O pending failed with error "
                              << eventError
                              << Term.cReset << std::endl;    
                } 
                break;                
            }
                
            case ComState::InputPending:
            {
                if (!eventOverlapResult)
                {
                    eventError = GetLastError();                
                    switch (eventError)
                    {
                        case ERROR_IO_INCOMPLETE:
                        {                                
                            // Force Input to Console or IO-Completion Time-One                             
                            cStep.Next = ComState::InputPending;
                            
                            if (pendingCycleCount > 100)
                            {
                                EventReset(eventOverlap);
                                cStep.Next = ComState::Idel;
                                break;
                            }

                            if (pendingCycleCount <= 0)
                            {
                                std::cout << Term.cInfo
                                          << "IO Completion ... "
                                          << pendingCycleCount
                                          << Term.cReset;
                            }
                            else
                            {
                                std::cout << "\x1b[1A"  // Move cursor up one
                                          << "\x1b[2K" // Delete the entire line
                                          << Term.cInfo
                                          << "IO Completion ... "
                                          << pendingCycleCount
                                          << Term.cReset;
                            }
                            pendingCycleCount += 1;
                            break;
                        }
                        default:
                        {
                            std::cout << Term.cError
                                      << "Undefined Error: " << eventError
                                      << Term.cReset;
                        }
                    }                        
                }

                vKeyStatus = GetKeyState(VK_SPACE);
                if (vKeyStatus & KeyDown)
                {
                    EventReset(eventOverlap);
                    cStep.Next = ComState::Idel;
                    break;
                }
                
                if (eventMask)
                {
                    cStep.Next = ComState::InputFinished;
                    ClearCommError(deviceHandle, &eventError, &comState);
                    EventReset(eventOverlap);                    
                }
                SetLastError(0);
                break;
            }

            case ComState::InputFinished: 
           { 
               cStep.Next = ComState::Idel;
                if (eventMask & EV_CTS)
                {
                    logInfo("CTS changed state detected");
                }
                if (eventMask & EV_DSR)
                {
                    logInfo("DSR changed State detected");   
                }
                if (eventMask & EV_RXFLAG)
                {
                    clog("RXFLAG changed state detected", Term.cBlue);
                    cStep.Next = ComState::InputReadBuffer;
                }
                if (eventMask & EV_RXCHAR)
                {
                    clog("RXCHAR changed detected", Term.cBlue);
                    cStep.Next = ComState::InputReadBuffer;
                }
                if (eventMask & EV_RLSD)
                {
                    clog("EV_RLSD detectec§", Term.cBlue);
                }
                eventMask = 0;
                break;
            }

            case ComState::InputReadBuffer:
            {
                cStep.Next = ComState::Idel;
                if (ClearCommError(deviceHandle, &eventError, &comState))
                {
                    bufferSize = comState.cbInQue;
                    logWarn("Comm Error Size: " + std::to_string(bufferSize));
                }
                else
                {
                    logError("No Comm Error Size: " + std::to_string(bufferSize));
                }

                BOOL readResult = ReadFileEx(
                    deviceHandle,
                    deviceBuffer,
                    bufferSize - 1,
                    &readOverlap,
                    onReadIOCompletion
                );
                        
                if (!readResult)
                {
                    std::cout << Term.cError
                              << "Termnal failure: Unable to read from File: " << GetLastError()
                              << Term.cReset;
                    CloseHandle(deviceHandle);
                    return (5);
                }

                // Asynchronous Wait starts hier
                SleepEx(INFINITE, TRUE);
                bytesRead = ReadBytesTransferred;
                cStep.Next = ComState::InputWriteConsole;
                break;
            }

            case ComState::InputWriteConsole:
            {        
                // logInfo("Inpute Write to Console");
                cStep.Next = ComState::Idel;
                if (bytesRead > 0 && bytesRead <= bufferSize - 1)
                {
                    
                    std::cout << Term.cCyan
                              << deviceBuffer
                              << Term.cReset;
                
                    bytesRead = 0;
                    
                    // NOTE: this is temporary
                    std::memset(deviceBuffer, 0, BUFFERSIZE);
                    cStep.Next = ComState::InputDone;
                    break;
                }
                else if (bytesRead == 0)
                {
                    // FIXME: NUll byte has to be handled better than
                    // verbose logging.                    
                    logError("No data read from devicePort");
                }
                else
                {
                    logWarn("Unexpected value for dwBytesRead");
                }
                break;
            }

            case ComState::InputDone:
            {
                cStep.Next = ComState::Idel;
                break;
            }

            case ComState::InputError:
            {
                break;
            }

            case ComState::Output:
            {
                cStep.Next = ComState::Idel;
                std::cout << Term.cWarn;
                std::getline(std::cin, ConsoleInput);
                    
                if (!ConsoleInput.length())
                {
                    break;
                }
                if (ConsoleInput == "exit")
                {
                    goto Terminate;
                }

                // Remove Empty line caused by Enter Key
                //std::cout << "\x1b[1A";  // Move cursor up one
                                            
                BOOL writeResult = WriteFileEx(
                    deviceHandle,
                    ConsoleInput.data(),
                    static_cast<DWORD>(ConsoleInput.length()),
                    &writeOverlap,
                    onWriteIOCompletion
                );
                if (!writeResult)
                {
                    std::cout << Term.cError
                              << "Termnal failure: Unable to write from File " << GetLastError()
                              << Term.cReset;
                    CloseHandle(deviceHandle);
                    return (6);
                }
                    
                SleepEx(10000, TRUE);
                // NOTE: RXCAHR Mask is triggerd When we Write Files
                //       therefor eventOverlap ought to be reseted
                EventReset(eventOverlap);
                cStep.Next = ComState::OutputDone;
                break;
            }

            case ComState::OutputDone:
            {
                cStep.Next = ComState::Idel;
                break;
            }

            case ComState::OutputError:
            {
                break;
            }
        };
        // StepChain Transition
        if(cStep.Active != cStep.Next)
        {
            cStep.Previous = cStep.Active;
            cStep.Active = cStep.Next;
            std::string stp = "Step: " + logStep(cStep);
            logInfo(stp);
        }

        Sleep(50);
    }

Terminate:
    logInfo("Programm Terminating");
    CloseHandle(deviceHandle);
    return (0);
}
