/////////////////////////////////////////////////////////////////////////////
/// File: ctrl.hpp
/// Data: 03.01.2026
/// Autor: Chukwunonso Bob-Anyeji
/// Description: This file contains the 'main' function.
///              Program execution begins and ends there. 
/////////////////////////////////////////////////////////////////////////////

#include "ctrl.hpp"

#include <time.h>
#include <iostream>
#include <string>
#include <initializer_list>
#include <bitset>


//DWORD ReadBytesTransferred = 0;
//DWORD WriteBytesTransferred = 0;

time_t TimeStamp = time(NULL);
std::string ConsoleInput = "";
char TimeStampBuffer[32];
Terminal Term;

//BYTE  vKeyBoardState[256] = {};
short vKeyStatus = 0;

/////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////
/// TODO: 1) Add Refactor logging
///       2) Add timestamps with ctime or chrono
///       3) Refactor / Pull out code chunks in seprate functions
///       4) Auto set Baudrate
///       5) Consider using a state Maschine (DONE)
///       6) Encalsulate dangling attributes within Namespaces
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
      //<< text << GetLastError()
              << Term.cReset;
}

std::string  StepToString(Step stp)
{
    switch (stp.Active) {
        case ComState::Init: return "Initialization";
        case ComState::Idel: return "Idel";
        case ComState::Input: return "Input";            
        case ComState::InputPending: return "InputPending";     
        case ComState::InputReady: return "InputReady";        
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

/*
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

BOOL  KeyboardPressed(BOOL Init = true)
{
    // Init Keyboard State
    if(Init)
    {
        GetKeyState(0);        
    }
    
    // Get keyboard Input
    if (!GetKeyboardState(vKeyBoardState))
    {
        logGetLastError("Get Key Board State failed with error:  ", Term.cWarn);
    }
    for (int i = 26; i <= 90; i++)
    {                    
        if (vKeyBoardState[i] & KeyDown)
        {
            return true;
        }
    }
    return false;
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
*/

int main(int argc, const char* argv[])
{
    // NOTE: Refractor, Refractor, Refractor
    
    std::cout << Term.cInfo
              <<"(* programm Started *)"
              << Term.cReset;

    // Activate verborse logging
    if(!strcmp(argv[argc-1], "-v") || !strcmp(argv[argc-1], "--verbose"))
    {
        Term.Debug = true;
    }
    if(!strcmp(argv[argc-1], "-v-sm") || !strcmp(argv[argc-1], "--verbose-sm"))
    {
        Term.Debug = true;
        Term.DebugSM = true;            
    }
    
    std::initializer_list<Message<int>> infc = {{"Argument Count: ", argc}};
    logInfos<int>(infc);
    
    std::string devicePort = "COM1";
    for (int i = 0; i < argc; i++)
    {       
        std::initializer_list<Message<const char*>> infv = {{"Argument Value: ", argv[i]}};
        logInfos<const char*>(infv);
        
        if (!strcmp(argv[i], "-c") || !strcmp(argv[i],"--com"))
        {
            if ((i+1 < argc) && (strlen(argv[i+1]) > 0))
            {
                devicePort = "COM" + static_cast<std::string>(argv[i+1]);
            }            
        }
    }

    int x;
    std::cin >> x;

    /*    
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
        logGetLastError("GetCommState failed with error ");
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
        logGetLastError("SetCommState failed with error ");
        CloseHandle(deviceHandle);
        return (3);
    }
    logInfo("Serial port " + devicePort + " successfully reconfigured.");
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
        logGetLastError("SetCommMask failed with error ");
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
    
    COMSTAT comState;
    DWORD bufferSize = BUFFERSIZE;
    
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
       case ComState::InputReady: 
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
                    logInfo("RXFLAG changed state detected");
                    cStep.Next = ComState::InputReadBuffer;
                }
                if (eventMask & EV_RXCHAR)
                {
                    logInfo("RXCHAR changed detected");
                    cStep.Next = ComState::InputReadBuffer;
                }
                if (eventMask & EV_RLSD)
                {
                    logInfo("EV_RLSD detectec§");
                }
                eventMask = 0;
                break;
            }

            case ComState::InputReadBuffer:
            {
                Sleep(100);
                cStep.Next = ComState::Idel;
                if (ClearCommError(deviceHandle, &eventError, &comState))
                {
                    bufferSize = comState.cbInQue;
                    logInfo("Comm Error Size: " + std::to_string(bufferSize));
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
                    logGetLastError("Termnal failure: Unable to read from File: ");
                    CloseHandle(deviceHandle);
                    return (5);
                }

                // Asynchronous Wait starts hier
                SleepEx(IO_TIMEOUT, TRUE);
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
                              << Term.cReset
                              << std::endl;                
                    
                    cStep.Next = ComState::InputDone;
                    break;
                }
                else if (bytesRead == 0)
                {
                    // FIXME: NUll byte has to be handled better than
                    // verbose logging.                    
                    logWarn("No data read from devicePort");
                }
                else
                {
                    logWarn("Unexpected value for dwBytesRead");
                }
                break;
            }

            case ComState::InputDone:
            {
                comState = {};
                eventMask = 0;
                // NOTE: this is temporary
                bytesRead = 0;
                std::memset(deviceBuffer, 0, BUFFERSIZE);
                
                cStep.Next = ComState::Idel;
                break;
            }

            case ComState::InputError:
            {
                break;
            }

            case ComState::Output:
            {
                logInfo("Waiting for Keyboard Input ... ");
                std::cout << Term.cGetLine;
                std::getline(std::cin, ConsoleInput);
                    
                if (!ConsoleInput.length())
                {
                    cStep.Next = ComState::Idel;
                    break;
                }
                if (ConsoleInput == "exit")
                {
                    goto Terminate;
                }
                           
                BOOL writeResult = WriteFileEx(
                    deviceHandle,
                    ConsoleInput.data(),
                    static_cast<DWORD>(ConsoleInput.length()),
                    &writeOverlap,
                    onWriteIOCompletion
                );
                if (!writeResult)
                {
                    logGetLastError("Termnal failure: Unable to write from File ");
                    CloseHandle(deviceHandle);
                    return (6);
                }
                    
                SleepEx(IO_TIMEOUT, TRUE);
                // NOTE: RXCAHR Mask is triggerd When we Write Files
                //       therefor eventOverlap ought to be reseted
                EventReset(eventOverlap);
                cStep.Next = ComState::OutputDone;
                break;
            }

            case ComState::OutputDone:
            {
                if(!KeyboardPressed())
                {
                    cStep.Next = ComState::Idel;                    
                }
                
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
            if(Term.DebugSM)
            {
                std::string stp = "Step: " + StepToString(cStep);
                logInfo(stp);                
            }
        }
    }
Terminate:
    std::cout << Term.cInfo
              << "(* Programm Terminating *)"
              << Term.cReset;
    CloseHandle(deviceHandle);
    return (0);
    */
}
