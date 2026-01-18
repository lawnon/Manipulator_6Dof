///
/// File: ctrl.hpp
/// Data: 03.01.2026
/// Autor: Chukwunonso Bob-Anyeji
/// Description: This file contains the 'main' function.
///              Program execution begins and ends there. 
///

#include "ctrl.hpp"

DWORD ReadBytesTransferred = 0;
DWORD WriteBytesTransferred = 0;
std::string ConsoleInput;


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

void PrintDeviceState(DCB deviceCtrlBlk)
{
    logInfo("=>DCB[" + std::to_string((int)deviceCtrlBlk.DCBlength) + "]:= "
            + "BaudRate=" + std::to_string((int)deviceCtrlBlk.BaudRate) + " |"
            + "ByteSize=" + std::to_string((int)deviceCtrlBlk.ByteSize) + " |"
            + "Parity="   + std::to_string((int)deviceCtrlBlk.Parity  ) + " |" 
            + "StopBits=" + std::to_string((int)deviceCtrlBlk.StopBits));
}

void CALLBACK onReadIOCompletion(
    __in DWORD errorCode,
    __in DWORD nrOfBytes,
    __in LPOVERLAPPED overlaped)
{
    if (errorCode > 0)
    {
        logError(std::format("ErrorCode: {:b} |Number of bytes: {} |Overlap: {}",
                             errorCode ,
                             nrOfBytes,
                             static_cast<void*>(overlaped)));
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
        logError(std::format("ErrorCode: {:b} |Number of bytes: {} |Overlap: {}",
                             errorCode ,
                             nrOfBytes,
                             static_cast<void*>(overlaped)));
    }
    WriteBytesTransferred = nrOfBytes;
    EventReset(*overlaped);
}

int main(int argc, const char* argv[], const char* envp[])
{
    // NOTE: Refractor, Refractor, Refractor
    
    // Print Debug Info
    logInfo("programm Started :");
    std::cout << "argc: " << argc << std::endl;
    //for (int i = 0 : size_t(argc))
    for (int i = 0; i < argc; i++)
    {
        logInfo(std::format("argv: {}",argv[i]));
        logInfo(std::format("envp: {}",envp[i]));
    } 

    DCB deviceCtrlBlk;
    HANDLE deviceHandle;
    std::string devicePort = std::format("COM{}",5);
    char deviceBuffer[BUFFERSIZE] = {0};
    
    //OVERLAPPED write
    DWORD bytesRead = 0;
    DWORD eventMask = 0;
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
        std::cout << cRed << "=>CreateFile failed with error "
                  << GetLastError() << cReset << std::endl;
        CloseHandle(deviceHandle);
        return (1);
    }

    //  Initialize the DCB structure.
    std::cout << std::endl << "=>Initializing DCB Structure" << std::endl;
    SecureZeroMemory(&deviceCtrlBlk, sizeof(DCB));
    deviceCtrlBlk.DCBlength = sizeof(DCB);

    result = GetCommState(deviceHandle, &deviceCtrlBlk);
    if (!result)
    {
        std::cout << cRed << "GetCommState failed with error "
                  << GetLastError() << cReset << std::endl;
        CloseHandle(deviceHandle);
        return (2);
    }

    deviceCtrlBlk.BaudRate = CBR_115200;     //  baud rate
    deviceCtrlBlk.ByteSize = 8;             //  data size, xmit and rcv
    deviceCtrlBlk.Parity = NOPARITY;      //  parity bit
    deviceCtrlBlk.StopBits = ONESTOPBIT;    //  stop bit

    result = SetCommState(deviceHandle, &deviceCtrlBlk);
    if (!result)
    {
        std::cout << cRed << "SetCommState failed with error "
                  << GetLastError() << cReset << std::endl;
        CloseHandle(deviceHandle);
        return (3);
    }   
    std::cout << "=>Serial port " << devicePort
              << " successfully reconfigured." << std::endl;
    PrintDeviceState(deviceCtrlBlk);

    result = SetCommMask(deviceHandle,
                         EV_CTS    | // clear to send
                         EV_DSR    | // data set ready
                         EV_RXFLAG | // Received certain character ie. XonChar
                         EV_RXCHAR   // Recieved a character           
    );
    if (!result)
    {
        std::cout << "SetCommMask failed with error "
                  << GetLastError() << std::endl;
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
    
    BOOL idle = true;
    while(true){
        if (!idle)
        {
            logInfo("Waiting of Console Input... ");
            std::getline(std::cin, ConsoleInput);

            if (!ConsoleInput.length())
            {
                continue;
            }
            if (ConsoleInput == "exit")
            {
                return 0;
            }

            ConsoleInput += "\n";
                
            BOOL writeResult = WriteFileEx(
                deviceHandle,
                ConsoleInput.data(),
                static_cast<DWORD>(ConsoleInput.length()),
                &writeOverlap,
                onWriteIOCompletion
            );

            if (!writeResult)
            {
                logError(std::format("Termnal failure: Unable to write from File {}", GetLastError()));
                CloseHandle(deviceHandle);
                return (6);
            }

            SleepEx(INFINITE, TRUE);
            // NOTE: RXCAHR Mask is triggerd When we Write Files
            // therefor eventOverlap ougjt to be reseted
            EventReset(eventOverlap);
            //logWarn(std::format("Data Writen: {}", ConsoleInput));
            
            idle = true;
            continue;
        }

        BOOL eventResult = WaitCommEvent(deviceHandle, &eventMask, &eventOverlap);
        if (!eventResult)
        {
            DWORD eventError = GetLastError();
            if (eventError == ERROR_IO_PENDING)
            {
                //logInfo("IO is pending...");
                idle = false;
                BOOL eventPending = true;
                long cycleCounter = 0;
                while(eventPending)
                {
                    eventPending = false;
                    cycleCounter += 1;

                    BOOL overlapResult = GetOverlappedResult(deviceHandle,
                                                             &eventOverlap,
                                                             &bytesRead,
                                                             false);
                    eventError = GetLastError();
                
                    if (!overlapResult)
                    {
                        switch (eventError)
                        {
                            case ERROR_IO_INCOMPLETE:
                            {
                                //std::cout
                                //    << "\x1b[1A"  // Move cursor up one
                                //    << "\x1b[2K"; // Delete the entire line
                                //logInfo(std::format("IO Completion... {}",
                                //                    cycleCounter));

                                // NOTE: eventPending ought to be true
                                // and we ought to use Mutlthreading to make
                                // this it work :)
                                eventPending = true;

                                if (cycleCounter >= 100)
                                {
                                    eventPending = false;
                                    EventReset(eventOverlap);
                                }
                                break;
                            }
                            default:
                            {
                                logInfo(std::format("Undefined Error: {:b}", eventError));
                            }
                        }
                    }
                    else
                    {
                        EventReset(eventOverlap);
                    
                        if (eventMask & EV_CTS)
                        {
                            std::cout << "CTS changed state detected" << std::endl;
                        }
                        if (eventMask & EV_DSR)
                        {
                            std::cout << "DSR changed State detected" << std::endl;   
                        }
                        if (eventMask & EV_RXFLAG)
                        {
                            logInfo("RXFLAG changed state detected");
                        }
                        if (eventMask & EV_RXCHAR)
                        {
                            //logInfo("RXCHAR changed detected");
                            COMSTAT comState;
                            DWORD BufferSize = BUFFERSIZE;
                            if (ClearCommError(deviceHandle, &eventError, &comState))
                            {
                                BufferSize = comState.cbInQue;
                            }                            

                            BOOL readResult = ReadFileEx(
                                deviceHandle,
                                deviceBuffer,
                                BufferSize - 1,
                                &readOverlap,
                                onReadIOCompletion
                            );
                        
                            if (!readResult)
                            {
                                logError(std::format("Termnal failure: Unable to read from File {0}", GetLastError()));
                                CloseHandle(deviceHandle);
                                return (5);
                            }

                            // Asynchronous Wait starts hier
                            SleepEx(INFINITE, TRUE);
                            bytesRead = ReadBytesTransferred;
        
                            if (bytesRead > 0 && bytesRead <= BufferSize -1)
                            {
                                deviceBuffer[bytesRead]='\n';
                                std::cout << cCyan
                                          << deviceBuffer
                                          << cReset;
                                bytesRead = 0;

                                // NOTE: this is temporary
                                std::memset(deviceBuffer, 0, BUFFERSIZE);
                            }
                            else if (bytesRead == 0)
                            {
                                // FIXME: NUll byte has to be handled better than
                                // verbose logging.
                                
                                //logInfo(std::format("\nNo data read from {}", devicePort));
                            }
                            else
                            {
                                logWarn("\n ** Unexpected value for dwBytesRead ** \n");
                            }
                            
                            idle = true;
                        }                        
                    }
                    Sleep(1);
                }
            }
            else
            {
                // FIXME: Logging is to verbose, can to be done better
                //std::cout << cRed << "I\\O pending failed with error "
                //          << eventError
                //          << cReset << std::endl; 
            }  
        }
        else
        {
            logWarn("Wow i really got here");
        }
        
    }
    
    logInfo("Programm Terminating");
    CloseHandle(deviceHandle);
    return (0);
}
