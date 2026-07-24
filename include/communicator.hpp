#pragma once
#include <Arduino.h>

// Basic UART link to an ESP32-CAM (or any serial peer). Messages are plain
// text terminated by '\n'. Receiving is fully non-blocking: poll() drains
// whatever bytes have arrived and reassembles complete lines, so it never
// stalls the control loop.
//
// Threading: like the IMU, drive this from ONE task only - HardwareSerial is
// not safe to touch concurrently from multiple tasks.
class Communicator {
    public:
        // uart        : which hardware UART to use, e.g. &Serial1 (NOT Serial,
        //               that's the USB debug port).
        // rxPin/txPin : this board's GPIOs. Wire this RX to the ESP-CAM's TX and
        //               this TX to the ESP-CAM's RX (crossed), with common GND.
        // baud        : must match the ESP-CAM's UART baud.
        Communicator(HardwareSerial& uart, uint8_t rxPin, uint8_t txPin, uint32_t baud = 115200);

        void begin(); // call once in setup

        // Send one message; a trailing '\n' is added. Near-instant unless the
        // TX buffer fills, so keep messages short.
        void send(const char* msg);
        void send(const String& msg) { send(msg.c_str()); }

        // Non-blocking: reads all available bytes into the line buffer. Returns
        // true when a COMPLETE new message just finished this call. Call often
        // (e.g. every control cycle). If several lines arrive between polls,
        // only the newest is kept - poll fast enough if you can't miss any.
        bool poll();

        // Valid after poll() returns true: the most recent complete message,
        // null-terminated, without the newline.
        const char* lastMessage() const { return mMessage; }

    private:
        HardwareSerial& mUart;
        uint8_t mRxPin;
        uint8_t mTxPin;
        uint32_t mBaud;

        static constexpr size_t BUF_SIZE = 128;
        char mBuffer[BUF_SIZE]; // line being assembled
        size_t mLen = 0;
        char mMessage[BUF_SIZE] = {0}; // last complete line
};
