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

        // printf-style send: formats into a fixed buffer, then send(). Do NOT
        // put a trailing '\n' in fmt - send() adds it. Returns false and sends
        // NOTHING if the formatted message would exceed BUF_SIZE-1 chars, so a
        // truncated half-line never goes out looking complete. The format
        // attribute makes the compiler check the args against fmt (like printf).
        bool sendf(const char* fmt, ...) __attribute__((format(printf, 2, 3)));

        // Non-blocking: reads buffered bytes and returns true as soon as ONE
        // complete message is ready, leaving anything after it untouched for
        // the next call. Call in a loop - `while (comm.poll()) { ... }` - or
        // you will fall behind a fast sender by exactly the lines you skip.
        bool poll();

        // Valid after poll() returns true: the most recent complete message,
        // null-terminated, without the newline.
        const char* lastMessage() const { return mMessage; }

    private:
        HardwareSerial& mUart;
        uint8_t mRxPin;
        uint8_t mTxPin;
        uint32_t mBaud;

        static constexpr size_t BUF_SIZE = 256; // holds the full telemetry line
        // A partial line idle longer than this is noise, not a line: a full row
        // transmits in ~6 ms, so any gap this large mid-line means the sender
        // was not actually mid-line. See poll().
        static constexpr uint32_t STALE_LINE_MS = 100;

        char mBuffer[BUF_SIZE]; // line being assembled
        size_t mLen = 0;
        uint32_t mLastByteMs = 0; // when the last byte landed in mBuffer
        char mMessage[BUF_SIZE] = {0}; // last complete line
};
