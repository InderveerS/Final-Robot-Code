#include "communicator.hpp"
#include <stdarg.h>

Communicator::Communicator(HardwareSerial& uart, uint8_t rxPin, uint8_t txPin, uint32_t baud)
    : mUart(uart), mRxPin(rxPin), mTxPin(txPin), mBaud(baud) {}

void Communicator::begin() {
    // Bigger RX buffer than the 256-byte default so a burst from the ESP-CAM
    // isn't dropped if poll() is briefly late. Must be set before begin().
    mUart.setRxBufferSize(512);
    mUart.begin(mBaud, SERIAL_8N1, mRxPin, mTxPin);
}

void Communicator::send(const char* msg) {
    mUart.print(msg);
    mUart.print('\n');
}

bool Communicator::sendf(const char* fmt, ...) {
    char buf[BUF_SIZE];
    va_list args;
    va_start(args, fmt);
    int n = vsnprintf(buf, sizeof(buf), fmt, args); // n = chars that WOULD be written
    va_end(args);

    // Drop rather than transmit a truncated line the receiver would read as
    // complete. n < 0 is a formatting error; n >= buffer size means truncation.
    if (n < 0 || n >= (int)sizeof(buf)) {
        return false;
    }
    send(buf);
    return true;
}

bool Communicator::poll() {
    while (mUart.available()) {
        char c = (char)mUart.read();
        uint32_t now = millis();

        // A real line arrives as one burst (~130 bytes is ~6 ms at 230400). A
        // partial line older than this is not a line at all - it is noise the
        // UART framed while the peer's TX pin floated during ITS reset. Drop it
        // instead of gluing it onto the front of the next good line. Without
        // this, junk with no newline in it sits here indefinitely and corrupts
        // whatever arrives next, which is always the CSV header.
        if (mLen > 0 && (now - mLastByteMs) > STALE_LINE_MS) {
            mLen = 0;
        }
        mLastByteMs = now;

        if (c == '\n' || c == '\r') {
            if (mLen > 0) {                       // ignore empty lines / bare \r\n
                mBuffer[mLen] = '\0';
                memcpy(mMessage, mBuffer, mLen + 1);
                mLen = 0;
                return true;                      // ONE line per call - see below
            }
        } else if (c < 0x20 || c > 0x7E) {
            // Non-printable, so framing garbage rather than payload - this link
            // only ever carries printable CSV. Dropping it here is what keeps a
            // stray 0x00 out of the buffer: writers use strlen, so a leading NUL
            // silently truncates the whole line to nothing.
            continue;
        } else if (mLen < BUF_SIZE - 1) {
            mBuffer[mLen++] = c;
        } else {
            mLen = 0;                             // overrun: drop the too-long line
        }
    }
    return false;
}
// Returning at the newline instead of draining is what makes back-to-back
// lines survive. Anything still unread stays in the driver's ring buffer and
// the partial line stays in mBuffer/mLen, so the next call picks up exactly
// where this one stopped. Callers must loop until poll() returns false.
