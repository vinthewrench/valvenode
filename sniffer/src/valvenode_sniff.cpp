/*
 * valvenode_sniff.cpp
 *
 * ValveNode RS-485 packet sniffer / decoder.
 *
 * Default:
 *      port = /dev/tty.usbserial-BG01DITM
 *      baud = 9600
 *
 * Usage:
 *      ./valvenode_sniff [port] [baud]
 *
 * Examples:
 *      ./valvenode_sniff
 *      ./valvenode_sniff /dev/tty.usbserial-BG01DITM
 *      ./valvenode_sniff /dev/ttyUSB0 9600
 *
 * Protocol:
 *      RS-485 half-duplex
 *      9600 8N1
 *      ASCII frames:
 *
 *          :DDC...[KK]\r
 *
 *      DD = address as two ASCII hex digits
 *      C  = command/reply character
 *      KK = checksum, 8-bit sum of ASCII body bytes before checksum
 *
 * Known examples:
 *      :01AA2      -> <01> ACK
 *      :01WB8      -> <01> WHO_REPLY
 *      :02WB9      -> <02> WHO_REPLY
 *      :01R1C27    -> <01> STATUS valve=1 CLOSED
 *
 * Output:
 *      HH:MM:SS.mmm  DIR  RAW_PACKET  ADDR  DECODE
 *
 * Direction:
 *      -->  master to node / broadcast
 *      <--  node to master
 *      ???  malformed, short, garbage, partial, or ambiguous
 */

#include <cerrno>
#include <csignal>
#include <cstdint>
#include <cstdlib>
#include <cstring>

#include <chrono>
#include <fcntl.h>
#include <iomanip>
#include <iostream>
#include <poll.h>
#include <sstream>
#include <string>
#include <termios.h>
#include <unistd.h>

static constexpr const char* DEFAULT_PORT = "/dev/ttyUSB0";
static constexpr int DEFAULT_BAUD = 9600;
static constexpr int FRAME_TIMEOUT_MS = 250;

static volatile sig_atomic_t g_exitRequested = 0;

/**
 * @brief Signal handler used to request a clean shutdown.
 *
 * @param signalNumber Signal number. Currently unused.
 */
static void signalHandler(int signalNumber)
{
    (void)signalNumber;

    g_exitRequested = 1;
}

/**
 * @brief Return the current local time formatted for packet logging.
 *
 * The sniffer intentionally prints only time-of-day, not the date, because
 * this is meant for live bus watching.
 *
 * @return Timestamp string in HH:MM:SS.mmm format.
 */
static std::string timeStamp()
{
    using namespace std::chrono;

    auto now = system_clock::now();
    auto nowTime = system_clock::to_time_t(now);
    auto ms = duration_cast<milliseconds>(now.time_since_epoch()) % 1000;

    struct tm localTime {};
    localtime_r(&nowTime, &localTime);

    std::ostringstream oss;
    oss << std::put_time(&localTime, "%H:%M:%S")
        << "."
        << std::setw(3)
        << std::setfill('0')
        << ms.count();

    return oss.str();
}

/**
 * @brief Check whether a character is an ASCII hexadecimal digit.
 *
 * @param c Character to test.
 * @return true if c is 0-9, a-f, or A-F.
 */
static bool isHexChar(char c)
{
    return (c >= '0' && c <= '9') ||
           (c >= 'a' && c <= 'f') ||
           (c >= 'A' && c <= 'F');
}

/**
 * @brief Convert one ASCII hexadecimal digit to its integer value.
 *
 * @param c ASCII hex digit.
 * @return Value 0-15 on success, -1 if c is not a hex digit.
 */
static int hexValue(char c)
{
    if (c >= '0' && c <= '9') {
        return c - '0';
    }

    if (c >= 'a' && c <= 'f') {
        return 10 + c - 'a';
    }

    if (c >= 'A' && c <= 'F') {
        return 10 + c - 'A';
    }

    return -1;
}

/**
 * @brief Parse two ASCII hexadecimal characters into one byte.
 *
 * @param text Source text.
 * @param offset Offset of the first hex character.
 * @param valueOut Receives parsed byte on success.
 * @return true if two valid hex characters were parsed.
 */
static bool parseHexByte(const std::string& text, size_t offset, uint8_t& valueOut)
{
    if (offset + 2 > text.size()) {
        return false;
    }

    if (!isHexChar(text[offset]) || !isHexChar(text[offset + 1])) {
        return false;
    }

    int hi = hexValue(text[offset]);
    int lo = hexValue(text[offset + 1]);

    if (hi < 0 || lo < 0) {
        return false;
    }

    valueOut = static_cast<uint8_t>((hi << 4) | lo);
    return true;
}

/**
 * @brief Format a byte as two uppercase ASCII hexadecimal characters.
 *
 * @param value Byte value.
 * @return Two-character uppercase hex string.
 */
static std::string hexByte(uint8_t value)
{
    std::ostringstream oss;
    oss << std::uppercase
        << std::hex
        << std::setw(2)
        << std::setfill('0')
        << static_cast<unsigned int>(value);

    return oss.str();
}

/**
 * @brief Calculate the ValveNode packet checksum.
 *
 * The checksum is the low 8 bits of the sum of all ASCII bytes in the body
 * before the checksum field. The leading ':' is not included.
 *
 * Examples:
 *      "01A"   -> A2
 *      "01W"   -> B8
 *      "02W"   -> B9
 *      "01R1C" -> 27
 *
 * @param body Packet body without ':' and without checksum.
 * @return 8-bit checksum.
 */
static uint8_t checksum8(const std::string& body)
{
    uint32_t sum = 0;

    for (unsigned char c : body) {
        sum += c;
    }

    return static_cast<uint8_t>(sum & 0xff);
}

/**
 * @brief Convert an integer baud rate into a termios speed constant.
 *
 * @param baud Integer baud rate.
 * @return termios speed constant, or B0 if unsupported.
 */
static speed_t baudToSpeed(int baud)
{
    switch (baud) {
        case 1200:   return B1200;
        case 2400:   return B2400;
        case 4800:   return B4800;
        case 9600:   return B9600;
        case 19200:  return B19200;
        case 38400:  return B38400;
        case 57600:  return B57600;
        case 115200: return B115200;
        default:     return B0;
    }
}

/**
 * @brief Configure an opened serial port for raw ValveNode traffic.
 *
 * The port is configured for raw 8N1 with no hardware flow control.
 *
 * @param fd Open serial file descriptor.
 * @param baud Baud rate.
 * @return true on success.
 */
/**
 * @brief Configure an opened serial port for raw ValveNode traffic.
 *
 * The port is configured for raw 8N1 with no hardware flow control.
 *
 * @param fd Open serial file descriptor.
 * @param baud Baud rate.
 * @return true on success.
 */
static bool configureSerial(int fd, int baud)
{
    struct termios tio {};

    if (tcgetattr(fd, &tio) != 0) {
        std::cerr << "tcgetattr failed: " << strerror(errno) << "\n";
        return false;
    }

    cfmakeraw(&tio);

    speed_t speed = baudToSpeed(baud);
    if (speed == B0) {
        std::cerr << "unsupported baud rate: " << baud << "\n";
        return false;
    }

    if (cfsetispeed(&tio, speed) != 0 || cfsetospeed(&tio, speed) != 0) {
        std::cerr << "cfsetispeed/cfsetospeed failed: " << strerror(errno) << "\n";
        return false;
    }

    /*
     * 8N1, no flow control.
     */
    tio.c_cflag &= ~PARENB;
    tio.c_cflag &= ~CSTOPB;
    tio.c_cflag &= ~CSIZE;
    tio.c_cflag |= CS8;

#ifdef CRTSCTS
    tio.c_cflag &= ~CRTSCTS;
#endif

    tio.c_cflag |= CLOCAL;
    tio.c_cflag |= CREAD;

    /*
     * Non-canonical, non-blocking-ish reads.
     * poll() is still used in the main loop.
     */
    tio.c_cc[VMIN] = 0;
    tio.c_cc[VTIME] = 0;

    if (tcsetattr(fd, TCSANOW, &tio) != 0) {
        std::cerr << "tcsetattr failed: " << strerror(errno) << "\n";
        return false;
    }

    /*
     * No tcflush() here.
     *
     * This program is a passive sniffer and never writes to the serial port.
     * Flushing output is pointless, and flushing input can throw away exactly
     * the bytes we are trying to observe. If stale bytes are already buffered,
     * the parser will report them as GARBAGE/PARTIAL and resync on the next
     * ':' frame marker.
     */
    return true;
}

/**
 * @brief Parsed representation of one ValveNode ASCII frame.
 */
struct ParsedFrame {
    std::string raw;
    std::string body;
    std::string payloadBody;

    bool malformed = false;
    std::string malformedReason;

    bool shortFrame = false;

    bool hasChecksum = false;
    bool checksumOk = false;
    uint8_t gotChecksum = 0;
    uint8_t calcChecksum = 0;

    bool hasAddress = false;
    uint8_t address = 0;

    char cmd = 0;
    std::string payload;
};

/**
 * @brief Parse a raw ValveNode frame into decoded fields.
 *
 * The frame may include a leading ':' and may have already had CR/LF stripped.
 * This function accepts short bench-test frames without checksum, but validates
 * checksum whenever a plausible final two-hex-character checksum is present.
 *
 * @param rawFrame Raw frame text.
 * @return Parsed frame structure.
 */
static ParsedFrame parseFrame(const std::string& rawFrame)
{
    ParsedFrame p;
    p.raw = rawFrame;

    std::string text = rawFrame;

    while (!text.empty() && (text.back() == '\r' || text.back() == '\n')) {
        text.pop_back();
    }

    if (text.empty() || text[0] != ':') {
        p.malformed = true;
        p.malformedReason = "no_start";
        return p;
    }

    p.body = text.substr(1);
    p.payloadBody = p.body;

    if (p.body.size() < 3) {
        p.shortFrame = true;

        if (p.body.size() >= 2) {
            uint8_t addr = 0;
            if (parseHexByte(p.body, 0, addr)) {
                p.hasAddress = true;
                p.address = addr;
            }
        }

        return p;
    }

    uint8_t addr = 0;
    if (!parseHexByte(p.body, 0, addr)) {
        p.malformed = true;
        p.malformedReason = "bad_address";
        return p;
    }

    p.hasAddress = true;
    p.address = addr;

    /*
     * Checksum rule:
     *
     * For normal checked frames, the final two characters are ASCII hex.
     * The checksum is the low 8 bits of the sum of all ASCII body bytes
     * before the checksum.
     *
     * Short command examples like :01P or :01O1 are also accepted without
     * checksum because they are useful during bench testing.
     */
    if (p.body.size() >= 5) {
        uint8_t got = 0;

        if (parseHexByte(p.body, p.body.size() - 2, got)) {
            std::string withoutChecksum = p.body.substr(0, p.body.size() - 2);
            uint8_t calc = checksum8(withoutChecksum);

            p.hasChecksum = true;
            p.gotChecksum = got;
            p.calcChecksum = calc;
            p.checksumOk = (got == calc);
            p.payloadBody = withoutChecksum;
        }
    }

    if (p.payloadBody.size() < 3) {
        p.shortFrame = true;
        return p;
    }

    p.cmd = p.payloadBody[2];

    if (p.payloadBody.size() > 3) {
        p.payload = p.payloadBody.substr(3);
    }

    return p;
}

/**
 * @brief Format the address column for display.
 *
 * Normal node addresses are displayed as <01> through <FE>.
 * Broadcast FF is displayed as BRDC.
 *
 * @param p Parsed frame.
 * @return Four-character address tag.
 */
static std::string addressTag(const ParsedFrame& p)
{
    if (!p.hasAddress) {
        return "----";
    }

    if (p.address == 0xff) {
        return "BRDC";
    }

    std::ostringstream oss;
    oss << "<"
        << std::uppercase
        << std::hex
        << std::setw(2)
        << std::setfill('0')
        << static_cast<unsigned int>(p.address)
        << ">";

    return oss.str();
}

/**
 * @brief Infer packet direction from the ValveNode command contract.
 *
 * The USB-RS485 adapter is receive-only from the sniffer's point of view.
 * Direction is inferred from the packet fields:
 *
 *      -->  master to node / broadcast
 *      <--  node to master
 *      ???  malformed, short, garbage, partial, or ambiguous
 *
 * @param p Parsed frame.
 * @return Direction tag.
 */
static std::string directionTag(const ParsedFrame& p)
{
    if (p.malformed || p.shortFrame || !p.hasAddress || p.cmd == 0) {
        return "???";
    }

    /*
     * Broadcast frames are master-originated in this protocol.
     *
     * Known examples:
     *      :FFW...     WHO scan
     *      :FFC...     close all
     */
    if (p.address == 0xff) {
        return "-->";
    }

    /*
     * Definite node replies.
     */
    switch (p.cmd) {
        case 'A':
        case 'R':
            return "<--";

        case 'W':
            /*
             * Non-broadcast W is a WHO reply.
             */
            return "<--";

        case 'V':
            /*
             * V with no payload is a version request.
             * V with payload is a version reply.
             */
            return p.payload.empty() ? "-->" : "<--";

        default:
            break;
    }

    /*
     * Definite master requests.
     */
    switch (p.cmd) {
        case 'P':
        case 'O':
        case 'C':
        case 'S':
        case 'I':
        case 'M':
        case 'N':
        case 'X':
            return "-->";

        default:
            break;
    }

    return "???";
}

/**
 * @brief Convert a ValveNode status state character to display text.
 *
 * @param state State character from an R reply.
 * @return Human-readable state string.
 */
static std::string stateName(char state)
{
    switch (state) {
        case 'O':
            return "OPEN";

        case 'C':
            return "CLOSED";

        case '1':
            return "OPEN";

        case '0':
            return "CLOSED";

        default:
            return std::string("state=") + state;
    }
}

/**
 * @brief Decode one parsed frame into a compact human-readable string.
 *
 * Bad checksum is shown only when present and wrong. Good checksum is silent
 * to keep the live output clean.
 *
 * @param p Parsed frame.
 * @return Decode string.
 */
static std::string decodeFrame(const ParsedFrame& p)
{
    if (p.malformed) {
        return "MALFORMED " + p.malformedReason;
    }

    if (p.shortFrame) {
        return "SHORT";
    }

    std::ostringstream oss;

    switch (p.cmd) {
        case 'A':
            oss << "ACK";
            break;

        case 'P':
            oss << "PING";
            break;

        case 'W':
            if (p.address == 0xff) {
                oss << "WHO_SCAN";
            } else {
                oss << "WHO_REPLY";
            }
            break;

        case 'O':
            if (!p.payload.empty()) {
                oss << "OPEN valve=" << p.payload[0];
            } else {
                oss << "OPEN SHORT";
            }
            break;

        case 'C':
            if (p.address == 0xff && p.payload.empty()) {
                oss << "CLOSE_ALL";
            } else if (!p.payload.empty()) {
                oss << "CLOSE valve=" << p.payload[0];
            } else {
                oss << "CLOSE SHORT";
            }
            break;

        case 'S':
            if (!p.payload.empty()) {
                oss << "STATUS? valve=" << p.payload[0];
            } else {
                oss << "STATUS? SHORT";
            }
            break;

        case 'R':
            if (p.payload.size() >= 2) {
                oss << "STATUS valve=" << p.payload[0] << " " << stateName(p.payload[1]);
            } else {
                oss << "STATUS SHORT";
            }
            break;

        case 'V':
            if (p.payload.empty()) {
                oss << "VERSION?";
            } else if (p.payload.size() == 4 &&
                       isHexChar(p.payload[0]) &&
                       isHexChar(p.payload[1]) &&
                       isHexChar(p.payload[2]) &&
                       isHexChar(p.payload[3])) {
                uint8_t major = 0;
                uint8_t minor = 0;

                if (parseHexByte(p.payload, 0, major) &&
                    parseHexByte(p.payload, 2, minor)) {
                    oss << "VERSION "
                        << static_cast<unsigned int>(major)
                        << "."
                        << static_cast<unsigned int>(minor);
                } else {
                    oss << "VERSION payload=" << p.payload;
                }
            } else {
                oss << "VERSION payload=" << p.payload;
            }
            break;

        case 'I':
            oss << "IDENTIFY";
            break;

        case 'M':
            if (p.payload.empty()) {
                oss << "CONFIG";
            } else {
                oss << "CONFIG payload=" << p.payload;
            }
            break;

        case 'N':
            if (p.payload.empty()) {
                oss << "ASSIGN";
            } else {
                oss << "ASSIGN payload=" << p.payload;
            }
            break;

        case 'X':
            oss << "CANCEL";
            break;

        default:
            oss << "UNKNOWN cmd=" << p.cmd;

            if (!p.payload.empty()) {
                oss << " payload=" << p.payload;
            }

            break;
    }

    if (p.hasChecksum && !p.checksumOk) {
        oss << "  BAD_CKSUM got=" << hexByte(p.gotChecksum)
            << " calc=" << hexByte(p.calcChecksum);
    }

    return oss.str();
}

/**
 * @brief Print one complete decoded frame.
 *
 * @param rawFrame Complete raw frame, normally without trailing CR/LF.
 */
static void printDecodedFrame(const std::string& rawFrame)
{
    ParsedFrame p = parseFrame(rawFrame);

    std::cout << timeStamp()
              << "  "
              << directionTag(p)
              << "  "
              << std::left
              << std::setw(12)
              << rawFrame
              << "  "
              << std::setw(4)
              << addressTag(p)
              << "  "
              << decodeFrame(p)
              << "\n";

    std::cout.flush();
}

/**
 * @brief Print non-frame bytes that were seen outside a ':' started frame.
 *
 * @param garbage Bytes discarded before a frame start marker.
 */
static void printGarbage(const std::string& garbage)
{
    if (garbage.empty()) {
        return;
    }

    std::cout << timeStamp()
              << "  ???  "
              << std::left
              << std::setw(12)
              << garbage
              << "  ----  GARBAGE discarded=no_start\n";

    std::cout.flush();
}

/**
 * @brief Print a frame that started but did not finish before timeout.
 *
 * @param partial Partial frame text.
 */
static void printPartial(const std::string& partial)
{
    if (partial.empty()) {
        return;
    }

    std::cout << timeStamp()
              << "  ???  "
              << std::left
              << std::setw(12)
              << partial
              << "  ----  PARTIAL timeout_before_CR\n";

    std::cout.flush();
}

/**
 * @brief Print command-line usage.
 *
 * @param programName argv[0].
 */
static void printUsage(const char* programName)
{
    std::cerr
        << "usage: " << programName << " [port] [baud]\n"
        << "\n"
        << "defaults:\n"
        << "  port: " << DEFAULT_PORT << "\n"
        << "  baud: " << DEFAULT_BAUD << "\n"
        << "\n"
        << "examples:\n"
        << "  " << programName << "\n"
        << "  " << programName << " /dev/tty.usbserial-BG01DITM\n"
        << "  " << programName << " /dev/ttyUSB0 9600\n";
}

/**
 * @brief Program entry point.
 *
 * Opens the selected serial port, configures it for ValveNode RS-485 traffic,
 * and continuously prints decoded packets until interrupted.
 *
 * @param argc Argument count.
 * @param argv Argument vector.
 * @return 0 on clean exit, non-zero on setup/runtime failure.
 */
int main(int argc, char* argv[])
{
    const char* port = DEFAULT_PORT;
    int baud = DEFAULT_BAUD;

   std::cout.setf(std::ios::unitbuf);

    if (argc >= 2) {
        std::string arg = argv[1];

        if (arg == "-h" || arg == "--help") {
            printUsage(argv[0]);
            return 0;
        }

        port = argv[1];
    }

    if (argc >= 3) {
        char* end = nullptr;
        long parsedBaud = strtol(argv[2], &end, 10);

        if (end == argv[2] || *end != '\0' || parsedBaud <= 0 || parsedBaud > 1000000) {
            std::cerr << "invalid baud rate: " << argv[2] << "\n";
            printUsage(argv[0]);
            return 1;
        }

        baud = static_cast<int>(parsedBaud);
    }

    if (argc > 3) {
        printUsage(argv[0]);
        return 1;
    }

    signal(SIGINT, signalHandler);
    signal(SIGTERM, signalHandler);

    int fd = open(port, O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd < 0) {
        std::cerr << "open failed for " << port << ": " << strerror(errno) << "\n";
        return 1;
    }

    if (!configureSerial(fd, baud)) {
        close(fd);
        return 1;
    }

    std::cout << "ValveNode RS-485 sniffer\n";
    std::cout << "port=" << port << "  baud=" << baud << "  mode=8N1\n\n";
    std::cout.flush();

    std::string frame;
    bool inFrame = false;
    auto lastByteTime = std::chrono::steady_clock::now();

    while (!g_exitRequested) {
        struct pollfd pfd {};
        pfd.fd = fd;
        pfd.events = POLLIN;

        int pollResult = poll(&pfd, 1, 50);

        if (pollResult < 0) {
            if (errno == EINTR) {
                continue;
            }

            std::cerr << "poll failed: " << strerror(errno) << "\n";
            break;
        }

        if (pollResult == 0) {
            if (inFrame) {
                auto now = std::chrono::steady_clock::now();
                auto ageMs = std::chrono::duration_cast<std::chrono::milliseconds>(
                    now - lastByteTime
                ).count();

                if (ageMs >= FRAME_TIMEOUT_MS) {
                    printPartial(frame);
                    frame.clear();
                    inFrame = false;
                }
            }

            continue;
        }

        if ((pfd.revents & POLLIN) == 0) {
            continue;
        }

        char buffer[128];
        ssize_t bytesRead = read(fd, buffer, sizeof(buffer));

        if (bytesRead < 0) {
            if (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR) {
                continue;
            }

            std::cerr << "read failed: " << strerror(errno) << "\n";
            break;
        }

        if (bytesRead == 0) {
            continue;
        }

        for (ssize_t i = 0; i < bytesRead; ++i) {
            char c = buffer[i];
            lastByteTime = std::chrono::steady_clock::now();

            if (!inFrame) {
                if (c == ':') {
                    frame.clear();
                    frame.push_back(c);
                    inFrame = true;
                    continue;
                }

                if (c == '\r' || c == '\n') {
                    continue;
                }

                std::string garbage;
                garbage.push_back(c);

                /*
                 * Coalesce immediately-available non-frame bytes from this
                 * same read buffer until a start marker or terminator.
                 */
                while ((i + 1) < bytesRead) {
                    char next = buffer[i + 1];

                    if (next == ':' || next == '\r' || next == '\n') {
                        break;
                    }

                    garbage.push_back(next);
                    ++i;
                }

                printGarbage(garbage);
                continue;
            }

            frame.push_back(c);

            if (c == '\r' || c == '\n') {
                while (!frame.empty() && (frame.back() == '\r' || frame.back() == '\n')) {
                    frame.pop_back();
                }

                printDecodedFrame(frame);

                frame.clear();
                inFrame = false;
                continue;
            }

            /*
             * A new ':' while already in a frame usually means the previous
             * frame got truncated or the sniffer attached mid-stream.
             */
            if (c == ':' && frame.size() > 1) {
                std::string previous = frame.substr(0, frame.size() - 1);
                printPartial(previous);

                frame.clear();
                frame.push_back(':');
                inFrame = true;
                continue;
            }

            /*
             * Hard sanity limit. ValveNode packets are tiny. If this grows,
             * something is wrong on the wire.
             */
            if (frame.size() > 80) {
                printPartial(frame);
                frame.clear();
                inFrame = false;
            }
        }
    }

    if (inFrame && !frame.empty()) {
        printPartial(frame);
    }

    close(fd);
    return 0;
}
