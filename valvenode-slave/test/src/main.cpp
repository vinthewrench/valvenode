/**
 * @file main.cpp
 * @brief Valve Node RS-485 serial test tool.
 *
 * This utility talks directly to a working valve-node slave through a
 * USB-to-RS485 adapter. It is intended for bench testing, node provisioning,
 * address reassignment, and direct protocol debugging without the valve master
 * board in the path.
 *
 * Protocol summary
 * ----------------
 *
 * Request format:
 *   :DDC[A]\r
 *
 * Reply format:
 *   :DDC[A...][KK]\r
 *
 * Field meanings:
 *   - ':'  start of frame
 *   - DD   two hex digit node address
 *   - C    one character command
 *   - A    optional one character argument
 *   - KK   two hex digit checksum on replies
 *
 * Request checksums are omitted because the current slave firmware accepts
 * checksumless requests. Reply checksums are always verified.
 *
 * Normal examples:
 *
 *   DEV=/dev/cu.usbserial-BG01DITM
 *
 ./vnode -d "$DEV" who

 ./vnode -d "$DEV" ping 1
 ./vnode -d "$DEV" ping 2

 ./vnode -d "$DEV" status 1 1
 ./vnode -d "$DEV" status 1 2
 ./vnode -d "$DEV" status 2 1
 ./vnode -d "$DEV" status 2 2

 ./vnode -d "$DEV" open 1 1
 ./vnode -d "$DEV" close 1 1
 ./vnode -d "$DEV" open 1 2
 ./vnode -d "$DEV" close 1 2

 ./vnode -d "$DEV" open 2 1
 ./vnode -d "$DEV" close 2 1
 ./vnode -d "$DEV" open 2 2
 ./vnode -d "$DEV" close 2 2

 ./vnode -d "$DEV" version 1
 ./vnode -d "$DEV" version 2

 ./vnode -d "$DEV" identify 1
 ./vnode -d "$DEV" identify 2
 ./vnode -d "$DEV" cancel


 * Provisioning examples:
 *
 *   # Put an unassigned node into config mode. Use only when exactly one
 *   # unassigned node is on the bus.
 *   ./vnode -d "$DEV" config
 *
 *   # Assign the node currently in config mode to address 3.
 *   ./vnode -d "$DEV" assign 3
 *
 *   # Move an already-addressed node from 2 to 5.
 *   ./vnode -d "$DEV" move 2 5
 *
 * Debug example:
 *
 *   ./vnode -d "$DEV" raw :FFX
 */

#include <cerrno>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <vector>

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <sys/select.h>

/** @brief Default serial device used when -d is omitted. */
static const char *DEFAULT_DEVICE = "/dev/ttyUSB0";

/** @brief Default UART baud rate used for the valve-node protocol. */
static const int DEFAULT_BAUD = 9600;

/** @brief Default per-command reply timeout, in milliseconds. */
static const int DEFAULT_TIMEOUT_MS = 500;

/**
 * @brief Broadcast WHO collection timeout, in milliseconds.
 *
 * The slave firmware staggers WHO replies by node address using 20 ms slots.
 * Six seconds covers node addresses through 254 with margin.
 */
static const int WHO_TIMEOUT_MS = 6000;

/**
 * @brief Print command-line usage text.
 *
 * @param prog Program name from argv[0].
 */
static void usage(const char *prog)
{
    std::fprintf(stderr,
        "usage:\n"
        "  %s [-d device] [-b baud] ping <node>\n"
        "  %s [-d device] [-b baud] open <node> <channel>\n"
        "  %s [-d device] [-b baud] close <node> <channel>\n"
        "  %s [-d device] [-b baud] closeall\n"
        "  %s [-d device] [-b baud] status <node> <channel>\n"
        "  %s [-d device] [-b baud] version <node>\n"
        "  %s [-d device] [-b baud] identify <node>\n"
        "  %s [-d device] [-b baud] who\n"
        "\n"
        "provisioning:\n"
        "  %s [-d device] [-b baud] config [node]\n"
        "  %s [-d device] [-b baud] assign <new-node>\n"
        "  %s [-d device] [-b baud] move <old-node> <new-node>\n"
        "  %s [-d device] [-b baud] cancel\n"
        "\n"
        "debug:\n"
        "  %s [-d device] [-b baud] raw <frame>\n"
        "\n"
        "examples:\n"
        "  %s -d /dev/ttyUSB0 ping 4\n"
        "  %s -d /dev/cu.usbserial-XXXX open 4 1\n"
        "  %s -d /dev/cu.usbserial-XXXX config\n"
        "  %s -d /dev/cu.usbserial-XXXX assign 3\n"
        "  %s -d /dev/cu.usbserial-XXXX move 2 5\n",
        prog, prog, prog, prog, prog, prog, prog, prog,
        prog, prog, prog, prog,
        prog,
        prog, prog, prog, prog, prog);
}

/**
 * @brief Convert an integer baud rate into a termios speed constant.
 *
 * @param baud Baud rate requested by the user.
 * @return Matching termios speed value, or 0 if unsupported.
 */
static speed_t baud_to_speed(int baud)
{
    switch (baud) {
        case 9600:   return B9600;
        case 19200:  return B19200;
        case 38400:  return B38400;
        case 57600:  return B57600;
        case 115200: return B115200;
        default:     return 0;
    }
}

/**
 * @brief Parse an unsigned 8-bit integer from a C string.
 *
 * The parser accepts decimal, octal, or hex input as supported by strtol()
 * with base 0.
 *
 * @param s Input string.
 * @param[out] out Parsed value on success.
 * @retval true Input was valid and within 0..255.
 * @retval false Input was missing, malformed, or out of range.
 */
static bool parse_u8(const char *s, uint8_t &out)
{
    if (!s || !*s) {
        return false;
    }

    char *end = nullptr;
    long v = std::strtol(s, &end, 0);

    if (*end != '\0') {
        return false;
    }

    if (v < 0 || v > 255) {
        return false;
    }

    out = static_cast<uint8_t>(v);
    return true;
}

/**
 * @brief Parse a valid assigned node address.
 *
 * Valid assigned nodes are 1..254. Address 0 is unassigned and address 255 is
 * broadcast, so both are rejected for normal directed commands.
 *
 * @param s Input string.
 * @param[out] node Parsed node address.
 * @retval true Node address is valid for directed commands.
 * @retval false Node address is invalid.
 */
static bool parse_node(const char *s, uint8_t &node)
{
    if (!parse_u8(s, node)) {
        return false;
    }

    if (node == 0 || node == 255) {
        return false;
    }

    return true;
}

/**
 * @brief Parse a human-facing channel number.
 *
 * The test tool accepts channels 1..16. Current two-channel slave firmware only
 * implements channels 1 and 2, but the tool keeps the command surface wider for
 * future node variants.
 *
 * @param s Input string.
 * @param[out] channel Parsed channel number.
 * @retval true Channel is within 1..16.
 * @retval false Channel is invalid.
 */
static bool parse_channel(const char *s, uint8_t &channel)
{
    if (!parse_u8(s, channel)) {
        return false;
    }

    return channel >= 1 && channel <= 16;
}

/**
 * @brief Convert a human-facing channel number into the one-character wire code.
 *
 * Existing working slaves use '1' for channel 1 and '2' for channel 2. To keep
 * that behavior while allowing sixteen possible channel codes, this mapper uses
 * '1'..'9' for channels 1..9, 'A'..'F' for channels 10..15, and '0' for
 * channel 16.
 *
 * @param channel Human-facing channel number, 1..16.
 * @param[out] out One-character protocol argument.
 * @retval true Channel was converted.
 * @retval false Channel was out of range.
 */
static bool channel_to_wire_char(uint8_t channel, char &out)
{
    if (channel < 1 || channel > 16) {
        return false;
    }

    if (channel <= 9) {
        out = static_cast<char>('0' + channel);
    } else if (channel <= 15) {
        out = static_cast<char>('A' + (channel - 10));
    } else {
        out = '0';
    }

    return true;
}

/**
 * @brief Convert a one-character wire channel code into a channel number.
 *
 * @param c Wire channel character from a reply.
 * @param[out] channel Decoded human-facing channel number.
 * @retval true Character was a recognized channel code.
 * @retval false Character was not a valid channel code.
 */
static bool wire_char_to_channel(char c, uint8_t &channel)
{
    if (c >= '1' && c <= '9') {
        channel = static_cast<uint8_t>(c - '0');
        return true;
    }

    if (c >= 'A' && c <= 'F') {
        channel = static_cast<uint8_t>(10 + (c - 'A'));
        return true;
    }

    if (c >= 'a' && c <= 'f') {
        channel = static_cast<uint8_t>(10 + (c - 'a'));
        return true;
    }

    if (c == '0') {
        channel = 16;
        return true;
    }

    return false;
}

/**
 * @brief Compute the protocol checksum for a body string.
 *
 * The checksum is the 8-bit sum of the ASCII body bytes. The ':' start byte and
 * line ending are not included.
 *
 * @param body Frame body without ':' and without line ending.
 * @return 8-bit checksum.
 */
static uint8_t checksum_body(const std::string &body)
{
    uint8_t sum = 0;

    for (unsigned char c : body) {
        sum = static_cast<uint8_t>(sum + c);
    }

    return sum;
}

/**
 * @brief Convert one ASCII hex digit to a numeric nibble value.
 *
 * @param c ASCII character to parse.
 * @return 0..15 on success, or -1 if not a hex digit.
 */
static int hex_value(char c)
{
    if (c >= '0' && c <= '9') {
        return c - '0';
    }

    if (c >= 'A' && c <= 'F') {
        return 10 + (c - 'A');
    }

    if (c >= 'a' && c <= 'f') {
        return 10 + (c - 'a');
    }

    return -1;
}

/**
 * @brief Parse two ASCII hex digits into one byte.
 *
 * @param s Pointer to at least two ASCII characters.
 * @param[out] out Parsed byte value.
 * @retval true Both characters were valid hex digits.
 * @retval false Either character was invalid.
 */
static bool parse_hex_byte(const char *s, uint8_t &out)
{
    int hi = hex_value(s[0]);
    int lo = hex_value(s[1]);

    if (hi < 0 || lo < 0) {
        return false;
    }

    out = static_cast<uint8_t>((hi << 4) | lo);
    return true;
}

/**
 * @brief Build a checksumless request frame.
 *
 * @param dest Destination node address. Use 0xFF for broadcast.
 * @param cmd One-character command code.
 * @param arg Optional one-character command argument, or '\0' for none.
 * @return Complete request frame ending in carriage return.
 */
static std::string make_request(uint8_t dest, char cmd, char arg = '\0')
{
    char buf[16];

    if (arg != '\0') {
        std::snprintf(buf, sizeof(buf), ":%02X%c%c\r", dest, cmd, arg);
    } else {
        std::snprintf(buf, sizeof(buf), ":%02X%c\r", dest, cmd);
    }

    return std::string(buf);
}

/**
 * @brief Normalize a user-supplied raw frame.
 *
 * Adds a leading ':' if missing and appends '\r' if no line ending is present.
 *
 * @param s Raw user input frame body or complete frame.
 * @return Normalized frame suitable for transmission.
 */
static std::string normalize_raw_frame(const char *s)
{
    if (s == nullptr) {
        return std::string();
    }

    std::string frame(s);

    if (frame.empty()) {
        return frame;
    }

    if (frame[0] != ':') {
        frame.insert(frame.begin(), ':');
    }

    if (frame.empty() || (frame.back() != '\r' && frame.back() != '\n')) {
        frame.push_back('\r');
    }

    return frame;
}

/**
 * @brief Open and configure a POSIX serial port for 8N1 raw I/O.
 *
 * @param device Serial device path.
 * @param baud Baud rate.
 * @return Open file descriptor on success, or -1 on failure.
 */
static int open_serial(const char *device, int baud)
{
    int fd = ::open(device, O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd < 0) {
        std::fprintf(stderr, "open failed for %s: %s\n", device, std::strerror(errno));
        return -1;
    }

    speed_t speed = baud_to_speed(baud);
    if (speed == 0) {
        std::fprintf(stderr, "unsupported baud rate: %d\n", baud);
        ::close(fd);
        return -1;
    }

    struct termios tty;
    std::memset(&tty, 0, sizeof(tty));

    if (tcgetattr(fd, &tty) != 0) {
        std::fprintf(stderr, "tcgetattr failed: %s\n", std::strerror(errno));
        ::close(fd);
        return -1;
    }

    cfmakeraw(&tty);

    cfsetispeed(&tty, speed);
    cfsetospeed(&tty, speed);

    tty.c_cflag |= static_cast<tcflag_t>(CLOCAL | CREAD);
    tty.c_cflag &= static_cast<tcflag_t>(~PARENB);
    tty.c_cflag &= static_cast<tcflag_t>(~CSTOPB);
    tty.c_cflag &= static_cast<tcflag_t>(~CSIZE);
    tty.c_cflag |= CS8;

#ifdef CRTSCTS
    tty.c_cflag &= static_cast<tcflag_t>(~CRTSCTS);
#endif

    tty.c_cc[VMIN] = 0;
    tty.c_cc[VTIME] = 0;

    if (tcsetattr(fd, TCSANOW, &tty) != 0) {
        std::fprintf(stderr, "tcsetattr failed: %s\n", std::strerror(errno));
        ::close(fd);
        return -1;
    }

    tcflush(fd, TCIOFLUSH);

    int flags = fcntl(fd, F_GETFL, 0);
    if (flags >= 0) {
        fcntl(fd, F_SETFL, flags & ~O_NONBLOCK);
    }

    return fd;
}

/**
 * @brief Write an entire string to a file descriptor.
 *
 * Retries interrupted writes and calls tcdrain() before returning so the serial
 * driver has accepted the full frame for transmission.
 *
 * @param fd Open serial file descriptor.
 * @param s Data to write.
 * @retval true Full string was written.
 * @retval false A write error occurred.
 */
static bool write_all(int fd, const std::string &s)
{
    const char *p = s.data();
    size_t left = s.size();

    while (left > 0) {
        ssize_t n = ::write(fd, p, left);
        if (n < 0) {
            if (errno == EINTR) {
                continue;
            }

            std::fprintf(stderr, "write failed: %s\n", std::strerror(errno));
            return false;
        }

        p += n;
        left -= static_cast<size_t>(n);
    }

    tcdrain(fd);
    return true;
}

/**
 * @brief Read one non-empty CR/LF-terminated line from the serial port.
 *
 * @param fd Open serial file descriptor.
 * @param[out] line Received line without CR or LF.
 * @param timeout_ms Timeout for each wait interval, in milliseconds.
 * @retval true A complete non-empty line was read.
 * @retval false Timeout or read/select error occurred.
 */
static bool read_line(int fd, std::string &line, int timeout_ms)
{
    line.clear();

    while (true) {
        fd_set rfds;
        FD_ZERO(&rfds);
        FD_SET(fd, &rfds);

        struct timeval tv;
        tv.tv_sec = timeout_ms / 1000;
        tv.tv_usec = (timeout_ms % 1000) * 1000;

        int rc = select(fd + 1, &rfds, nullptr, nullptr, &tv);
        if (rc < 0) {
            if (errno == EINTR) {
                continue;
            }

            std::fprintf(stderr, "select failed: %s\n", std::strerror(errno));
            return false;
        }

        if (rc == 0) {
            return false;
        }

        char c = 0;
        ssize_t n = ::read(fd, &c, 1);
        if (n < 0) {
            if (errno == EINTR) {
                continue;
            }

            std::fprintf(stderr, "read failed: %s\n", std::strerror(errno));
            return false;
        }

        if (n == 0) {
            continue;
        }

        if (c == '\r' || c == '\n') {
            if (!line.empty()) {
                return true;
            }

            continue;
        }

        if (line.size() < 128) {
            line.push_back(c);
        }
    }
}

/**
 * @brief Verify the checksum on a received reply line.
 *
 * Replies are expected to begin with ':' and end with two ASCII hex checksum
 * digits. The checksum covers the reply body after ':' and before the checksum.
 *
 * @param line Reply line without CR/LF.
 * @retval true Reply checksum is valid.
 * @retval false Reply is malformed or checksum does not match.
 */
static bool verify_reply_checksum(const std::string &line)
{
    if (line.size() < 5) {
        return false;
    }

    if (line[0] != ':') {
        return false;
    }

    const size_t len = line.size();

    uint8_t got = 0;
    if (!parse_hex_byte(line.c_str() + len - 2, got)) {
        return false;
    }

    std::string body = line.substr(1, len - 3);
    uint8_t expect = checksum_body(body);

    return got == expect;
}

/**
 * @brief Check whether a reply is a valid ACK from a specific node.
 *
 * @param line Reply line without CR/LF.
 * @param expected_node Node address expected to send the ACK.
 * @retval true Reply is a checksum-valid ACK from expected_node.
 * @retval false Reply is not the expected ACK.
 */
static bool reply_is_ack_from(const std::string &line, uint8_t expected_node)
{
    if (!verify_reply_checksum(line)) {
        return false;
    }

    if (line.size() < 6 || line[0] != ':') {
        return false;
    }

    uint8_t node = 0;
    if (!parse_hex_byte(line.c_str() + 1, node)) {
        return false;
    }

    return node == expected_node && line[3] == 'A';
}

/**
 * @brief Print and decode a received reply line.
 *
 * The function prints the raw reply, verifies its checksum, extracts the node
 * address, and displays a short human-readable decode of known reply types.
 *
 * @param line Reply line without CR/LF.
 */
static void print_reply_decode(const std::string &line)
{
    std::printf("rx: %s", line.c_str());

    if (!verify_reply_checksum(line)) {
        std::printf("  [bad checksum]\n");
        return;
    }

    std::printf("  [checksum ok]");

    if (line.size() < 6 || line[0] != ':') {
        std::printf("\n");
        return;
    }

    uint8_t node = 0;
    if (!parse_hex_byte(line.c_str() + 1, node)) {
        std::printf("\n");
        return;
    }

    char cmd = line[3];

    switch (cmd) {
        case 'A':
            std::printf(" node=%u ACK", node);
            break;

        case 'E':
            std::printf(" node=%u ERROR", node);
            break;

        case 'W':
            std::printf(" node=%u WHO", node);
            break;

        case 'B':
            std::printf(" node=%u BOOT", node);
            break;

        case 'V':
            std::printf(" node=%u VERSION", node);
            if (line.size() >= 10) {
                std::printf(" %c%c%c%c", line[4], line[5], line[6], line[7]);
            }
            break;

        case 'I':
            std::printf(" node=%u IDENTIFY", node);
            break;

        case 'R':
            if (line.size() >= 8) {
                uint8_t channel = 0;
                if (wire_char_to_channel(line[4], channel)) {
                    std::printf(" node=%u CHANNEL %u %s",
                        node,
                        channel,
                        line[5] == 'O' ? "OPEN" :
                        line[5] == 'C' ? "CLOSED" : "UNKNOWN");
                } else {
                    std::printf(" node=%u CHANNEL %c %s",
                        node,
                        line[4],
                        line[5] == 'O' ? "OPEN" :
                        line[5] == 'C' ? "CLOSED" : "UNKNOWN");
                }
            } else {
                std::printf(" node=%u STATE", node);
            }
            break;

        default:
            std::printf(" node=%u cmd=%c", node, cmd);
            break;
    }

    std::printf("\n");
}

/**
 * @brief Print a transmitted frame in readable form.
 *
 * CR and LF are printed as escaped sequences so the exact transmitted frame is
 * visible in the terminal.
 *
 * @param frame Frame being transmitted.
 */
static void print_tx(const std::string &frame)
{
    std::printf("tx: ");
    for (char c : frame) {
        if (c == '\r') {
            std::printf("\\r");
        } else if (c == '\n') {
            std::printf("\\n");
        } else {
            std::putchar(c);
        }
    }
    std::printf("\n");
}

/**
 * @brief Send one frame and wait for one reply.
 *
 * @param fd Open serial file descriptor.
 * @param frame Request frame to transmit.
 * @param timeout_ms Reply timeout in milliseconds.
 * @retval true A reply was received and printed.
 * @retval false Transmission failed or no reply arrived.
 */
static bool send_and_read_one(int fd, const std::string &frame, int timeout_ms)
{
    print_tx(frame);

    if (!write_all(fd, frame)) {
        return false;
    }

    std::string line;
    if (!read_line(fd, line, timeout_ms)) {
        std::fprintf(stderr, "timeout waiting for reply\n");
        return false;
    }

    print_reply_decode(line);
    return true;
}

/**
 * @brief Send one frame and require an ACK from a specific node.
 *
 * Used by the move command so address reassignment stops if the old node did
 * not enter config mode successfully.
 *
 * @param fd Open serial file descriptor.
 * @param frame Request frame to transmit.
 * @param expected_node Node expected to return ACK.
 * @param timeout_ms Reply timeout in milliseconds.
 * @retval true Expected ACK was received.
 * @retval false Transmission failed, timeout occurred, or reply was not ACK.
 */
static bool send_and_read_ack_from(int fd, const std::string &frame, uint8_t expected_node, int timeout_ms)
{
    print_tx(frame);

    if (!write_all(fd, frame)) {
        return false;
    }

    std::string line;
    if (!read_line(fd, line, timeout_ms)) {
        std::fprintf(stderr, "timeout waiting for ACK from node %u\n", expected_node);
        return false;
    }

    print_reply_decode(line);

    if (!reply_is_ack_from(line, expected_node)) {
        std::fprintf(stderr, "expected ACK from node %u\n", expected_node);
        return false;
    }

    return true;
}

/**
 * @brief Send a frame for a command that does not reply.
 *
 * The current slave firmware does not ACK broadcast cancel. A short delay after
 * tcdrain() gives USB serial adapters time to finish handing off the frame
 * before the program closes the port.
 *
 * @param fd Open serial file descriptor.
 * @param frame Frame to transmit.
 * @retval true Frame was written.
 * @retval false Transmission failed.
 */
static bool send_no_reply_expected(int fd, const std::string &frame)
{
    print_tx(frame);

    if (!write_all(fd, frame)) {
        return false;
    }

    usleep(20000);

    std::printf("no reply expected\n");
    return true;
}

/**
 * @brief Send one frame and collect zero or more replies until timeout.
 *
 * Used for broadcast WHO, where multiple nodes reply in address-based time
 * slots.
 *
 * @param fd Open serial file descriptor.
 * @param frame Request frame to transmit.
 * @param total_timeout_ms Total collection window in milliseconds.
 * @retval true At least one reply was received.
 * @retval false No replies arrived or transmission failed.
 */
static bool send_and_read_many(int fd, const std::string &frame, int total_timeout_ms)
{
    print_tx(frame);

    if (!write_all(fd, frame)) {
        return false;
    }

    bool any = false;

    int remaining_ms = total_timeout_ms;
    while (remaining_ms > 0) {
        std::string line;

        int slice_ms = remaining_ms > 250 ? 250 : remaining_ms;
        if (read_line(fd, line, slice_ms)) {
            print_reply_decode(line);
            any = true;
        }

        remaining_ms -= slice_ms;
    }

    if (!any) {
        std::fprintf(stderr, "no replies\n");
        return false;
    }

    return true;
}

/**
 * @brief Program entry point.
 *
 * Parses command-line options, builds the requested valve-node protocol frame,
 * opens the serial port, executes the transaction, and returns process status.
 *
 * @param argc Argument count.
 * @param argv Argument vector.
 * @return 0 on success, 1 on runtime failure, or 2 on usage error.
 */
int main(int argc, char **argv)
{
    const char *device = DEFAULT_DEVICE;
    int baud = DEFAULT_BAUD;

    int argi = 1;
    while (argi < argc) {
        if (std::strcmp(argv[argi], "-d") == 0) {
            if (argi + 1 >= argc) {
                usage(argv[0]);
                return 2;
            }

            device = argv[argi + 1];
            argi += 2;
            continue;
        }

        if (std::strcmp(argv[argi], "-b") == 0) {
            if (argi + 1 >= argc) {
                usage(argv[0]);
                return 2;
            }

            baud = std::atoi(argv[argi + 1]);
            argi += 2;
            continue;
        }

        break;
    }

    if (argi >= argc) {
        usage(argv[0]);
        return 2;
    }

    const char *cmd = argv[argi++];

    std::string frame;
    int timeout_ms = DEFAULT_TIMEOUT_MS;
    bool read_many = false;
    bool no_reply_expected = false;
    bool move_command = false;
    uint8_t move_old_node = 0;
    uint8_t move_new_node = 0;

    if (std::strcmp(cmd, "ping") == 0) {
        if (argi + 1 != argc) {
            usage(argv[0]);
            return 2;
        }

        uint8_t node = 0;
        if (!parse_node(argv[argi], node)) {
            std::fprintf(stderr, "invalid node: %s\n", argv[argi]);
            return 2;
        }

        frame = make_request(node, 'P');

    } else if (std::strcmp(cmd, "open") == 0 || std::strcmp(cmd, "close") == 0) {
        if (argi + 2 != argc) {
            usage(argv[0]);
            return 2;
        }

        uint8_t node = 0;
        uint8_t channel = 0;

        if (!parse_node(argv[argi], node)) {
            std::fprintf(stderr, "invalid node: %s\n", argv[argi]);
            return 2;
        }

        if (!parse_channel(argv[argi + 1], channel)) {
            std::fprintf(stderr, "invalid channel: %s\n", argv[argi + 1]);
            return 2;
        }

        char c = (std::strcmp(cmd, "open") == 0) ? 'O' : 'C';
        char channel_char = 0;
        if (!channel_to_wire_char(channel, channel_char)) {
            std::fprintf(stderr, "invalid channel: %u\n", channel);
            return 2;
        }

        frame = make_request(node, c, channel_char);

    } else if (std::strcmp(cmd, "closeall") == 0) {
        if (argi != argc) {
            usage(argv[0]);
            return 2;
        }

        /*
         * Broadcast close-all.
         *
         * Frame:
         *   :FFC\r
         *
         * Meaning:
         *   FF = broadcast address
         *   C  = close command with no channel argument
         *
         * No reply is expected. Nodes that support this command should close
         * their local valve outputs and remain silent to avoid bus collisions.
         */
        frame = make_request(0xFF, 'C');
        no_reply_expected = true;

    } else if (std::strcmp(cmd, "status") == 0) {
        if (argi + 2 != argc) {
            usage(argv[0]);
            return 2;
        }

        uint8_t node = 0;
        uint8_t channel = 0;

        if (!parse_node(argv[argi], node)) {
            std::fprintf(stderr, "invalid node: %s\n", argv[argi]);
            return 2;
        }

        if (!parse_channel(argv[argi + 1], channel)) {
            std::fprintf(stderr, "invalid channel: %s\n", argv[argi + 1]);
            return 2;
        }

        char channel_char = 0;
        if (!channel_to_wire_char(channel, channel_char)) {
            std::fprintf(stderr, "invalid channel: %u\n", channel);
            return 2;
        }

        frame = make_request(node, 'S', channel_char);

    } else if (std::strcmp(cmd, "version") == 0) {
        if (argi + 1 != argc) {
            usage(argv[0]);
            return 2;
        }

        uint8_t node = 0;
        if (!parse_node(argv[argi], node)) {
            std::fprintf(stderr, "invalid node: %s\n", argv[argi]);
            return 2;
        }

        frame = make_request(node, 'V');

    } else if (std::strcmp(cmd, "identify") == 0) {
        if (argi + 1 != argc) {
            usage(argv[0]);
            return 2;
        }

        uint8_t node = 0;
        if (!parse_node(argv[argi], node)) {
            std::fprintf(stderr, "invalid node: %s\n", argv[argi]);
            return 2;
        }

        frame = make_request(node, 'I');

    } else if (std::strcmp(cmd, "who") == 0) {
        if (argi != argc) {
            usage(argv[0]);
            return 2;
        }

        frame = make_request(0xFF, 'W');
        timeout_ms = WHO_TIMEOUT_MS;
        read_many = true;

    } else if (std::strcmp(cmd, "config") == 0) {
        uint8_t node = 0;

        if (argi == argc) {
            node = 0;
        } else if (argi + 1 == argc) {
            if (!parse_node(argv[argi], node)) {
                std::fprintf(stderr, "invalid node: %s\n", argv[argi]);
                return 2;
            }
        } else {
            usage(argv[0]);
            return 2;
        }

        if (node == 0) {
            std::fprintf(stderr,
                "warning: config with no node sends :00M and may put every unassigned node on the bus into config mode.\n");
        }

        frame = make_request(node, 'M');

    } else if (std::strcmp(cmd, "assign") == 0) {
        if (argi + 1 != argc) {
            usage(argv[0]);
            return 2;
        }

        uint8_t new_node = 0;
        if (!parse_node(argv[argi], new_node)) {
            std::fprintf(stderr, "invalid new-node: %s\n", argv[argi]);
            return 2;
        }

        std::fprintf(stderr,
            "assigning address %u to whichever node is currently in config mode.\n",
            new_node);

        frame = make_request(new_node, 'N');

    } else if (std::strcmp(cmd, "move") == 0) {
        if (argi + 2 != argc) {
            usage(argv[0]);
            return 2;
        }

        if (!parse_node(argv[argi], move_old_node)) {
            std::fprintf(stderr, "invalid old-node: %s\n", argv[argi]);
            return 2;
        }

        if (!parse_node(argv[argi + 1], move_new_node)) {
            std::fprintf(stderr, "invalid new-node: %s\n", argv[argi + 1]);
            return 2;
        }

        if (move_old_node == move_new_node) {
            std::fprintf(stderr, "old-node and new-node are the same\n");
            return 2;
        }

        move_command = true;

    } else if (std::strcmp(cmd, "cancel") == 0) {
        if (argi != argc) {
            usage(argv[0]);
            return 2;
        }

        frame = make_request(0xFF, 'X');
        no_reply_expected = true;

    } else if (std::strcmp(cmd, "raw") == 0) {
        if (argi + 1 != argc) {
            usage(argv[0]);
            return 2;
        }

        frame = normalize_raw_frame(argv[argi]);

    } else {
        std::fprintf(stderr, "unknown command: %s\n", cmd);
        usage(argv[0]);
        return 2;
    }

    int fd = open_serial(device, baud);
    if (fd < 0) {
        return 1;
    }

    std::printf("device: %s\n", device);
    std::printf("baud:   %d\n", baud);

    bool ok = false;

    if (move_command) {
        std::printf("moving node %u to node %u\n", move_old_node, move_new_node);

        std::string config_frame = make_request(move_old_node, 'M');
        std::string assign_frame = make_request(move_new_node, 'N');

        ok = send_and_read_ack_from(fd, config_frame, move_old_node, DEFAULT_TIMEOUT_MS);
        if (ok) {
            ok = send_and_read_ack_from(fd, assign_frame, move_new_node, DEFAULT_TIMEOUT_MS);
        }

        if (ok) {
            std::printf("node moved: %u -> %u\n", move_old_node, move_new_node);
        }
    } else if (no_reply_expected) {
        ok = send_no_reply_expected(fd, frame);
    } else if (read_many) {
        ok = send_and_read_many(fd, frame, timeout_ms);
    } else {
        ok = send_and_read_one(fd, frame, timeout_ms);
    }

    ::close(fd);
    return ok ? 0 : 1;
}
