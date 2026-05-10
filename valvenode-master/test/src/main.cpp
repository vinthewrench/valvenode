/*
 * main.cpp
 *
 * Valve Master I2C test tool.
 *
 * Talks to the ATmega88PB Valve Master over I2C.
 *
 * Current target:
 *   firmware/valvenode_master.c on the real valve-master PCB.
 *
 * Notes:
 *   Field power is host-controlled. Commands that talk to valve nodes
 *   require field power to be turned on first:
 *
 *     ./valve power on
 *     ./valve ping 1
 *     ./valve closeall
 *     ./valve power off
 */

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cstdint>
#include <csignal>

#include "Valve_master.hpp"

int gVerbose_flag = 0;
int gDebug_flag   = 0;

Valve_master gDevice;

void handler(int signal)
{
    if (gDevice.isOpen()) {
        gDevice.stop();
    }

    exit(signal);
}

static void usage(const char* prog)
{
    std::printf(
        "usage:\n"
        "  %s [options] status\n"
        "  %s [options] power on\n"
        "  %s [options] power off\n"
        "  %s [options] who\n"
        "  %s [options] map\n"
        "  %s [options] ping <node 1-254>\n"
        "  %s [options] set <node 1-254> <channel 1-16> <on|off>\n"
        "  %s [options] closeall\n"
        "  %s [options] channel <node 1-254> <channel 1-16> status\n"
        "  %s [options] version master\n"
        "  %s [options] version <node 1-254>\n"
        "  %s [options] identify <node 1-254>\n"
        "  %s [options] cancel\n"
        "\n"
        "provisioning:\n"
        "  %s [options] config [node 1-254]\n"
        "  %s [options] assign <new-node 1-254>\n"
        "  %s [options] move <old-node 1-254> <new-node 1-254>\n"
        "\n"
        "fault/testing:\n"
        "  %s [options] fault set\n"
        "  %s [options] fault clear\n"
        "\n"
        "common flow:\n"
        "  %s power on\n"
        "  %s ping 1\n"
        "  %s closeall\n"
        "  %s power off\n"
        "\n"
        "options:\n"
        "  -a, --addr <addr>  I2C address, default 0x09. Accepts decimal or hex.\n"
        "  -v, --verbose      enable verbose output\n"
        "  -d, --debug        enable debug output\n",
        prog, prog, prog, prog, prog, prog, prog, prog, prog,
        prog, prog, prog, prog,
        prog, prog, prog,
        prog, prog,
        prog, prog, prog, prog
    );
}

static bool parse_u8(const char* s, uint8_t& out)
{
    if (s == nullptr || *s == '\0') {
        return false;
    }

    char* end = nullptr;
    unsigned long v = std::strtoul(s, &end, 0);

    if (*end != '\0' || v > 255) {
        return false;
    }

    out = static_cast<uint8_t>(v);
    return true;
}

static bool parse_node(const char* s, uint8_t& node)
{
    if (!parse_u8(s, node)) {
        return false;
    }

    return Valve_master::validNode(node);
}

static bool parse_channel(const char* s, uint8_t& channel)
{
    if (!parse_u8(s, channel)) {
        return false;
    }

    return Valve_master::validChannel(channel);
}

static int parse_global_options(int argc, char* argv[], int& cmd_index, uint8_t& addr)
{
    cmd_index = 1;

    while (cmd_index < argc) {
        const char* arg = argv[cmd_index];

        if (std::strcmp(arg, "-v") == 0 || std::strcmp(arg, "--verbose") == 0) {
            gVerbose_flag++;
            cmd_index++;
            continue;
        }

        if (std::strcmp(arg, "-d") == 0 || std::strcmp(arg, "--debug") == 0) {
            gDebug_flag++;
            cmd_index++;
            continue;
        }

        if (std::strcmp(arg, "-a") == 0 || std::strcmp(arg, "--addr") == 0) {
            if ((cmd_index + 1) >= argc) {
                std::fprintf(stderr, "missing argument for %s\n", arg);
                return -1;
            }

            if (!parse_u8(argv[cmd_index + 1], addr)) {
                std::fprintf(stderr, "bad I2C address: %s\n", argv[cmd_index + 1]);
                return -1;
            }

            cmd_index += 2;
            continue;
        }

        break;
    }

    return 0;
}

static const char* result_name(uint8_t result)
{
    switch (result) {
    case Valve_master::RESULT_OK:                  return "OK";
    case Valve_master::RESULT_BAD_COMMAND:         return "BAD_COMMAND";
    case Valve_master::RESULT_BAD_NODE:            return "BAD_NODE";
    case Valve_master::RESULT_BAD_CHANNEL:         return "BAD_CHANNEL";
    case Valve_master::RESULT_NODE_NOT_FOUND:      return "NODE_NOT_FOUND";
    case Valve_master::RESULT_UNSUPPORTED_CHANNEL: return "UNSUPPORTED_CHANNEL";
    case Valve_master::RESULT_CONFIG_REQUIRED:     return "CONFIG_REQUIRED";
    case Valve_master::RESULT_ADDRESS_IN_USE:      return "ADDRESS_IN_USE";
    case Valve_master::RESULT_BUSY:                return "BUSY";

#ifdef VALVE_MASTER_HAS_RESULT_RS485_TIMEOUT
    case Valve_master::RESULT_RS485_TIMEOUT:       return "RS485_TIMEOUT";
#endif

#ifdef VALVE_MASTER_HAS_RESULT_RS485_BAD_CHECKSUM
    case Valve_master::RESULT_RS485_BAD_CHECKSUM:  return "RS485_BAD_CHECKSUM";
#endif

#ifdef VALVE_MASTER_HAS_RESULT_RS485_BAD_REPLY
    case Valve_master::RESULT_RS485_BAD_REPLY:     return "RS485_BAD_REPLY";
#endif

#ifdef VALVE_MASTER_HAS_RESULT_POWER_OFF
    case Valve_master::RESULT_POWER_OFF:           return "POWER_OFF";
#endif

    default:
        /*
         * Keep local fallbacks here so this test tool still decodes newer
         * firmware results even if Valve_master.hpp has not been updated yet.
         */
        switch (result) {
        case 0x09: return "RS485_TIMEOUT";
        case 0x0A: return "RS485_BAD_CHECKSUM";
        case 0x0B: return "RS485_BAD_REPLY";
        case 0x0C: return "RESERVED_0C";
        case 0x0E: return "POWER_OFF";
        default:   return "UNKNOWN";
        }
    }
}

static void print_status()
{
    uint8_t status = 0;
    uint8_t result = 0;
    uint8_t power = 0;

    if (!gDevice.getStatus(status)) {
        std::fprintf(stderr, "failed to read status\n");
        return;
    }

    if (!gDevice.getLastResult(result)) {
        std::fprintf(stderr, "failed to read result\n");
        return;
    }

    if (!gDevice.getPowerState(power)) {
        std::fprintf(stderr, "failed to read power state\n");
        return;
    }

    std::printf("status: 0x%02X\n", status);
    std::printf("  busy:  %s\n", (status & Valve_master::STATUS_BUSY) ? "yes" : "no");
    std::printf("  error: %s\n", (status & Valve_master::STATUS_ERROR) ? "yes" : "no");
    std::printf("  power: %s\n", (status & Valve_master::STATUS_POWER_ON) ? "on" : "off");
    std::printf("power_state: %u\n", power);
    std::printf("result: 0x%02X %s\n", result, result_name(result));
}

static void print_reply()
{
    Valve_master::Reply reply{};

    if (!gDevice.getReply(reply)) {
        std::fprintf(stderr, "failed to read reply registers\n");
        return;
    }

    std::printf("reply: node=%u cmd=", reply.node);

    if (reply.cmd >= 32 && reply.cmd <= 126) {
        std::printf("%c", static_cast<char>(reply.cmd));
    } else {
        std::printf("0x%02X", reply.cmd);
    }

    std::printf(" arg0=0x%02X arg1=0x%02X", reply.arg0, reply.arg1);

    if (reply.cmd == 'R') {
        std::printf(" channel=%u state=%s",
                    reply.arg0,
                    reply.arg1 == 'O' ? "OPEN" :
                    reply.arg1 == 'C' ? "CLOSED" : "UNKNOWN");
    } else if (reply.cmd == 'V') {
        uint16_t version = static_cast<uint16_t>((reply.arg0 << 8) | reply.arg1);
        std::printf(" version=%02u.%02u",
                    Valve_master::versionMajor(version),
                    Valve_master::versionMinor(version));
    }

    std::printf("\n");
}

static void print_map()
{
    uint8_t nodes[Valve_master::NODE_MAP_BYTES];
    uint8_t count = 0;

    if (!gDevice.readNodeMap(nodes, count)) {
        std::fprintf(stderr, "failed to read node map\n");
        return;
    }

    std::printf("nodes: %u\n", count);

    for (uint8_t i = 0; i < count; ++i) {
        std::printf("  %u\n", nodes[i]);
    }
}

static void print_failure_status(const char* what)
{
    uint8_t result = 0;

    if (gDevice.getLastResult(result)) {
        std::fprintf(stderr, "%s failed: 0x%02X %s\n", what, result, result_name(result));
    } else {
        std::fprintf(stderr, "%s failed\n", what);
    }

    if (gDebug_flag || gVerbose_flag) {
        print_status();
    }
}

int main(int argc, char* argv[])
{
    signal(SIGINT, handler);
    signal(SIGTERM, handler);

    uint8_t addr = Valve_master::DEFAULT_I2C_ADDR;

    int cmd_index = 1;
    if (parse_global_options(argc, argv, cmd_index, addr) != 0) {
        usage(argv[0]);
        return 1;
    }

    if (cmd_index >= argc) {
        usage(argv[0]);
        return 1;
    }

    if (gVerbose_flag) {
        std::printf("verbose enabled\n");
    }

    if (gDebug_flag) {
        std::printf("debug enabled\n");
    }

    if (gVerbose_flag || gDebug_flag) {
        std::printf("I2C address: 0x%02X\n", addr);
    }

    const char* cmd = argv[cmd_index];

    if (!gDevice.begin(addr)) {
        std::printf("Failed to open Valve Master at I2C address 0x%02X\n", addr);
        return 1;
    }

    if (gVerbose_flag || gDebug_flag) {
        std::printf("Opened Valve Master at I2C address 0x%02X\n", gDevice.getDevAddr());
    }

    int exit_code = 0;

    if (std::strcmp(cmd, "status") == 0) {
        if ((argc - cmd_index) != 1) {
            usage(argv[0]);
            exit_code = 1;
            goto done;
        }

        print_status();
        goto done;
    }

    if (std::strcmp(cmd, "power") == 0) {
        if ((argc - cmd_index) != 2) {
            usage(argv[0]);
            exit_code = 1;
            goto done;
        }

        const char* state = argv[cmd_index + 1];

        if (std::strcmp(state, "on") == 0) {
            if (!gDevice.powerOn()) {
                print_failure_status("POWER_ON");
                exit_code = 1;
            } else {
                print_status();
            }
            goto done;
        }

        if (std::strcmp(state, "off") == 0) {
            if (!gDevice.powerOff()) {
                print_failure_status("POWER_OFF");
                exit_code = 1;
            } else {
                print_status();
            }
            goto done;
        }

        usage(argv[0]);
        exit_code = 1;
        goto done;
    }

    if (std::strcmp(cmd, "who") == 0) {
        if ((argc - cmd_index) != 1) {
            usage(argv[0]);
            exit_code = 1;
            goto done;
        }

        if (!gDevice.whoScan()) {
            print_failure_status("WHO");
            exit_code = 1;
        } else {
            print_map();
            print_reply();
        }
        goto done;
    }

    if (std::strcmp(cmd, "map") == 0) {
        if ((argc - cmd_index) != 1) {
            usage(argv[0]);
            exit_code = 1;
            goto done;
        }

        print_map();
        goto done;
    }

    if (std::strcmp(cmd, "ping") == 0) {
        if ((argc - cmd_index) != 2) {
            usage(argv[0]);
            exit_code = 1;
            goto done;
        }

        uint8_t node = 0;
        if (!parse_node(argv[cmd_index + 1], node)) {
            std::fprintf(stderr, "bad node: %s\n", argv[cmd_index + 1]);
            exit_code = 1;
            goto done;
        }

        if (!gDevice.pingNode(node)) {
            char what[64];
            std::snprintf(what, sizeof(what), "PING node %u", node);
            print_failure_status(what);
            exit_code = 1;
        } else {
            print_reply();
        }

        goto done;
    }

    if (std::strcmp(cmd, "set") == 0) {
        if ((argc - cmd_index) != 4) {
            usage(argv[0]);
            exit_code = 1;
            goto done;
        }

        uint8_t node = 0;
        uint8_t channel = 0;

        if (!parse_node(argv[cmd_index + 1], node)) {
            std::fprintf(stderr, "bad node: %s\n", argv[cmd_index + 1]);
            exit_code = 1;
            goto done;
        }

        if (!parse_channel(argv[cmd_index + 2], channel)) {
            std::fprintf(stderr, "bad channel: %s\n", argv[cmd_index + 2]);
            exit_code = 1;
            goto done;
        }

        const char* state_text = argv[cmd_index + 3];

        bool on = false;

        if (std::strcmp(state_text, "on") == 0) {
            on = true;
        } else if (std::strcmp(state_text, "off") == 0) {
            on = false;
        } else {
            std::fprintf(stderr, "bad state: %s, expected on or off\n", state_text);
            exit_code = 1;
            goto done;
        }

        if (!gDevice.setChannel(node, channel, on)) {
            char what[96];
            std::snprintf(what, sizeof(what),
                          "SET node %u channel %u %s",
                          node,
                          channel,
                          on ? "on" : "off");
            print_failure_status(what);
            exit_code = 1;
        } else {
            print_reply();
        }

        goto done;
    }

    if (std::strcmp(cmd, "closeall") == 0) {
        if ((argc - cmd_index) != 1) {
            usage(argv[0]);
            exit_code = 1;
            goto done;
        }

        if (!gDevice.closeAll()) {
            print_failure_status("CLOSE_ALL");
            exit_code = 1;
        } else {
            print_status();
        }

        goto done;
    }

    if (std::strcmp(cmd, "channel") == 0) {
        if ((argc - cmd_index) != 4 || std::strcmp(argv[cmd_index + 3], "status") != 0) {
            usage(argv[0]);
            exit_code = 1;
            goto done;
        }

        uint8_t node = 0;
        uint8_t channel = 0;
        uint8_t state = 0;

        if (!parse_node(argv[cmd_index + 1], node)) {
            std::fprintf(stderr, "bad node: %s\n", argv[cmd_index + 1]);
            exit_code = 1;
            goto done;
        }

        if (!parse_channel(argv[cmd_index + 2], channel)) {
            std::fprintf(stderr, "bad channel: %s\n", argv[cmd_index + 2]);
            exit_code = 1;
            goto done;
        }

        if (!gDevice.getChannelStatus(node, channel, state)) {
            char what[96];
            std::snprintf(what, sizeof(what),
                          "CHANNEL STATUS node %u channel %u",
                          node,
                          channel);
            print_failure_status(what);
            exit_code = 1;
        } else {
            std::printf("node=%u channel=%u state=%s\n",
                        node,
                        channel,
                        state == 'O' ? "OPEN" :
                        state == 'C' ? "CLOSED" : "UNKNOWN");
            print_reply();
        }

        goto done;
    }

    if (std::strcmp(cmd, "version") == 0) {
        if ((argc - cmd_index) != 2) {
            usage(argv[0]);
            exit_code = 1;
            goto done;
        }

        Valve_master::FirmwareVersion version = 0;

        if (std::strcmp(argv[cmd_index + 1], "master") == 0) {
            if (!gDevice.getMasterVersion(version)) {
                print_failure_status("MASTER VERSION");
                exit_code = 1;
            } else {
                std::printf("master version: %02u.%02u (0x%04X)\n",
                            Valve_master::versionMajor(version),
                            Valve_master::versionMinor(version),
                            version);
            }

            goto done;
        }

        uint8_t node = 0;
        if (!parse_node(argv[cmd_index + 1], node)) {
            std::fprintf(stderr, "bad node: %s\n", argv[cmd_index + 1]);
            exit_code = 1;
            goto done;
        }

        if (!gDevice.getNodeVersion(node, version)) {
            char what[64];
            std::snprintf(what, sizeof(what), "VERSION node %u", node);
            print_failure_status(what);
            exit_code = 1;
        } else {
            std::printf("node %u version: %02u.%02u (0x%04X)\n",
                        node,
                        Valve_master::versionMajor(version),
                        Valve_master::versionMinor(version),
                        version);
            print_reply();
        }

        goto done;
    }

    if (std::strcmp(cmd, "identify") == 0) {
        if ((argc - cmd_index) != 2) {
            usage(argv[0]);
            exit_code = 1;
            goto done;
        }

        uint8_t node = 0;
        if (!parse_node(argv[cmd_index + 1], node)) {
            std::fprintf(stderr, "bad node: %s\n", argv[cmd_index + 1]);
            exit_code = 1;
            goto done;
        }

        if (!gDevice.identifyNode(node)) {
            char what[64];
            std::snprintf(what, sizeof(what), "IDENTIFY node %u", node);
            print_failure_status(what);
            exit_code = 1;
        } else {
            print_reply();
        }

        goto done;
    }

    if (std::strcmp(cmd, "cancel") == 0) {
        if ((argc - cmd_index) != 1) {
            usage(argv[0]);
            exit_code = 1;
            goto done;
        }

        if (!gDevice.cancel()) {
            print_failure_status("CANCEL");
            exit_code = 1;
        } else {
            print_reply();
        }

        goto done;
    }

    if (std::strcmp(cmd, "config") == 0) {
        if ((argc - cmd_index) != 1 && (argc - cmd_index) != 2) {
            usage(argv[0]);
            exit_code = 1;
            goto done;
        }

        uint8_t node = 0;

        if ((argc - cmd_index) == 2) {
            if (!parse_node(argv[cmd_index + 1], node)) {
                std::fprintf(stderr, "bad node: %s\n", argv[cmd_index + 1]);
                exit_code = 1;
                goto done;
            }
        }

        if (!gDevice.config(node)) {
            char what[64];
            std::snprintf(what, sizeof(what), "CONFIG node %u", node);
            print_failure_status(what);
            exit_code = 1;
        } else {
            print_reply();
        }

        goto done;
    }

    if (std::strcmp(cmd, "assign") == 0) {
        if ((argc - cmd_index) != 2) {
            usage(argv[0]);
            exit_code = 1;
            goto done;
        }

        uint8_t node = 0;
        if (!parse_node(argv[cmd_index + 1], node)) {
            std::fprintf(stderr, "bad new node: %s\n", argv[cmd_index + 1]);
            exit_code = 1;
            goto done;
        }

        if (!gDevice.assign(node)) {
            char what[64];
            std::snprintf(what, sizeof(what), "ASSIGN node %u", node);
            print_failure_status(what);
            exit_code = 1;
        } else {
            print_reply();
            print_map();
        }

        goto done;
    }

    if (std::strcmp(cmd, "move") == 0) {
        if ((argc - cmd_index) != 3) {
            usage(argv[0]);
            exit_code = 1;
            goto done;
        }

        uint8_t old_node = 0;
        uint8_t new_node = 0;

        if (!parse_node(argv[cmd_index + 1], old_node)) {
            std::fprintf(stderr, "bad old node: %s\n", argv[cmd_index + 1]);
            exit_code = 1;
            goto done;
        }

        if (!parse_node(argv[cmd_index + 2], new_node)) {
            std::fprintf(stderr, "bad new node: %s\n", argv[cmd_index + 2]);
            exit_code = 1;
            goto done;
        }

        if (!gDevice.moveNode(old_node, new_node)) {
            char what[64];
            std::snprintf(what, sizeof(what), "MOVE %u -> %u", old_node, new_node);
            print_failure_status(what);
            exit_code = 1;
        } else {
            std::printf("node moved: %u -> %u\n", old_node, new_node);
            print_map();
        }

        goto done;
    }

    if (std::strcmp(cmd, "fault") == 0) {
        if ((argc - cmd_index) != 2) {
            usage(argv[0]);
            exit_code = 1;
            goto done;
        }

        const char* state = argv[cmd_index + 1];

        if (std::strcmp(state, "set") == 0) {
            if (!gDevice.setError()) {
                print_failure_status("SET_ERROR");
                exit_code = 1;
            } else {
                print_status();
            }

            goto done;
        }

        if (std::strcmp(state, "clear") == 0) {
            if (!gDevice.clearError()) {
                print_failure_status("CLEAR_ERROR");
                exit_code = 1;
            } else {
                print_status();
            }

            goto done;
        }

        usage(argv[0]);
        exit_code = 1;
        goto done;
    }

    usage(argv[0]);
    exit_code = 1;

done:
    if (gDevice.isOpen()) {
        gDevice.stop();
    }

    return exit_code;
}
