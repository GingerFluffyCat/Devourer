// APFPV station probe — Linux/WSL harness.
// Opens the AU dongle over libusb, runs the FULL ApfpvStation connect chain
// (scan -> arm -> auth -> assoc -> WPA2 -> DHCP) exactly like the Android JNI
// (apfpv_jni.cpp ensureStation + nativeStaConnect), and prints the state funnel.
// Used to capture the STATION-path TX in usbmon and find why the auth gets no reply.
//   Build: cmake + make ApfpvProbe.  Run: DEVOURER_PID=0x881a ./ApfpvProbe
#ifdef _WIN32
#define WIN32_LEAN_AND_MEAN
#include <winsock2.h>
#include <ws2tcpip.h>
#endif
#include "WiFiDriver.h"
#include "RtlJaguarDevice.h"
#include "RtlUsbAdapter.h"
#include "ApfpvStation.h"
#include "logger.h"
#include <libusb.h>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <thread>
#include <chrono>
#include <atomic>

static const char* stateName(int s) {
    static const char* n[] = {"Idle","Scanning","Arming","Authenticating","Associating",
        "Handshaking","Dhcp","Streaming","FailNoAp","FailTx","FailNoAck","FailAuth",
        "FailDhcp","LinkLost","Reconnecting"};
    return (s >= 0 && s < 15) ? n[s] : "?";
}

#ifdef _WIN32
// Local TCP relay: plink connects to 127.0.0.1:<localPort>, we bridge raw bytes to the VTX's
// dropbear (:22) over the dongle's devourer TCP stack. This is the "SSH over the dongle" path —
// no PC network route to the VTX needed. Runs on the main thread for up to runSecs.
static void runSshProxy(apfpv::ApfpvStation& station, uint32_t vtx, uint16_t dport, int localPort, int runSecs) {
    WSADATA w; if (WSAStartup(MAKEWORD(2,2), &w)) { printf("[sshproxy] WSAStartup fail\n"); return; }
    SOCKET ls = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    BOOL yes = 1; setsockopt(ls, SOL_SOCKET, SO_REUSEADDR, (char*)&yes, sizeof(yes));
    sockaddr_in a{}; a.sin_family = AF_INET; a.sin_port = htons((u_short)localPort);
    a.sin_addr.s_addr = htonl(0x7f000001);   // 127.0.0.1
    if (bind(ls, (sockaddr*)&a, sizeof(a)) || listen(ls, 1)) {
        printf("[sshproxy] bind/listen fail %d\n", WSAGetLastError()); closesocket(ls); WSACleanup(); return;
    }
    printf("[sshproxy] listening 127.0.0.1:%d  ->  VTX:%u over the dongle (%ds window)\n",
           localPort, dport, runSecs); fflush(stdout);
    auto tEnd = std::chrono::steady_clock::now() + std::chrono::seconds(runSecs);
    while (std::chrono::steady_clock::now() < tEnd) {
        fd_set rf; FD_ZERO(&rf); FD_SET(ls, &rf); timeval tv{1, 0};
        if (select(0, &rf, nullptr, nullptr, &tv) <= 0) continue;
        SOCKET cs = accept(ls, nullptr, nullptr);
        if (cs == INVALID_SOCKET) continue;
        printf("[sshproxy] client connected; SYN -> VTX:%u\n", dport); fflush(stdout);
        if (!station.tcpConnect(vtx, dport, 5000)) { printf("[sshproxy] tcpConnect FAILED\n"); closesocket(cs); continue; }
        printf("[sshproxy] VTX TCP established; relaying\n"); fflush(stdout);
        u_long nb = 1; ioctlsocket(cs, FIONBIO, &nb);
        bool open = true; long upBytes = 0, downBytes = 0; const char* why = "window";
        while (open && std::chrono::steady_clock::now() < tEnd) {
            char buf[2048];
            int r = recv(cs, buf, sizeof(buf), 0);
            if (r > 0)       { station.tcpSend((const uint8_t*)buf, (size_t)r); upBytes += r; }
            else if (r == 0) { why = "client-FIN"; break; }           // client closed
            else if (WSAGetLastError() != WSAEWOULDBLOCK) { why = "client-err"; break; }
            std::string down; int got = station.tcpPoll(down, 20);
            if (!down.empty()) { send(cs, down.data(), (int)down.size(), 0); downBytes += (long)down.size(); }
            if (got < 0) { open = false; why = "vtx-close"; }         // VTX closed
        }
        station.tcpClose();
        closesocket(cs);
        printf("[sshproxy] session ended (%s) up=%ld down=%ld bytes\n", why, upBytes, downBytes); fflush(stdout);
    }
    closesocket(ls); WSACleanup();
}
#endif

int main(int argc, char** argv) {
    auto logger = std::make_shared<Logger>();
    libusb_context* ctx = nullptr;
    libusb_init(&ctx);

    uint16_t vid = 0x0bda, pid = 0x881a;
    if (const char* p = std::getenv("DEVOURER_PID")) pid = (uint16_t)strtoul(p, nullptr, 0);
    libusb_device_handle* handle = libusb_open_device_with_vid_pid(ctx, vid, pid);
    if (!handle) { printf("ERROR: no device %04x:%04x\n", vid, pid); return 1; }
    if (libusb_kernel_driver_active(handle, 0)) libusb_detach_kernel_driver(handle, 0);
    if (!std::getenv("DEVOURER_SKIP_RESET")) libusb_reset_device(handle);
    if (libusb_claim_interface(handle, 0) != 0) { printf("ERROR: claim_interface\n"); return 1; }

    // libusb EVENT LOOP — REQUIRED: RtlUsbAdapter::send_packet uses async
    // libusb_submit_transfer; without a thread pumping libusb_handle_events the
    // TX URB is submitted but never completed -> the auth frame never reaches the
    // wire (0 bulk-OUT). The APFPV path (apfpv_jni) currently lacks this.
    std::atomic<bool> evtRun{true};
    std::thread evtThread([&]() {
        while (evtRun.load()) {
            struct timeval tv { 0, 100000 };
            libusb_handle_events_timeout_completed(ctx, &tv, nullptr);
        }
    });

    WiFiDriver driver(logger);
    auto rtl = driver.CreateRtlDevice(handle);
    if (!rtl) { printf("ERROR: CreateRtlDevice null\n"); evtRun=false; evtThread.join(); return 1; }

    std::atomic<int> last{-1};
    auto onState = [&last](apfpv::ApfpvStation::State s) {
        int si = (int)s;
        if (si != last.exchange(si)) { printf("STATE -> %d (%s)\n", si, stateName(si)); fflush(stdout); }
    };
    // APFPV_RTP_DUMP=/path → append raw RTP payloads (rig video-validity check: parse H265 NAL types
    // to confirm HW-decrypt produces decodable H.265, not mangled bytes).
    FILE* rtpDump = nullptr;
    if (const char* p = std::getenv("APFPV_RTP_DUMP")) rtpDump = std::fopen(p, "wb");
    auto onRtp = [rtpDump](const uint8_t* d, size_t n) {
        if (rtpDump && d && n && n < 65536) {   // length-prefixed (2B BE) so we can split packets offline
            uint8_t len[2] = { (uint8_t)(n >> 8), (uint8_t)(n & 0xff) };
            std::fwrite(len, 1, 2, rtpDump); std::fwrite(d, 1, n, rtpDump); std::fflush(rtpDump);
        }
    };

    apfpv::ApfpvStation station(&rtl->adapter(), &rtl->radioManager(), onRtp, onState);
    station.setDevice(rtl.get());

    apfpv::ApfpvStation::Params prm;
    prm.ssid = "OpenIPC";
    prm.passphrase = "12345678";
    if (const char* s = std::getenv("APFPV_SSID")) prm.ssid = s;
    if (const char* p = std::getenv("APFPV_PASS")) prm.passphrase = p;
    prm.bandwidth = 80;              // 80 MHz for full throughput (phone app uses 80; the
                                     // Params default is 20 — without this the probe caps at 20 MHz)
    if (const char* b = std::getenv("APFPV_BW")) prm.bandwidth = atoi(b);
    prm.scan = true;                 // sweep to find the AP's real channel + BSSID
    prm.channel = 6;                 // hint
    if (const char* c = std::getenv("APFPV_CHANNEL")) prm.channel = atoi(c);
    // Static-IP (skip DHCP) — for the Windows ICS hotspot that won't lease.
    // APFPV_STATIC_IP="192.168.137.50" APFPV_STATIC_GW="192.168.137.1"
    if (const char* sip = std::getenv("APFPV_STATIC_IP")) {
        unsigned a,b,c2,d; if (sscanf(sip,"%u.%u.%u.%u",&a,&b,&c2,&d)==4) prm.staticIp=(a<<24)|(b<<16)|(c2<<8)|d;
        prm.staticNetmask = 0xFFFFFF00u;
        if (const char* gw = std::getenv("APFPV_STATIC_GW")) { unsigned e,f,g,h; if (sscanf(gw,"%u.%u.%u.%u",&e,&f,&g,&h)==4) prm.staticGateway=(e<<24)|(f<<16)|(g<<8)|h; }
    }
    prm.haveBssid = false;

    printf("=== APFPV station probe: connecting to \"%s\" ===\n", prm.ssid.c_str()); fflush(stdout);
    station.connect(prm);
    if (prm.staticIp) { fprintf(stderr, "ip=%u.%u.%u.%u (static)\n", (prm.staticIp>>24)&255,(prm.staticIp>>16)&255,(prm.staticIp>>8)&255,prm.staticIp&255); fflush(stderr); }

    int secs = 25;
    if (const char* s = std::getenv("APFPV_SECONDS")) secs = atoi(s);
    // APFPV_HTTP=<path>: after the link is up, issue an HTTP GET to the VTX (192.168.0.1:80) OVER
    // THE DONGLE (devourer's own TCP stack) — used to probe the WebUI + drive aalink/bitrate CGIs
    // without any PC-side network route to the VTX. APFPV_HTTP_PORT overrides the port.
    if (const char* hp = std::getenv("APFPV_HTTP")) {
        std::this_thread::sleep_for(std::chrono::seconds(7));   // let it reach STREAMING + get the lease
        uint32_t vtx = station.leaseServerIp();
        if (!vtx) vtx = 0xC0A80001u;                            // 192.168.0.1 fallback
        uint16_t port = 80; if (const char* pp = std::getenv("APFPV_HTTP_PORT")) port = (uint16_t)atoi(pp);
        printf("=== HTTP GET '%s' -> %u.%u.%u.%u:%u (over dongle) ===\n",
               hp, (vtx>>24)&255,(vtx>>16)&255,(vtx>>8)&255,vtx&255, port); fflush(stdout);
        std::string resp = station.httpGet(vtx, port, hp, 6000);
        printf("=== HTTP RESP (%zu bytes) ===\n%s\n=== END HTTP ===\n",
               resp.size(), resp.substr(0, 1500).c_str()); fflush(stdout);
        int rem = secs - 7; if (rem > 0) std::this_thread::sleep_for(std::chrono::seconds(rem));
    } else if (const char* sp = std::getenv("APFPV_SSHPROXY")) {
#ifdef _WIN32
        // Wait for the DHCP lease (tcpConnect needs it) — up to 40s — instead of a fixed sleep.
        uint32_t vtx = 0;
        for (int i = 0; i < 80; ++i) {
            vtx = station.leaseServerIp();
            if (station.leaseIp() && vtx) { printf("[sshproxy] lease up (ip=%u.%u.%u.%u) after %ds\n",
                (station.leaseIp()>>24)&255,(station.leaseIp()>>16)&255,(station.leaseIp()>>8)&255,station.leaseIp()&255, i/2); fflush(stdout); break; }
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
        }
        if (!vtx) vtx = 0xC0A80001u;
        int lport = atoi(sp); if (lport <= 0) lport = 2222;
        uint16_t dport = 22; if (const char* dp = std::getenv("APFPV_SSHPROXY_DPORT")) dport = (uint16_t)atoi(dp);
        int runSecs = secs > 10 ? secs - 8 : 30;
        runSshProxy(station, vtx, dport, lport, runSecs);
#else
        printf("APFPV_SSHPROXY only supported on Windows\n");
        std::this_thread::sleep_for(std::chrono::seconds(secs));
#endif
    } else {
        std::this_thread::sleep_for(std::chrono::seconds(secs));
    }

    station.disconnect();
    evtRun = false;
    if (evtThread.joinable()) evtThread.join();
    libusb_release_interface(handle, 0);
    libusb_close(handle);
    libusb_exit(ctx);
    printf("=== probe done ===\n");
    return 0;
}
