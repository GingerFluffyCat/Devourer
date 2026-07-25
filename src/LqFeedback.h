#pragma once
#include <cstdint>
#include <functional>
namespace apfpv {
class LqFeedback {
public:
    enum class Mode { FixedTimer, FrameDriven };
    // send_interval_ms: RSSI link-feedback uplink rate to greg's aalink on UDP 192.168.0.1:12345.
    // NOTE: this is greg's *aalink*, NOT the public OpenIPC/adaptive-link *alink* (port 9999,
    // colon score 1000-2000, ~1-5 Hz) — different protocol, don't conflate the two.
    //
    // 250 ms (4 Hz), NOT aalink's own RSSI_SAMPLE_INTERVAL_MS of 33 ms. Matching aalink's internal
    // sampling rate was cargo-culting: on the dongle path every LQ datagram is a real USB bulk-OUT
    // on the SAME pipe video RX uses, so 33 ms = 30 TX/s continuously interrupting RX for the whole
    // session. aalink cannot act on that resolution anyway — its own decision cadence is
    // UP_COOLDOWN_MS=2000 (2 s), so 33 ms is ~60x finer than anything it can use. 250 ms removes
    // ~90% of those RX interruptions while staying well inside aalink's UDP_TIMEOUT_LOOPS=10 (~1 s)
    // window before it falls back to proc-RSSI. Measured on-device (OnePlus CPH2651 + RTL8812AU vs
    // a live OpenIPC AP, 1080p H265 @90fps) together with the reorder-hold cut: parse 8.18->5.70ms,
    // decode-path sum 15.15->12.70ms, decFail 3.09->2.21%, lost 0.040->0.026%, renderFps unchanged
    // ~90. Keep any override <=500 ms to stay inside that 1 s fallback; DEVOURER_LQ_MS (host) /
    // debug.pixelpilot.lqms (Android) tune it live without a rebuild.
    struct Config { Mode mode = Mode::FixedTimer; int send_interval_ms = 250;
        int min_interval_ms = 20; int keepalive_ms = 0; };
    static int rssiPct(int dbm);
    LqFeedback() ; explicit LqFeedback(Config cfg);
    ~LqFeedback() { stop(); }   // join thread + close socket on destroy (reconnect-safe)
    bool start(const char* airIp = "192.168.0.1", uint16_t port = 12345);
    // Route each LQ datagram through the caller instead of a host UDP socket. Needed on the
    // libusb dongle path (Win/WSL): the host has NO route to the VTX, so ::sendto dead-ends;
    // the sink lets ApfpvStation TX the payload over the dongle's own IP stack (sendIpPacket).
    void setSink(std::function<void(const char* buf, int len)> fn);
    void stop();
    void update(int rssiA_dbm, int rssiB_dbm = INT32_MIN);
};
}
