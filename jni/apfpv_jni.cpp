// ============================================================================
//  apfpv_jni.cpp — JNI bindings: ApfpvStaLink.java  <->  native ApfpvStation
#include <sys/un.h>
//  This is the REAL connective tissue: PixelPilot's Java calls these; they
//  drive the forked devourer station stack. Mirrors the existing WfbNgLink JNI
//  registration pattern (libusb fd via wrap_sys_device -> RtlUsbAdapter).
//  Goes in PixelPilot app/wfbngrtl8812/src/main/cpp/ alongside the wfb JNI.
// ============================================================================
#include <jni.h>
#include <libusb.h>
#include <sys/un.h>
#include <memory>
#include <string>
#include <thread>
#include <chrono>
#include <atomic>
#include <android/log.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <cstring>
#include <pthread.h>
#include <sched.h>
#include <sys/resource.h>
#include <unistd.h>

#include "ApfpvStation.h"
#include "WiFiDriver.h"
#include "RtlUsbAdapter.h"
#include "RtlJaguarDevice.h"

#define LOGI(...) __android_log_print(ANDROID_LOG_INFO,  "apfpv-jni", __VA_ARGS__)
#define LOGE(...) __android_log_print(ANDROID_LOG_ERROR, "apfpv-jni", __VA_ARGS__)

using namespace apfpv;

// One station per app instance (APFPV uses a single link).
struct StaCtx {
    libusb_context*               usb = nullptr;
    libusb_device_handle*         handle = nullptr;
    int                           wrappedFd = -1;   // fd ctx->handle was wrapped from (replug detection)
    std::unique_ptr<WiFiDriver>   driver;
    std::unique_ptr<RtlJaguarDevice> rtl;   // owns adapter + radio manager
    std::unique_ptr<ApfpvStation> station;
    JavaVM*                       jvm = nullptr;
    jobject                       jlink = nullptr;   // global ref to ApfpvStaLink
    int                           rtpSock = -1;     // AF_UNIX DGRAM → "\0my_socket" (UDSReceiver)
    sockaddr_un                   rtpDst{};
    socklen_t                     rtpDstLen = 0;
    // General-IP bridge for the ApfpvVpnService TUN (SSH/any TCP+UDP to the VTX over the
    // dongle). ADDITIVE + off by default — only armed when the VpnService calls
    // nativeStaSetIpBridge(true), so the RTP video path is untouched. Downlink: the RxDeframe
    // IP sink sends each decrypted IPv4 packet to UDP 127.0.0.1:5601 (the VpnService reads +
    // writes the TUN). Uplink: nativeStaSendIp() feeds TUN packets to station->sendIpPacket().
    int                           ipDownSock = -1;  // UDP → 127.0.0.1:5601 (downlink to VpnService)
    // libusb event loop: drives the async RX URB pool + async TX (send_packet)
    // for the kernel-style station I/O. Without it the async transfers never
    // complete -> the auth never radiates.
    std::thread                   evtThread;
    std::atomic<bool>             evtRun{false};
    // Set true while RX is paused/draining for a cal/TX so the event thread yields
    // instead of contending the event lock with the cal's synchronous control reads
    // (daemon-vs-direct-call serialization; critical here since tv=100ms locks long).
    std::shared_ptr<std::atomic<bool>> rxQuiesce = std::make_shared<std::atomic<bool>>(false);
    std::atomic<bool>             connecting{false};  // reject concurrent/replug double-fire connects (CreateRtlDevice race)
};

// Forward state to Java (onNativeState). The state callback can fire either on
// an internal worker thread (not attached to the JVM) OR synchronously on the
// thread that called nativeStaConnect (already attached, owned by ART). We must
// only Attach/Detach a thread WE attached — unconditionally detaching the
// caller's thread is what aborted the app (SIGABRT via ART) after libusb init.
static void postState(StaCtx* ctx, ApfpvStation::State s) {
    if (!ctx || !ctx->jvm || !ctx->jlink) return;
    JNIEnv* env = nullptr;
    bool weAttached = false;
    jint r = ctx->jvm->GetEnv(reinterpret_cast<void**>(&env), JNI_VERSION_1_6);
    if (r == JNI_EDETACHED) {
        if (ctx->jvm->AttachCurrentThread(&env, nullptr) != JNI_OK || !env) return;
        weAttached = true;
    } else if (r != JNI_OK || !env) {
        return;
    }
    jclass cls = env->GetObjectClass(ctx->jlink);
    if (cls) {
        jmethodID m = env->GetMethodID(cls, "onNativeState", "(I)V");
        if (m) env->CallVoidMethod(ctx->jlink, m, (jint)s);
        env->DeleteLocalRef(cls);
    }
    if (weAttached) ctx->jvm->DetachCurrentThread();  // never detach the caller's thread
}

// Build the device + station ONCE and keep it for the ctx's lifetime. Both the
// scan and the connect reuse it — rebuilding it (the old code did) destroyed a
// device whose RX read thread was still running, so that thread then locked a
// freed mutex -> 'pthread_mutex_lock on a destroyed mutex' SIGABRT.
static bool ensureStation(StaCtx* ctx) {
    if (ctx->station) return true;
    if (!ctx->usb || !ctx->handle) return false;
    if (ctx->rtpSock < 0) {
        ctx->rtpSock = ::socket(AF_UNIX, SOCK_DGRAM, 0);
        int sndBuf = 4 * 1024 * 1024;
        ::setsockopt(ctx->rtpSock, SOL_SOCKET, SO_SNDBUF, &sndBuf, sizeof(sndBuf));
        std::memset(&ctx->rtpDst, 0, sizeof(ctx->rtpDst));
        ctx->rtpDst.sun_family = AF_UNIX;
        ctx->rtpDst.sun_path[0] = '\0';  // abstract namespace
        strcpy(ctx->rtpDst.sun_path + 1, "my_socket");
        ctx->rtpDstLen = offsetof(sockaddr_un, sun_path) + 1 + strlen("my_socket");
    }
    StaCtx* c = ctx;
    // Per-packet RTP forward to 127.0.0.1:5600 (the proven pre-throughput path).
    // NOTE: the previous sendmmsg batching DROPPED partial batches on any >2ms inter-packet
    // gap and only flushed when the NEXT packet arrived — it shredded the RTP stream so the
    // player could not reassemble NALUs (decoded 0 frames). A blocking-free sendto to
    // loopback is a few µs and never reorders/drops, so video reassembles correctly.
    auto onRtp = [c](const uint8_t* rtp, size_t len) {
        if (c->rtpSock < 0 || !rtp || !len) return;
        // AF_UNIX abstract namespace: bypasses IP stack, ~2-3× faster than UDP loopback.
        // App's UDSReceiver already listens on "\0my_socket" (always started in VideoPlayer).
        ::sendto(c->rtpSock, rtp, len, MSG_DONTWAIT, (sockaddr*)&c->rtpDst, c->rtpDstLen);
    };
    auto onState = [c](ApfpvStation::State s){ postState(c, s); };
    try {
        // CLAIM THE USB INTERFACE before bringing the chip up — the same step the
        // working WFB path (WfbngLink.cpp) and the devourer demo do. Without it,
        // control transfers (channel tuning) still work but the BULK RX endpoint
        // delivers NOTHING — which is exactly why the scan saw 0 beacons. Detach
        // any kernel driver first, then claim interface 0. (No libusb_reset_device
        // on Android — we hold an adopted fd, not an owned device.)
        if (libusb_kernel_driver_active(ctx->handle, 0) == 1) {
            int d = libusb_detach_kernel_driver(ctx->handle, 0);
            LOGI("detach_kernel_driver: %d", d);
        }
        int cr = libusb_claim_interface(ctx->handle, 0);
        LOGI("claim_interface(0): %d", cr);

        ctx->driver = std::make_unique<WiFiDriver>(nullptr /*Logger_t*/);
        ctx->rtl = ctx->driver->CreateRtlDevice(ctx->handle);
        if (!ctx->rtl) { LOGE("CreateRtlDevice returned null"); return false; }
        // Hand the raw usbfs fd to the adapter for direct USBDEVFS_BULK OUT.
        // DEFAULT ON — bypasses libusb's OUT-behind-IN queue so the kernel USB
        // stack interleaves our OUT (ADDBA Response etc.) with the async IN URBs
        // at the hardware level. USBDEVFS_BULK is synchronous in the kernel and
        // does NOT touch libusb's async reap queue, so they coexist on one fd.
        // DEVOURER_NO_RAW_TX disables for A/B comparison.
        if (!std::getenv("DEVOURER_NO_RAW_TX") && ctx->wrappedFd >= 0) {
            uint8_t outEp = 0x03;
            if (const char* e = std::getenv("DEVOURER_TX_EP")) outEp = (uint8_t)strtoul(e, nullptr, 0);
            ctx->rtl->adapter().setTxFd(ctx->wrappedFd, outEp);
            LOGI("raw-fd TX enabled: fd=%d ep=0x%02x", ctx->wrappedFd, outEp);
        }
        ctx->station = std::make_unique<ApfpvStation>(
            &ctx->rtl->adapter(), &ctx->rtl->radioManager(), onRtp, onState);
        ctx->station->setDevice(ctx->rtl.get());
        // Clear any STUCK RX-pause carried over from a prior (mid-TX/stream) device — on a
        // replug the StaCtx (and this shared flag) is reused; if it's left true the event loop
        // yields forever -> auth TX radiates but no RX reply -> FAIL_NO_AP. A full app restart
        // "fixed" it only because it made a fresh StaCtx. Reset on every (re)build.
        ctx->rxQuiesce->store(false);
        ctx->rtl->adapter().setQuiesceFlag(ctx->rxQuiesce);  // event thread yields during cal
        // Start the libusb event loop (drives async RX URBs + async TX).
        if (!ctx->evtRun.load()) {
            ctx->evtRun.store(true);
            ctx->evtThread = std::thread([ctx]() {
                // Lever B (kernel-match): the USB reaper must be scheduled the instant a URB
                // completes, to recycle buffers back into the submit queue with minimal latency
                // (our userspace stand-in for the kernel's in-IRQ resubmit). Try SCHED_FIFO RT
                // priority; if the app lacks CAP_SYS_NICE (typical), fall back to the lowest nice
                // so the scheduler still favours us over the decode/render threads.
                {
                    struct sched_param sp{}; sp.sched_priority = 80;
                    if (pthread_setschedparam(pthread_self(), SCHED_FIFO, &sp) != 0) {
                        setpriority(PRIO_PROCESS, 0, -19);   // nice -19 fallback
                    }
                }
                while (ctx->evtRun.load()) {
                    // Yield while a cal/TX has RX paused so we don't hold the event lock
                    // against its synchronous control reads (serialization fix).
                    if (ctx->rxQuiesce->load(std::memory_order_acquire)) {
                        std::this_thread::sleep_for(std::chrono::milliseconds(1));
                        continue;
                    }
                    // Short timeout: handle_events returns immediately when a URB completes
                    // (poll-driven); the timeout only bounds idle latency for the rxQuiesce check.
                    struct timeval tv { 0, 2000 };   // 2ms (was 100ms)
                    libusb_handle_events_timeout_completed(ctx->usb, &tv, nullptr);
                }
            });
        }
    } catch (...) { LOGE("ensureStation: device setup threw"); return false; }
    return true;
}

extern "C" {

JNIEXPORT jlong JNICALL
Java_com_openipc_wfbngrtl8812_ApfpvStaLink_nativeStaInitialize(JNIEnv* env, jclass, jobject /*context*/) {
    auto* ctx = new StaCtx();
    env->GetJavaVM(&ctx->jvm);

    // Unrooted Android: we adopt a USB fd from the Java UsbManager, so libusb must
    // NOT try to enumerate /dev/bus/usb. This option MUST be set BEFORE libusb_init
    // (exactly as the WFB path does) — otherwise init fails on the device scan and
    // leaves ctx->usb null, and the later libusb_wrap_sys_device dereferences that
    // null context (the SIGSEGV at libusb1.0.so seen when switching to APFPV).
    libusb_set_option(NULL, LIBUSB_OPTION_NO_DEVICE_DISCOVERY);
    int r = libusb_init(&ctx->usb);
    if (r < 0 || !ctx->usb) {
        LOGE("libusb_init failed (%d) — APFPV station unavailable", r);
        ctx->usb = nullptr;
    }
    LOGI("ApfpvStation ctx initialized (usb=%p)", (void*)ctx->usb);
    return reinterpret_cast<jlong>(ctx);
}

JNIEXPORT void JNICALL
Java_com_openipc_wfbngrtl8812_ApfpvStaLink_nativeStaConnect(
        JNIEnv* env, jclass, jlong inst, jobject jlink, jint fd,
        jint channel, jint bandwidth, jstring jssid, jstring jpass, jstring jbssid, jstring jStaticIp) {
    auto* ctx = reinterpret_cast<StaCtx*>(inst);
    if (!ctx) return;
    // Reject a CONCURRENT connect: replug fires onUsbHotplug on ATTACHED+PERMISSION -> two
    // "apfpv-connect" threads -> two ensureStation/CreateRtlDevice on the shared ctx -> CRASH,
    // and they tear down each other's device mid-arm -> endless Reconnecting/Arming LOOP.
    bool _expected = false;
    if (!ctx->connecting.compare_exchange_strong(_expected, true)) {
        LOGI("connect already in progress — ignoring concurrent call");
        return;
    }
    struct ConnGuard { std::atomic<bool>& f; ~ConnGuard() { f.store(false); } } _cg{ctx->connecting};
    if (!ctx->jlink) ctx->jlink = env->NewGlobalRef(jlink);

    // libusb must have initialized in nativeStaInitialize (option set before init).
    // If it didn't, bail with a FAIL state instead of dereferencing a null context.
    if (!ctx->usb) {
        LOGE("libusb context null — cannot connect"); postState(ctx, ApfpvStation::State::FailNoAp); return;
    }
    // REPLUG: a re-inserted dongle is a NEW fd. The old ctx->handle drives the DEAD device
    // (no TX -> FAIL_NO_AP "not in range"). If the fd changed, fully tear down the stale
    // station/device/handle so we rebuild on the fresh fd — both the wrap-once guard below
    // AND ensureStation's "build once" guard would otherwise keep using the dead handle.
    if (ctx->handle && ctx->wrappedFd != fd) {
        LOGI("replug: fd %d -> %d, re-initialising device", ctx->wrappedFd, fd);
        if (ctx->station) ctx->station->disconnect();   // stops supervisor + RX (URB pool + sync thread)
        // Stop the libusb event loop BEFORE freeing the device — otherwise its in-flight
        // RX URB callbacks dereference the freed RtlUsbAdapter (the SIGABRT on replug).
        // ensureStation restarts the thread (its `if (!evtRun)` guard) on the new handle.
        if (ctx->evtRun.exchange(false) && ctx->evtThread.joinable()) ctx->evtThread.join();
        ctx->station.reset();
        ctx->rtl.reset();
        ctx->driver.reset();
        libusb_close(ctx->handle);
        ctx->handle = nullptr;
        // Repeatedly wrap/close adopted fds on the SAME libusb context corrupts its internal
        // device list — the 2nd replug's wrapped handle then has a NULL device -> SEGV in the
        // new RtlUsbAdapter ctor's first libusb_control_transfer. Give the new fd a FRESH
        // libusb context (same init the first connect used).
        if (ctx->usb) { libusb_exit(ctx->usb); ctx->usb = nullptr; }
        libusb_set_option(nullptr, LIBUSB_OPTION_NO_DEVICE_DISCOVERY);
        if (libusb_init(&ctx->usb) < 0 || !ctx->usb) {
            LOGE("replug: libusb re-init failed");
            postState(ctx, ApfpvStation::State::FailNoAp); return;
        }
    }
    // Unrooted USB-host path (same as the wfb monitor path): adopt the fd.
    // The scan may already have wrapped it — don't re-wrap (that leaks handles).
    if (!ctx->handle) {
        if (libusb_wrap_sys_device(ctx->usb, (intptr_t)fd, &ctx->handle) < 0 || !ctx->handle) {
            LOGE("libusb_wrap_sys_device failed (fd=%d)", fd);
            postState(ctx, ApfpvStation::State::FailNoAp); return;
        }
        ctx->wrappedFd = fd;
    }

    // Build (or reuse) the device + station — same instance the scan used, so we
    // never destroy a device whose RX thread is still running.
    if (!ensureStation(ctx)) { postState(ctx, ApfpvStation::State::FailNoAp); return; }

    const char* ssid = env->GetStringUTFChars(jssid, nullptr);
    const char* pass = env->GetStringUTFChars(jpass, nullptr);
    const char* bstr = jbssid ? env->GetStringUTFChars(jbssid, nullptr) : nullptr;
    LOGI("nativeStaConnect: channel=%d bw=%d ssid=%s bssid=%s", (int)channel, (int)bandwidth, ssid ? ssid : "(null)", bstr ? bstr : "(null)");
    try {
        // Stop any prior scan/supervisor on this device before a fresh connect.
        ctx->station->disconnect();
        ApfpvStation::Params p;
        p.channel = channel; p.bandwidth = bandwidth;
        p.ssid = ssid; p.passphrase = pass; p.lqFeedback = true;
        // Phone-assisted: parse "aa:bb:cc:dd:ee:ff" -> skip the dongle sweep.
        if (bstr && bstr[0]) {
            unsigned b[6];
            if (sscanf(bstr, "%x:%x:%x:%x:%x:%x", &b[0],&b[1],&b[2],&b[3],&b[4],&b[5]) == 6) {
                for (int i = 0; i < 6; ++i) p.bssid[i] = (uint8_t)b[i];
                p.haveBssid = true; p.scan = false;
                LOGI("BSSID parsed: %02x:%02x:%02x:%02x:%02x:%02x, haveBssid=1 scan=0",
                     p.bssid[0],p.bssid[1],p.bssid[2],p.bssid[3],p.bssid[4],p.bssid[5]);
            } else {
                LOGI("BSSID parse FAILED for '%s'", bstr);
            }
        } else {
            LOGI("No BSSID provided — will scan");
        }
        // Static-IP mode: a non-empty "a.b.c.d" from JNI -> SKIP DHCP and bind it; empty/null -> DHCP.
        if (jStaticIp) {
            const char* sip = env->GetStringUTFChars(jStaticIp, nullptr);
            unsigned a,b,c,d;
            if (sip && sscanf(sip, "%u.%u.%u.%u", &a,&b,&c,&d) == 4)
                p.staticIp = ((uint32_t)a<<24)|((uint32_t)b<<16)|((uint32_t)c<<8)|(uint32_t)d;
            if (sip) env->ReleaseStringUTFChars(jStaticIp, sip);
        }
        ctx->station->connect(p);     // runs the gated chain; states -> Java
    } catch (const std::exception& e) {
        LOGE("APFPV connect threw: %s", e.what());
        postState(ctx, ApfpvStation::State::FailNoAp);
    } catch (...) {
        LOGE("APFPV connect threw an unknown exception");
        postState(ctx, ApfpvStation::State::FailNoAp);
    }

    env->ReleaseStringUTFChars(jssid, ssid);
    env->ReleaseStringUTFChars(jpass, pass);
    if (bstr) env->ReleaseStringUTFChars(jbssid, bstr);
}

JNIEXPORT void JNICALL
Java_com_openipc_wfbngrtl8812_ApfpvStaLink_nativeStaDisconnect(JNIEnv*, jclass, jlong inst, jint /*fd*/) {
    auto* ctx = reinterpret_cast<StaCtx*>(inst);
    if (!ctx) return;
    if (ctx->station) ctx->station->disconnect();   // stops supervisor + RX + connect chain
    // App-resume reconnect: the Java side closes the UsbDeviceConnection right after this,
    // so the adopted fd is about to become invalid. Invalidate wrappedFd so the NEXT
    // nativeStaConnect ALWAYS takes the replug teardown+rebuild path (fresh libusb ctx +
    // re-wrap) — even if Android hands back the SAME fd number, we must not reuse the stale
    // handle that wraps the now-closed fd.
    ctx->wrappedFd = -1;
}

// All-SSID scan for the picker UI. Adopts the fd + builds the device/station if
// needed (scan happens BEFORE connect), then channel-hops collecting beacons and
// calls Java onNativeScanResult(ssid, channel, rssi) per discovered SSID.
// BLOCKS for the whole sweep — Java must call this on a worker thread.
JNIEXPORT void JNICALL
Java_com_openipc_wfbngrtl8812_ApfpvStaLink_nativeStaScan(
        JNIEnv* env, jclass, jlong inst, jobject jlink, jint fd, jint perChannelMs,
        jboolean includeDfs) {
    auto* ctx = reinterpret_cast<StaCtx*>(inst);
    if (!ctx || !ctx->usb) return;
    if (!ctx->jlink) ctx->jlink = env->NewGlobalRef(jlink);
    if (!ctx->handle) {
        if (libusb_wrap_sys_device(ctx->usb, (intptr_t)fd, &ctx->handle) < 0 || !ctx->handle) {
            LOGE("scan: libusb_wrap_sys_device failed (fd=%d)", fd); return;
        }
    }
    if (!ensureStation(ctx)) { LOGE("scan: ensureStation failed"); return; }
    jobject link = ctx->jlink;
    auto onAp = [ctx, link](const std::string& ssid, const ApInfo& info) {
        if (!ctx->jvm || !link) return;
        JNIEnv* e = nullptr; bool att = false;
        jint r = ctx->jvm->GetEnv(reinterpret_cast<void**>(&e), JNI_VERSION_1_6);
        if (r == JNI_EDETACHED) { if (ctx->jvm->AttachCurrentThread(&e, nullptr) != JNI_OK || !e) return; att = true; }
        else if (r != JNI_OK || !e) return;
        jclass cls = e->GetObjectClass(link);
        if (cls) {
            jmethodID m = e->GetMethodID(cls, "onNativeScanResult", "(Ljava/lang/String;II)V");
            if (m) { jstring js = e->NewStringUTF(ssid.c_str());
                     e->CallVoidMethod(link, m, js, (jint)info.channel, (jint)info.rssi);
                     if (js) e->DeleteLocalRef(js); }
            e->DeleteLocalRef(cls);
        }
        if (att) ctx->jvm->DetachCurrentThread();
    };
    // Stop any running connect/reconnect supervisor first — otherwise the
    // supervisor thread and the scan both drive the same USB device and the
    // register reads collide (rtw_read throws). disconnect() is a no-op if idle.
    ctx->station->disconnect();
    try { ctx->station->scanAll(perChannelMs > 0 ? perChannelMs : 250,
                                includeDfs == JNI_TRUE, onAp); }
    catch (...) { LOGE("scanAll threw"); }
}

// VRX EIRP-cal beacon: make the dongle broadcast `ssid` on `channel` at `txIndex`
// so a second phone can scan + measure its EIRP. Adopts the fd + builds the
// device if needed (same as scan). Non-blocking — beacons on a native thread.
JNIEXPORT void JNICALL
Java_com_openipc_wfbngrtl8812_ApfpvStaLink_nativeStaStartBeaconCal(
        JNIEnv* env, jclass, jlong inst, jint fd, jstring jssid, jint channel, jint txIndex) {
    auto* ctx = reinterpret_cast<StaCtx*>(inst);
    if (!ctx || !ctx->usb) return;
    if (!ctx->handle) {
        if (libusb_wrap_sys_device(ctx->usb, (intptr_t)fd, &ctx->handle) < 0 || !ctx->handle) {
            LOGE("beacon: libusb_wrap_sys_device failed (fd=%d)", fd); return;
        }
    }
    if (!ensureStation(ctx)) { LOGE("beacon: ensureStation failed"); return; }
    const char* ssid = env->GetStringUTFChars(jssid, nullptr);
    try { ctx->station->startBeaconCal(ssid ? ssid : "APFPV-VRX-CAL", channel, txIndex); }
    catch (...) { LOGE("startBeaconCal threw"); }
    if (ssid) env->ReleaseStringUTFChars(jssid, ssid);
}

JNIEXPORT void JNICALL
Java_com_openipc_wfbngrtl8812_ApfpvStaLink_nativeStaStopBeaconCal(JNIEnv*, jclass, jlong inst) {
    auto* ctx = reinterpret_cast<StaCtx*>(inst);
    if (ctx && ctx->station) ctx->station->stopBeaconCal();
}

// Dongle-as-AP (SoftAP) — WIP test path. Brings the dongle up as an 802.11 AP: beacons the SSID,
// answers probe/auth/assoc, runs the WPA2 authenticator, and (with DEVOURER_AP_HWACK) the Jaguar1
// HW software-beacon + per-STA ACK. Mirrors nativeStaStartBeaconCal's fd-wrap + ensureStation.
JNIEXPORT void JNICALL
Java_com_openipc_wfbngrtl8812_ApfpvStaLink_nativeStaStartAp(
        JNIEnv* env, jclass, jlong inst, jint fd, jstring jssid, jint channel, jstring jpass) {
    auto* ctx = reinterpret_cast<StaCtx*>(inst);
    if (!ctx || !ctx->usb) return;
    if (!ctx->handle) {
        if (libusb_wrap_sys_device(ctx->usb, (intptr_t)fd, &ctx->handle) < 0 || !ctx->handle) {
            LOGE("ap: libusb_wrap_sys_device failed (fd=%d)", fd); return;
        }
    }
    if (!ensureStation(ctx)) { LOGE("ap: ensureStation failed"); return; }
    const char* ssid = env->GetStringUTFChars(jssid, nullptr);
    const char* pass = jpass ? env->GetStringUTFChars(jpass, nullptr) : nullptr;
    // forceHwBeacon=true: Android can't set DEVOURER_AP_HWACK, and the whole point of the app
    // button is to exercise the Jaguar1 HW software-beacon path, so enable it explicitly.
    try { ctx->station->startAp(ssid ? ssid : "APFPV-AP", channel, pass ? pass : "", /*forceHwBeacon=*/true); }
    catch (...) { LOGE("startAp threw"); }
    if (ssid) env->ReleaseStringUTFChars(jssid, ssid);
    if (pass) env->ReleaseStringUTFChars(jpass, pass);
}

JNIEXPORT void JNICALL
Java_com_openipc_wfbngrtl8812_ApfpvStaLink_nativeStaStopAp(JNIEnv*, jclass, jlong inst) {
    auto* ctx = reinterpret_cast<StaCtx*>(inst);
    if (ctx && ctx->station) ctx->station->stopAp();
}

JNIEXPORT jint JNICALL
Java_com_openipc_wfbngrtl8812_ApfpvStaLink_nativeStaGetState(JNIEnv*, jclass, jlong inst) {
    auto* ctx = reinterpret_cast<StaCtx*>(inst);
    return (ctx && ctx->station) ? (jint)ctx->station->state() : 0;
}

JNIEXPORT jint JNICALL
Java_com_openipc_wfbngrtl8812_ApfpvStaLink_nativeStaGetChannel(JNIEnv*, jclass, jlong inst) {
    auto* ctx = reinterpret_cast<StaCtx*>(inst);
    return (ctx && ctx->station) ? (jint)ctx->station->channel() : 0;
}

JNIEXPORT jint JNICALL
Java_com_openipc_wfbngrtl8812_ApfpvStaLink_nativeStaGetRssi(JNIEnv*, jclass, jlong inst) {
    auto* ctx = reinterpret_cast<StaCtx*>(inst);
    return (ctx && ctx->station) ? (jint)ctx->station->rssiDbm() : -99;
}

// The dongle's actual DHCP lease IP (host byte order), or 0 if none yet. The VpnService TUN must
// use THIS as its address — the dongle's ARP responder + CCMP TX identify as the lease IP, so a TUN
// hardcoded to a different IP (192.168.0.10) makes the VTX unable to ARP/route replies back → SSH
// times out and aalink ignores the LQ. 0 => caller falls back to the .10 static.
JNIEXPORT jint JNICALL
Java_com_openipc_wfbngrtl8812_ApfpvStaLink_nativeStaLeaseIp(JNIEnv*, jclass, jlong inst) {
    auto* ctx = reinterpret_cast<StaCtx*>(inst);
    return (ctx && ctx->station) ? (jint)ctx->station->leaseIp() : 0;
}

JNIEXPORT void JNICALL
Java_com_openipc_wfbngrtl8812_ApfpvStaLink_nativeStaSetLqFeedback(JNIEnv*, jclass, jlong /*inst*/, jboolean /*en*/) {
    // toggles ApfpvStation's LqFeedback; stored in Params at connect for now.
}

JNIEXPORT void JNICALL
Java_com_openipc_wfbngrtl8812_ApfpvStaLink_nativeStaSetTxPower(JNIEnv*, jclass, jlong inst, jint power) {
    // GS dongle TX power index (0-63) for the station-mode dongle. Same chip and
    // same scale as the WFB-ng path; routes to the RtlJaguarDevice's SetTxPower.
    // (Phone-Wi-Fi mode never reaches here — it has no dongle and no StaCtx.)
    auto* ctx = reinterpret_cast<StaCtx*>(inst);
    if (ctx && ctx->rtl) {
        ctx->rtl->SetTxPower(static_cast<uint8_t>(power));
    }
}

JNIEXPORT jint JNICALL
Java_com_openipc_wfbngrtl8812_ApfpvStaLink_nativeStaGetTxPower(JNIEnv*, jclass, jlong inst) {
    // Reads the dongle's REAL reference TX power index (0-63) from the driver:
    // EFUSE-calibrated value when the dongle is connected, else the fallback.
    // Returns -1 if no device yet (caller should keep its own default then).
    auto* ctx = reinterpret_cast<StaCtx*>(inst);
    if (ctx && ctx->rtl) {
        return static_cast<jint>(ctx->rtl->GetTxPower());
    }
    return -1;
}

// ---- General-IP bridge for the ApfpvVpnService TUN (SSH-over-dongle) ---------------------
// Uplink: TUN → this → station->sendIpPacket (CCMP-encrypt + TX the raw IPv4 datagram).
JNIEXPORT void JNICALL
Java_com_openipc_wfbngrtl8812_ApfpvStaLink_nativeStaSendIp(JNIEnv* env, jclass, jlong inst, jbyteArray pkt, jint len) {
    auto* ctx = reinterpret_cast<StaCtx*>(inst);
    if (!ctx || !ctx->station || !pkt || len <= 0) return;
    jbyte* p = env->GetByteArrayElements(pkt, nullptr);
    if (p) {
        ctx->station->sendIpPacket(reinterpret_cast<const uint8_t*>(p), (size_t) len);
        env->ReleaseByteArrayElements(pkt, p, JNI_ABORT);
    }
}

// Arm/disarm the downlink: route decrypted general-IP packets to UDP 127.0.0.1:5601 so the
// VpnService can write them to the TUN. Off = clear the sink (RTP path unaffected either way).
JNIEXPORT void JNICALL
Java_com_openipc_wfbngrtl8812_ApfpvStaLink_nativeStaSetIpBridge(JNIEnv*, jclass, jlong inst, jboolean on) {
    auto* ctx = reinterpret_cast<StaCtx*>(inst);
    if (!ctx || !ctx->station) return;
    if (on) {
        if (ctx->ipDownSock < 0) {
            ctx->ipDownSock = ::socket(AF_INET, SOCK_DGRAM | SOCK_NONBLOCK, 0);
        }
        int sock = ctx->ipDownSock;
        StaCtx* c = ctx;
        ctx->station->setIpSink([c, sock](const uint8_t* ip, size_t n) {
            if (sock < 0 || !ip || !n) return;
            sockaddr_in dst{}; dst.sin_family = AF_INET; dst.sin_port = htons(5601);
            dst.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
            ::sendto(sock, ip, n, MSG_DONTWAIT, (sockaddr*) &dst, sizeof(dst));
        });
        LOGI("APFPV IP bridge ARMED (downlink -> udp 127.0.0.1:5601)");
    } else {
        ctx->station->setIpSink(nullptr);
        if (ctx->ipDownSock >= 0) { ::close(ctx->ipDownSock); ctx->ipDownSock = -1; }
        LOGI("APFPV IP bridge disarmed");
    }
}

} // extern "C"
