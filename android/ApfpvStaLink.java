package com.openipc.wfbngrtl8812;

import android.content.Context;
import android.hardware.usb.UsbManager;

/**
 * ApfpvStaLink — the integration seam PixelPilot must ADD to drive a
 * station-capable devourer (APFPV access-point client mode).
 *
 * WHY A NEW CLASS (not an extension of WfbNgLink):
 * WfbNgLink models WFB-ng's STATELESS BROADCAST receiver:
 *     nativeRun(channel, bandwidth, fd)  — just start listening
 *     nativeRefreshKey / nativeSetUseFec / nativeStartAdaptivelink
 *     (gs.key, FEC, alink-over-wfb-tunnel — none exist in APFPV)
 *
 * APFPV station mode is a CREDENTIALED, STATEFUL CONNECTION. It needs things
 * WfbNgLink has no vocabulary for: an SSID + WPA2 passphrase, an association
 * lifecycle that can FAIL, and a connection-state the UI must observe. Hence a
 * parallel link class. The VIDEO path is unchanged — once connected, devourer
 * de-encapsulates 802.11->UDP->RTP into the SAME port-5600 / onNewRTPData
 * pipeline PixelPilot already has. Only link establishment is new.
 *
 * What PixelPilot must add around this class:
 *   1. Credential UI (SSID + passphrase; default OpenIPC / 12345678)
 *   2. Connection-state UI + OSD (states below can fail — must be surfaced)
 *   3. Mode toggle WFB-ng <-> APFPV-station (which link class to instantiate)
 *   4. OSD: show dongle RSSI (from devourer radiotap) as the link metric
 *   5. Nothing for video decode — reused as-is.
 */
public class ApfpvStaLink {

    static {
        System.loadLibrary("WfbngRtl8812");   // same .so; adds STA entrypoints
    }

    // ---- connection lifecycle states the UI MUST be able to render ----------
    // (WFB-ng had none of these — it just streamed.)
    public enum StaState {
        IDLE,
        SCANNING,          // looking for the SSID on the set channel
        ARMING,            // SetStationMode: MSR=STATION + BSSID + RCR  <-- auto-ACK
        AUTHENTICATING,    // 802.11 open auth
        ASSOCIATING,       // assoc-req/resp
        HANDSHAKING,       // WPA2 4-way
        DHCP,              // obtaining 192.168.0.10
        STREAMING,         // RTP flowing to port 5600 -> decoder
        // failure terminal states (each needs a distinct UI message):
        FAIL_NO_AP,        // SSID never seen           (check channel/range)
        FAIL_TX,           // TXFAIL_NoAuthResp         (descriptor/BMC bug)
        FAIL_NO_ACK,       // NOGO_Deauthed             (hardware won't auto-ACK)
        FAIL_AUTH,         // wrong passphrase / 4-way failed
        FAIL_DHCP,         // associated but no IP
        LINK_LOST          // mid-flight deauth/disassoc
    }

    public interface StaStatusListener {
        void onStateChanged(StaState state);
        void onRssi(int dbm);          // dongle radiotap RSSI -> OSD + LQ sender
        void onError(String detail);
    }

    private final long nativeStaLink;

    /** Native ApfpvStation handle for the manager to pass to native calls. */
    public long handle() { return nativeStaLink; }

    public ApfpvStaLink(Context context) {
        nativeStaLink = nativeStaInitialize(context);
    }

    // ---- NEW native surface (cf. WfbNgLink.nativeRun) -----------------------
    // Carries CREDENTIALS — the thing WfbNgLink never needed.
    public static native long nativeStaInitialize(Context context);

    public static native void nativeStaConnect(long inst, ApfpvStaLink link, int fd, int channel, int bandwidth, String ssid, String pass, String bssid, String staticIp);

    public static native void nativeStaDisconnect(long inst, int fd);

    // Poll/observe the lifecycle (delivered via registered callback below).
    public static native int  nativeStaGetState(long inst);   // -> StaState ordinal
    public static native int  nativeStaGetRssi(long inst);    // dongle dBm

    // The LQ-feedback sender (dongle RSSI -> VTX 192.168.0.1:12345) lives native;
    // toggle it here. Makes the fork BETTER than stock phone-APFPV (which sends
    // no feedback), because the receiver that DECODES is the one reporting LQ.
    public static native void nativeStaSetLqFeedback(long inst, boolean enabled);

    // GS dongle TX power index (0-63, RTL8812AU scale) for the station-mode
    // dongle. Same index the WFB-ng path uses; applied to the same chip. Sets
    // the uplink/back-channel power. Mirrors WfbNgLink.nativeSetTxPower.
    public static native void nativeStaSetTxPower(long inst, int power);

    // Reads the dongle's real reference TX power index (0-63) from the driver
    // (EFUSE-calibrated when connected). Returns -1 if no device is connected
    // yet, so the caller can keep its own default until the dongle is up.
    public static native int nativeStaGetTxPower(long inst);

    // status callback (native -> java), same pattern as WfbNgLink.nativeCallBack
    private StaStatusListener listener;
    public void setStatusListener(StaStatusListener l) { this.listener = l; }

    private int currentFd = -1;
    /** Instance wrapper: associate this dongle (fd) to the AP in station mode. */
    public void connect(int fd, int channel, int bandwidth, String ssid, String pass) {
        this.currentFd = fd;
        nativeStaConnect(nativeStaLink, this, fd, channel, bandwidth, ssid, pass,
                         "50:e6:36:7d:54:f3", "");  // Taiga BSSID -> skip DFS scan
    }
    /** Instance wrapper: tear down the station link. */
    public void disconnect() {
        // Call UNCONDITIONALLY: the fd arg is ignored natively (teardown uses the ctx
        // station), and ApfpvLinkManager connects via the STATIC nativeStaConnect (not
        // this.connect), so currentFd was never set. The old `if (currentFd >= 0)` guard
        // therefore skipped teardown entirely -> on app pause the stale station kept the
        // dongle fd + supervisor alive -> on resume the reconnect couldn't re-open the
        // device. Always tear down so resume gets a clean dongle.
        nativeStaDisconnect(nativeStaLink, currentFd);
        currentFd = -1;
    }
    public int getState() { return nativeStaGetState(nativeStaLink); }
    public int getRssi()  { return nativeStaGetRssi(nativeStaLink); }
    /** Instance wrapper: set GS dongle TX power index (0-63) in station mode. */
    public void setTxPower(int power) { nativeStaSetTxPower(nativeStaLink, power); }
    /** Instance wrapper: read the dongle's real TX power index (0-63), or -1 if
     *  no device is connected yet. */
    public int getTxPower() { return nativeStaGetTxPower(nativeStaLink); }

    // called FROM native on state/rssi changes
    @SuppressWarnings("unused")
    private void onNativeState(int ordinal) {
        if (listener != null) listener.onStateChanged(StaState.values()[ordinal]);
    }
    @SuppressWarnings("unused")
    private void onNativeRssi(int dbm) {
        if (listener != null) listener.onRssi(dbm);
    }

    // ---- usage sketch -------------------------------------------------------
    // val usb = ctx.getSystemService(UsbManager::class) ...
    // val fd  = usbManager.openDevice(dev).fileDescriptor
    // link.setStatusListener(...)              // drive the connection-state UI
    // ApfpvStaLink.nativeStaConnect(inst, ctx, fd, 40, 20, "OpenIPC", "12345678")
    // ... onStateChanged(STREAMING) -> video already on port 5600 -> decoder
    //
    // NOTE: nativeStaConnect runs the WHOLE gated chain natively:
    //   ARMING (StationMode::arm) -> auth -> assoc -> WPA2 (Wpa2Supplicant)
    //   -> DHCP -> RTP de-encap -> port 5600. Each failure maps to a FAIL_*
    //   state above so the UI can tell the user WHICH step broke — the same
    //   diagnostic split the hardened probe produces.
}
