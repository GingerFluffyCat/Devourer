// ============================================================================
//  RxDeframe.cpp — 802.11 data -> UDP/RTP -> port 5600 + RSSI tap (impl)
//  Declarations in RxDeframe.h. Wire onPacket() as devourer's RX callback.
// ============================================================================
#include "RxDeframe.h"
#include "ApfpvStation.h"
#include "PhydmWatchdog.h"
#include <cstring>
#include <cstdio>
#include <chrono>
#include <android/log.h>

// --- throughput bottleneck diag: logs timing breakdown every 500 packets ---
#define RX_DIAG_INTERVAL 500
namespace apfpv {

RxDeframe::RxDeframe(const Mac& self, const Mac& bssid, Wpa2Supplicant* wpa,
                     LqFeedback* lq, OnRtpFn onRtp)
    : _self(self), _bssid(bssid), _wpa(wpa), _lq(lq), _onRtp(std::move(onRtp)) {}

int RxDeframe::toDbm(uint8_t r) {
    // RxAtrib.rssi[] is devourer's gain_trsw byte for the path: the RX
    // descriptor's {TRSW(bit7), gain[6:0]} field (FrameParser.cpp fills it from
    // driver_data.gain_trsw). It is a GAIN index, not dBm — so the placeholder
    // r/2-95 was wrong. The RTL8812 (Jaguar) phystatus path in the Realtek HAL
    // converts the OFDM RX gain to power as:  rx_pwr_dBm = gain_index - 110
    // (the phydm rx_pwr_all base; LNA/VGA gain index maps ~1:1 to dB, offset by
    // the front-end reference of -110 dBm at gain 0). We mask the TRSW bit and
    // apply that. Clamp to a sane RF window.
    int gain = r & 0x7F;                 // strip TRSW flag, keep gain[6:0]
    int dbm  = gain - 110;               // Jaguar OFDM gain-index -> dBm
    if (dbm < -100) dbm = -100;
    if (dbm > -10)  dbm = -10;
    return dbm;
}

void RxDeframe::onPacket(const Packet& pkt) {
    // Layer diagnostic: count frames entering RxDeframe + forwarded to RTP
    static uint64_t inFrames=0, fwdFrames=0, decryptOk=0, fwdBytes=0;
    static auto layerClock = std::chrono::steady_clock::now();
    inFrames++;
    if (_station) _station->notifyRxAlive();   // any RX = link alive (supervisor)
    if (_lq) _lq->update(toDbm(pkt.RxAtrib.rssi[0]), toDbm(pkt.RxAtrib.rssi[1]));
    // Feed live RX RSSI to DIG so it uses phydm *connected-mode* boundaries
    // (IGI floor tracks RSSI ~0x37 @ -45 dBm) instead of monitor coverage
    // bounds (capped 0x2a → over-gained → FA storm → AP rate-caps us).
    PhydmWatchdog::SetLinkRssi(toDbm(pkt.RxAtrib.rssi[0]));

    const uint8_t* f = pkt.Data.data();
    size_t len = pkt.Data.size();
    if (len < 24) return;

    uint16_t fc = f[0] | (f[1] << 8);
    // mgmt deauth/disassoc from our AP -> immediate link-loss signal
    if (((fc >> 2) & 0x3) == 0x0) {
        uint8_t sub = (fc >> 4) & 0xF;
        if (sub == 0xC || sub == 0xA) {
            if (_station) { // deauth/disassoc from AP -> immediate link-loss
                if (!_wpa || !_wpa->pmfActive() || _wpa->verifyProtectedMgmt(f, len))
                    _station->notifyDeauth();
            }
            return;
        }
        // 802.11 Action frame (subtype 0xD): BlockAck frames
        if (sub == 0xD && len >= 27) {
            const uint8_t* b = f + 24;
            // Full hex dump of ADDBA Request (cat=3 act=0) to compare buffer size/policy
            if (b[0] == 0x03 && b[1] == 0x00 && len >= 24 + 9) {
                u16 p = b[3] | (b[4] << 8);
                __android_log_print(4, "apfpv-scan",
                    "ADDBA-REQ-IN dialog=%u param=0x%04x policy=%u tid=%u bufsz=%u timeout=%u startSeq=%u",
                    b[2], p, (p>>1)&1, (p>>2)&0x0f, (p>>6)&0x3ff,
                    b[5]|(b[6]<<8), (b[7]|(b[8]<<8))>>4);
            } else if (b[0] == 0x03 && b[1] == 0x02 && len >= 24 + 6) {
                // DELBA: param[2-3] = {initiator bit11, TID bits12-15}, reason[4-5].
                u16 dp = b[2] | (b[3] << 8);
                __android_log_print(4, "apfpv-scan", "DELBA-IN tid=%u initiator=%u reason=%u",
                    (dp >> 12) & 0xf, (dp >> 11) & 1, (unsigned)(b[4] | (b[5] << 8)));
            } else {
                __android_log_print(4, "apfpv-scan", "ACTION-RX cat=%u act=%u", b[0], b[1]);
            }
        }
        if (sub == 0xD && _station && len >= 24 + 3) {
            const uint8_t* body = f + 24; // 3-addr mgmt header
            if (body[0] == 0x03) { // BlockAck category
                if (body[1] == 0x00) { // ADDBA Request from AP -> send Response
                    _station->handleAddbaRequest(f, len);
                } else if (body[1] == 0x01) { // ADDBA Response from AP -> BA session established!
                    // AP accepted our ADDBA Request. Enable reorder for this TID.
                    u16 param = body[5] | (body[6] << 8);  // BA Parameter Set
                    u8  tid   = (param >> 2) & 0x0f;
                    u16 status = body[3] | (body[4] << 8);
                    if (status == 0 && tid < 16) {
                        _reorder[tid].enable = true;
                        _reorder[tid].wsize_b = 64;
                        _reorder[tid].indicate_seq = 0xffff;  // accept any start seq
                        _reorder[tid].pending.clear();
                        __android_log_print(4, "apfpv-scan",
                            "ADDBA Response ACCEPTED tid=%u — A-MPDU enabled!", tid);
                    }
                }
            }
        }
        return;
    }
    if (((fc >> 2) & 0x3) != 0x2) return;          // data frames only
    if (!(fc & 0x0200)) return;                    // from-DS (AP->STA)
    // fprintf removed: blocks RX thread at 65Mbps — use rxd-diag instead
    // Accept unicast-to-us OR group-addressed (I/G bit in A1[0]): DHCP OFFER/ACK + ARP are
    // broadcast and FPV video may be multicast. Group frames decrypt with the GTK below.
    if (!(f[4] & 0x01) && std::memcmp(f + 4, _self.data(), 6) != 0) return;

    size_t hdrLen = 24;
    if ((fc & 0x0300) == 0x0300) hdrLen += 6;      // 4-addr
    if (fc & 0x0080) hdrLen += 2;                  // QoS-data (subtype>=8): 2B QoS Control
                                                   // (was 0x8000 = Order flag — wrong; EAPOL
                                                   // M1 is QoS-data fc=0x0288, so this was the
                                                   // header-offset bug that hid the handshake)
    if (len <= hdrLen) return;
    // A-MSDU Present = bit 7 of the QoS Control field (the last 2B of the header, present only
    // for QoS data). When set, the (decrypted) MSDU body is a chain of [DA|SA|len|sub-MSDU]
    // subframes, NOT a bare LLC/SNAP. The greg VTX uses this WITH rtw_ampdu_enable=0 to
    // aggregate several packets into one normally-ACKed MPDU — throughput gain without BlockAck.
    const bool amsdu = (fc & 0x0080) && (f[hdrLen - 2] & 0x80);

    const uint8_t* body = f + hdrLen; size_t bodyLen = len - hdrLen;
    // Pre-allocated buffer per thread — avoids heap alloc per packet (5600/s at 65Mbps)
    static thread_local std::vector<uint8_t> plainBuf(2048);
    const uint8_t* llc; size_t llcLen;
    if (fc & 0x4000) {                             // Protected
        // Lever C.2: if HW CCMP decrypt is on AND the chip already decrypted this frame
        // (descriptor SWDEC=0 -> bdecrypted), skip the SW AES entirely — the body is
        // [CCMP hdr 8][plaintext LLC+payload][MIC 8]. This is the lean kernel-style path.
        // GATED: env cached once; default OFF so the proven SW path stays active.
        static const bool kHwDecrypt = std::getenv("DEVOURER_HW_DECRYPT") != nullptr;  // OFF by default (no tput gain)
        // DIAG: count HW-decrypted vs SW-fallback frames so we can SEE whether the chip is
        // actually HW-decrypting (bdecrypted=1) after the SECCFG=0x010c fix. Logged every 4000.
        static thread_local uint32_t hwDec = 0, swDec = 0, diagN = 0, grp = 0, uni = 0;
        if (kHwDecrypt) {
            if (pkt.RxAtrib.bdecrypted) hwDec++; else swDec++;
            if (f[4] & 0x01) grp++; else uni++;   // A1 group-addressed vs unicast-to-us
            if ((++diagN % 4000) == 0)
                fprintf(stderr, "[rxd-hwdec] bdecrypted(HW)=%u SW-fallback=%u | group=%u unicast=%u (last4000)\n",
                        hwDec, swDec, grp, uni), hwDec = swDec = grp = uni = 0;
        }
        if (kHwDecrypt && pkt.RxAtrib.bdecrypted) {
            if (bodyLen <= 16) return;             // need CCMP hdr(8) + MIC(8)
            llc = body + 8; llcLen = bodyLen - 16; // strip CCMP header + trailing MIC
            decryptOk++;
        } else {
            if (!_wpa || !_wpa->ready()) return;
            if (!_wpa->decryptData(f, len, plainBuf)) {
                _dbgDecFail++;
                return;
            }
            decryptOk++;
            llc = plainBuf.data(); llcLen = plainBuf.size();
        }
    } else { llc = body; llcLen = bodyLen; }

    // Deliver ONE MSDU (LLC/SNAP + payload) up the stack. Called once for a normal frame, or
    // once per subframe for an A-MSDU. Early `return` = skip THIS MSDU (move to the next).
    auto deliverMsdu = [&](const uint8_t* llc, size_t llcLen) {
    if (llcLen < 8) return;
    static const uint8_t snap[6] = {0xaa,0xaa,0x03,0x00,0x00,0x00};
    if (std::memcmp(llc, snap, 6) != 0) return;
    uint16_t ethertype = (llc[6] << 8) | llc[7];
    // EAPOL (0x888E): the WPA2 4-way handshake. MUST route to the supplicant or
    // the handshake never completes and the link stalls at Handshaking.
    if (ethertype == 0x888E) {
        if (_wpa) _wpa->onEapolKey(llc + 8, llcLen - 8);
        return;
    }
    // A-MPDU reorder disabled — AP sends single frames (Block-Ack HW not working).
    // The reorder buffer (processReorder) is ready; enable when HW BA works.
    #if 0
    if ((fc & 0x0088) == 0x0088 && ethertype == 0x0800 && llcLen >= 28) {
        const uint8_t* ip = llc + 8; size_t ipl = llcLen - 8;
        if (ipl >= 20 && (ip[0] >> 4) == 4 && ip[9] == 17) {
            size_t ihl = (ip[0] & 0x0f) * 4;
            if (ipl >= ihl + 8) {
                const uint8_t* u = ip + ihl;
                if (((u[2] << 8) | u[3]) == 5600) {
                    processReorder((hdrLen>=26)?(f[24]&0xf):0,
                        (uint16_t)((f[23]<<4)|(f[22]>>4)), llc, llcLen);
                    return;
                }
            }
        }
    }
    #endif

    // ARP responder: reply to "who has <ourIp>?" so peers keep a fresh entry for us and the
    // unicast RTP/SSH stream doesn't stall when their STALE entry needs re-validation.
    if (ethertype == 0x0806 && _ourIp && _arpSend && _wpa && llcLen >= 8 + 28) {
        const uint8_t* a = llc + 8;                          // ARP payload
        uint32_t tip = (a[24]<<24)|(a[25]<<16)|(a[26]<<8)|a[27];
        if (a[6]==0x00 && a[7]==0x01 && tip == _ourIp) {     // a request for OUR IP
            uint8_t r[28]; std::memcpy(r, a, 28);
            r[6]=0x00; r[7]=0x02;                            // oper = reply
            std::memcpy(r+8,  _self.data(), 6);              // sender HW = us
            std::memcpy(r+14, a+24, 4);                      // sender IP = our IP
            std::memcpy(r+18, a+8, 6);                       // target HW = requester
            std::memcpy(r+24, a+14, 4);                      // target IP = requester
            auto m = _wpa->buildEncryptedData(r, 28, 0x0806);
            if (!m.empty()) _arpSend(m);
        }
        return;
    }
    if (ethertype != 0x0800) return; // IPv4
    const uint8_t* ip = llc + 8; size_t ipLen = llcLen - 8;
    // De-dup 802.11 retransmissions of RTP BEFORE any sink: as a station we RX unicast RTP
    // and the AP retransmits each frame until ACKed (~6x). This must run ahead of the _onIp
    // (TUN) path below — otherwise the duplicates reach the decoder via the OS route. Retries
    // reuse the RTP seq, so drop a UDP/5600 packet whose (payload-type, seq) repeats the last.
    if (ipLen >= 20 && (ip[0] >> 4) == 4 && ip[9] == 17) {
        size_t ihl0 = (ip[0] & 0x0f) * 4;
        if (ipLen >= ihl0 + 8 + 4) {
            const uint8_t* u = ip + ihl0;
            if (((u[2] << 8) | u[3]) == 5600) {
                const uint8_t* r = u + 8;
                uint8_t  pt = r[1] & 0x7f;
                uint16_t sq = (uint16_t)((r[2] << 8) | r[3]);
                _dbgRx++;
                // NOTE: the RTP-seq de-dup loop (128-elem scan/pkt) was REMOVED — it was dead
                // code (dropDup=0 every run). The CCMP PN-replay window already drops every
                // 802.11 retransmit (same PN) at decrypt time, before this point. One less
                // 716k-iter/s linear scan in the hot path.
                // RX-seq-gap loss: how many packets between this and the last UNIQUE one for this pt.
                if (_lastSeqV[pt]) {
                    int gap = (int)(uint16_t)(sq - _lastSeq[pt]);
                    if (gap > 1 && gap < 2000) _dbgLoss += (gap - 1);
                }
                _lastSeq[pt] = sq; _lastSeqV[pt] = true;
                // Health summary every 120 unique pkts: received / dup-dropped / decrypt-failed / lost.
                if ((_dbgRx % 120) == 0)
                    __android_log_print(ANDROID_LOG_INFO, "rxd-health",
                        "rx=%d dropDup=%d decFail=%d lost=%d amsdu=%d sub=%d (pt=%u seq=%u) rxrate=%u bw=%u",
                        _dbgRx, _dbgDrop, _dbgDecFail, _dbgLoss, _dbgAmsdu, _dbgAmsduSub, pt, sq,
                        (unsigned)pkt.RxAtrib.data_rate, (unsigned)pkt.RxAtrib.bw);
                        // rxrate: 0-3=CCK 1/2/5.5/11, 4-11=OFDM 6..54, 12-27=HT MCS0-15,
                        // 28+=VHT. Low (<12) => rate-control collapsed (ACK problem);
                        // high MCS but sparse rx => A-MPDU/Block-Ack aggregation missing.
            }
        }
    }
    // GENERAL-IP BRIDGE: hand the WHOLE IPv4 packet to the TUN sink so SSH / any TCP+UDP
    // traverses the dongle (it becomes a full L3 interface, not RTP-only). The OS then
    // routes it. The RTP->5600 + DHCP shortcuts below stay for the dongle-only demo path.
    if (_onIp) _onIp(ip, ipLen);
    if (ipLen < 20 || (ip[0] >> 4) != 4 || ip[9] != 17) return; // IPv4/UDP
    size_t ihl = (ip[0] & 0x0f) * 4;
    if (ipLen < ihl + 8) return;
    const uint8_t* udp = ip + ihl;
    uint16_t dport = (udp[2] << 8) | udp[3];
    uint16_t udlen = (udp[4] << 8) | udp[5];
    // DHCP: replies (OFFER/ACK) arrive on UDP port 68 (BOOTP client). Forward
    // the UDP payload (BOOTP message) to the DHCP state machine so DORA can
    // complete. Without this, the client never sees OFFER and DHCP stalls.
    if (dport == 68) {
        if (_onDhcp && udlen >= 8 && (size_t)udlen <= ipLen - ihl)
            _onDhcp(udp + 8, udlen - 8);
        return;
    }
    if (dport != 5600) return;   // video port
    uint16_t ulen = udlen;
    if (ulen < 8 || (size_t)ulen > ipLen - ihl) return;
    const uint8_t* rtp = udp + 8; size_t rtpLen = ulen - 8;
    if (rtpLen && _onRtp) { _onRtp(rtp, rtpLen); fwdFrames++; fwdBytes += rtpLen; }
    // Layer health every 1 second
    auto now = std::chrono::steady_clock::now();
    auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - layerClock).count();
    if (elapsed >= 1000) {
      float secs = (float)elapsed / 1000.0f;
      __android_log_print(4, "rx-layer",
        "IN=%d/s FWD=%d/s (%.1f%%) decOK=%d/s | GOODPUT=%.1f Mbps",
        (int)(inFrames/secs), (int)(fwdFrames/secs),
        (float)fwdFrames*100.f/(float)(inFrames+1),
        (int)(decryptOk/secs),
        (float)fwdBytes * 8.0f / (secs * 1e6f));
      inFrames=fwdFrames=decryptOk=0; fwdBytes=0; layerClock = now;
    }
    // (removed: per-frame t0..t3 timing + rxd-diag breakdown — 4 steady_clock::now()/frame of
    //  pure profiling overhead; the 1s rx-layer GOODPUT line above is enough.)
    };  // end deliverMsdu

    if (amsdu) {
        // Walk A-MSDU subframes: [DA(6)|SA(6)|len(2)|MSDU(len bytes)|pad to 4B] (no pad on last).
        // Each MSDU is its own LLC/SNAP packet — the AP coalesced several into one ACKed MPDU.
        _dbgAmsdu++;
        size_t off = 0;
        while (off + 14 <= llcLen) {
            uint16_t sub = (uint16_t)((llc[off + 12] << 8) | llc[off + 13]);
            if (sub < 8 || off + 14 + sub > llcLen) break;     // truncated/garbage -> stop
            _dbgAmsduSub++;
            deliverMsdu(llc + off + 14, sub);                  // MSDU starts with its LLC/SNAP
            off = (off + 14 + sub + 3) & ~size_t(3);           // next subframe is 4-byte aligned
        }
    } else {
        deliverMsdu(llc, llcLen);                              // normal single-MSDU frame
    }
}

// A-MPDU per-TID reorder buffer (kernel: recv_indicatepkt_reorder).
// Delivers frames in-order within a sliding window of 64 seq numbers.
// QoS data frames from A-MPDU aggregates may arrive out-of-order; without
// this buffer, the RTP stream would see corrupted/unordered NALUs.
// 12-bit sequence space (802.11 QoS), modulo 4096.
bool RxDeframe::processReorder(uint8_t tid, uint16_t seq, const uint8_t* llc, size_t llcLen) {
    if (tid >= 16) return false;
    auto& rc = _reorder[tid];
    uint16_t sq = seq & 0xfff;  // 12-bit sequence number

    // First frame on this TID: initialize window at this seq.
    if (!rc.enable) {
        rc.enable = true;
        rc.indicate_seq = sq;
        rc.pending.clear();
    }

    uint16_t ind = rc.indicate_seq;

    // Already delivered or too old (outside window, below indicate_seq).
    // Sequence comparison with 12-bit wrap: (a - b) mod 4096.
    int16_t sn_diff = (int16_t)(sq - ind);
    if (sn_diff < 0) {
        // Too old — already passed this seq. Drop.
        return false;
    }

    // In-order: deliver immediately.
    if (sn_diff == 0) {
        // Deliver this frame
        if (llcLen >= 8 && _onRtp) {
            const uint8_t* ip = llc + 8;  // skip SNAP header
            size_t ipl = llcLen - 8;
            // Quick IP/UDP/RTP extraction (same as main onPacket path)
            if (ipl >= 20 && (ip[0] >> 4) == 4) {
                size_t ihl = (ip[0] & 0x0f) * 4;
                if (ipl >= ihl + 8 && ip[9] == 17) {  // UDP
                    const uint8_t* u = ip + ihl;
                    uint16_t dport = (u[2] << 8) | u[3];
                    if (dport == 5600) {
                        const uint8_t* rtp = u + 8;
                        size_t rtpLen = ((u[4] << 8) | u[5]) - 8;  // UDP len minus 8B header = RTP len
                        if (rtpLen > 0 && rtpLen <= ipl - ihl - 8)
                            _onRtp(rtp, rtpLen);
                    }
                }
            }
        }

        // Advance window to next expected seq
        rc.indicate_seq = (ind + 1) & 0xfff;

        // Drain pending queue: deliver any consecutive buffered frames
        while (!rc.pending.empty()) {
            auto it = rc.pending.begin();
            uint16_t next_ind = rc.indicate_seq;
            if (it->first == next_ind) {
                // Deliver queued frame (IP/UDP/RTP extraction similar to above)
                auto& buf = it->second;
                if (buf.size() >= 8) {
                    const uint8_t* ip = buf.data() + 8;
                    size_t ipl = buf.size() - 8;
                    if (ipl >= 20 && (ip[0] >> 4) == 4) {
                        size_t ihl = (ip[0] & 0x0f) * 4;
                        if (ipl >= ihl + 8 && ip[9] == 17) {
                            const uint8_t* u = ip + ihl;
                            uint16_t dport = (u[2] << 8) | u[3];
                            if (dport == 5600) {
                                const uint8_t* rtp = u + 8;
                                size_t rtpLen = (u[4] << 8) | u[5];
                                if (rtpLen >= 8 && (size_t)(rtpLen) <= ipl - ihl && _onRtp)
                                    _onRtp(rtp, rtpLen);
                            }
                        }
                    }
                }
                rc.indicate_seq = (next_ind + 1) & 0xfff;
                rc.pending.erase(it);
            } else {
                break;  // gap in pending queue — wait for the missing frame
            }
        }
        return true;
    }

    // Out-of-order but within window: buffer for later delivery.
    if (sn_diff < (int16_t)rc.wsize_b) {
        // Check if already queued (duplicate)
        if (rc.pending.count(sq) > 0) return false;
        rc.pending[sq] = std::vector<uint8_t>(llc, llc + llcLen);
        return false;  // queued, not delivered yet
    }

    // Outside window — too far ahead. Advance window and deliver.
    // (This handles the case where many frames were dropped and we need to skip ahead.)
    rc.indicate_seq = (sq + 1) & 0xfff;
    rc.pending.clear();  // discard stale pending frames
    if (llcLen >= 8 && _onRtp) {
        const uint8_t* ip = llc + 8;
        size_t ipl = llcLen - 8;
        if (ipl >= 20 && (ip[0] >> 4) == 4) {
            size_t ihl = (ip[0] & 0x0f) * 4;
            if (ipl >= ihl + 8 && ip[9] == 17) {
                const uint8_t* u = ip + ihl;
                if (((u[2] << 8) | u[3]) == 5600) {
                    const uint8_t* rtp = u + 8;
                    size_t rtpLen = (u[4] << 8) | u[5];
                    if (rtpLen >= 8 && (size_t)(rtpLen) <= ipl - ihl)
                        _onRtp(rtp, rtpLen);
                }
            }
        }
    }
    return true;
}

} // namespace apfpv
