#ifndef APFPV_RS_FEC_H
#define APFPV_RS_FEC_H
//  RsFec — systematic Reed-Solomon erasure code over GF(2^8), K data + (N-K)
//  parity per block. Recovers the K source packets from ANY K of the N
//  received (data or parity). This is the classic Rizzo/zfec scheme wfb-ng
//  uses; it is the SHARED codec for both the VTX encoder and the ground
//  (RxDeframe) decoder so their matrices are guaranteed identical.
//
//  Symbol = one byte; a "packet" is a length-`sz` vector coded per-byte-
//  position, so all packets in a block MUST be padded to the same `sz`.
//
//  Usage (encoder):  RsFec fec(k, n);
//                    for (i in 0..n-1) fec.encode(src[0..k-1], out_i, i, sz);
//  Usage (decoder):  give it any k received packets + their indices;
//                    decode() rebuilds the k source packets in place.
#include <cstdint>
#include <cstddef>
#include <vector>

class RsFec {
public:
  // k = number of source packets per block, n = total (k + parity). k<=n<=255.
  RsFec(int k, int n);

  int k() const { return _k; }
  int n() const { return _n; }

  // Produce encoded packet `index` (0..n-1) of length sz into dst.
  // index < k  -> systematic copy of src[index]; index >= k -> parity.
  // src is an array of k pointers, each to sz bytes.
  void encode(const uint8_t* const* src, uint8_t* dst, int index, size_t sz) const;

  // Recover the k source packets. On entry pkt[i] points to the i-th RECEIVED
  // packet (length sz) and idx[i] is its original index (0..n-1); there must be
  // exactly k of them. On return the first k pkt[] buffers hold source packets
  // 0..k-1 in order. Buffers are rearranged/overwritten in place. Returns true
  // on success. (Received packets may be reordered by index internally.)
  bool decode(uint8_t** pkt, int* idx, size_t sz) const;

  // Self-test: build a block, erase up to (n-k) packets, verify recovery.
  // Returns true if all tested erasure patterns recovered exactly.
  static bool SelfTest();

private:
  int _k, _n;
  // Encoding matrix, n*k, row-major. Top k rows = identity (systematic).
  std::vector<uint8_t> _enc;
};

#endif // APFPV_RS_FEC_H
