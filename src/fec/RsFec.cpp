//  RsFec.cpp — GF(2^8) systematic Reed-Solomon erasure codec.
//  Port of Luigi Rizzo's fec (public domain, 1997), the same algorithm in
//  zfec / wfb-ng. Primitive polynomial x^8+x^4+x^3+x^2+1 (0x11d).
#include "RsFec.h"
#include <cstring>
#include <cstdio>

namespace {

// ---- GF(2^8) tables ------------------------------------------------------
uint8_t GF_EXP[512];   // antilog, doubled so we can index [0..2*255) w/o mod
uint8_t GF_LOG[256];
uint8_t GF_INV[256];
bool    g_gfReady = false;

void gfInit() {
  if (g_gfReady) return;
  const int PRIM = 0x11d;
  int x = 1;
  for (int i = 0; i < 255; i++) {
    GF_EXP[i] = (uint8_t)x;
    GF_LOG[x] = (uint8_t)i;
    x <<= 1;
    if (x & 0x100) x ^= PRIM;
  }
  GF_LOG[0] = 0;                 // log(0) undefined; never used as a factor
  for (int i = 255; i < 512; i++) GF_EXP[i] = GF_EXP[i - 255];
  GF_INV[0] = 0;
  for (int i = 1; i < 256; i++) GF_INV[i] = GF_EXP[255 - GF_LOG[i]];
  g_gfReady = true;
}

inline uint8_t gmul(uint8_t a, uint8_t b) {
  if (a == 0 || b == 0) return 0;
  return GF_EXP[GF_LOG[a] + GF_LOG[b]];
}

// dst[] ^= c * src[]   (byte-vector multiply-accumulate over GF256)
void addmul(uint8_t* dst, const uint8_t* src, uint8_t c, size_t sz) {
  if (c == 0) return;
  const uint8_t* T = &GF_EXP[GF_LOG[c]];   // T[GF_LOG[x]] = c*x
  for (size_t i = 0; i < sz; i++) {
    uint8_t s = src[i];
    if (s) dst[i] ^= T[GF_LOG[s]];
  }
}

// ---- matrix helpers (row-major, GF256) -----------------------------------
void matMul(const uint8_t* a, const uint8_t* b, uint8_t* c, int n, int k, int m) {
  // a is n*k, b is k*m, c is n*m
  for (int r = 0; r < n; r++)
    for (int col = 0; col < m; col++) {
      uint8_t acc = 0;
      for (int i = 0; i < k; i++) acc ^= gmul(a[r * k + i], b[i * m + col]);
      c[r * m + col] = acc;
    }
}

// Invert k*k matrix in place via Gauss-Jordan. Returns false if singular.
bool matInvert(uint8_t* m, int k) {
  std::vector<uint8_t> id((size_t)k * k, 0);
  for (int i = 0; i < k; i++) id[i * k + i] = 1;
  for (int col = 0; col < k; col++) {
    // find pivot
    int piv = -1;
    for (int r = col; r < k; r++) if (m[r * k + col]) { piv = r; break; }
    if (piv < 0) return false;
    if (piv != col)
      for (int c = 0; c < k; c++) {
        std::swap(m[piv * k + c], m[col * k + c]);
        std::swap(id[piv * k + c], id[col * k + c]);
      }
    uint8_t inv = GF_INV[m[col * k + col]];
    for (int c = 0; c < k; c++) { m[col * k + c] = gmul(m[col * k + c], inv);
                                  id[col * k + c] = gmul(id[col * k + c], inv); }
    for (int r = 0; r < k; r++) {
      if (r == col) continue;
      uint8_t f = m[r * k + col];
      if (!f) continue;
      for (int c = 0; c < k; c++) {
        m[r * k + c] ^= gmul(f, m[col * k + c]);
        id[r * k + c] ^= gmul(f, id[col * k + c]);
      }
    }
  }
  std::memcpy(m, id.data(), (size_t)k * k);
  return true;
}

} // namespace

// Build the systematic encoding matrix: top k rows = identity, bottom n-k
// rows = a Vandermonde block made systematic (Vandermonde * inverse of its
// top k*k). This guarantees any k rows are invertible (MDS property).
RsFec::RsFec(int k, int n) : _k(k), _n(n), _enc((size_t)n * k, 0) {
  gfInit();
  // Vandermonde V (n*k): V[r][c] = (r+1)^c  ... use r as the field element.
  std::vector<uint8_t> V((size_t)n * k);
  for (int r = 0; r < n; r++) {
    uint8_t a = (uint8_t)r;           // distinct field elements 0..n-1
    uint8_t p = 1;
    for (int c = 0; c < k; c++) { V[r * k + c] = p; p = gmul(p, a); }
  }
  // top = V[0..k), invert it, enc = V * inv(top). Then top k rows become I.
  std::vector<uint8_t> top(V.begin(), V.begin() + (size_t)k * k);
  matInvert(top.data(), k);
  matMul(V.data(), top.data(), _enc.data(), n, k, k);
}

void RsFec::encode(const uint8_t* const* src, uint8_t* dst, int index, size_t sz) const {
  if (index < _k) { std::memcpy(dst, src[index], sz); return; }
  std::memset(dst, 0, sz);
  const uint8_t* row = &_enc[(size_t)index * _k];
  for (int j = 0; j < _k; j++) addmul(dst, src[j], row[j], sz);
}

bool RsFec::decode(uint8_t** pkt, int* idx, size_t sz) const {
  // If we already hold all k systematic packets (indices 0..k-1), nothing to do
  // beyond ordering. Build the k*k matrix from the encoding rows of the
  // received indices, invert it, and reconstruct each source packet.
  // First place received packets so slot i holds the packet for decode row i.
  // Build decode matrix D (k*k) = encoding rows for the received idx[].
  std::vector<uint8_t> D((size_t)_k * _k);
  for (int i = 0; i < _k; i++) {
    const uint8_t* row = &_enc[(size_t)idx[i] * _k];
    std::memcpy(&D[(size_t)i * _k], row, _k);
  }
  if (!matInvert(D.data(), _k)) return false;
  // out[r] = sum_i D[r][i] * pkt[i]
  std::vector<std::vector<uint8_t>> out(_k, std::vector<uint8_t>(sz, 0));
  for (int r = 0; r < _k; r++)
    for (int i = 0; i < _k; i++)
      addmul(out[r].data(), pkt[i], D[(size_t)r * _k + i], sz);
  for (int r = 0; r < _k; r++) { std::memcpy(pkt[r], out[r].data(), sz); idx[r] = r; }
  return true;
}

bool RsFec::SelfTest() {
  gfInit();
  const int K = 8, N = 12, SZ = 64;   // 8 data + 4 parity
  RsFec fec(K, N);
  // source packets
  std::vector<std::vector<uint8_t>> src(K, std::vector<uint8_t>(SZ));
  for (int i = 0; i < K; i++) for (int j = 0; j < SZ; j++) src[i][j] = (uint8_t)(i * 31 + j * 7 + 1);
  std::vector<const uint8_t*> srcp(K);
  for (int i = 0; i < K; i++) srcp[i] = src[i].data();
  // encode all N
  std::vector<std::vector<uint8_t>> enc(N, std::vector<uint8_t>(SZ));
  for (int i = 0; i < N; i++) fec.encode(srcp.data(), enc[i].data(), i, SZ);
  // Try every pattern of exactly (N-K) erasures among N; recover from the rest.
  int patterns = 0, ok = 0;
  // simple: erase the first e packets for e = 0..N-K, plus a few interior sets
  int eraseSets[][4] = { {0,1,2,3}, {8,9,10,11}, {0,3,7,11}, {1,2,8,10}, {4,5,6,7} };
  for (auto& es : eraseSets) {
    patterns++;
    bool erased[N] = {false};
    for (int e = 0; e < N - K; e++) erased[es[e]] = true;
    // gather first K non-erased
    std::vector<std::vector<uint8_t>> buf;
    std::vector<uint8_t*> pkt; std::vector<int> idx;
    for (int i = 0; i < N && (int)idx.size() < K; i++) if (!erased[i]) {
      buf.push_back(enc[i]); idx.push_back(i);
    }
    for (auto& b : buf) pkt.push_back(b.data());
    if (!fec.decode(pkt.data(), idx.data(), SZ)) continue;
    bool good = true;
    for (int i = 0; i < K; i++) if (std::memcmp(pkt[i], src[i].data(), SZ) != 0) { good = false; break; }
    if (good) ok++;
  }
  std::printf("[RsFec::SelfTest] %d/%d erasure patterns recovered (K=%d N=%d)\n", ok, patterns, K, N);
  return ok == patterns;
}
