#include "platform/file_hash.hpp"

#include <array>
#include <cstdint>
#include <fstream>
#include <stdexcept>

namespace dt46 {
namespace {
constexpr std::array<std::uint32_t, 64> constants = {
    0x428a2f98, 0x71374491, 0xb5c0fbcf, 0xe9b5dba5, 0x3956c25b, 0x59f111f1, 0x923f82a4, 0xab1c5ed5,
    0xd807aa98, 0x12835b01, 0x243185be, 0x550c7dc3, 0x72be5d74, 0x80deb1fe, 0x9bdc06a7, 0xc19bf174,
    0xe49b69c1, 0xefbe4786, 0x0fc19dc6, 0x240ca1cc, 0x2de92c6f, 0x4a7484aa, 0x5cb0a9dc, 0x76f988da,
    0x983e5152, 0xa831c66d, 0xb00327c8, 0xbf597fc7, 0xc6e00bf3, 0xd5a79147, 0x06ca6351, 0x14292967,
    0x27b70a85, 0x2e1b2138, 0x4d2c6dfc, 0x53380d13, 0x650a7354, 0x766a0abb, 0x81c2c92e, 0x92722c85,
    0xa2bfe8a1, 0xa81a664b, 0xc24b8b70, 0xc76c51a3, 0xd192e819, 0xd6990624, 0xf40e3585, 0x106aa070,
    0x19a4c116, 0x1e376c08, 0x2748774c, 0x34b0bcb5, 0x391c0cb3, 0x4ed8aa4a, 0x5b9cca4f, 0x682e6ff3,
    0x748f82ee, 0x78a5636f, 0x84c87814, 0x8cc70208, 0x90befffa, 0xa4506ceb, 0xbef9a3f7, 0xc67178f2};

std::uint32_t rotr(std::uint32_t value, int bits) { return (value >> bits) | (value << (32 - bits)); }

void compress(const std::uint8_t* data, std::array<std::uint32_t, 8>& state) {
    std::array<std::uint32_t, 64> schedule{};
    for (int i = 0; i < 16; ++i)
        schedule[i] = (std::uint32_t(data[4 * i]) << 24) | (std::uint32_t(data[4 * i + 1]) << 16) |
                      (std::uint32_t(data[4 * i + 2]) << 8) | std::uint32_t(data[4 * i + 3]);
    for (int i = 16; i < 64; ++i) {
        const auto low = rotr(schedule[i - 15], 7) ^ rotr(schedule[i - 15], 18) ^ (schedule[i - 15] >> 3);
        const auto high = rotr(schedule[i - 2], 17) ^ rotr(schedule[i - 2], 19) ^ (schedule[i - 2] >> 10);
        schedule[i] = schedule[i - 16] + low + schedule[i - 7] + high;
    }
    auto [a, b, c, d, e, f, g, h] = state;
    for (int i = 0; i < 64; ++i) {
        const auto sum1 = rotr(e, 6) ^ rotr(e, 11) ^ rotr(e, 25);
        const auto choose = (e & f) ^ (~e & g);
        const auto temp1 = h + sum1 + choose + constants[i] + schedule[i];
        const auto sum0 = rotr(a, 2) ^ rotr(a, 13) ^ rotr(a, 22);
        const auto majority = (a & b) ^ (a & c) ^ (b & c);
        const auto temp2 = sum0 + majority;
        h = g; g = f; f = e; e = d + temp1; d = c; c = b; b = a; a = temp1 + temp2;
    }
    state[0] += a; state[1] += b; state[2] += c; state[3] += d;
    state[4] += e; state[5] += f; state[6] += g; state[7] += h;
}
}

std::string sha256(const std::filesystem::path& path) {
    std::ifstream input(path, std::ios::binary);
    if (!input) throw std::runtime_error("Cannot open " + path.string());
    std::array<std::uint32_t, 8> state = {0x6a09e667, 0xbb67ae85, 0x3c6ef372, 0xa54ff53a,
                                          0x510e527f, 0x9b05688c, 0x1f83d9ab, 0x5be0cd19};
    std::array<std::uint8_t, 64> block{};
    std::size_t filled = 0;
    std::uint64_t bits = 0;
    std::array<char, 65536> chunk{};
    while (input) {
        input.read(chunk.data(), std::streamsize(chunk.size()));
        const auto read = std::size_t(input.gcount());
        bits += std::uint64_t(read) * 8;
        for (std::size_t i = 0; i < read; ++i) {
            block[filled++] = std::uint8_t(chunk[i]);
            if (filled == block.size()) { compress(block.data(), state); filled = 0; }
        }
    }
    block[filled++] = 0x80;
    if (filled > 56) {
        while (filled < block.size()) block[filled++] = 0;
        compress(block.data(), state);
        filled = 0;
    }
    while (filled < 56) block[filled++] = 0;
    for (int i = 0; i < 8; ++i) block[56 + i] = std::uint8_t((bits >> (56 - 8 * i)) & 0xff);
    compress(block.data(), state);
    static const char* digits = "0123456789abcdef";
    std::string result;
    result.reserve(64);
    for (auto value : state)
        for (int shift = 28; shift >= 0; shift -= 4) result.push_back(digits[(value >> shift) & 0xf]);
    return result;
}
}
