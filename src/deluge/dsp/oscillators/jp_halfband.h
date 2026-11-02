/*
 * Copyright © 2026 Jonathan Riley
 *
 * This file is part of The Synthstrom Audible Deluge Firmware.
 *
 * The Synthstrom Audible Deluge Firmware is free software: you can redistribute it and/or modify it under the
 * terms of the GNU General Public License as published by the Free Software Foundation,
 * either version 3 of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY;
 * without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along with this program.
 * If not, see <https://www.gnu.org/licenses/>.
 */
#pragma once

// 2:1 half-band decimator for the JP-mode oscillators: 88.2 kHz internal -> 44.1 kHz.
// This stage is ours, not the JP-8000's (the JP runs its whole voice at 88.2 kHz and its DAC filters the rest).
//
// 63 taps, equiripple (Parks-McClellan), passband 0..19 kHz, stopband 25.1..44.1 kHz. Every second tap is zero,
// so each output costs 16 symmetric pairs plus the centre tap. Coefficients are Q31, centre tap 2^30, DC gain
// exactly 1. Measured response of these integer coefficients: passband ripple 0.003 dB, stopband -75 dB. Group
// delay 15.5 output samples.
//
// Input is split into even and odd internal frames, each with history in front:
//   even[0 .. kEvenHistory + n), odd[0 .. kOddHistory + n). Output m (0 <= m < n):
//   acc = 2^30 * odd[m] + sum_j kCoef[j] * (even[m + 15 - j] + even[m + 16 + j]) + 2^30 (rounding)
//   y = sat32(acc >> 31), out = sat32(y << 7)          (24-bit JP word to q31, 1 bit of headroom)

#include <cstdint>

#if defined(__ARM_NEON)
#include <arm_neon.h>
#define JP_OSC_NEON 1
#elif defined(JP_OSC_SIMDE)
#define SIMDE_ENABLE_NATIVE_ALIASES
#include <simde/arm/neon.h>
#define JP_OSC_NEON 1
#else
#define JP_OSC_NEON 0
#endif

namespace deluge::dsp::jp {

class HalfBand {
public:
	static constexpr int32_t kPairs = 16;
	static constexpr int32_t kEvenHistory = 2 * kPairs - 1; // 31
	static constexpr int32_t kOddHistory = kPairs;          // 16
	static constexpr int32_t kOutputShift = 7;
	static constexpr int32_t kCoef[kPairs] = {681689222, -221788039, 126808058, -84203931, 59351759, -42851435,
	                                          31110580,  -22447349,  15959470,  -11094527, 7485088,  -4855622,
	                                          2994570,   -1723706,   899869,    -463095};

	struct State {
		int32_t even[kEvenHistory] = {};
		int32_t odd[kOddHistory] = {};
	};

	static inline int32_t sat32(int64_t v) {
		return v > INT32_MAX ? INT32_MAX : (v < INT32_MIN ? INT32_MIN : static_cast<int32_t>(v));
	}

	/// One output, plain C. The NEON path gives the same bits.
	static inline int32_t processOne(const int32_t* even, const int32_t* odd, int32_t m) {
		int64_t acc = (int64_t{1} << 30) + (static_cast<int64_t>(odd[m]) << 30);
		for (int32_t j = 0; j < kPairs; j++) {
			acc += static_cast<int64_t>(kCoef[j]) * (even[m + 15 - j] + even[m + 16 + j]);
		}
		return sat32(static_cast<int64_t>(sat32(acc >> 31)) << kOutputShift);
	}

	static inline void process(const int32_t* even, const int32_t* odd, int32_t* out, int32_t n) {
		int32_t m = 0;
#if JP_OSC_NEON
		int32x2_t c[kPairs / 2];
		for (int32_t j = 0; j < kPairs / 2; j++) {
			c[j] = vld1_s32(kCoef + 2 * j);
		}
		for (; m + 4 <= n; m += 4) {
			// even[m .. m + 36) once; each tap's four-sample window is a vext of two neighbours
			int32x4_t e[9];
			for (int32_t k = 0; k < 9; k++) {
				e[k] = vld1q_s32(even + m + 4 * k);
			}
			auto window = [&](int32_t o) __attribute__((always_inline)) {
				switch (o & 3) {
				case 1:
					return vextq_s32(e[o >> 2], e[(o >> 2) + 1], 1);
				case 2:
					return vextq_s32(e[o >> 2], e[(o >> 2) + 1], 2);
				case 3:
					return vextq_s32(e[o >> 2], e[(o >> 2) + 1], 3);
				default:
					return e[o >> 2];
				}
			};
			int32x4_t o = vld1q_s32(odd + m);
			int64x2_t lo = vdupq_n_s64(int64_t{1} << 30);
			int64x2_t hi = lo;
			lo = vmlal_n_s32(lo, vget_low_s32(o), 1 << 30);
			hi = vmlal_n_s32(hi, vget_high_s32(o), 1 << 30);
#pragma GCC unroll 8
			for (int32_t j = 0; j < kPairs; j += 2) {
				int32x4_t s0 = vaddq_s32(window(15 - j), window(16 + j));
				int32x4_t s1 = vaddq_s32(window(14 - j), window(17 + j));
				lo = vmlal_lane_s32(lo, vget_low_s32(s0), c[j / 2], 0);
				hi = vmlal_lane_s32(hi, vget_high_s32(s0), c[j / 2], 0);
				lo = vmlal_lane_s32(lo, vget_low_s32(s1), c[j / 2], 1);
				hi = vmlal_lane_s32(hi, vget_high_s32(s1), c[j / 2], 1);
			}
			int32x4_t y = vcombine_s32(vqshrn_n_s64(lo, 31), vqshrn_n_s64(hi, 31));
			vst1q_s32(out + m, vqshlq_n_s32(y, kOutputShift));
		}
#endif
		for (; m < n; m++) {
			out[m] = processOne(even, odd, m);
		}
	}
};

} // namespace deluge::dsp::jp
