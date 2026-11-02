/*
 * Copyright © 2026 Jonathan Riley
 *
 * This file is part of The Synthstrom Audible Deluge Firmware.
 */
#pragma once

// Integer core of the JP-style cascaded 12+12 filter. No Deluge headers, so host tests can include it.
#include "dsp/oscillators/jp_osc_constants.h"
#include <cstdint>
#include <climits>

namespace deluge::dsp::jp {

inline int32_t filtSat(int64_t v) {
	return v > INT32_MAX ? INT32_MAX : (v < INT32_MIN ? INT32_MIN : static_cast<int32_t>(v));
}

inline int32_t filtClip(int32_t x) {
	constexpr int32_t lim = 3 << 29; // 0.75, keeps self-oscillation from running away
	if (x > lim) {
		return lim;
	}
	if (x < -lim) {
		return -lim;
	}
	return x;
}

inline void filtStage12(int32_t in, int32_t& low, int32_t& band, int32_t f, int32_t damp) {
	low = filtClip(filtSat(static_cast<int64_t>(low) + ((static_cast<int64_t>(f) * band) >> 31)));
	int32_t high = filtSat(static_cast<int64_t>(in) - low - ((static_cast<int64_t>(damp) * band) >> 31));
	band = filtClip(filtSat(static_cast<int64_t>(band) + ((static_cast<int64_t>(f) * high) >> 31)));
}

/// Two cascaded 12 dB stages. noise is unused unless self-osc (caller may add it).
inline int32_t filtProcess(int32_t input, int32_t& low1, int32_t& band1, int32_t& low2, int32_t& band2, int32_t f,
                           int32_t damp) {
	filtStage12(input, low1, band1, f, damp);
	filtStage12(low1, low2, band2, f, damp);
	return low2;
}

} // namespace deluge::dsp::jp
