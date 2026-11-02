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

// JP-8000 Feedback Oscillator: a pitched delay loop with two interpolated taps.
// Wave index is CTRL 1 (second tap position), phase width is CTRL 2 (feedback).
// Delay contents are never cleared on note-on. Numbers: jp_osc_constants.h.

#include "dsp/oscillators/jp_osc_constants.h"
#include <cstdint>

namespace deluge::dsp::jp {

class Feedback {
public:
	struct State {
		int32_t buf[kFbDelaySize] = {};
		uint32_t write = 0;
		uint32_t phase = 0;
		bool seeded = false;
	};

	static constexpr int32_t tapStep(int32_t waveIndex) {
		int64_t v = waveIndex < -(1 << 30) ? -(1 << 30) : (waveIndex > (1 << 30) - 1 ? (1 << 30) - 1 : waveIndex);
		return static_cast<int32_t>(((v + (1 << 30)) * 127 + (int64_t{1} << 30)) >> 31);
	}
	static constexpr int32_t fbStep(uint32_t pulseWidth) {
		uint32_t a = (static_cast<int32_t>(pulseWidth) >= 0) ? pulseWidth : -pulseWidth;
		return static_cast<int32_t>((static_cast<uint64_t>(a) * 127 + (uint64_t{1} << 30)) >> 31);
	}

	static State* lease(const void* key);
	static void unlease(State* state);

	static void seed(State& state, uint32_t random);
	static void render(int32_t* buffer, int32_t numSamples, uint32_t phaseIncrement, int32_t waveIndex,
	                   uint32_t pulseWidth, State& state);
};

} // namespace deluge::dsp::jp
