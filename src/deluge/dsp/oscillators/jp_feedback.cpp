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
#include "dsp/oscillators/jp_feedback.h"
#include <climits>

#if __has_include("definitions.h")
#include "definitions.h"
#endif
#ifndef PLACE_SDRAM_BSS
#define PLACE_SDRAM_BSS
#endif

namespace deluge::dsp::jp {
namespace {

inline int32_t sat32(int64_t v) {
	return v > INT32_MAX ? INT32_MAX : (v < INT32_MIN ? INT32_MIN : static_cast<int32_t>(v));
}

// Linear interpolation, delay16 is 16.16 samples behind write.
inline int32_t lerp(const int32_t* buf, uint32_t write, uint32_t delay16) {
	uint32_t pos16 = (write << 16) - delay16;
	uint32_t i = (pos16 >> 16) & static_cast<uint32_t>(kFbDelayMask);
	uint32_t frac = pos16 & 0xffff;
	int32_t a = buf[i];
	int32_t b = buf[(i + 1) & static_cast<uint32_t>(kFbDelayMask)];
	return a + static_cast<int32_t>(((static_cast<int64_t>(b - a) * frac) >> 16));
}

uint32_t period16FromIncrement(uint32_t phaseIncrement) {
	if (phaseIncrement == 0) {
		return static_cast<uint32_t>(kFbDelaySize) << 16;
	}
	uint64_t p = (uint64_t{1} << 48) / phaseIncrement;
	uint32_t maxP = static_cast<uint32_t>(kFbDelaySize) << 16;
	if (p > maxP) {
		return maxP;
	}
	if (p < 0x20000u) {
		return 0x20000u; // at least two samples so the taps are distinct
	}
	return static_cast<uint32_t>(p);
}

struct Slot {
	Feedback::State state;
	const void* key = nullptr;
	bool leased = false;
};

PLACE_SDRAM_BSS Slot bank[kFbBankSlots];
uint32_t seedState = 0x243f6a88;

} // namespace

Feedback::State* Feedback::lease(const void* key) {
	if (key == nullptr) {
		return nullptr;
	}
	Slot* found = nullptr;
	Slot* unused = nullptr;
	Slot* idle = nullptr;
	for (Slot& slot : bank) {
		if (slot.key == key) {
			found = &slot;
			break;
		}
		if (unused == nullptr && slot.key == nullptr) {
			unused = &slot;
		}
		if (idle == nullptr && !slot.leased) {
			idle = &slot;
		}
	}
	Slot* slot = found != nullptr ? found : (unused != nullptr ? unused : idle);
	if (slot == nullptr) {
		return nullptr;
	}
	if (slot != found) {
		slot->key = key;
	}
	slot->leased = true;
	return &slot->state;
}

void Feedback::unlease(State* state) {
	if (state == nullptr) {
		return;
	}
	for (Slot& slot : bank) {
		if (&slot.state == state) {
			slot.leased = false;
			return;
		}
	}
}

void Feedback::seed(State& state, uint32_t random) {
	state = State{};
	for (int32_t& s : state.buf) {
		random ^= random << 13;
		random ^= random >> 17;
		random ^= random << 5;
		s = static_cast<int32_t>(random) >> 8; // quiet noise so the loop has energy
	}
	state.write = random;
	state.phase = random << 1;
	state.seeded = true;
}

void Feedback::render(int32_t* buffer, int32_t numSamples, uint32_t phaseIncrement, int32_t waveIndex,
                      uint32_t pulseWidth, State& state) {
	if (!state.seeded) {
		seedState = seedState * 1664525u + 1013904223u;
		seed(state, seedState | 1u);
	}

	const uint32_t period16 = period16FromIncrement(phaseIncrement);
	uint32_t tap2 = static_cast<uint32_t>((static_cast<uint64_t>(period16) * static_cast<uint32_t>(tapStep(waveIndex)))
	                                      >> 7);
	if (tap2 < 0x10000u) {
		tap2 = 0x10000u;
	}
	const int32_t fb = static_cast<int32_t>((static_cast<int64_t>(fbStep(pulseWidth)) * kFbMaxFeedback) / 127);
	uint32_t write = state.write;
	uint32_t phase = state.phase;
	int32_t* buf = state.buf;

	for (int32_t n = 0; n < numSamples; n++) {
		int32_t y1 = lerp(buf, write, period16);
		int32_t y2 = lerp(buf, write, tap2);
		int32_t sum = sat32(static_cast<int64_t>(y1) + y2);
		int32_t loop = static_cast<int32_t>((static_cast<int64_t>(sum) * fb) >> 31);
		phase += phaseIncrement;
		int32_t excite = static_cast<int32_t>(phase) >> kFbExciteShift;
		int32_t w = sat32(static_cast<int64_t>(loop) + excite);
		buf[write & static_cast<uint32_t>(kFbDelayMask)] = w;
		write++;
		buffer[n] = sum;
	}
	state.write = write;
	state.phase = phase;
}

} // namespace deluge::dsp::jp
