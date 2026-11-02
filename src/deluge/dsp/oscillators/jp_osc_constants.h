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

// Constants for the JP-8000 style oscillators (JP mode of OscType::SUPERSAW, OscType::JP_FEEDBACK)
// and the JP-style filter / effects.
//
// Two builds:
//  - Public defaults (this file alone). Values come from published descriptions of the JP-8000 Super Saw:
//    atosynth, "The Super Saw code" and "A deepdive into the JP8000 and TC170C140 ESP" (detune and mix tables,
//    detune offsets, centre gain, 88.2 kHz internal rate, 24-bit counters), and Adam Szabo, "How to Emulate the
//    Super Saw" (2010) (pitch-tracked high-pass at the fundamental). Where those sources are silent (knob
//    smoother rate, level compensation, high-pass coefficients) the values here are our own placeholders and
//    are marked as such. The public build is a faithful structure with approximate numbers, not a JP-8000.
//  - Traced constants. If <jp_osc_data/constants.h> is on the include path (a separate, private data package;
//    set JP_OSC_DATA_DIR in CMake), its values replace the placeholders and JP_TRACED_CONSTANTS is 1.
//    No such data is part of this repository.

#include <cstdint>

#if __has_include(<jp_osc_data/constants.h>)
#include <jp_osc_data/constants.h>
#define JP_TRACED_CONSTANTS 1
#else
#define JP_TRACED_CONSTANTS 0
#endif

namespace deluge::dsp::jp {

/// The JP-8000 voice DSP runs one frame per 384 clocks of 33.8688 MHz.
inline constexpr int32_t kInternalRateHz = 88200;

/// Detune offsets per saw (saw 0 is the centre), applied as (kDetuneTable[i] * detuneBase) >> 7 (atosynth).
inline constexpr int32_t kDetuneTable[7] = {0, 128, -128, 408, -412, 704, -720};

/// Centre saw gain, (saw * 25) >> 7 (atosynth). Side saws use the mix word's bits 23..16.
inline constexpr int32_t kCentreGain = 25;

/// Mix word written for CONTROL 2 step s: 102400 + 16384 * s (atosynth).
inline constexpr int32_t kMixTargetBase = 102400;
inline constexpr int32_t kMixTargetStep = 16384;

/// Detune value (DSP word >> 9) for CONTROL 1 step, built from atosynth's published rules:
/// 0 -> 1; 1..63 +1 every second step; 64..80 +1; 81..120 +2; 121..123 +8; 124 +16; 125 +32; 126 +96; 127 -> 321.
/// Step 103 reads 79 (the published "103 quirk", 40448 = 79 * 512).
constexpr int32_t publishedDetuneValue(int32_t step) {
	if (step == 103) {
		return 79;
	}
	int32_t v = 1;
	for (int32_t m = 1; m <= step; m++) {
		if (m <= 63) {
			v += (m % 2 == 0) ? 1 : 0;
		}
		else if (m <= 80) {
			v += 1;
		}
		else if (m <= 120) {
			v += 2;
		}
		else if (m <= 123) {
			v += 8;
		}
		else if (m == 124) {
			v += 16;
		}
		else if (m == 125) {
			v += 32;
		}
		else if (m == 126) {
			v += 96;
		}
		else {
			v = 321;
		}
	}
	return v;
}

#if JP_TRACED_CONSTANTS
inline constexpr bool kTracedConstants = true;
constexpr int32_t detuneTarget(int32_t step) {
	return static_cast<int32_t>(jp_osc_data::kDetuneTarget[step]) << 9;
}
inline constexpr int kSmootherShift = jp_osc_data::kSsSmootherShift;
inline constexpr bool kLevelCompensation = true;
inline constexpr int32_t kLevelMul = jp_osc_data::kSsGainMul;
inline constexpr int kLevelShift = jp_osc_data::kSsGainShift;
inline constexpr int32_t kHpfPitchMul = jp_osc_data::kHpfPitchMul;
inline constexpr int32_t kHpfIn = jp_osc_data::kSsHpfIn;
inline constexpr int32_t kHpfMid = jp_osc_data::kSsHpfMid;
inline constexpr int32_t kHpfOut = jp_osc_data::kSsHpfOut;
/// Per-note centre increment table, or nullptr to use the formula.
inline constexpr const uint32_t* kPitchTable = jp_osc_data::kPitch;
inline constexpr int32_t kPitchTableTopNote = 116; // notes above share the clamped value
#else
inline constexpr bool kTracedConstants = false;
constexpr int32_t detuneTarget(int32_t step) {
	return publishedDetuneValue(step) << 9;
}
// Placeholders (not published, not the JP's values): a one-pole smoother with a ~11.6 ms time constant,
// no level compensation, and a 4-pole high-pass with its -3 dB point near the fundamental (Szabo), ~1 dB bump.
inline constexpr int kSmootherShift = 10;
inline constexpr bool kLevelCompensation = false;
inline constexpr int32_t kLevelMul = 0;
inline constexpr int kLevelShift = 0;
inline constexpr int32_t kHpfPitchMul = 114;
inline constexpr int32_t kHpfIn = 32;
inline constexpr int32_t kHpfMid = 64;
inline constexpr int32_t kHpfOut = 120;
inline constexpr const uint32_t* kPitchTable = nullptr;
inline constexpr int32_t kPitchTableTopNote = 0;
#endif

// --- Feedback Oscillator -------------------------------------------------------
// Published: Roland JP-8000 Owner's Manual (Feedback Osc; CTRL 1 = Harmonics,
// CTRL 2 = Feedback). Public write-ups describe a pitched delay loop with two
// interpolated taps (second tap position = CTRL 1, feedback gain = CTRL 2).
// Delay length, excitation and the max feedback word are our defaults (not
// published, not the JP's values). Optional overrides: jp_osc_data/feedback.h.

#if __has_include(<jp_osc_data/feedback.h>)
#include <jp_osc_data/feedback.h>
inline constexpr int32_t kFbDelaySize = jp_osc_data::kFbDelaySize;
inline constexpr int32_t kFbMaxFeedback = jp_osc_data::kFbMaxFeedback;
inline constexpr int32_t kFbExciteShift = jp_osc_data::kFbExciteShift;
#else
inline constexpr int32_t kFbDelaySize = 4096;     // 44.1 kHz samples; notes below ~10.8 Hz clamp
inline constexpr int32_t kFbMaxFeedback = 2130706432; // ~0.992 Q31: self-oscillates, saturates
inline constexpr int32_t kFbExciteShift = 6;      // quiet saw into the loop (placeholder)
#endif
inline constexpr int32_t kFbDelayMask = kFbDelaySize - 1;
inline constexpr int32_t kFbBankSlots = 48;

// --- JP-style -24 dB filter ----------------------------------------------------
// Published character: JP-8000 24 dB/oct is two cascaded 12 dB/oct stages, an
// exponential cutoff, and resonance that self-oscillates (Roland JP-8000
// Owner's Manual; typical descriptions of the JP filter). Frequency range and
// the max-resonance damping word are our defaults (not published).
// Optional overrides: jp_osc_data/filter.h.

#if __has_include(<jp_osc_data/filter.h>)
#include <jp_osc_data/filter.h>
inline constexpr float kJpFiltFminHz = jp_osc_data::kJpFiltFminHz;
inline constexpr float kJpFiltFmaxHz = jp_osc_data::kJpFiltFmaxHz;
inline constexpr int32_t kJpFiltMinDamp = jp_osc_data::kJpFiltMinDamp;
inline constexpr int32_t kJpFiltMaxDamp = jp_osc_data::kJpFiltMaxDamp;
#else
inline constexpr float kJpFiltFminHz = 20.0f;
inline constexpr float kJpFiltFmaxHz = 18000.0f;
// q31 damping at max resonance. ~3–5 s ring at 1 kHz for a 2-pole (placeholder).
inline constexpr int32_t kJpFiltMinDamp = 220000;
inline constexpr int32_t kJpFiltMaxDamp = 1073741824; // 0.5
#endif

} // namespace deluge::dsp::jp
