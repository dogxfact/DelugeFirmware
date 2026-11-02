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

// JP mode of OscType::SUPERSAW: the JP-8000 Super Saw as a fixed-point model of its voice DSP program.
//
// Seven free-running 24-bit counters at 88.2 kHz, the JP's detune and mix tables (128 steps each, CONTROL 1 =
// detune from the wave index, CONTROL 2 = mix from the phase width), a saturating 24-bit sum, level compensation,
// a pitch-tracked high-pass and the DSP's own knob smoother, then our 2:1 half-band to 44.1 kHz and q31 out.
// Counters are never reset: no osc sync, no retrigger phase, no phase randomising per note.
// Numbers come from jp_osc_constants.h (public defaults, or traced values if that data package is present).

#include "dsp/oscillators/jp_halfband.h"
#include "dsp/oscillators/jp_osc_constants.h"
#include <cstdint>

namespace deluge::dsp::jp {

class SuperSaw {
public:
	static constexpr int32_t kNumSaws = 7;
	static constexpr int32_t kChunk = 64; // output samples per internal pass, bounds the stack buffers

	/// The 88.2 kHz voice. Counters are 24-bit, kept shifted up by 8 so they wrap in 32 bits.
	struct Core {
		uint32_t phase[kNumSaws] = {};
		int32_t detune = 0; // smoothed detune word
		int32_t mix = 0;    // smoothed mix word
		int32_t hpf[4] = {};
	};

	/// Host words, as the JP's CPU writes them: centre increment, detune and mix smoother targets.
	struct Controls {
		int32_t pitch;
		int32_t detuneTarget;
		int32_t mixTarget;
	};

	struct State {
		Core core;
		HalfBand::State decimator;
		bool seeded = false;
	};

	/// Wave index (bipolar +-2^30) to CONTROL 1 step 0..127.
	static constexpr int32_t detuneStep(int32_t waveIndex) {
		int64_t v = waveIndex < -(1 << 30) ? -(1 << 30) : (waveIndex > (1 << 30) - 1 ? (1 << 30) - 1 : waveIndex);
		return static_cast<int32_t>(((v + (1 << 30)) * 127 + (int64_t{1} << 30)) >> 31);
	}
	/// Phase width (as passed to renderOsc) to CONTROL 2 step 0..127.
	static constexpr int32_t mixStep(uint32_t pulseWidth) {
		uint32_t a = (static_cast<int32_t>(pulseWidth) >= 0) ? pulseWidth : -pulseWidth;
		return static_cast<int32_t>((static_cast<uint64_t>(a) * 127 + (uint64_t{1} << 30)) >> 31);
	}
	static constexpr int32_t mixTarget(int32_t step) { return kMixTargetBase + kMixTargetStep * step; }

	/// Deluge phase increment (2^32 per cycle at 44.1 kHz) to the JP's 24-bit increment at 88.2 kHz. With the
	/// traced table, in-tune notes use the JP's own per-note value.
	static int32_t pitchFromPhaseIncrement(uint32_t phaseIncrement);

	/// One 88.2 kHz frame, plain C. Returns the 24-bit output word.
	static int32_t frame(Core& core, const Controls& controls);

	/// numPairs * 2 frames. Frame 2t goes to even[t], frame 2t + 1 to odd[t]. Same bits as calling frame().
	static void renderInternal(Core& core, const Controls& controls, int32_t* even, int32_t* odd, int32_t numPairs);

	/// Overwrites buffer[0 .. numSamples) with q31 at 44.1 kHz.
	static void render(int32_t* buffer, int32_t numSamples, uint32_t phaseIncrement, int32_t waveIndex,
	                   uint32_t pulseWidth, State& state);

	/// The pitch-tracked high-pass alone, one frame (for tests).
	static int32_t highPassFrame(int32_t* hpfState, int32_t pitch, int32_t in);

	/// First use of a voice: random counters, smoothers on target, filters empty.
	static void seed(State& state, const Controls& controls, uint32_t random);
};

} // namespace deluge::dsp::jp
