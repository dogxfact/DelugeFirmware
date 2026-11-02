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
#include "dsp/oscillators/jp_supersaw.h"
#include <cmath>
#include <cstring>

namespace deluge::dsp::jp {
namespace {

using Core = SuperSaw::Core;
using Controls = SuperSaw::Controls;

// The JP's arithmetic: 24-bit words, (int24 * int8) >> shift with floor shifts, 24-bit saturation.
inline int32_t sat24(int32_t v) {
#if defined(__ARM_ARCH) && !defined(__aarch64__)
	return __builtin_arm_ssat(v, 24);
#else
	return v > 0x7fffff ? 0x7fffff : (v < -0x800000 ? -0x800000 : v);
#endif
}
inline int32_t signExtend30(int32_t v) {
	return static_cast<int32_t>(static_cast<uint32_t>(v) << 2) >> 2;
}
inline int32_t mulShift(int32_t x, int32_t c, int32_t shift) {
	return (x * static_cast<int32_t>(static_cast<int8_t>(c))) >> shift;
}
// Two-part multiply by a 24-bit coefficient word: its top byte, then 7 more bits on x >> 7.
struct SplitCoef {
	int32_t high;
	int32_t low;
	explicit SplitCoef(int32_t k) : high(static_cast<int8_t>(k >> 16)), low((k >> 9) & 0x7f) {}
	int32_t apply(int32_t x, int32_t shift) const { return ((x * high) >> shift) + (((x >> 7) * low) >> shift); }
};
inline int32_t phaseValue(uint32_t phase) {
	return static_cast<int32_t>(phase) >> 8;
}
inline int32_t smootherStep(int32_t target, int32_t value) {
	return sat24(signExtend30(target - value)) >> kSmootherShift;
}
// Level compensation: inverse of the scaled mix word, as the DSP's one's-complement trick.
inline int32_t levelCoef(int32_t mix) {
	int32_t r = sat24(mulShift(mix, kLevelMul, kLevelShift));
	return r >= 0 ? (~r & 0x7fffff) : ~(r & 0x7fffff);
}

struct Derived {
	int32_t pitch;
	SplitCoef hpf;
	explicit Derived(int32_t pitch_) : pitch(pitch_), hpf(sat24(mulShift(pitch_, kHpfPitchMul, 6))) {}
};

inline void increments(const Core& core, int32_t pitch, int32_t* inc) {
	SplitCoef detune(core.detune);
	int32_t base = signExtend30(detune.apply(pitch, 7));
	int32_t baseSat = sat24(base);
	inc[0] = pitch;
	inc[1] = pitch + base;
	for (int32_t i = 2; i < SuperSaw::kNumSaws; i++) {
		inc[i] = pitch + static_cast<int32_t>((static_cast<int64_t>(kDetuneTable[i]) * baseSat) >> 7);
	}
}

// Two 2-pole sections in one structure, cutoff word from the pitch; four 24-bit states.
inline int32_t highPass(int32_t* h, const SplitCoef& k, int32_t in) {
	int32_t a = h[0];
	int32_t b = h[1];
	int32_t c = h[2];
	int32_t d = h[3];
	int32_t b2 = sat24(k.apply(a, 7) + b);
	int32_t u = sat24(b2 + a);
	int32_t d2 = sat24(k.apply(c, 6) + d);
	int32_t w = sat24(d2 + c);
	int32_t x = sat24(-u + mulShift(in, kHpfIn, 6));
	int32_t y = sat24(-w + mulShift(x, kHpfMid, 6));
	h[0] = sat24(k.apply(x, 6) + a);
	h[1] = b2;
	h[2] = sat24(c + k.apply(y, 6));
	h[3] = d2;
	return sat24(mulShift(y, kHpfOut, 6));
}

inline int32_t levelStage(int32_t sum, const SplitCoef& level) {
	if constexpr (kLevelCompensation) {
		return sat24(level.apply(sum, 6));
	}
	return sum;
}

inline int32_t frameWith(Core& core, const Controls& controls, const Derived& derived) {
	core.detune = sat24(core.detune + smootherStep(controls.detuneTarget, core.detune));
	core.mix = sat24(core.mix + smootherStep(controls.mixTarget, core.mix));
	int32_t inc[SuperSaw::kNumSaws];
	increments(core, derived.pitch, inc);
	for (int32_t i = 0; i < SuperSaw::kNumSaws; i++) {
		core.phase[i] += static_cast<uint32_t>(inc[i]) << 8;
	}
	int32_t g = core.mix >> 16;
	int32_t sum = mulShift(phaseValue(core.phase[0]), kCentreGain, 7);
	for (int32_t i = 1; i < SuperSaw::kNumSaws; i++) {
		sum += mulShift(phaseValue(core.phase[i]), g, 7);
	}
	int32_t s = levelStage(sat24(sum), SplitCoef(levelCoef(core.mix)));
	return highPass(core.hpf, derived.hpf, s);
}

inline bool settled(const Core& core, const Controls& controls) {
	return smootherStep(controls.detuneTarget, core.detune) == 0 && smootherStep(controls.mixTarget, core.mix) == 0;
}

inline void store(int32_t* even, int32_t* odd, int32_t t, int32_t v) {
	((t & 1) ? odd : even)[t >> 1] = v;
}

uint32_t seedState = 0x9e3779b9;

} // namespace

int32_t SuperSaw::pitchFromPhaseIncrement(uint32_t phaseIncrement) {
	uint32_t p = (phaseIncrement >> 9) + ((phaseIncrement >> 8) & 1);
	if (p > 0x7fffff) {
		p = 0x7fffff;
	}
#if JP_TRACED_CONSTANTS
	// In-tune notes (within ~0.4 cent of equal temperament, A = 440 Hz) take the JP's own increment.
	float f = static_cast<float>(phaseIncrement) * (44100.0f / 4294967296.0f);
	if (f > 7.0f) {
		float semis = 12.0f * std::log2(f * (1.0f / 440.0f)) + 69.0f;
		int32_t note = static_cast<int32_t>(std::lround(semis));
		if (note >= 0 && note <= kPitchTableTopNote && std::fabs(semis - static_cast<float>(note)) < 0.004f) {
			return static_cast<int32_t>(kPitchTable[note]);
		}
	}
#endif
	return static_cast<int32_t>(p);
}

int32_t SuperSaw::frame(Core& core, const Controls& controls) {
	return frameWith(core, controls, Derived(controls.pitch));
}

void SuperSaw::renderInternal(Core& core, const Controls& controls, int32_t* even, int32_t* odd, int32_t numPairs) {
	const Derived derived(controls.pitch);
	const int32_t total = numPairs * 2;
	int32_t t = 0;

	// While a smoother is moving the increments change every frame.
	// Then one more if needed, so the vector loop starts on an even frame.
	for (; t < total && (!settled(core, controls) || (t & 1)); t++) {
		store(even, odd, t, frameWith(core, controls, derived));
	}

#if JP_OSC_NEON
	if (total - t >= 4) {
		// Settled: increments are fixed, so four frames of all seven counters go in one vector each.
		int32_t inc[kNumSaws];
		increments(core, derived.pitch, inc);
		const int32_t g = core.mix >> 16;
		const SplitCoef level(levelCoef(core.mix));
		const int32x4_t lo24 = vdupq_n_s32(-0x800000);
		const int32x4_t hi24 = vdupq_n_s32(0x7fffff);
		uint32x4_t ph[kNumSaws];
		uint32x4_t step[kNumSaws];
		for (int32_t i = 0; i < kNumSaws; i++) {
			uint32_t d = static_cast<uint32_t>(inc[i]) << 8;
			const uint32_t lanes[4] = {core.phase[i] + d, core.phase[i] + 2 * d, core.phase[i] + 3 * d,
			                           core.phase[i] + 4 * d};
			ph[i] = vld1q_u32(lanes);
			step[i] = vdupq_n_u32(4 * d);
		}
		for (; total - t >= 4; t += 4) {
			int32x4_t sum = vshrq_n_s32(vmulq_n_s32(vshrq_n_s32(vreinterpretq_s32_u32(ph[0]), 8), kCentreGain), 7);
			for (int32_t i = 1; i < kNumSaws; i++) {
				sum = vsraq_n_s32(sum, vmulq_n_s32(vshrq_n_s32(vreinterpretq_s32_u32(ph[i]), 8), g), 7);
			}
			sum = vminq_s32(vmaxq_s32(sum, lo24), hi24);
			if constexpr (kLevelCompensation) {
				int32x4_t a = vshrq_n_s32(vmulq_n_s32(sum, level.high), 6);
				a = vsraq_n_s32(a, vmulq_n_s32(vshrq_n_s32(sum, 7), level.low), 6);
				sum = vminq_s32(vmaxq_s32(a, lo24), hi24);
			}
			int32_t s[4];
			vst1q_s32(s, sum);
			for (int32_t i = 0; i < kNumSaws; i++) {
				ph[i] = vaddq_u32(ph[i], step[i]);
			}
			// t is even here, so frames t, t+1, t+2, t+3 go to even, odd, even, odd
			even[t >> 1] = highPass(core.hpf, derived.hpf, s[0]);
			odd[t >> 1] = highPass(core.hpf, derived.hpf, s[1]);
			even[(t >> 1) + 1] = highPass(core.hpf, derived.hpf, s[2]);
			odd[(t >> 1) + 1] = highPass(core.hpf, derived.hpf, s[3]);
		}
		for (int32_t i = 0; i < kNumSaws; i++) {
			// lane 0 is one increment past the last frame rendered
			core.phase[i] = vgetq_lane_u32(ph[i], 0) - (static_cast<uint32_t>(inc[i]) << 8);
		}
	}
#endif

	for (; t < total; t++) {
		store(even, odd, t, frameWith(core, controls, derived));
	}
}

int32_t SuperSaw::highPassFrame(int32_t* hpfState, int32_t pitch, int32_t in) {
	return highPass(hpfState, Derived(pitch).hpf, in);
}

void SuperSaw::seed(State& state, const Controls& controls, uint32_t random) {
	state = State{};
	for (uint32_t& phase : state.core.phase) {
		random ^= random << 13;
		random ^= random >> 17;
		random ^= random << 5;
		phase = random & 0xffffff00u;
	}
	state.core.detune = controls.detuneTarget;
	state.core.mix = controls.mixTarget;
	state.seeded = true;
}

void SuperSaw::render(int32_t* buffer, int32_t numSamples, uint32_t phaseIncrement, int32_t waveIndex,
                      uint32_t pulseWidth, State& state) {
	const Controls controls{pitchFromPhaseIncrement(phaseIncrement), detuneTarget(detuneStep(waveIndex)),
	                        mixTarget(mixStep(pulseWidth))};
	if (!state.seeded) {
		seedState = seedState * 1664525u + 1013904223u;
		seed(state, controls, seedState | 1u);
	}

	int32_t even[HalfBand::kEvenHistory + kChunk];
	int32_t odd[HalfBand::kOddHistory + kChunk];
	while (numSamples > 0) {
		const int32_t n = numSamples < kChunk ? numSamples : kChunk;
		std::memcpy(even, state.decimator.even, sizeof(state.decimator.even));
		std::memcpy(odd, state.decimator.odd, sizeof(state.decimator.odd));
		renderInternal(state.core, controls, even + HalfBand::kEvenHistory, odd + HalfBand::kOddHistory, n);
		HalfBand::process(even, odd, buffer, n);
		std::memcpy(state.decimator.even, even + n, sizeof(state.decimator.even));
		std::memcpy(state.decimator.odd, odd + n, sizeof(state.decimator.odd));
		buffer += n;
		numSamples -= n;
	}
}

} // namespace deluge::dsp::jp
