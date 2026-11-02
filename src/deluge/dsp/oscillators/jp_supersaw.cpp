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

// Counter steps (increment << 8). Only the low 24 bits of an increment reach a counter, so the products are
// taken mod 2^32: bits 7..30 of (kDetuneTable[i] * base) are all that survive the >> 7 and << 8.
inline void steps(int32_t detune, int32_t pitch, uint32_t* d) {
	SplitCoef split(detune);
	int32_t base = split.apply(pitch, 7);
	uint32_t baseSat = static_cast<uint32_t>(sat24(signExtend30(base)));
	d[0] = static_cast<uint32_t>(pitch) << 8;
	d[1] = static_cast<uint32_t>(pitch + base) << 8;
	for (int32_t i = 2; i < SuperSaw::kNumSaws; i++) {
		int32_t x = static_cast<int32_t>(static_cast<uint32_t>(kDetuneTable[i]) * baseSat) >> 7;
		d[i] = static_cast<uint32_t>(pitch + x) << 8;
	}
}

// Two 2-pole sections in one structure, cutoff word from the pitch; four 24-bit states. Kept in locals across
// a block so they stay in registers.
struct Hpf {
	int32_t a, b, c, d;
	explicit Hpf(const int32_t* h) : a(h[0]), b(h[1]), c(h[2]), d(h[3]) {}
	void save(int32_t* h) const {
		h[0] = a;
		h[1] = b;
		h[2] = c;
		h[3] = d;
	}
	int32_t run(const SplitCoef& k, int32_t in) {
		int32_t b2 = sat24(k.apply(a, 7) + b);
		int32_t u = sat24(b2 + a);
		int32_t d2 = sat24(k.apply(c, 6) + d);
		int32_t w = sat24(d2 + c);
		int32_t x = sat24(-u + mulShift(in, kHpfIn, 6));
		int32_t y = sat24(-w + mulShift(x, kHpfMid, 6));
		a = sat24(k.apply(x, 6) + a);
		b = b2;
		c = sat24(c + k.apply(y, 6));
		d = d2;
		return sat24(mulShift(y, kHpfOut, 6));
	}
};

inline int32_t levelStage(int32_t sum, const SplitCoef& level) {
	if constexpr (kLevelCompensation) {
		return sat24(level.apply(sum, 6));
	}
	return sum;
}

inline void smooth(Core& core, const Controls& controls) {
	core.detune = sat24(core.detune + smootherStep(controls.detuneTarget, core.detune));
	core.mix = sat24(core.mix + smootherStep(controls.mixTarget, core.mix));
}

// Counters advanced by d[], then the mix and level stages; the high-pass comes after.
inline int32_t mixFrame(Core& core, const uint32_t* d, int32_t g, const SplitCoef& level) {
	for (int32_t i = 0; i < SuperSaw::kNumSaws; i++) {
		core.phase[i] += d[i];
	}
	int32_t sum = mulShift(phaseValue(core.phase[0]), kCentreGain, 7);
	for (int32_t i = 1; i < SuperSaw::kNumSaws; i++) {
		sum += mulShift(phaseValue(core.phase[i]), g, 7);
	}
	return levelStage(sat24(sum), level);
}

inline int32_t frameWith(Core& core, const Controls& controls, const Derived& derived, Hpf& hpf) {
	smooth(core, controls);
	uint32_t d[SuperSaw::kNumSaws];
	steps(core.detune, derived.pitch, d);
	return hpf.run(derived.hpf, mixFrame(core, d, core.mix >> 16, SplitCoef(levelCoef(core.mix))));
}

inline bool settled(const Core& core, const Controls& controls) {
	return smootherStep(controls.detuneTarget, core.detune) == 0 && smootherStep(controls.mixTarget, core.mix) == 0;
}

#if JP_OSC_NEON
inline int32x4_t sat24q(int32x4_t v) {
	return vshrq_n_s32(vqshlq_n_s32(v, 8), 8);
}

// Four frames' counter values from four frames' steps: an inclusive prefix sum across the lanes.
inline uint32x4_t advance(uint32_t& phase, uint32x4_t d) {
	const uint32x4_t zero = vdupq_n_u32(0);
	d = vaddq_u32(d, vextq_u32(zero, d, 3));
	d = vaddq_u32(d, vextq_u32(zero, d, 2));
	uint32x4_t ph = vaddq_u32(d, vdupq_n_u32(phase));
	phase = vgetq_lane_u32(ph, 3);
	return ph;
}

// Sum of the seven saws for four frames. g is per lane (it moves with the mix smoother).
inline int32x4_t mixSum(const uint32x4_t* ph, int32x4_t g) {
	int32x4_t sum = vshrq_n_s32(vmulq_n_s32(vshrq_n_s32(vreinterpretq_s32_u32(ph[0]), 8), kCentreGain), 7);
#pragma GCC unroll 8
	for (int32_t i = 1; i < SuperSaw::kNumSaws; i++) {
		sum = vsraq_n_s32(sum, vmulq_s32(vshrq_n_s32(vreinterpretq_s32_u32(ph[i]), 8), g), 7);
	}
	return sat24q(sum);
}

inline int32x4_t levelStageq(int32x4_t sum, int32x4_t high, int32x4_t low) {
	if constexpr (kLevelCompensation) {
		int32x4_t a = vshrq_n_s32(vmulq_s32(sum, high), 6);
		a = vsraq_n_s32(a, vmulq_s32(vshrq_n_s32(sum, 7), low), 6);
		return sat24q(a);
	}
	return sum;
}

// Four frames of a moving voice: smoothers stepped per frame, everything that follows across four lanes.
inline int32x4_t movingBlock(Core& core, const Controls& controls, int32_t pitch) {
	int32x2_t sm = vset_lane_s32(core.mix, vdup_n_s32(core.detune), 1);
	const int32x2_t target = vset_lane_s32(controls.mixTarget, vdup_n_s32(controls.detuneTarget), 1);
	int32_t trace[8];
	for (int32_t l = 0; l < 4; l++) {
		int32x2_t diff = vshr_n_s32(vshl_n_s32(vsub_s32(target, sm), 2), 2);
		diff = vshr_n_s32(vshr_n_s32(vqshl_n_s32(diff, 8), 8), kSmootherShift);
		sm = vshr_n_s32(vqshl_n_s32(vadd_s32(sm, diff), 8), 8);
		vst1_s32(trace + 2 * l, sm);
	}
	core.detune = vget_lane_s32(sm, 0);
	core.mix = vget_lane_s32(sm, 1);
	const int32x4x2_t dm = vld2q_s32(trace); // val[0]: detune per frame, val[1]: mix per frame

	const int32x4_t high = vshrq_n_s32(vshlq_n_s32(dm.val[0], 8), 24);
	const int32x4_t low = vandq_s32(vshrq_n_s32(dm.val[0], 9), vdupq_n_s32(0x7f));
	const int32x4_t base = vsraq_n_s32(vshrq_n_s32(vmulq_n_s32(high, pitch), 7), vmulq_n_s32(low, pitch >> 7), 7);
	const int32x4_t baseSat = sat24q(vshrq_n_s32(vshlq_n_s32(base, 2), 2));
	const int32x4_t p = vdupq_n_s32(pitch);

	uint32x4_t ph[SuperSaw::kNumSaws];
	ph[0] = advance(core.phase[0], vdupq_n_u32(static_cast<uint32_t>(pitch) << 8));
	ph[1] = advance(core.phase[1], vreinterpretq_u32_s32(vshlq_n_s32(vaddq_s32(p, base), 8)));
#pragma GCC unroll 8
	for (int32_t i = 2; i < SuperSaw::kNumSaws; i++) {
		int32x4_t inc = vsraq_n_s32(p, vmulq_n_s32(baseSat, kDetuneTable[i]), 7);
		ph[i] = advance(core.phase[i], vreinterpretq_u32_s32(vshlq_n_s32(inc, 8)));
	}
	int32x4_t sum = mixSum(ph, vshrq_n_s32(dm.val[1], 16));
	if constexpr (kLevelCompensation) {
		// levelCoef() per lane: r >= 0 ? ~r & 0x7fffff : ~(r & 0x7fffff)
		const int32x4_t mask = vdupq_n_s32(0x7fffff);
		int32x4_t r =
		    sat24q(vshlq_s32(vmulq_n_s32(dm.val[1], static_cast<int8_t>(kLevelMul)), vdupq_n_s32(-kLevelShift)));
		int32x4_t coef = vandq_s32(vmvnq_s32(vandq_s32(r, mask)), vorrq_s32(mask, vshrq_n_s32(r, 31)));
		sum =
		    levelStageq(sum, vshrq_n_s32(vshlq_n_s32(coef, 8), 24), vandq_s32(vshrq_n_s32(coef, 9), vdupq_n_s32(0x7f)));
	}
	return sum;
}
#endif

uint32_t ssSeedState = 0x9e3779b9;

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
	Hpf hpf(core.hpf);
	int32_t out = frameWith(core, controls, Derived(controls.pitch), hpf);
	hpf.save(core.hpf);
	return out;
}

void SuperSaw::renderInternal(Core& core, const Controls& controls, int32_t* even, int32_t* odd, int32_t numPairs) {
	const Derived derived(controls.pitch);
	Hpf hpf(core.hpf);
	int32_t* const evenEnd = even + numPairs;

#if JP_OSC_NEON
	auto filterFour = [&](int32x4_t sum) __attribute__((always_inline)) {
		// frames t, t+1, t+2, t+3 (t a multiple of 4) go to even, odd, even, odd
		even[0] = hpf.run(derived.hpf, vgetq_lane_s32(sum, 0));
		odd[0] = hpf.run(derived.hpf, vgetq_lane_s32(sum, 1));
		even[1] = hpf.run(derived.hpf, vgetq_lane_s32(sum, 2));
		odd[1] = hpf.run(derived.hpf, vgetq_lane_s32(sum, 3));
		even += 2;
		odd += 2;
	};
	// While a smoother is moving, four frames at a time with per-frame controls.
	while (evenEnd - even >= 2 && !settled(core, controls)) {
		filterFour(movingBlock(core, controls, derived.pitch));
	}
	if (evenEnd - even >= 2) {
		// Settled, and it stays so for this call (the targets are fixed): steps, gain and level are constants.
		uint32_t d[kNumSaws];
		steps(core.detune, derived.pitch, d);
		const int32x4_t g = vdupq_n_s32(core.mix >> 16);
		const SplitCoef level(levelCoef(core.mix));
		const int32x4_t levelHigh = vdupq_n_s32(level.high);
		const int32x4_t levelLow = vdupq_n_s32(level.low);
		const uint32x4_t ramp = {1, 2, 3, 4};
		uint32x4_t ph[kNumSaws];
		uint32x4_t step[kNumSaws];
		for (int32_t i = 0; i < kNumSaws; i++) {
			ph[i] = vmlaq_n_u32(vdupq_n_u32(core.phase[i]), ramp, d[i]);
			step[i] = vdupq_n_u32(4 * d[i]);
		}
		do {
			int32x4_t sum = levelStageq(mixSum(ph, g), levelHigh, levelLow);
			for (int32_t i = 0; i < kNumSaws; i++) {
				ph[i] = vaddq_u32(ph[i], step[i]);
			}
			filterFour(sum);
		} while (evenEnd - even >= 2);
		for (int32_t i = 0; i < kNumSaws; i++) {
			// lane 0 is one step past the last frame rendered
			core.phase[i] = vgetq_lane_u32(ph[i], 0) - d[i];
		}
	}
	if (even < evenEnd) {
		*even = frameWith(core, controls, derived, hpf);
		*odd = frameWith(core, controls, derived, hpf);
	}
#else
	while (even < evenEnd && !settled(core, controls)) {
		*even++ = frameWith(core, controls, derived, hpf);
		*odd++ = frameWith(core, controls, derived, hpf);
	}
	if (even < evenEnd) {
		uint32_t d[kNumSaws];
		steps(core.detune, derived.pitch, d);
		const int32_t g = core.mix >> 16;
		const SplitCoef level(levelCoef(core.mix));
		do {
			*even++ = hpf.run(derived.hpf, mixFrame(core, d, g, level));
			*odd++ = hpf.run(derived.hpf, mixFrame(core, d, g, level));
		} while (even < evenEnd);
	}
#endif
	hpf.save(core.hpf);
}

int32_t SuperSaw::highPassFrame(int32_t* hpfState, int32_t pitch, int32_t in) {
	Hpf hpf(hpfState);
	int32_t out = hpf.run(Derived(pitch).hpf, in);
	hpf.save(hpfState);
	return out;
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
	state.lastPitch = controls.pitch;
	state.seeded = true;
}

void SuperSaw::advancePhases(Core& core, int32_t pitch, uint32_t numInternalFrames) {
	if (numInternalFrames == 0 || pitch == 0) {
		return;
	}
	uint32_t d[kNumSaws];
	steps(core.detune, pitch, d);
	for (int32_t i = 0; i < kNumSaws; i++) {
		core.phase[i] += d[i] * numInternalFrames;
	}
}

namespace {
// Persistent bank: ObjectPool reconstructs on acquire (and recycle runs the destructor), which
// reseeds every note. Slots here keep their Core after unlease. Key is the VoiceUnisonPartSource
// address, which is stable across in-place Voice steals.
constexpr int32_t kBankSlots = 48;
struct SsSlot {
	SuperSaw::State state;
	const void* key = nullptr;
	bool leased = false;
	uint32_t lastNow = 0;
};
SsSlot ssBank[kBankSlots];
} // namespace

SuperSaw::State* SuperSaw::lease(const void* key, uint32_t nowSamples) {
	if (key == nullptr) {
		return nullptr;
	}
	SsSlot* found = nullptr;
	SsSlot* unused = nullptr;
	SsSlot* idle = nullptr;
	for (SsSlot& slot : ssBank) {
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
	SsSlot* slot = found != nullptr ? found : (unused != nullptr ? unused : idle);
	if (slot == nullptr) {
		return nullptr; // bank full: caller falls back to the stock supersaw
	}
	if (slot != found) {
		slot->key = key;
	}
	else if (slot->state.seeded && slot->state.lastPitch != 0) {
		uint32_t dt = nowSamples - slot->lastNow;
		if (dt != 0) {
			advancePhases(slot->state.core, slot->state.lastPitch, dt * 2u);
			if (dt > 16u) {
				slot->state.decimator = {};
			}
		}
	}
	slot->leased = true;
	slot->lastNow = nowSamples;
	return &slot->state;
}

void SuperSaw::unlease(State* state, uint32_t nowSamples) {
	if (state == nullptr) {
		return;
	}
	for (SsSlot& slot : ssBank) {
		if (&slot.state == state) {
			slot.leased = false;
			slot.lastNow = nowSamples;
			return;
		}
	}
}

void SuperSaw::render(int32_t* buffer, int32_t numSamples, uint32_t phaseIncrement, int32_t waveIndex,
                      uint32_t pulseWidth, State& state) {
	const Controls controls{pitchFromPhaseIncrement(phaseIncrement), detuneTarget(detuneStep(waveIndex)),
	                        mixTarget(mixStep(pulseWidth))};
	if (!state.seeded) {
		ssSeedState = ssSeedState * 1664525u + 1013904223u;
		seed(state, controls, ssSeedState | 1u);
	}
	state.lastPitch = controls.pitch;

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
