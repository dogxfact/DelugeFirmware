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
// Host tests for the JP-mode Super Saw (A1..A9 of the JP Super Saw plan, decimator, free-running phase).
// Build: see run_tests.sh. Builds with public defaults unless <jp_osc_data/constants.h> is on the include path.
#include "dsp/oscillators/jp_supersaw.h"
#include <cmath>
#include <complex>
#include <cstdio>
#include <cstdlib>
#include <vector>

using namespace deluge::dsp::jp;
static int failures = 0;
#define CHECK(cond, ...)                                                                                               \
	do {                                                                                                               \
		if (!(cond)) {                                                                                                 \
			failures++;                                                                                                \
			printf("  FAIL: " __VA_ARGS__);                                                                            \
			printf("\n");                                                                                              \
		}                                                                                                              \
	} while (0)

static uint32_t rng = 2463534242u;
static uint32_t rnd() {
	rng ^= rng << 13;
	rng ^= rng >> 17;
	rng ^= rng << 5;
	return rng;
}
static int32_t se24(uint32_t v) {
	return static_cast<int32_t>(v << 8) >> 8;
}
static int32_t etPitch(int note) {
	return static_cast<int32_t>(std::lround(440.0 * std::pow(2.0, (note - 69) / 12.0) * 16777216.0 / 88200.0));
}
static int32_t jpPitch(int note) {
#if JP_TRACED_CONSTANTS
	return static_cast<int32_t>(kPitchTable[note]);
#else
	return etPitch(note);
#endif
}
static SuperSaw::Core settledCore(const SuperSaw::Controls& k) {
	SuperSaw::Core c;
	for (auto& p : c.phase) {
		p = rnd() & 0xffffff00u;
	}
	c.detune = k.detuneTarget;
	c.mix = k.mixTarget;
	return c;
}
// Detune base, written out from atosynth's description (split multiply, floor shifts).
static int32_t refDetuneBase(int32_t pitch, int32_t d) {
	return ((pitch * static_cast<int8_t>(d >> 16)) >> 7) + (((pitch >> 7) * ((d >> 9) & 0x7f)) >> 7);
}

static void testTables() {
	printf("A1 control tables (%s constants)\n", kTracedConstants ? "traced" : "public");
	int bad = 0;
	for (int s = 0; s < 128; s++) {
		if (!kTracedConstants) {
			bad += detuneTarget(s) != (publishedDetuneValue(s) << 9);
		}
		bad += SuperSaw::mixTarget(s) != 102400 + 16384 * s;
	}
	CHECK(bad == 0, "%d table entries differ", bad);
	if (!kTracedConstants) {
		CHECK(detuneTarget(0) == 512 && detuneTarget(127) == 164352 && detuneTarget(103) == 40448,
		      "published spot values");
	}
	// Knob mapping: wave index and phase width reach all 128 steps, ends map to 0 and 127
	CHECK(SuperSaw::detuneStep(-(1 << 30)) == 0 && SuperSaw::detuneStep((1 << 30) - 1) == 127, "detune ends");
	CHECK(SuperSaw::mixStep(0) == 0 && SuperSaw::mixStep(0x80000000u) == 127 && SuperSaw::mixStep(0x7fffffffu) == 127,
	      "mix ends");
	int seen = 0;
	for (int s = 0; s < 128; s++) {
		int64_t wi = -(int64_t{1} << 30) + ((int64_t{s} << 31) + 63) / 127;
		if (wi > (1 << 30) - 1) {
			wi = (1 << 30) - 1;
		}
		seen += SuperSaw::detuneStep(static_cast<int32_t>(wi)) == s;
	}
	CHECK(seen == 128, "detune steps reachable %d/128", seen);
	printf("  detune targets 0/64/103/127 = %d %d %d %d, mix targets 0/127 = %d %d\n", detuneTarget(0),
	       detuneTarget(64), detuneTarget(103), detuneTarget(127), SuperSaw::mixTarget(0), SuperSaw::mixTarget(127));
}

static void testIncrements() {
	printf("A2 increments, notes 0..117 x detune 0..127 x 7 saws\n");
	long bad = 0, n = 0;
	for (int note = 0; note <= 117; note++) {
		for (int step = 0; step < 128; step++) {
			SuperSaw::Controls k{jpPitch(note), detuneTarget(step), SuperSaw::mixTarget(64)};
			SuperSaw::Core c = settledCore(k);
			SuperSaw::Core before = c;
			SuperSaw::frame(c, k);
			int32_t base = refDetuneBase(k.pitch, k.detuneTarget);
			for (int i = 0; i < 7; i++) {
				int32_t expect = k.pitch + static_cast<int32_t>((int64_t{kDetuneTable[i]} * base) >> 7);
				int32_t got = se24((c.phase[i] - before.phase[i]) >> 8);
				bad += se24(expect) != got;
				n++;
			}
		}
	}
	CHECK(bad == 0, "%ld of %ld increments differ", bad, n);
	printf("  %ld increments, %ld differ\n", n, bad);
}

// Reference frame written independently of the firmware core, on top of the firmware high-pass.
static int32_t refFrame(SuperSaw::Core& c, const SuperSaw::Controls& k) {
	auto sat = [](int64_t v) {
		return static_cast<int32_t>(v > 0x7fffff ? 0x7fffff : (v < -0x800000 ? -0x800000 : v));
	};
	auto smooth = [&](int32_t t, int32_t s) {
		int32_t e = sat(static_cast<int32_t>(static_cast<uint32_t>(t - s) << 2) >> 2);
		return sat(int64_t{s} + (e >> kSmootherShift));
	};
	c.detune = smooth(k.detuneTarget, c.detune);
	c.mix = smooth(k.mixTarget, c.mix);
	int32_t base = refDetuneBase(k.pitch, c.detune);
	int32_t g = c.mix >> 16;
	int32_t sum = 0;
	for (int i = 0; i < 7; i++) {
		int32_t b = (i == 1) ? base : sat(base);
		int32_t inc = k.pitch + static_cast<int32_t>((int64_t{kDetuneTable[i]} * b) >> 7);
		c.phase[i] += static_cast<uint32_t>(inc) << 8;
		sum += (se24(c.phase[i] >> 8) * (i == 0 ? kCentreGain : g)) >> 7;
	}
	int32_t s = sat(sum);
	if (kLevelCompensation) {
		int32_t r = sat((c.mix * kLevelMul) >> kLevelShift);
		int32_t G = r >= 0 ? (~r & 0x7fffff) : ~(r & 0x7fffff);
		s = sat(((s * static_cast<int8_t>(G >> 16)) >> 6) + (((s >> 7) * ((G >> 9) & 0x7f)) >> 6));
	}
	return SuperSaw::highPassFrame(c.hpf, k.pitch, s);
}

static void testInternalRender() {
	printf("A3 internal render, 4096 frames, notes 24/60/96 x detune 0/64/103/127 x mix 0/64/127\n");
	const int notes[3] = {24, 60, 96}, dets[4] = {0, 64, 103, 127}, mixes[3] = {0, 64, 127};
	long bad = 0, badBlock = 0;
	int32_t peak = 0;
	static int32_t ev[2048], od[2048];
	for (int note : notes) {
		for (int det : dets) {
			for (int mix : mixes) {
				SuperSaw::Controls k{jpPitch(note), detuneTarget(det), SuperSaw::mixTarget(mix)};
				SuperSaw::Core a = settledCore(k);
				// start the smoothers off target so the moving path is covered too
				a.detune = k.detuneTarget / 2;
				a.mix = k.mixTarget / 3;
				SuperSaw::Core b = a, r = a;
				SuperSaw::renderInternal(b, k, ev, od, 2048);
				for (int t = 0; t < 4096; t++) {
					int32_t x = SuperSaw::frame(a, k), y = refFrame(r, k), z = (t & 1) ? od[t >> 1] : ev[t >> 1];
					bad += x != y;
					badBlock += z != x;
					peak = std::max(peak, std::abs(x));
				}
			}
		}
	}
	CHECK(bad == 0 && badBlock == 0, "frame vs reference %ld, block vs frame %ld", bad, badBlock);
	printf("  frame vs independent reference: %ld differ; block (%s) vs frame: %ld differ; peak %d\n", bad,
	       JP_OSC_NEON ? "NEON" : "scalar", badBlock, peak);
}

static void testHighPass() {
	printf("A4 high-pass sine gain (dB, relative to 32 x f0)\n");
	const double mults[] = {0.25, 0.5, 0.7, 1, 2, 4, 16, 32};
	for (int note : {24, 60, 96}) {
		int32_t pitch = jpPitch(note);
		double f0 = pitch * 88200.0 / 16777216.0;
		double g[8];
		for (int m = 0; m < 8; m++) {
			double f = f0 * mults[m];
			int32_t h[4] = {};
			double acc = 0;
			int n = 88200, settle = 44100;
			for (int t = 0; t < n + settle; t++) {
				int32_t in = static_cast<int32_t>(std::lround(0x200000 * std::sin(2 * M_PI * f * t / 88200.0)));
				int32_t out = SuperSaw::highPassFrame(h, pitch, in);
				if (t >= settle) {
					acc += double(out) * out;
				}
			}
			g[m] = 10 * std::log10(acc / n / (0.5 * 0x200000 * 0x200000.0p0));
		}
		printf("  note %3d f0 %8.2f Hz:", note, f0);
		for (int m = 0; m < 8; m++) {
			printf(" %.2fx %6.2f", mults[m], g[m] - g[7]);
		}
		printf("  (abs %.2f dB at 32x)\n", g[7]);
		CHECK(g[0] - g[7] < -30 && std::fabs(g[6] - g[7]) < 0.5, "note %d high-pass shape", note);
	}
}

static void testDecimator() {
	printf("A5 decimator (63-tap half-band, ours)\n");
	// integer coefficient response
	double ripMax = -1e9, ripMin = 1e9, stop = -1e9;
	for (int i = 0; i <= 4410; i++) {
		double f = i * 10.0, w = 2 * M_PI * f / 88200.0;
		double H = 0.5;
		for (int j = 0; j < HalfBand::kPairs; j++) {
			H += 2 * (HalfBand::kCoef[j] / 2147483648.0) * std::cos(w * (2 * j + 1));
		}
		double db = 20 * std::log10(std::fabs(H) + 1e-12);
		if (f <= 19000) {
			ripMax = std::max(ripMax, db);
			ripMin = std::min(ripMin, db);
		}
		if (f >= 25100) {
			stop = std::max(stop, db);
		}
	}
	printf("  passband 0..19 kHz: %+.4f..%+.4f dB, stopband >= 25.1 kHz: %.1f dB\n", ripMin, ripMax, stop);
	CHECK(ripMax - ripMin < 0.01 && stop < -70, "response");
	// vector vs scalar, and vs a double-precision FIR on the same input
	const int n = 4096;
	std::vector<int32_t> ev(HalfBand::kEvenHistory + n), od(HalfBand::kOddHistory + n), out(n);
	for (auto& v : ev) {
		v = se24(rnd());
	}
	for (auto& v : od) {
		v = se24(rnd());
	}
	HalfBand::process(ev.data(), od.data(), out.data(), n);
	int vecBad = 0, worst = 0;
	for (int m = 0; m < n; m++) {
		vecBad += out[m] != HalfBand::processOne(ev.data(), od.data(), m);
		double acc = 0.5 * od[m];
		for (int j = 0; j < HalfBand::kPairs; j++) {
			acc += (HalfBand::kCoef[j] / 2147483648.0) * (double(ev[m + 15 - j]) + ev[m + 16 + j]);
		}
		long ref = std::lround(std::max(-2147483648.0, std::min(2147483647.0, acc * 128)));
		worst = std::max<long>(worst, std::labs(ref - out[m]) >> 7);
	}
	CHECK(vecBad == 0 && worst <= 1, "vector mismatches %d, worst %d LSB", vecBad, worst);
	printf("  %s vs scalar: %d differ; vs double FIR: worst %d LSB (24-bit)\n", JP_OSC_NEON ? "NEON" : "scalar", vecBad,
	       worst);
	// sines through the decimator: level in the passband, rejection of what would alias
	printf("  sine level after decimation (dB):");
	for (double f : {1000.0, 10000.0, 18000.0, 21000.0, 23100.0, 26000.0, 30000.0, 40000.0}) {
		std::vector<int32_t> e(HalfBand::kEvenHistory + n), o(HalfBand::kOddHistory + n), y(n);
		for (int t = 0; t < 2 * (n + 31); t++) {
			int32_t v = static_cast<int32_t>(std::lround(0x400000 * std::sin(2 * M_PI * f * t / 88200.0)));
			int idx = t >> 1;
			if (t & 1) {
				if (idx - 15 >= 0 && idx - 15 < (int)o.size()) {
					o[idx - 15] = v;
				}
			}
			else if (idx < (int)e.size()) {
				e[idx] = v;
			}
		}
		HalfBand::process(e.data(), o.data(), y.data(), n);
		double acc = 0;
		for (int m = 64; m < n; m++) {
			acc += double(y[m] >> 7) * (y[m] >> 7);
		}
		printf(" %.0fk %.2f", f / 1000, 10 * std::log10(acc / (n - 64) / (0.5 * 0x400000 * 0x400000.0p0)));
	}
	printf("\n");
}

static void testSpectralDetune() {
	printf("A6 saw frequencies over 1 s at note 60 (Hz, measured from counters vs table)\n");
	double worst = 0;
	for (int det : {0, 64, 103, 127}) {
		SuperSaw::Controls k{jpPitch(60), detuneTarget(det), SuperSaw::mixTarget(64)};
		SuperSaw::Core c = settledCore(k);
		uint64_t total[7] = {};
		uint32_t last[7];
		for (int i = 0; i < 7; i++) {
			last[i] = c.phase[i];
		}
		for (int t = 0; t < 88200; t++) {
			SuperSaw::frame(c, k);
			for (int i = 0; i < 7; i++) {
				total[i] += (c.phase[i] - last[i]) >> 8;
				last[i] = c.phase[i];
			}
		}
		int32_t base = refDetuneBase(k.pitch, k.detuneTarget);
		printf("  detune %3d:", det);
		for (int i = 0; i < 7; i++) {
			double hz = total[i] / 16777216.0;
			double expect = (k.pitch + ((int64_t{kDetuneTable[i]} * base) >> 7)) * 88200.0 / 16777216.0;
			worst = std::max(worst, std::fabs(hz - expect));
			printf(" %.3f", hz);
		}
		printf("\n");
	}
	CHECK(worst <= 0.01, "worst %.4f Hz", worst);
}

static void testMixGains() {
	printf("A7 side/centre gain vs (mix >> 16) / 25\n");
	double worst = 0;
	for (int s = 0; s < 128; s++) {
		SuperSaw::Controls k{jpPitch(60), detuneTarget(64), SuperSaw::mixTarget(s)};
		SuperSaw::Core c = settledCore(k);
		c.mix = SuperSaw::mixTarget(s > 0 ? s - 1 : 0); // approach from below, as after turning the knob up
		for (int t = 0; t < 20000; t++) {
			SuperSaw::frame(c, k);
		}
		double got = 20 * std::log10((c.mix >> 16) / 25.0);
		double expect = 20 * std::log10(((102400 + 16384 * s) >> 16) / 25.0);
		worst = std::max(worst, std::fabs(got - expect));
	}
	CHECK(worst <= 0.05, "worst %.3f dB", worst);
	printf("  worst difference over 128 steps: %.3f dB\n", worst);
}

static void testSzabo() {
	printf("A8 Szabo cross-check (informational): detune spread vs Szabo's polynomial\n");
	const double poly[] = {10028.7312891634,  -50818.8652045924, 111363.4808729368, -138150.6761080548,
	                       106649.6679158292, -53046.9642751875, 17019.9518580080,  -3425.0836591318,
	                       404.2703938388,    -24.1878824391,    0.6717417634,      0.0030115596};
	double dot = 0, aa = 0, bb = 0;
	for (int s = 0; s < 128; s++) {
		double x = s / 127.0, sz = 0;
		for (double c : poly) {
			sz = sz * x + c;
		}
		double ours = detuneTarget(s) / double(detuneTarget(127));
		dot += ours * sz;
		aa += ours * ours;
		bb += sz * sz;
	}
	printf("  shape correlation %.4f (Szabo's curve is a fit to recordings, not the table)\n",
	       dot / std::sqrt(aa * bb));
}

static void testPhase() {
	printf("A9 / free-running phase: 100 note-ons, no resets, 7 saws at every phase width\n");
	SuperSaw::State st;
	SuperSaw::Controls k0{jpPitch(60), detuneTarget(64), SuperSaw::mixTarget(64)};
	SuperSaw::seed(st, k0, 99);
	int32_t buf[128];
	long bad = 0;
	for (int noteOn = 0; noteOn < 100; noteOn++) {
		int note = 24 + (rnd() % 80);
		uint32_t inc =
		    static_cast<uint32_t>(std::lround(440.0 * std::pow(2.0, (note - 69) / 12.0) * 4294967296.0 / 44100));
		uint32_t pw = rnd();
		int32_t wi = static_cast<int32_t>(rnd()) >> 1;
		SuperSaw::Controls k{SuperSaw::pitchFromPhaseIncrement(inc), detuneTarget(SuperSaw::detuneStep(wi)),
		                     SuperSaw::mixTarget(SuperSaw::mixStep(pw))};
		SuperSaw::Core before = st.core;
		int n = 1 + rnd() % 128;
		SuperSaw::render(buf, n, inc, wi, pw, st);
		// replay with frame(): the counters must have moved on from where they were, never reset
		SuperSaw::Core replay = before;
		for (int t = 0; t < 2 * n; t++) {
			SuperSaw::frame(replay, k);
		}
		for (int i = 0; i < 7; i++) {
			bad += replay.phase[i] != st.core.phase[i];
		}
		// all seven saws sound whatever the phase width: distinct increments, side gain above zero
		int32_t base = refDetuneBase(k.pitch, st.core.detune);
		bad += (st.core.mix >> 16) <= 0;
		bad += base == 0 && SuperSaw::detuneStep(wi) > 0;
	}
	CHECK(bad == 0, "%ld phase or saw-count faults", bad);
	// chunking: one 128-sample call equals 37 + 91
	SuperSaw::State a, b;
	SuperSaw::seed(a, k0, 7);
	b = a;
	int32_t x[128], y[128];
	uint32_t inc = 0x1000000;
	SuperSaw::render(x, 128, inc, 0, 0x40000000, a);
	SuperSaw::render(y, 37, inc, 0, 0x40000000, b);
	SuperSaw::render(y + 37, 91, inc, 0, 0x40000000, b);
	int diff = 0;
	for (int i = 0; i < 128; i++) {
		diff += x[i] != y[i];
	}
	CHECK(diff == 0, "chunked render differs in %d samples", diff);
	printf("  100 note-ons: %ld faults; 128 vs 37+91 samples: %d differ\n", bad, diff);
}

static void testPitchMapping() {
	printf("Pitch: Deluge phase increment -> JP increment\n");
	int snapped = 0;
	double worstCents = 0;
	for (int note = 0; note <= 116; note++) {
		uint32_t inc =
		    static_cast<uint32_t>(std::lround(440.0 * std::pow(2.0, (note - 69) / 12.0) * 4294967296.0 / 44100));
		int32_t p = SuperSaw::pitchFromPhaseIncrement(inc);
		snapped += p == jpPitch(note);
		worstCents = std::max(worstCents, std::fabs(1200 * std::log2(double(p) / etPitch(note))));
	}
	printf("  in-tune notes 0..116 on the JP value: %d/117; worst distance from equal temperament %.3f cents\n",
	       snapped, worstCents);
	CHECK(snapped == 117, "snapped %d", snapped);
}

int main() {
	printf("JP Super Saw host tests, %s constants, %s\n", kTracedConstants ? "traced" : "public",
	       JP_OSC_NEON ? "NEON (simde on host)" : "scalar");
	testTables();
	testIncrements();
	testInternalRender();
	testHighPass();
	testDecimator();
	testSpectralDetune();
	testMixGains();
	testSzabo();
	testPhase();
	testPitchMapping();
	printf(failures ? "FAILED: %d\n" : "ALL PASSED\n", failures);
	return failures ? 1 : 0;
}
