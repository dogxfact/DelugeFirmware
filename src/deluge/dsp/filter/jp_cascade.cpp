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
#include "dsp/filter/jp_cascade.h"
#include "dsp/filter/jp_filter_core.h"
#include <cmath>

namespace deluge::dsp::filter {

q31_t JpCascadeFilter::process(q31_t input, q31_t& low1, q31_t& band1, q31_t& low2, q31_t& band2, q31_t f, q31_t damp,
                               uint32_t& noise) {
	if (damp <= jp::kJpFiltMinDamp * 2) {
		noise = noise * 1664525u + 1013904223u;
		input = jp::filtSat(static_cast<int64_t>(input) + (static_cast<int32_t>(noise) >> 20));
	}
	return jp::filtProcess(input, low1, band1, low2, band2, f, damp);
}

q31_t JpCascadeFilter::setConfig(q31_t frequency, q31_t resonance, FilterMode, q31_t, q31_t filterGain) {
	float knob = static_cast<float>(frequency) / ONE_Q31f;
	if (knob < 0.0f) {
		knob = 0.0f;
	}
	if (knob > 1.0f) {
		knob = 1.0f;
	}
	float hz = jp::kJpFiltFminHz * std::pow(jp::kJpFiltFmaxHz / jp::kJpFiltFminHz, knob);
	float ff = 2.0f * std::sin(static_cast<float>(M_PI) * hz / 44100.0f);
	if (ff > 0.75f) {
		ff = 0.75f; // Chamberlin stability
	}
	f = static_cast<q31_t>(ff * 2147483648.0f);

	// resonance 0..~2^28 (same raw range as the other filters). Map to damp.
	int32_t r = resonance < 0 ? 0 : resonance;
	if (r > (1 << 28)) {
		r = 1 << 28;
	}
	int32_t t = r << 3; // ~0..2^31
	// damp = max - t * (max - min) / 2^31
	int32_t span = jp::kJpFiltMaxDamp - jp::kJpFiltMinDamp;
	damp = jp::kJpFiltMaxDamp - static_cast<int32_t>(((int64_t)span * t) >> 31);
	if (damp < jp::kJpFiltMinDamp) {
		damp = jp::kJpFiltMinDamp;
	}
	return filterGain;
}

void JpCascadeFilter::doFilter(q31_t* startSample, q31_t* endSample, int32_t sampleIncrement) {
	q31_t low1 = l.low1, band1 = l.band1, low2 = l.low2, band2 = l.band2;
	q31_t cf = f, cd = damp;
	uint32_t n = noise;
	q31_t* s = startSample;
	do {
		*s = process(*s, low1, band1, low2, band2, cf, cd, n);
		s += sampleIncrement;
	} while (s < endSample);
	l.low1 = low1;
	l.band1 = band1;
	l.low2 = low2;
	l.band2 = band2;
	noise = n;
}

void JpCascadeFilter::doFilterStereo(q31_t* startSample, q31_t* endSample) {
	q31_t l1 = l.low1, b1 = l.band1, l2 = l.low2, b2 = l.band2;
	q31_t r1 = r.low1, rb1 = r.band1, r2 = r.low2, rb2 = r.band2;
	q31_t cf = f, cd = damp;
	uint32_t n = noise;
	q31_t* s = startSample;
	do {
		s[0] = process(s[0], l1, b1, l2, b2, cf, cd, n);
		s[1] = process(s[1], r1, rb1, r2, rb2, cf, cd, n);
		s += 2;
	} while (s < endSample);
	l.low1 = l1;
	l.band1 = b1;
	l.low2 = l2;
	l.band2 = b2;
	r.low1 = r1;
	r.band1 = rb1;
	r.low2 = r2;
	r.band2 = rb2;
	noise = n;
}

} // namespace deluge::dsp::filter
