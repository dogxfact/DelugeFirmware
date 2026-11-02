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

#include "dsp/filter/filter.h"
#include "dsp/oscillators/jp_osc_constants.h"
#include "util/fixedpoint.h"

namespace deluge::dsp::filter {

/// JP-style -24 dB/oct: two cascaded 12 dB Chamberlin stages, exponential cutoff.
class JpCascadeFilter : public Filter<JpCascadeFilter> {
public:
	JpCascadeFilter() = default;
	q31_t setConfig(q31_t frequency, q31_t resonance, FilterMode lpfMode, q31_t lpfMorph, q31_t filterGain);
	void doFilter(q31_t* startSample, q31_t* endSample, int32_t sampleIncrement);
	void doFilterStereo(q31_t* startSample, q31_t* endSample);
	void resetFilter() {
		l = {};
		r = {};
		noise = 1;
	}

	/// One sample. Exposed for host tests.
	static inline q31_t process(q31_t input, q31_t& low1, q31_t& band1, q31_t& low2, q31_t& band2, q31_t f, q31_t damp,
	                            uint32_t& noise);

private:
	struct State {
		q31_t low1 = 0;
		q31_t band1 = 0;
		q31_t low2 = 0;
		q31_t band2 = 0;
	};
	State l;
	State r;
	q31_t f = 0;
	q31_t damp = jp::kJpFiltMaxDamp;
	uint32_t noise = 1;
};

} // namespace deluge::dsp::filter
