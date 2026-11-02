/*
 * Copyright © 2014-2023 Synthstrom Audible Limited
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

#include "definitions_cxx.hpp"
#include "dsp/oscillators/jp_supersaw.h"
#include "dsp/oscillators/supersaw.h"
#include "model/sample/sample.h"
#include <gsl/gsl>

class TimeStretcher;
class VoiceSamplePlaybackGuide;
class Source;
class Voice;
class VoiceSample;
class LivePitchShifter;
class DxVoice;

class VoiceUnisonPartSource {
public:
	VoiceUnisonPartSource() = default;
	~VoiceUnisonPartSource();
	bool noteOn(Voice* voice, Source* source, VoiceSamplePlaybackGuide* voiceSource, uint32_t samplesLate,
	            uint32_t oscPhase, bool resetEverything, SynthMode synthMode, uint8_t velocity);
	void unassign(bool deletingSong);
	bool getPitchAndSpeedParams(Source* source, VoiceSamplePlaybackGuide* voiceSource, uint32_t* phaseIncrement,
	                            uint32_t* timeStretchRatio, uint32_t* noteLengthInSamples);
	uint32_t getSpeedParamForNoSyncing(Source* source, int32_t phaseIncrement, int32_t pitchAdjustNeutralValue);

	uint32_t oscPos; // FKA phase. No longer used for Sample playback / rate conversion position. Only waves, including
	                 // wavetable.
	deluge::dsp::Supersaw::SidePhases supersawPhases = deluge::dsp::Supersaw::sidePhasesFrom(0);
	// JP mode: lease of a free-running Super Saw (same address always gets the same counters).
	// Null if JP mode is off or the bank is full; then the stock supersaw renders.
	gsl::owner<deluge::dsp::jp::SuperSaw::State*> jpSuperSaw = nullptr;
	uint32_t phaseIncrementStoredValue;
	int32_t carrierFeedback;
	bool active;
	VoiceSample* voiceSample = nullptr;
	LivePitchShifter* livePitchShifter = nullptr;
	gsl::owner<DxVoice*> dxVoice = nullptr;
};
