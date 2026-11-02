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
// Instruction-count harness for the JP Super Saw render on Cortex-A9. Cross-compiled freestanding and run
// under an instruction-counting emulator by count_instructions.py. Not part of the firmware build.
#include "dsp/oscillators/jp_supersaw.h"

using deluge::dsp::jp::SuperSaw;

static SuperSaw::State state;
static int32_t buffer[128];

extern "C" {
// called once: seeds the voice and runs it long enough that the decimator history is warm
void benchSetup() {
	SuperSaw::render(buffer, 128, 0x1000000, 0, 0x40000000, state);
}
// steady note: smoothers settled
void benchSettled(int32_t blocks) {
	for (int32_t b = 0; b < blocks; b++) {
		SuperSaw::render(buffer, 128, 0x1000000, 0, 0x40000000, state);
	}
}
// knob moving every block: the smoothers never settle
void benchMoving(int32_t blocks) {
	for (int32_t b = 0; b < blocks; b++) {
		SuperSaw::render(buffer, 128, 0x1000000, (b & 1) ? (1 << 29) : -(1 << 29), (b & 1) ? 0x10000000 : 0x70000000,
		                 state);
	}
}
}
