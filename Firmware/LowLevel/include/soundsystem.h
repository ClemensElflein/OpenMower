// Created by Elmar Elflein on 18/07/22.
// Copyright (c) 2022 Elmar Elflein. All rights reserved.
//
// This work is licensed under a Creative Commons Attribution-NonCommercial-ShareAlike 4.0 International License.
//
// Feel free to use the design in your private/educational projects, but don't try to sell the design or products based on it without getting my consent first.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
//
//
#ifndef _SOUND_SYSTEM_H_
#define _SOUND_SYSTEM_H_

#include <Arduino.h>
#include <stdint.h>

#include <list>
#include <map>
#include <string>

#include "datatypes.h"

namespace soundSystem {

enum class TrackType {
    BACKGROUND = 1,  // Background tracks are stored in folder mp3 and get interrupted/aborted by higher priority sound like advert
    ADVERT,          // Advert tracks are stored in language specific folder, i.e. "01" US or "49" German, and interrupt/stop background sounds
};
struct CardSources {
    bool origin : 1;         // This sound track is available on origin SD-Card
    bool improve_sound : 1;  // Available since ImproveSound PR
};
struct TrackFlags {
    bool repeat : 1;           // Repeat this track. This flag is limited to background sounds!
    bool stop_background : 1;  // Stop replaying of a current running background track after this sound got played
};
struct TrackDef {
    uint16_t num;  // SD-Card's track number
    TrackType type;
    CardSources card_sources;
    TrackFlags track_flags;
    unsigned long pauseAfter = 0;     // Cosmetic pause in ms, after advert track got played, before the next sound get processed from queue.
    int32_t repeatDuration = 180000;  // How long (ms) to repeat a background sound. Default to 180 sec. noise pollution (i.e. VdS 2300)
};

// For better reading, let's use track macro names with the detailed track definition
// clang-format off
#define SOUND_TRACK_BGD_OM_BOOT              TrackDef{ 2, TrackType::BACKGROUND, CardSources{.improve_sound=true}, TrackFlags{.repeat=true}}
#define SOUND_TRACK_ADV_HI_IM_STEVE          TrackDef{ 1, TrackType::ADVERT,     CardSources{.origin=true, .improve_sound=true}, .pauseAfter=1500}
#define SOUND_TRACK_ADV_IMU_INIT_FAILED      TrackDef{19, TrackType::ADVERT,     CardSources{.improve_sound=true}, .pauseAfter=500}
#define SOUND_TRACK_BGD_OM_ALARM             TrackDef{15, TrackType::BACKGROUND, CardSources{.improve_sound=true}, TrackFlags{.repeat=true}, .repeatDuration=20000}
#define SOUND_TRACK_ADV_EMERGENCY_STOP       TrackDef{ 8, TrackType::ADVERT,     CardSources{.origin=true, .improve_sound=true}, TrackFlags{.stop_background=true}, .pauseAfter = 500}
#define SOUND_TRACK_ADV_EMERGENCY_LIFT       TrackDef{ 9, TrackType::ADVERT,     CardSources{.origin=true, .improve_sound=true}, TrackFlags{.stop_background=true}, .pauseAfter = 500}
#define SOUND_TRACK_ADV_EMERGENCY_ROS        TrackDef{24, TrackType::ADVERT,     CardSources{.improve_sound=true}, TrackFlags{.stop_background=true}, .pauseAfter = 500}
#define SOUND_TRACK_ADV_EMERGENCY_CLEARED    TrackDef{23, TrackType::ADVERT,     CardSources{.improve_sound=true}, TrackFlags{.stop_background=true}, .pauseAfter = 500}
#define SOUND_TRACK_BGD_EMERGENCY_ALARM      TrackDef{ 9, TrackType::BACKGROUND, CardSources{.improve_sound=true}}
#define SOUND_TRACK_ADV_OM_STARTUP_SUCCESS   TrackDef{ 2, TrackType::ADVERT,     CardSources{.origin=true, .improve_sound=true}, .pauseAfter = 1500}
#define SOUND_TRACK_ADV_ROS_INIT             TrackDef{ 3, TrackType::ADVERT,     CardSources{.origin=true, .improve_sound=true}, TrackFlags{.stop_background=true}, .pauseAfter = 500}
#define SOUND_TRACK_BGD_ROS_BOOT             TrackDef{ 5, TrackType::BACKGROUND, CardSources{.improve_sound=true}, TrackFlags{.repeat=true}}
#define SOUND_TRACK_ADV_ROS_STARTUP_SUCCESS  TrackDef{16, TrackType::ADVERT,     CardSources{.improve_sound=true}, TrackFlags{.stop_background=true}}
#define SOUND_TRACK_ADV_ROS_STOPPED          TrackDef{17, TrackType::ADVERT,     CardSources{.improve_sound=true}, TrackFlags{.stop_background=true}}
#define SOUND_TRACK_ADV_MAP_RECORD_START     TrackDef{ 4, TrackType::ADVERT,     CardSources{.origin=true, .improve_sound=true}, TrackFlags{.stop_background=true}}
#define SOUND_TRACK_ADV_AUTONOMOUS_START     TrackDef{12, TrackType::ADVERT,     CardSources{.origin=true, .improve_sound=true}, TrackFlags{.stop_background=true}, .pauseAfter = 1500}
#define SOUND_TRACK_ADV_RAIN                 TrackDef{10, TrackType::ADVERT,     CardSources{.origin=true, .improve_sound=true}, TrackFlags{.stop_background=true}, .pauseAfter = 3000}
#define SOUND_TRACK_ADV_RTKGPS_WAIT          TrackDef{ 5, TrackType::ADVERT,     CardSources{.origin=true, .improve_sound=true}, .pauseAfter = 1500}
#define SOUND_TRACK_ADV_RTKGPS_POOR          TrackDef{20, TrackType::BACKGROUND, CardSources{.improve_sound=true}}
#define SOUND_TRACK_ADV_RTKGPS_MODERATE      TrackDef{21, TrackType::BACKGROUND, CardSources{.improve_sound=true}}
#define SOUND_TRACK_ADV_RTKGPS_GOOD          TrackDef{22, TrackType::BACKGROUND, CardSources{.improve_sound=true}}
#define SOUND_TRACK_BGD_MUSIC_PINK_PANTHER   TrackDef{12, TrackType::BACKGROUND, CardSources{.improve_sound=true}}
#define SOUND_TRACK_ADV_UP                   TrackDef{21, TrackType::ADVERT,     CardSources{.improve_sound=true}, .pauseAfter = 100}
#define SOUND_TRACK_ADV_DOWN                 TrackDef{20, TrackType::ADVERT,     CardSources{.improve_sound=true}, .pauseAfter = 100}
#define SOUND_TRACK_ADV_LANGUAGE             TrackDef{22, TrackType::ADVERT,     CardSources{.improve_sound=true}, .pauseAfter = 100}
#define SOUND_TRACK_ADV_MOW_DONE_DOCK        TrackDef{11, TrackType::ADVERT,     CardSources{.improve_sound=true}, TrackFlags{.stop_background=true}, .pauseAfter = 3000}
// clang-format on

bool begin();  // Init serial stream, soundmodule and sound_available_

void playSound(const TrackDef&);       // Play sound trackDef. This method writes sound trackDef in a list, the method processSounds() (has to run in loop) will play the sounds according to the list
void playSoundAdHoc(const TrackDef&);  // Play sound track number immediately without waiting until the end of sound

void setDFPis5V(const bool t_dfpis5v);  // Set if DFP is set to 5V Vcc
void setEnableBackground(const bool);   // Set if background sounds shall get played (true) or not (false)

void setLanguage(const iso639_1 language_p, const bool quiet = false);  // Set language to the pointing ISO639-1 (2 char) language code and announce if changed and not quiet

void setVolume(const uint8_t t_vol);  // Set volume (0-100%)
uint8_t setVolumeUp();                // Scale volume up by VOLUME_STEPS and return new volume (%)
uint8_t setVolumeDown();              // Scale volume down by VOLUME_STEPS and return new volume (%)

void applyConfig(const ll_high_level_config t_config, const bool quiet);  // Apply the volume specific config options

void processSounds(const ll_status t_ll_state, const bool t_ros_running, const ll_high_level_state t_hl_state);  // This method has to be called cyclic, e.g. every second.
}  // namespace soundSystem
#endif  // _SOUND_SYSTEM_H_  HEADER_FILE
