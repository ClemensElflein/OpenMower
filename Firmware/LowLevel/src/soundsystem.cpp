// Created by Elmar Elflein on 18/07/22.
// Copyright (c) 2022 Elmar Elflein. All rights reserved.
// Restructured by Jörg Ebeling on 10/16/23.
// Copyright (c) 2023, 2024 Jörg Ebeling. All rights reserved.
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

/*                          | Original      |        Old clones          |             Newer clones                |
 * DFPlayer Chip            | DFROBOT LISP3 | YX5200-24SS | MH2024K-24SS | MH2024K-16SS | GD3200B | JL AB23A799755 |
 * -----------------------------------------------------------------------------------------------------------------
 * Tested/working with OM   |      ok       |             |     ok       |              |  partly |       ok       |
 * Autoplay files *1        |      yes      |             |     no       |              |   no    |       no       |
 * Require reset()          |               |             |              |              |   YES   |                |
 * Reset->isOnline *2       |   800-850ms   |             |    750ms     |              |  600ms  |                |
 * Without SD Card          |   error 0x1   |             |              |              |         |                |
 * getTotalTrackCount()     |       0       |             |     24       |              |   24    |                |
 * getTotalFolderCount()    |       5       |             |      0       |              |    3    |                |
 * getStatus().source       |       2       |             |      0       |              |    0    |                |
 * playAdvertisement()      |     yes*4     |             |   yes*4      |              |   yes   |                |
 * setRepeatPlayAllInRoot() |   Plays ALL   |             |     yes      |              |         |                |
 * getCurrentTrack()        |      *3       |             |     *4       |              |    *3   |                |
 * OnPlayFinished()         | Play&Advert*5 |             |   *5,*6      |              | Play&Adv|                |
 * Auto increase volume *7  |      no       |             |     yes      |              |   yes   |      yes       |
 * -----------------------------------------------------------------------------------------------------------------
 * *1 = Autoplay (unasked) all (at least) root files (after reset()) when Pin-11 (IO_2) at GND (0.9.x-0.13.x)
 * *2 = Highly depends on SD-Card and content
 * *3 = Advert track get returned with an internal track number
 * *4 = Advert get only played with a current running play sound
 * *5 = Sometimes called a second time within 10-150ms
 * *6 = Two callbacks (advert & play) at the end of play sound
 * *7 = Auto increase volume if Pin-11 (IO_2) at GND (0.9.x-0.13.x)
 *
 * Conclusion by this evaluation:
 * DFPlayer's "advert" functionality is not usable for our requirements, as MH2024K-24S don't return a usable advert state.
 */

#include "soundsystem.h"

#include <Arduino.h>
#include <DFMiniMp3.h>
#include <etl/queue.h>

#include "debug.h"
#include "pins.h"

// Some quirky defines for old- sound-card-format detection
// clang-format off
#define DFP_DETECTION_BIT_END                (1 << 0)  // All detection phases ended
#define DFP_DETECTION_BIT_HAS_AUTOPLAY       (1 << 1)  // DFPlayer has played a track after reset
#define DFP_DETECTION_BIT_OLD_CARD_STRUCTURE (1 << 2)  // Detected old SD-Card structure
// clang-format on

#define DFP_HAS_AUTOPLAY_TIMEOUT 600                    // Timeout how long to test if DFPlayer is (wrongly) configured doing "autoplay" (IO2=GND)
#define DFP_AUTOPLAY_TIMEOUT 6000                      // Autoplayed track detection timeout. "Hi I'm Steve ..." is about 4.x seconds

namespace soundSystem {
namespace  // anonymous (private) namespace
{
class Mp3Notify_;                                // forward declaration
typedef DFMiniMp3<SerialPIO, Mp3Notify_> DfMp3;  // ... for a more readable/shorter DfMp3 typedef
                                                // typedef DFMiniMp3<SerialPIO, Mp3Notify, Mp3ChipMH2024K16SS> DfMp3; // Need to be tested if really required
SerialPIO soundSerial(PIN_SOUND_TX, PIN_SOUND_RX, 250);
DfMp3 myMP3(soundSerial);

etl::queue<TrackDef, SOUND_QUEUE_SIZE, etl::memory_model::MEMORY_MODEL_SMALL> active_sounds_;
bool sound_available_ = false;    // Sound module available as well as SD-Card with some kind of files
bool dfp_is_5v = false;           // Enable full sound if DFP is set to 5V Vcc
bool enable_background_ = false;  // Enable background sounds
uint8_t volume = VOLUME_DEFAULT;  // Last set volume (%)
std::string language_str = "en";  // Default ISO639-1 language string
uint8_t play_folder = 1;          // Default play folder, has to be related to language_str

uint16_t last_error_code_ = 0;  // Last DFPlayer error code. See DfMp3_Error for code meaning

ll_status last_ll_state = {};             // Last processed low-level state
ll_high_level_state last_hl_state_ = {};  // Last processed high-level state
unsigned long hl_mode_has_fix_ms_;         // Millis when the current high-level mode got his initial GPS fix. 0 if idle.
uint8_t hl_mode_flags_;                    // High level mode flags (assumptions), like initial GPS "fix", rain, docking...

TrackDef background_track_def_ = {};    // Current/last background track
TrackDef advert_track_def_ = {};        // Current/last playing advert track
bool current_playing_is_background_;     // Current/last playing sound is a background sound
unsigned long current_playing_started_;  // Millis when current/last playing sound got started

bool last_ros_running_ = false;        // Last ROS running state
unsigned long ros_running_since_ = 0;  // ros_running since millis state
unsigned long last_advert_end_;        // Millis when the last played advert sound ended. Used for pauseAfter calculation

// Describe specific (assumed) mode flags
enum ModeFlags : uint8_t {
    started = 0b1 << 0,       // Mode started
    primalGpsFix = 0b1 << 1,  // Got the primal GPS "fix", by which we can assume that i.e. the mower drive to his mowing start point
    rainDetected = 0b1 << 2,  // LL rain sensor signal received
    docking = 0b1 << 3,       // Heading back to base, i.e. due to rain detected
};

uint8_t dfp_detection_status = 0;              // Sound-card detection status bits i.e. for old- sound-card-format detection
unsigned long online_ms_ = 0;                   // millis() when module got detected as "online". Required for detection if module is configured (IO2) with autoplay

// Forward declarations
void playMowSound_();
bool handleAutoplay_(ll_status t_ll_state);
}  // namespace

bool begin() {
    myMP3.begin();

    // Some DFPlayer chips need a reset cmd for a defined 'ready' state, which is more precise than some kind of static (and large) 'delay(3000)'
    DEBUG_PRINTF("Reset...\n");
    myMP3.reset(false);  // Non-blocking reset

    unsigned long start_ms = millis();
    while (!myMP3.isOnline())  // loop till module is ready and "play media" is online
    {
        if (last_error_code_ == DfMp3_Error_Busy)  // Usually "media not found"
        {
            DEBUG_PRINTF("Busy error. No Media?\n");
            return false;
        }
        delay(50);
        myMP3.loop();
        if (millis() >= (start_ms + DFP_ONLINE_TIMEOUT)) {
            DEBUG_PRINTF("Reset timed out after %dms\n", millis() - start_ms);
            return false;
        }
    }
    DEBUG_PRINTF("Online after %dms\n", millis() - start_ms);
    online_ms_ = millis();

    uint16_t total_tracks = myMP3.getTotalTrackCount(DfMp3_PlaySource_Sd);
    DEBUG_PRINTF("getTotalTrackCount() = %d\n", total_tracks);

    uint16_t total_folder = myMP3.getTotalFolderCount();
    DEBUG_PRINTF("getTotalFolderCount() = %d\n", total_folder);

    // For future module evaluation/debugging
    DfMp3_Status status = myMP3.getStatus();
    DEBUG_PRINTF("getStatus().source = %d\n", status.source);

    if (!total_tracks && !total_folder) {
        DEBUG_PRINTF("Found no files nor folder on SD-Card!\n");
        return false;
    }
    sound_available_ = true;

    return sound_available_;
}

void setDFPis5V(const bool t_dfpis5v) {
    dfp_is_5v = t_dfpis5v;
}

void setEnableBackground(const bool t_bool) {
    enable_background_ = t_bool;
}

void setLanguage(const iso639_1 language_p, const bool quiet) {  // Set language to the pointing ISO639-1 (2 char) language code and announce if changed && not quiet
    uint8_t last_play_folder = play_folder;

    std::string language_str;
    for (size_t i = 0; i < sizeof(iso639_1); i++)
        language_str += language_p[i];

    if (auto it = language_to_playFolder_map.find(language_str.c_str()); it != language_to_playFolder_map.end())
        play_folder = it->second;

    if (!sound_available_ || play_folder == last_play_folder || quiet || !dfp_is_5v)
        return;

    playSoundAdHoc(tracks[SOUND_TRACK_ADV_LANGUAGE]);
}

void setVolume(const uint8_t t_vol)  // Set volume (0-100%)
{
    if (!sound_available_)
        return;

    // value of 30 is max equivalent to 100 %
    uint8_t val = (uint8_t)(30.0 / 100.0 * (double)t_vol);
    DEBUG_PRINTF("Set volume %d\n", val);
    myMP3.setVolume(val);
    volume = t_vol;
    delay(50);  // (sometimes) required for "DFR LISP3"
}

uint8_t setVolumeUp() {
    if (!sound_available_ || volume >= 100)
        return volume;

    setVolume(min(volume + VOLUME_STEPS, 100));
    if (dfp_is_5v)
        playSoundAdHoc(tracks[SOUND_TRACK_ADV_UP]);

    return volume;
}

uint8_t setVolumeDown() {
    if (!sound_available_ || volume < VOLUME_STEPS)
        return volume;

    setVolume(max(volume - VOLUME_STEPS, 0));
    if (dfp_is_5v)
        playSoundAdHoc(tracks[SOUND_TRACK_ADV_DOWN]);

    return volume;
}

void applyConfig(const ll_high_level_config t_config, const bool quiet) {
    setDFPis5V(t_config.options.dfp_is_5v == OptionState::ON);
    setVolume(t_config.volume);
    setLanguage(t_config.language, quiet);
    setEnableBackground(t_config.options.background_sounds == OptionState::ON || true);
}

void playSoundAdHoc(const TrackDef &t_track_def) {
    DEBUG_PRINTF("playSoundAdHoc(num %d, type %d, flags " PRINTF_BINARY_PATTERN_INT8 ")\n", t_track_def.num, t_track_def.type, PRINTF_BYTE_TO_BINARY_INT8(t_track_def.flags));

    if (!sound_available_ || !(dfp_detection_status & DFP_DETECTION_BIT_END))
        return;

    switch (t_track_def.type) {
        case TrackTypes::background:
            if (!enable_background_)
                return;
            myMP3.stop();
            delay(50);  // (sometimes) required for "MH2024K-24SS"
            myMP3.playMp3FolderTrack(t_track_def.num);
            delay(50);  // (sometimes) required for "MH2024K-24SS"
            background_track_def_ = t_track_def;
            current_playing_is_background_ = true;
            break;
        case TrackTypes::advert:
            myMP3.stop();
            delay(50);                                                          // (sometimes) required for "MH2024K-24SS"
            if (dfp_detection_status & DFP_DETECTION_BIT_OLD_CARD_STRUCTURE) {  // Old SD-Card detected (or assumed)
                // Simple support for old SD-Card format
                myMP3.playGlobalTrack(t_track_def.num);
            } else {
                // ATTENTION: Don't use playFolderTrack16() as it does NOT work reliable across DFPlayer clones (as of writing). But playFolderTrack() does.
                myMP3.playFolderTrack(play_folder, t_track_def.num);
            }
            delay(50);  // (sometimes) required for "MH2024K-24SS"
            advert_track_def_ = t_track_def;
            current_playing_is_background_ = false;
            break;
        default:
            // advert_raw not yet implemented
            return;
            break;
    }
    current_playing_started_ = millis();

    if (t_track_def.flags & TrackFlags::repeat) {
        myMP3.setRepeatPlayCurrentTrack(true);  // FIXME: Does NOT work reliable, see manual handling in Mp3Notify::OnPlayFinished()
    }

    if (t_track_def.flags & TrackFlags::stopBackground) {
        background_track_def_ = {};
    }
}

void playSound(const TrackDef &t_track_def) {
    DEBUG_PRINTF("playSound(num %d, type %d, flags " PRINTF_BINARY_PATTERN_INT8 ")\n", t_track_def.num, t_track_def.type, PRINTF_BYTE_TO_BINARY_INT8(t_track_def.flags));

    if (!sound_available_ || active_sounds_.full())
        return;

    active_sounds_.push(t_track_def);
}

/**
 * @brief Handle all kind of emergency sounds
 *
 * @param t_ll_state
 */
void handleEmergencies(const ll_status t_ll_state, const bool t_ros_running) {
    if ((last_ll_state.emergency_bitmask & LL_EMERGENCY_BIT_LATCH) == (t_ll_state.emergency_bitmask & LL_EMERGENCY_BIT_LATCH))
        return;  // Ignore stop button or wheel lift changes if latch didn't changed

    if (t_ll_state.v_charge > 20.0f ||                                   // No emergencies while docked
        !t_ros_running ||                                                // No emergencies as long as ROS does not run
        !ros_running_since_ ||                                           // Pico just started up, while ROS might already running (which happen i.e. after a FW update)
        millis() < (ros_running_since_ + ROS_RUNNING_BEFORE_EMERGENCY))  // No emergency handling before ROS is running that long
    {
        last_ll_state.emergency_bitmask = t_ll_state.emergency_bitmask;
        return;
    }

    if (!(last_ll_state.emergency_bitmask & LL_EMERGENCY_BIT_LATCH) && (t_ll_state.emergency_bitmask & LL_EMERGENCY_BIT_LATCH)) { // Latch changed from 0 -> 1
        if (t_ll_state.emergency_bitmask & LL_EMERGENCY_BIT_STOP) {
            playSoundAdHoc(tracks[SOUND_TRACK_ADV_EMERGENCY_STOP]);
            // Do not take attention via "Bida bida" as user pressed the button itself
        } else if (t_ll_state.emergency_bitmask & LL_EMERGENCY_BIT_LIFT) {
            playSoundAdHoc(tracks[SOUND_TRACK_ADV_EMERGENCY_LIFT]);
            playSound(tracks[SOUND_TRACK_BGD_EMERGENCY_ALARM]);
        } else {
            playSoundAdHoc(tracks[SOUND_TRACK_ADV_EMERGENCY_ROS]);
            playSound(tracks[SOUND_TRACK_BGD_EMERGENCY_ALARM]);
        }
    } else {
        // Latch changed from 1 -> 0
        playSoundAdHoc(tracks[SOUND_TRACK_ADV_EMERGENCY_CLEARED]);
    }
    last_ll_state.emergency_bitmask = t_ll_state.emergency_bitmask;
}

/**
 * @brief Handle GPS quality related sounds
 *
 * @param t_ll_state
 */
void handleGpsQuality(const ll_status t_ll_state, const bool t_ros_running, const ll_high_level_state t_hl_state) {
    static unsigned long next_gps_sound_cycle = millis();  // Next cycle when a GPS ping sound might get played

    if (!t_ros_running || t_hl_state.gps_quality == last_hl_state_.gps_quality || millis() < next_gps_sound_cycle)
        return;

    next_gps_sound_cycle = millis() + GPS_SOUND_CYCLETIME;

    switch (HighLevelState::getMode(t_hl_state.current_mode)) {
        case HighLevelState::Mode::Recording:
            // Ping only rated GPS quality changes
            if (t_hl_state.gps_quality < 50) {
                if (last_hl_state_.gps_quality >= 50)
                    playSound(tracks[SOUND_TRACK_ADV_RTKGPS_POOR]);  // GPS poor ping
            } else if (t_hl_state.gps_quality < 75) {
                if (last_hl_state_.gps_quality < 50 || last_hl_state_.gps_quality >= 75)
                    playSound(tracks[SOUND_TRACK_ADV_RTKGPS_MODERATE]);  // GPS moderate/acceptable ping
            } else {                                                     // Current GPS quality >= 75
                if (last_hl_state_.gps_quality < 75)
                    playSound(tracks[SOUND_TRACK_ADV_RTKGPS_GOOD]);  // GPS good ping

                hl_mode_flags_ |= ModeFlags::primalGpsFix;
                if (!hl_mode_has_fix_ms_)
                    hl_mode_has_fix_ms_ = millis();
            }
            break;

        case HighLevelState::Mode::Autonomous:
            // Stalking "Pink Panther" sound only once when starting to mow
            if (HighLevelState::getAutonomousSubMode(t_hl_state.current_mode) != HighLevelState::SubModeAutonomous::Mowing ||
                (hl_mode_flags_ & ModeFlags::primalGpsFix) ||
                (t_hl_state.gps_quality < 75))
                break;
            playSound(tracks[SOUND_TRACK_BGD_MUSIC_PINK_PANTHER]);  // Stalking "Pink Panther"
            hl_mode_flags_ |= ModeFlags::primalGpsFix;
            if (!hl_mode_has_fix_ms_)
                hl_mode_has_fix_ms_ = millis();
            break;

        default:
            break;
    }
    last_hl_state_.gps_quality = t_hl_state.gps_quality;
}

void processSounds(const ll_status t_ll_state, const bool t_ros_running, const ll_high_level_state t_hl_state) {
    if (!sound_available_)
        return;

    myMP3.loop();

    if (!handleAutoplay_(t_ll_state))  // Still in Autoplay detection and Sound-card-format (old/new) detection
        return;

    // Next cycle for sound processing reached?
    static unsigned long next_cycle;
    if (!(millis() > next_cycle)) {
        return;
    }
    next_cycle = millis() + PROCESS_CYCLETIME;

    // If get docked, shut up
    if (t_ll_state.v_charge > 20.0f && last_ll_state.v_charge < 10.0f) {
        myMP3.stop();
        background_track_def_ = {};
        active_sounds_.clear();
    }
    last_ll_state.v_charge = t_ll_state.v_charge;

    // Full sound support if DFP is set to 5V Vcc, but not before Pico is initialized.
    // The latter is required to ensure the right (user logical) order of sounds, because i.e. Raspi get powered before Pico reach initialized state,
    // but should be announced in a logic 
    // Also IMU failure should be 
    //if (dfp_is_5v && t_ll_state.status_bitmask & LL_STATUS_BIT_INITIALIZED) {
    if (dfp_is_5v && false) {
        handleEmergencies(t_ll_state, t_ros_running);

        // Handle LL status_bitmask changes
        const uint8_t changed_status = t_ll_state.status_bitmask ^ last_ll_state.status_bitmask;
        DEBUG_PRINTF("Changed status_bitmask " PRINTF_BINARY_PATTERN_INT8 " (new status " PRINTF_BINARY_PATTERN_INT8 " XOR last status " PRINTF_BINARY_PATTERN_INT8 "), HL mode %d\n",
                     PRINTF_BYTE_TO_BINARY_INT8(changed_status),
                     PRINTF_BYTE_TO_BINARY_INT8(t_ll_state.status_bitmask),
                     PRINTF_BYTE_TO_BINARY_INT8(last_ll_state.status_bitmask),
                     t_hl_state.current_mode);

        if (!(last_ll_state.status_bitmask & LL_STATUS_BIT_INITIALIZED) && (t_ll_state.status_bitmask & LL_STATUS_BIT_INITIALIZED)) {
            playSound(tracks[SOUND_TRACK_ADV_OM_STARTUP_SUCCESS]);  // OM startup successful
        }
        if (!t_ros_running && !(last_ll_state.status_bitmask & LL_STATUS_BIT_RASPI_POWER) && (t_ll_state.status_bitmask & LL_STATUS_BIT_RASPI_POWER)) {
            playSound(tracks[SOUND_TRACK_ADV_ROS_INIT]);  // Initializing ROS
            // We're in a new "Raspi/ROS" bootup phase, which might take longer. Change background sound for better identification
            playSound(tracks[SOUND_TRACK_BGD_ROS_BOOT]);
        }
        if (!(last_ll_state.status_bitmask & LL_STATUS_BIT_RAIN) && (t_ll_state.status_bitmask & LL_STATUS_BIT_RAIN)) {
            if (HighLevelState::getMode(t_hl_state.current_mode) == HighLevelState::Mode::Autonomous && !((hl_mode_flags_ & ModeFlags::rainDetected) || (hl_mode_flags_ & ModeFlags::docking))) {
                playSoundAdHoc(tracks[SOUND_TRACK_ADV_RAIN]);                                        // Rain detected, heading back to base
                playSound({.num = (uint16_t)(100 + (rand() % 3)), .type = TrackTypes::background});  // Play background track 100-102 by random
                hl_mode_flags_ |= ModeFlags::docking;
            }
            hl_mode_flags_ |= ModeFlags::rainDetected;
        }
        last_ll_state.status_bitmask = t_ll_state.status_bitmask;

        // ROS running changed
        if (!last_ros_running_ && t_ros_running && !ros_running_since_) {
            ros_running_since_ = millis();
            playSound(tracks[SOUND_TRACK_ADV_ROS_STARTUP_SUCCESS]);  // ROS startup successful
        } else if (last_ros_running_ && !t_ros_running) {
            ros_running_since_ = 0;
            playSound(tracks[SOUND_TRACK_ADV_ROS_STOPPED]);  // ROS stopped
        }
        last_ros_running_ = t_ros_running;

        // HL mode or sub-mode changed
        if (t_hl_state.current_mode != last_hl_state_.current_mode) {
            auto mode = HighLevelState::getMode(t_hl_state.current_mode);
            auto last_mode = HighLevelState::getMode(last_hl_state_.current_mode);
            auto sub_mode = HighLevelState::getSubMode(t_hl_state.current_mode);
            auto last_sub_mode = HighLevelState::getSubMode(last_hl_state_.current_mode);
            switch (HighLevelState::getMode(t_hl_state.current_mode)) {
                case HighLevelState::Mode::Recording:
                    hl_mode_flags_ |= ModeFlags::started;
                    playSound(tracks[SOUND_TRACK_ADV_MAP_RECORD_START]);  // Starting map area recording
                    if (t_hl_state.gps_quality < 75)
                        playSound(tracks[SOUND_TRACK_ADV_RTKGPS_WAIT]);  // Waiting for RTK GPS signal
                    break;

                case HighLevelState::Mode::Autonomous:
                    hl_mode_flags_ |= ModeFlags::started;
                    if (last_mode == HighLevelState::Mode::Idle && sub_mode == HighLevelState::SubModeAutonomous::Undocking) {
                        // IDLE => Autonomous-Undocking
                        playSound(tracks[SOUND_TRACK_ADV_AUTONOMOUS_START]);  // Stay back, autonomous robot mower in use
                        if (t_hl_state.gps_quality < 75)
                            playSound(tracks[SOUND_TRACK_ADV_RTKGPS_WAIT]);  // Waiting for RTK GPS signal
                    } else if (last_sub_mode != HighLevelState::SubModeAutonomous::Docking && sub_mode == HighLevelState::SubModeAutonomous::Docking) {
                        // !Docking => Docking
                        playSound(tracks[SOUND_TRACK_ADV_MOW_DONE_DOCK]);                                    // OM has completed mowing the lawn, heading back to docking station
                        playSound({.num = (uint16_t)(300 + (rand() % 4)), .type = TrackTypes::background});  // Play background track 300 to 203 by random
                    }
                    break;

                default:
                    hl_mode_has_fix_ms_ = 0;
                    hl_mode_flags_ = 0;
                    break;
            }
        }
        last_hl_state_.current_mode = t_hl_state.current_mode;

        // Handle GPS quality related sounds like "ping" or "Stalking Pink Panther"
        handleGpsQuality(t_ll_state, t_ros_running, t_hl_state);

        // Generic, state-change-independent sounds
        if (last_ros_running_)
            playMowSound_();
    }  // dfp_is_5v

    // Process sound queue
    if (active_sounds_.empty())
        return;

    DfMp3_Status status = myMP3.getStatus();
    uint16_t current_track = myMP3.getCurrentTrack();
    DEBUG_PRINTF("DFP-Status %#04x, playing track %d\n", status.state, current_track);

    // Do not interrupt advert sound if it's still playing (don't use busy signal from status_bitmask as it's not reliable set by DFPlayer clones)
    if (!current_playing_is_background_ && (status.state == DfMp3_StatusState_Playing || status.state == DfMp3_StatusState_Shuffling))
        return;

    // Cosmetic pause after advert sound
    if (advert_track_def_.pauseAfter && last_advert_end_ + advert_track_def_.pauseAfter > millis())
        return;

    // Play next in queue
    auto track_def = active_sounds_.front();
    DEBUG_PRINTF("Next (num %d, type %d, flags " PRINTF_BINARY_PATTERN_INT8 ")\n", track_def.num, track_def.type, PRINTF_BYTE_TO_BINARY_INT8(track_def.flags));
    playSoundAdHoc(track_def);
    active_sounds_.pop();
}

namespace  // anonymous (private) namespace
{
/**
 * @brief Handles all "autoplay" related stuff like if the module has autoplay,
 * which type of SD-Card is inserted as well as the case of a possibly already played track (Hi I'm Steve)
 *
 * @return true if it already got handled
 * @return false if not yet handled (because still in detection phase)
 */
bool handleAutoplay_(const ll_status t_ll_state) {
    static unsigned long autoplay_start = 0;  // start ms of auto-played track

    if (dfp_detection_status & DFP_DETECTION_BIT_END)
        return true;

    // Has "autoplay" detection for 600ms
    if (!(dfp_detection_status & DFP_DETECTION_BIT_HAS_AUTOPLAY) && millis() < online_ms_ + DFP_HAS_AUTOPLAY_TIMEOUT) {
        if (t_ll_state.status_bitmask & LL_STATUS_BIT_SOUND_BUSY) {
            dfp_detection_status |= DFP_DETECTION_BIT_HAS_AUTOPLAY;
            autoplay_start = millis();
        } else {
            return false;
        }
    }

    // If there's a snd got auto-started, measure the time for old/new-SD-Card detection
    if (autoplay_start) {
        if ((t_ll_state.status_bitmask & LL_STATUS_BIT_SOUND_BUSY) && (millis() < autoplay_start + DFP_AUTOPLAY_TIMEOUT)) {
            return false;
        } else {  // Auto-played snd stopped, or snd detection time timed out
            dfp_detection_status |= DFP_DETECTION_BIT_END;

            // The old SD-Card has "Hi I'm Steve" as track 1, which is approx. 4600ms,
            // whereas the new one has the long sequence of "Pings", which is approx. 29000ms
            if (millis() - autoplay_start < DFP_AUTOPLAY_TIMEOUT)
                dfp_detection_status |= DFP_DETECTION_BIT_OLD_CARD_STRUCTURE;

            myMP3.setRepeatPlayAllInRoot(false);     // FIXME: This would abort auto-playing title @ DFROBOT LISP3. Lib Error?
            myMP3.setRepeatPlayCurrentTrack(false);  // FIXME: This will stop auto-play of all root files @ DFROBOT LISP3. Lib Error?
            delay(50);                               // (sometimes) required for "DFR LISP3"
        }
    }

    dfp_detection_status |= DFP_DETECTION_BIT_END;
    DEBUG_PRINTF("dfp_detection_status 0b" PRINTF_BINARY_PATTERN_INT8 "\n", PRINTF_BYTE_TO_BINARY_INT8(dfp_detection_status));

    // Play "Hi I'm Steve ..." if ...
    if (!(dfp_detection_status & DFP_DETECTION_BIT_HAS_AUTOPLAY) || !(dfp_detection_status & DFP_DETECTION_BIT_OLD_CARD_STRUCTURE)) {
        playSound(tracks[SOUND_TRACK_ADV_HI_IM_STEVE]);  // Queue it instead of Adhoc, which save us another hacky delay

        // Full sound support if DFP is set to 5V Vcc
        if (dfp_is_5v)
            playSound(tracks[SOUND_TRACK_BGD_OM_BOOT]);
    }

    return true;
}

/**
 * @brief Play a randomized mow- background sound at randomized times
 */
void playMowSound_() {
    static unsigned long last_mow_sound_started_ms = 0;

    if (HighLevelState::getMode(last_hl_state_.current_mode) != HighLevelState::Mode::Autonomous || last_hl_state_.gps_quality < 50)
        return;

    unsigned long now = millis();
    if (now < (hl_mode_has_fix_ms_ + MOW_SOUND_INITIAL_FIX_DELAY) ||
        now < (last_mow_sound_started_ms + MOW_SOUND_MIN_PAUSE_AFTER))
        return;

    // Rand play on MOW_SOUND_CHANCE within next minute
    uint16_t tries_per_minute = 60000 / PROCESS_CYCLETIME;
    int dice = rand() % (tries_per_minute * 60000 / PROCESS_CYCLETIME);
    // DEBUG_PRINTF("tries_per_minute %u, chance %u, rand %i\n", tries_per_minute, MOW_SOUND_CHANCE, dice);
    if (dice > MOW_SOUND_CHANCE)
        return;  // No luck

    // Play sound
    playSound({.num = (uint16_t)(200 + (rand() % 6)), .type = TrackTypes::background});  // Play background track 200 to 205 by random
    last_mow_sound_started_ms = now;
}

/**
 * @brief Notification class required by DFMiniMP3's lib (see https://github.com/Makuna/DFMiniMp3/wiki/Notification-Method)
 *        Also handles background sound repeat.
 */
class Mp3Notify_ {
   public:
    static void OnError([[maybe_unused]] DfMp3 &mp3, uint16_t errorCode) {
        DEBUG_PRINTF("Error: %#06x\n", errorCode);
        last_error_code_ = errorCode;
    }

    static void OnPlayFinished([[maybe_unused]] DfMp3 &mp3, [[maybe_unused]] DfMp3_PlaySources source, uint16_t track) {
        static unsigned long last_finished_cb_call = 0;  // Last DFPlayer OnPlayFinished() callback call (ms). Required for redundant call detection
        static uint16_t last_finished_cb_track = 0;      // Last DFPlayer OnPlayFinished() callback track. Required for redundant call detection
        unsigned long now = millis();
        DEBUG_PRINTF("Finished track %d (now %lu ms)\n", track, now);

        // Redundant CB call protection
        if (track == last_finished_cb_track && now < last_finished_cb_call + DFP_REDUNDANT_ONPLAYFINISH_CB_MAX) {
            DEBUG_PRINTF("Redundant OnPLayFinish() call\n");
            return;
        }
        last_finished_cb_track = track;
        last_finished_cb_call = now;

        // Required for pauseAfter calculation
        if (!current_playing_is_background_) {
            last_advert_end_ = now;
        }

        // Repeat background sound handling
        if (background_track_def_.num && background_track_def_.flags & repeat) {
            DEBUG_PRINTF("Remaining repeat duration %i\n", background_track_def_.repeatDuration);
            if (background_track_def_.repeatDuration > 0) {
                unsigned long play_duration = now - current_playing_started_;
                background_track_def_.repeatDuration -= play_duration;
                playSoundAdHoc(background_track_def_);
            } else {
                delay(50);  // (sometimes) required for "DFR LISP3"
                myMP3.stop();
                background_track_def_ = {};
            }
        }
    }

    static void OnPlaySourceOnline([[maybe_unused]] DfMp3 &mp3, DfMp3_PlaySources source) {
        DEBUG_PRINTF("Play-source online %#04x\n", source);
    }

    static void OnPlaySourceInserted([[maybe_unused]] DfMp3 &mp3, DfMp3_PlaySources source) {
        DEBUG_PRINTF("Play-source inserted %#04x\n", source);
    }

    static void OnPlaySourceRemoved([[maybe_unused]] DfMp3 &mp3, DfMp3_PlaySources source) {
        DEBUG_PRINTF("Play-source removed %#04x\n", source);
    }
};
}  // namespace
}  // namespace soundSystem