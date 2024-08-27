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

#include <Arduino.h>
#include "debug.h"
#include <DFMiniMp3.h>
#include "pins.h"
#include "soundsystem.h"

// Some quirky defines for old- sound-card-format detection
#define DFP_DETECTION_BIT_END (1 << 0)                 // Detection phase ended
#define DFP_DETECTION_BIT_HAS_AUTOPLAY (1 << 1)        // DFPlayer has played a track after reset
#define DFP_DETECTION_BIT_OLD_CARD_STRUCTURE (1 << 2)  // Detected old SD-Card structure
#define DFP_DETECTION_BIT_HANDLED (1 << 3)             // Autoplay existence handled
#define DFP_AUTOPLAY_TIMEOUT 6000                      // Autoplayed track detection timeout. "Hi I'm Steve ..." is about 4.x seconds

namespace soundSystem
{
    namespace // anonymous (private) namespace
    {
        class Mp3Notify;                               // forward declaration
        typedef DFMiniMp3<SerialPIO, Mp3Notify> DfMp3; // ... for a more readable/shorter DfMp3 typedef
                                                       // typedef DFMiniMp3<SerialPIO, Mp3Notify, Mp3ChipMH2024K16SS> DfMp3; // Need to be tested if really required
        SerialPIO soundSerial(PIN_SOUND_TX, PIN_SOUND_RX, 250);
        DfMp3 myMP3(soundSerial);

        std::list<TrackDef> active_sounds_;
        bool sound_available_ = false;    // Sound module available as well as SD-Card with some kind of files
        bool dfp_is_5v = false;           // Enable full sound if DFP is set to 5V Vcc
        uint8_t volume = VOLUME_DEFAULT;  // Last set volume (%)
        std::string language_str = "en";  // Default ISO639-1 language string
        uint8_t play_folder = 1;          // Default play folder, has to be related to language_str

        uint16_t last_error_code_ = 0;  // Last DFPlayer error code. See DfMp3_Error for code meaning

        ll_status last_ll_state = {0};            // Last processed low-level state
        ll_high_level_state last_hl_state_ = {0}; // Last processed high-level state
        unsigned long hl_mode_has_fix_ms_;        // Millis when the current high-level mode got his initial GPS fix. 0 if idle.
        uint8_t hl_mode_flags_;                   // High level mode flags (assumptions), like initial GPS "fix", rain, docking...

        TrackDef background_track_def_ = {0};   // Current/last background track
        TrackDef advert_track_def_ = {0};       // Current/last playing advert track
        bool current_playing_is_background_;    // Current/last playing sound is a background sound
        unsigned long current_playing_started_; // Millis when current/last playing sound got started

        bool ros_running = false;              // ROS running state
        unsigned long ros_running_since_ = 0;  // ros_running since millis state
        unsigned long last_advert_end_;        // Millis when the last played advert sound ended. Used for pauseAfter calculation

        // Describe specific (assumed) mode flags
        enum ModeFlags : uint8_t
        {
            started = 0x01,       // Mode started
            initialGpsFix = 0x02, // Got an initial GPS "fix", by which we can assume that i.e. the mower drive to his mowing start point
            rainDetected = 0x04,  // LL rain sensor signal received
            docking = 0x08,       // Heading back to base, i.e. due to rain detected
        };

        uint8_t dfp_detection_status = 0;             // Sound-card detection status bits i.e. for old- sound-card-format detection
        unsigned long autoplay_detection_start = 0;   // start ms of auto-played track
        unsigned long autoplay_detection_timeout = 0; // When auto-play detection timeout (ms)

        // Forward declarations
        void playMowSound();
        bool handleAutoplay(ll_status t_ll_state);
    }

    bool begin()
    {
        myMP3.begin();

        // Some DFPlayer chips need a reset cmd for a defined 'ready' state, which is more precise than some kind of static (and large) 'delay(3000)'
        DEBUG_PRINTF("Reset...\n");
        myMP3.reset(false); // Non-blocking reset

        unsigned long start_ms = millis();
        while (!myMP3.isOnline()) // loop till module is ready and "play media" is online
        {
            if (last_error_code_ == DfMp3_Error_Busy) // Usually "media not found"
            {
                DEBUG_PRINTF("Busy error. No Media?\n");
                return false;
            }
            delay(50);
            myMP3.loop();
            if (millis() >= (start_ms + DFP_ONLINE_TIMEOUT))
            {
                DEBUG_PRINTF("Reset timed out after %dms\n", millis() - start_ms);
                return false;
            }
        }
        DEBUG_PRINTF("Online after %dms\n", millis() - start_ms);

        // Check max. 600ms if sound-busy get set, which would identify a "DFROBOT LISP3" chip, because only this chip auto-play if IO2 is pinned to GND
        start_ms = millis();
        while (millis() < start_ms + 600)
        {
            // Directly check if Sound-Busy bit is set.
            // We can't use the normal status_message.status_bitmask flag here, because DFPlayer::reset() occur in setup(), so loop1() isn't handled, as well
            // as IMU init follows DFPlayer init, which might take some more seconds, which we don't have to reliable detect a new or old SD-Card
            gpio_put_masked(0b111 << 13, 6 << 13);
            delay(1);
            if (!gpio_get(PIN_MUX_IN)) // DFPlayer is busy
            {
                dfp_detection_status |= DFP_DETECTION_BIT_HAS_AUTOPLAY;
                autoplay_detection_start = millis();
                autoplay_detection_timeout = autoplay_detection_start + DFP_AUTOPLAY_TIMEOUT;
                break;
            }
            delay(20);
        }

        uint16_t total_tracks = myMP3.getTotalTrackCount(DfMp3_PlaySource_Sd);
        DEBUG_PRINTF("getTotalTrackCount() = %d\n", total_tracks);

        uint16_t total_folder = myMP3.getTotalFolderCount();
        DEBUG_PRINTF("getTotalFolderCount() = %d\n", total_folder);

        // For future module evaluation/debugging
        DfMp3_Status status = myMP3.getStatus();
        DEBUG_PRINTF("getStatus().source = %d\n", status.source);

        if (!total_tracks && !total_folder)
        {
            DEBUG_PRINTF("Found no files nor folder on SD-Card!\n");
            return false;
        }
        sound_available_ = true;

        // myMP3.setRepeatPlayAllInRoot(false); // FIXME: This would abort auto-playing title @ DFROBOT LISP3. Lib Error?
        myMP3.setRepeatPlayCurrentTrack(false); // FIXME: This will stop auto-play of all root files @ DFROBOT LISP3. Lib Error?
        delay(50); // (sometimes) required for "DFR LISP3"
        return sound_available_;
    }

    void setDFPis5V(bool t_dfpis5v) {
        dfp_is_5v = t_dfpis5v;
    }

    void setLanguage(iso639_1 *language_p, bool quiet) {  // Set language to the pointing ISO639-1 (2 char) language code and announce if changed && not quiet
        uint8_t last_play_folder = play_folder;

        language_str = *language_p;
        language_str[sizeof(iso639_1)] = 0; // FIXME: Why does the previous command doesn't terminate the string in the right way?!
        if (auto it = language_to_playFolder_map.find(language_str.c_str()); it != language_to_playFolder_map.end()) {
            play_folder = it->second;
        }
        if (!sound_available_ || play_folder == last_play_folder || quiet || !dfp_is_5v)
            return;
            
        playSoundAdHoc(tracks[SOUND_TRACK_ADV_LANGUAGE]);
    }

    void setVolume(uint8_t t_vol)  // Set volume (0-100%)
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

    uint8_t setVolumeUp()
    {
        if (!sound_available_ || volume > (100 - VOLUME_STEPS))
            return volume;

        volume += VOLUME_STEPS;
        setVolume(volume);
        if (dfp_is_5v)
            playSoundAdHoc(tracks[SOUND_TRACK_ADV_UP]);
        return volume;
    }

    uint8_t setVolumeDown()
    {
        if (!sound_available_ || volume < VOLUME_STEPS)
            return volume;

        volume -= VOLUME_STEPS;
        setVolume(volume);
        if (dfp_is_5v)
            playSoundAdHoc(tracks[SOUND_TRACK_ADV_DOWN]);
        return volume;
    }

    void playSoundAdHoc(TrackDef t_track_def)
    {
        DEBUG_PRINTF("playSoundAdHoc(num %d, type %d, flags " PRINTF_BINARY_PATTERN_INT8 ")\n", t_track_def.num, t_track_def.type, PRINTF_BYTE_TO_BINARY_INT8(t_track_def.flags));

        if (!sound_available_ || !(dfp_detection_status & DFP_DETECTION_BIT_HANDLED))
            return;

        switch (t_track_def.type)
        {
        case TrackTypes::background:
            myMP3.stop();
            delay(50); // (sometimes) required for "MH2024K-24SS"
            myMP3.playMp3FolderTrack(t_track_def.num);
            delay(50); // (sometimes) required for "MH2024K-24SS"
            background_track_def_ = t_track_def;
            current_playing_is_background_ = true;
            break;
        case TrackTypes::advert:
            myMP3.stop();
            delay(50); // (sometimes) required for "MH2024K-24SS"
            if (dfp_detection_status & DFP_DETECTION_BIT_OLD_CARD_STRUCTURE) // Old SD-Card detected (or assumed)
            {
                // Simple support for old SD-Card format
                myMP3.playGlobalTrack(t_track_def.num);
            }
            else
            {
                // ATTENTION: Don't use playFolderTrack16() as it does NOT work reliable across DFPlayer clones (as of writing). But playFolderTrack() does.
                myMP3.playFolderTrack(play_folder, t_track_def.num);
            }
            delay(50); // (sometimes) required for "MH2024K-24SS"
            advert_track_def_ = t_track_def;
            current_playing_is_background_ = false;
            break;
        default:
            // advert_raw not yet implemented
            return;
            break;
        }
        current_playing_started_ = millis();

        if (t_track_def.flags & TrackFlags::repeat)
        {
            myMP3.setRepeatPlayCurrentTrack(true); // FIXME: Does NOT work reliable, see manual handling in Mp3Notify::OnPlayFinished()
        }

        if (t_track_def.flags & TrackFlags::stopBackground)
        {
            background_track_def_ = {0};
        }
    }

    void playSound(TrackDef t_track_def)
    {
        DEBUG_PRINTF("playSound(num %d, type %d, flags " PRINTF_BINARY_PATTERN_INT8 ")\n", t_track_def.num, t_track_def.type, PRINTF_BYTE_TO_BINARY_INT8(t_track_def.flags));

        if (!sound_available_ || (active_sounds_.size() == BUFFERSIZE))
            return;

        active_sounds_.push_front(t_track_def);
    }

    /**
     * @brief Handle all kind of emergency sounds
     *
     * @param t_ll_state
     */
    void handleEmergencies(ll_status t_ll_state, bool t_ros_running)
    {
        if ((last_ll_state.emergency_bitmask & LL_EMERGENCY_BIT_LATCH) == (t_ll_state.emergency_bitmask & LL_EMERGENCY_BIT_LATCH))
            return; // Ignore stop button or wheel lift changes if latch didn't changed

        if (t_ll_state.v_charge > 20.0f ||                                  // No emergencies while docked
            !t_ros_running ||                                               // No emergencies as long as ROS does not run
            !ros_running_since_ ||                                          // Pico just started up, while ROS might already running (which happen i.e. after a FW update)
            millis() < (ros_running_since_ + ROS_RUNNING_BEFORE_EMERGENCY)) // No emergency handling before ROS is running that long
        {
            last_ll_state.emergency_bitmask = t_ll_state.emergency_bitmask;
            return;
        }

        if (!(last_ll_state.emergency_bitmask & LL_EMERGENCY_BIT_LATCH) && (t_ll_state.emergency_bitmask & LL_EMERGENCY_BIT_LATCH))
        {
            // Latch changed from 0 -> 1
            if (t_ll_state.emergency_bitmask & LL_EMERGENCY_BITS_STOP)
            {
                playSoundAdHoc(tracks[SOUND_TRACK_ADV_EMERGENCY_STOP]);
                // Do not take attention via "Bida bida" as user pressed the button itself
            }
            else if (t_ll_state.emergency_bitmask & LL_EMERGENCY_BITS_LIFT)
            {
                playSoundAdHoc(tracks[SOUND_TRACK_ADV_EMERGENCY_LIFT]);
                playSound(tracks[SOUND_TRACK_BGD_EMERGENCY_ALARM]);
            }
            else
            {
                playSoundAdHoc(tracks[SOUND_TRACK_ADV_EMERGENCY_ROS]);
                playSound(tracks[SOUND_TRACK_BGD_EMERGENCY_ALARM]);
            }
        }
        else
        {
            // Latch changed from 1 -> 0
            playSoundAdHoc(tracks[SOUND_TRACK_ADV_EMERGENCY_CLEARED]);
        }
        last_ll_state.emergency_bitmask = t_ll_state.emergency_bitmask;
    }

    void processSounds(ll_status t_ll_state, bool t_ros_running, ll_high_level_state t_hl_state)
    {
        if (!sound_available_)
            return;

        myMP3.loop();

        if (!handleAutoplay(t_ll_state)) // Sound-card-format (old/new) detection
            return;

        static unsigned long next_cycle; // Next cycle for sound processing
        if (!(millis() > next_cycle))
        {
            return;
        }
        next_cycle = millis() + PROCESS_CYCLETIME;

        if (dfp_is_5v) { // Full sound support if DFP is set to 5V Vcc
            // Docked ?
            if (t_ll_state.v_charge > 20.0f) {
                if (last_ll_state.v_charge < 10.0f) {
                    myMP3.stop();
                    background_track_def_ = {0};
                    active_sounds_.clear();
                }
            }
            last_ll_state.v_charge = t_ll_state.v_charge;

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
                if (t_hl_state.current_mode == MODE_AUTONOMOUS && !((hl_mode_flags_ & ModeFlags::rainDetected) || (hl_mode_flags_ & ModeFlags::docking))) {
                    playSoundAdHoc(tracks[SOUND_TRACK_ADV_RAIN]);                                      // Rain detected, heading back to base
                    playSound({num : (uint16_t)(100 + (rand() % 3)), type : TrackTypes::background});  // Play background track 100-102 by random
                    hl_mode_flags_ |= ModeFlags::docking;
                }
                hl_mode_flags_ |= ModeFlags::rainDetected;
            }
            last_ll_state.status_bitmask = t_ll_state.status_bitmask;

            // ROS running changed
            if (!ros_running && t_ros_running && !ros_running_since_) {
                ros_running_since_ = millis();
                playSound(tracks[SOUND_TRACK_ADV_ROS_STARTUP_SUCCESS]);  // ROS startup successful
            } else if (ros_running && !t_ros_running) {
                ros_running_since_ = 0;
                playSound(tracks[SOUND_TRACK_ADV_ROS_STOPPED]);  // ROS stopped
            }
            ros_running = t_ros_running;

            // HL mode changed
            if (t_hl_state.current_mode != last_hl_state_.current_mode) {
                switch (t_hl_state.current_mode) {
                    case MODE_RECORDING:
                        hl_mode_flags_ |= ModeFlags::started;
                        playSound(tracks[SOUND_TRACK_ADV_MAP_RECORD_START]);  // Starting map area recording
                        if (t_hl_state.gps_quality < 75)
                            playSound(tracks[SOUND_TRACK_ADV_RTKGPS_WAIT]);  // Waiting for RTK GPS signal
                        break;

                    case MODE_AUTONOMOUS:
                        hl_mode_flags_ |= ModeFlags::started;
                        playSound(tracks[SOUND_TRACK_ADV_AUTONOMOUS_START]);  // Stay back, autonomous robot mower in use
                        if (t_hl_state.gps_quality < 75)
                            playSound(tracks[SOUND_TRACK_ADV_RTKGPS_WAIT]);  // Waiting for RTK GPS signal
                        break;

                    default:
                        hl_mode_has_fix_ms_ = 0;
                        hl_mode_flags_ = 0;
                        break;
                }
            }
            last_hl_state_.current_mode = t_hl_state.current_mode;

            // GPS quality changed
            static unsigned long next_gps_sound_cycle = millis();  // Next cycle when a GPS ping sound got played
            if (t_hl_state.gps_quality != last_hl_state_.gps_quality) {
                switch (t_hl_state.current_mode) {
                    case MODE_RECORDING:
                        if (millis() < next_gps_sound_cycle)
                            break;

                        // Ping only rated GPS quality changes
                        if (t_hl_state.gps_quality < 50) {
                            if (last_hl_state_.gps_quality >= 50)
                                playSound(tracks[SOUND_TRACK_ADV_RTKGPS_POOR]);  // GPS poor ping
                        } else if (t_hl_state.gps_quality < 75) {
                            if (last_hl_state_.gps_quality < 50 || last_hl_state_.gps_quality >= 75)
                                playSound(tracks[SOUND_TRACK_ADV_RTKGPS_MODERATE]);  // GPS moderate/acceptable ping
                        } else                                                       // GPS quality >= 75
                        {
                            if (last_hl_state_.gps_quality < 75)
                                playSound(tracks[SOUND_TRACK_ADV_RTKGPS_GOOD]);  // GPS good ping

                            hl_mode_flags_ |= ModeFlags::initialGpsFix;
                            if (!hl_mode_has_fix_ms_)
                                hl_mode_has_fix_ms_ = millis();
                        }
                        next_gps_sound_cycle = millis() + GPS_SOUND_CYCLETIME;
                        break;

                    case MODE_AUTONOMOUS:
                        if ((hl_mode_flags_ & ModeFlags::initialGpsFix) || (t_hl_state.gps_quality < 75))
                            break;
                        playSound(tracks[SOUND_TRACK_BGD_MUSIC_PINK_PANTHER]);  // Stalking "Pink Panther"
                        hl_mode_flags_ |= ModeFlags::initialGpsFix;
                        if (!hl_mode_has_fix_ms_)
                            hl_mode_has_fix_ms_ = millis();
                        break;

                    default:
                        break;
                }
            }
            last_hl_state_.gps_quality = t_hl_state.gps_quality;

            // Generic, state-change-independent sounds
            if(ros_running)
                playMowSound();
        }  // dfp_is_5v

        // Process sound queue
        int n = active_sounds_.size();
        if (n == 0)
            return;

        DfMp3_Status status = myMP3.getStatus();
        uint16_t current_track = myMP3.getCurrentTrack();
        DEBUG_PRINTF("Status %#04x, track %d\n", status.state, current_track);

        // Do not interrupt advert sound if it's still playing (don't use busy signal from status_bitmask as it's not reliable set my DFPlayer clones)
        if (!current_playing_is_background_ && (status.state == DfMp3_StatusState_Playing || status.state == DfMp3_StatusState_Shuffling))
            return;

        // Cosmetic pause after advert sound
        if (advert_track_def_.pauseAfter && last_advert_end_ + advert_track_def_.pauseAfter > millis())
            return;

        // Play next in queue
        TrackDef track_def = active_sounds_.back();
        DEBUG_PRINTF("Next (num %d, type %d, flags " PRINTF_BINARY_PATTERN_INT8 ")\n", track_def.num, track_def.type, PRINTF_BYTE_TO_BINARY_INT8(track_def.flags));
        playSoundAdHoc(track_def);
        active_sounds_.pop_back();
    }

    namespace // anonymous (private) namespace
    {
        /**
         * @brief Handles the case of a possibly already played track (Hi I'm Steve)
         *
         * @return true if it already got handled
         * @return false if not yet handled (because still in detection phase)
         */
        bool handleAutoplay(ll_status t_ll_state)
        {
            if (dfp_detection_status & DFP_DETECTION_BIT_HANDLED)
                return true;

            if (!(dfp_detection_status & DFP_DETECTION_BIT_END)) // Still in detection phase
            {
                if ((t_ll_state.status_bitmask & LL_STATUS_BIT_SOUND_BUSY) && // DFPlayer (still) busy, and ...
                    (millis() < autoplay_detection_timeout))                  // not timed out yet
                    return false;
                else
                {
                    dfp_detection_status |= DFP_DETECTION_BIT_END;
                    // The old SD-Card has "Hi I'm Steve" as track 1, which is approx. 4600ms,
                    // whereas the new one has the long sequence of "Pings", which is approx. 29000ms
                    if (millis() - autoplay_detection_start < DFP_AUTOPLAY_TIMEOUT)
                        dfp_detection_status |= DFP_DETECTION_BIT_OLD_CARD_STRUCTURE;
                }
            }
            dfp_detection_status |= DFP_DETECTION_BIT_HANDLED;

            // Play "Hi I'm Steve ..." if ...
            if (!(dfp_detection_status & DFP_DETECTION_BIT_HAS_AUTOPLAY) ||      // DFPlayer didn't auto-played, or ...
                !(dfp_detection_status & DFP_DETECTION_BIT_OLD_CARD_STRUCTURE))  // new SD-Card detected (or assumed)
            {
                playSoundAdHoc(tracks[SOUND_TRACK_ADV_HI_IM_STEVE]);
                if (dfp_is_5v)  // Full sound support if DFP is set to 5V Vcc
                    playSound(tracks[SOUND_TRACK_BGD_OM_BOOT]);
            }

            return true;
        }

        /**
         * @brief Play a randomized mow- background sound at randomized times
         */
        void playMowSound() {
            static unsigned long last_mow_sound_started_ms = 0;

            if (last_hl_state_.current_mode != MODE_AUTONOMOUS || last_hl_state_.gps_quality < 50)
                return;

            unsigned long now = millis();
            if (now < (hl_mode_has_fix_ms_ + MOW_SOUND_INITIAL_FIX_DELAY))
                return;
            if (now < (last_mow_sound_started_ms + MOW_SOUND_MIN_PAUSE_AFTER))
                return;

            // Rand play on MOW_SOUND_CHANCE within next minute
            uint16_t tries_per_minute = 60000 / PROCESS_CYCLETIME;
            int dice = rand() % (tries_per_minute * 60000 / PROCESS_CYCLETIME);
            // DEBUG_PRINTF("tries_per_minute %u, chance %u, rand %i\n", tries_per_minute, MOW_SOUND_CHANCE, dice);
            if (dice > MOW_SOUND_CHANCE)
                return;  // No luck

            // Play sound
            playSound({num : (uint16_t)(200 + (rand() % 7)), type : TrackTypes::background});  // Play background track 200 to 206 by random
            last_mow_sound_started_ms = now;
        }

        /**
         * @brief Notification class required by DFMiniMP3's lib (see https://github.com/Makuna/DFMiniMp3/wiki/Notification-Method)
         *        Also handles background sound repeat.
         */
        class Mp3Notify
        {
        public:
            static void OnError([[maybe_unused]] DfMp3 &mp3, uint16_t errorCode)
            {
                DEBUG_PRINTF("Error: %#06x\n", errorCode);
                last_error_code_ = errorCode;
            }

            static void OnPlayFinished([[maybe_unused]] DfMp3 &mp3, [[maybe_unused]] DfMp3_PlaySources source, uint16_t track)
            {
                static unsigned long last_finished_cb_call = 0; // Last DFPlayer OnPlayFinished() callback call (ms). Required for redundant call detection
                static uint16_t last_finished_cb_track = 0;     // Last DFPlayer OnPlayFinished() callback track. Required for redundant call detection
                unsigned long now = millis();
                DEBUG_PRINTF("Finished track %d (now %lu ms)\n", track, now);

                // Redundant CB call protection
                if (track == last_finished_cb_track && now < last_finished_cb_call + DFP_REDUNDANT_ONPLAYFINISH_CB_MAX)
                {
                    DEBUG_PRINTF("Redundant OnPLayFinish() call\n");
                    return;
                }
                last_finished_cb_track = track;
                last_finished_cb_call = now;

                // Required for pauseAfter calculation
                if (!current_playing_is_background_)
                {
                    last_advert_end_ = now;
                }

                // Repeat background sound handling
                if (background_track_def_.num && background_track_def_.flags & repeat)
                {
                    DEBUG_PRINTF("Remaining repeat duration %i\n", background_track_def_.repeatDuration);
                    if (background_track_def_.repeatDuration > 0)
                    {
                        unsigned long play_duration = now - current_playing_started_;
                        background_track_def_.repeatDuration -= play_duration;
                        playSoundAdHoc(background_track_def_);
                    }
                    else
                    {
                        delay(50); // (sometimes) required for "DFR LISP3"
                        myMP3.stop();
                        background_track_def_ = {0};
                    }
                }
            }

            static void OnPlaySourceOnline([[maybe_unused]] DfMp3 &mp3, DfMp3_PlaySources source)
            {
                DEBUG_PRINTF("Play-source online %#04x\n", source);
            }

            static void OnPlaySourceInserted([[maybe_unused]] DfMp3 &mp3, DfMp3_PlaySources source)
            {
                DEBUG_PRINTF("Play-source inserted %#04x\n", source);
            }

            static void OnPlaySourceRemoved([[maybe_unused]] DfMp3 &mp3, DfMp3_PlaySources source)
            {
                DEBUG_PRINTF("Play-source removed %#04x\n", source);
            }
        };
    }
}