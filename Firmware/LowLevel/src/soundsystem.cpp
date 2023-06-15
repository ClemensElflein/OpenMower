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

/*                          | Original      |        Old clones          |        Newer clones    | Unknown
 * DFPlayer Chip            | DFROBOT LISP3 | YX5200-24SS | MH2024K-24SS | MH2024K-16SS | GD3200B | YX6200-16S
 * -------------------------------------------------------------------------------------------------------
 * Tested                   |      ok       |             |     ok       |              |   ok    |
 * Autoplay files *1        |      yes      |             |     no       |              |   no    |
 * Require reset()          |               |             |              |              |   YES   |
 * Reset->isOnline *2       |   800-850ms   |             |    750ms     |              |  600ms  |
 * Without SD Card          |   error 0x1   |             |              |              |         |
 * getTotalTrackCount()     |       0       |             |     24       |              |   24    |
 * getTotalFolderCount()    |       5       |             |      0       |              |    3    |
 * getStatus().source       |       2       |             |      0       |              |    0    |
 * playAdvertisement()      |     yes*4     |             |   yes*4      |              |   yes   |
 * setRepeatPlayAllInRoot() |   Plays ALL   |             |     yes      |              |         |
 * getCurrentTrack()        |      *3       |             |     *4       |              |    *3   |
 * OnPlayFinished()         | Play&Advert*5 |             |   *5,*6      |              | Play&Adv|
 * -------------------------------------------------------------------------------------------------------
 * *1 = Autoplay (unasked) all (at least) root files.
 * *2 = Highly depends on SD-Card and content
 * *3 = Advert track get returned with an internal track number
 * *4 = Advert get only played with a current running play sound
 * *5 = Sometimes called a second time within 10-150ms
 * *6 = Two callbacks (advert & play) at the end of play sound
 *
 * Founded by this evaluation:
 * DFPlayer's "advert" functionality is not usable for our requirements, as MH2024K-24S don't return a usable advert state.
 * Nevertheless, DFPlayer's "advert" functionality might be used for unimportant messages like "GPS lost".
 */

#include <Arduino.h>
#include "debug.h"
#include "pins.h"
#include "soundsystem.h"

SerialPIO soundSerial(PIN_SOUND_TX, PIN_SOUND_RX, 250);
DfMp3 myMP3(soundSerial);

bool MP3Sound::begin()
{
    myMP3.begin();

    // Some DFPlayer chips need a reset cmd for a defined 'ready' state, which is more precise than some kind of static (and large) 'delay(3000)'
    DEBUG_PRINTF("Reset DFPlayer...\n");
    myMP3.reset(false);

    uint32_t start_ms = millis();
    while (!myMP3.isOnline()) // loop till module is ready and play media is online
    {
        if (last_error_code_ == DfMp3_Error_Busy) // Usually "media not found"
        {
            DEBUG_PRINTF("DFPlayer 'busy' error. No Media?\n");
            return false;
        }
        delay(50);
        myMP3.loop();
        if (millis() >= (start_ms + DFP_ONLINE_TIMEOUT))
        {
            DEBUG_PRINTF("DFPlayer reset timed out after %dms\n", millis() - start_ms);
            return false;
        }
    }
    DEBUG_PRINTF("DFPlayer online after %dms\n", millis() - start_ms);

    uint16_t total_tracks = myMP3.getTotalTrackCount(DfMp3_PlaySource_Sd);
    DEBUG_PRINTF("getTotalTrackCount() = %d\n", total_tracks);

    uint16_t total_folder = myMP3.getTotalFolderCount();
    DEBUG_PRINTF("getTotalFolderCount() = %d\n", total_folder);

    // For future module evaluation/debugging
    DfMp3_Status status = myMP3.getStatus();
    DEBUG_PRINTF("getStatus().source = %d\n", status.source);

    if (!total_tracks && !total_folder)
    {
        DEBUG_PRINTF("DFPlayer found no files nor folder on SD-Card!\n");
        return false;
    }
    this->sound_available_ = true;

    myMP3.setRepeatPlayCurrentTrack(false);
    myMP3.setRepeatPlayAllInRoot(false);
    playSoundAdHoc({num : 2, type : TrackTypes::background, flags : TrackFlags::repeat}); // Heartbeat "success" sound
    return this->sound_available_;
}

void MP3Sound::setVolume(uint8_t t_vol) // scales from 0 to 100 %
{
    // value of 30 is max equivalent to 100 %
    uint8_t val = (uint8_t)(30.0 / 100.0 * (double)t_vol);
    DEBUG_PRINTF("Set volume %d\n", val);
    myMP3.setVolume(val);
    delay(50); // (sometimes) required for "DFR LISP3"
}

void MP3Sound::playSoundAdHoc(TrackDef t_track_def)
{
    DEBUG_PRINTF("playSoundAdHoc(num %d, type %d, flags " PRINTF_BINARY_PATTERN_INT8 ")\n", t_track_def.num, t_track_def.type, PRINTF_BYTE_TO_BINARY_INT8(t_track_def.flags));

    if (!this->sound_available_)
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
        myMP3.playFolderTrack16(DFP_ADVERT_FOLDER, t_track_def.num);
        delay(50); // (sometimes) required for "MH2024K-24SS"
        advert_track_def_ = t_track_def;
        current_playing_is_background_ = false;
        break;

    default:
        // advert_raw not yet implemented
        return;
        break;
    }

    if (t_track_def.flags & TrackFlags::repeat)
    {
        myMP3.setRepeatPlayCurrentTrack(true);
    }

    if (t_track_def.flags & TrackFlags::stopBackground)
    {
        background_track_def_ = {0};
    }
}

void MP3Sound::playSound(TrackDef t_track_def)
{
    DEBUG_PRINTF("playSound(num %d, type %d, flags " PRINTF_BINARY_PATTERN_INT8 ")\n", t_track_def.num, t_track_def.type, PRINTF_BYTE_TO_BINARY_INT8(t_track_def.flags));

    if (!this->sound_available_ || (active_sounds_.size() == BUFFERSIZE))
        return;

    active_sounds_.push_front(t_track_def);
}

void MP3Sound::processSounds(ll_status t_ll_state, bool t_ros_running, ll_high_level_state t_hl_state)
{
    if (!this->sound_available_)
        return;

    myMP3.loop();

    if (millis() < next_process_cycle_)
    {
        return;
    }
    next_process_cycle_ = millis() + PROCESS_CYCLETIME;

    const uint8_t changed_status = t_ll_state.status_bitmask ^ last_ll_state_.status_bitmask;
    DEBUG_PRINTF("Changed status_bitmask " PRINTF_BINARY_PATTERN_INT8 " ", PRINTF_BYTE_TO_BINARY_INT8(changed_status));
    DEBUG_PRINTF("(new status " PRINTF_BINARY_PATTERN_INT8 " ", PRINTF_BYTE_TO_BINARY_INT8(t_ll_state.status_bitmask));
    DEBUG_PRINTF("XOR last status " PRINTF_BINARY_PATTERN_INT8 "), ", PRINTF_BYTE_TO_BINARY_INT8(last_ll_state_.status_bitmask));
    DEBUG_PRINTF("HL mode %d\n", t_hl_state.current_mode);

    // LL status_bitmask changed
    if (changed_status & StatusBitmask_initialized)
    {
        playSound({num : 2, type : TrackTypes::advert, pauseAfter : 1500}); // OM startup successful
    }
    if (changed_status & StatusBitmask_raspi_power)
    {
        playSound({num : 3, type : TrackTypes::advert, flags : TrackFlags::stopBackground}); // Initializing ROS
        // We're in a new "Raspi/ROS" bootup phase, which might take longer. Change background sound for better identification
        playSound({num : 5, type : TrackTypes::background, flags : TrackFlags::repeat});
    }
    last_ll_state_.status_bitmask = t_ll_state.status_bitmask;

    // ROS running changed
    if (t_ros_running && !last_ros_running_)
    {
        playSound({num : 16, type : TrackTypes::advert, flags : TrackFlags::stopBackground}); // ROS startup successful
    }
    last_ros_running_ = t_ros_running;

    // HL mode changed
    if (t_hl_state.current_mode != last_hl_state_.current_mode)
    {
        switch (t_hl_state.current_mode)
        {
        case MODE_RECORDING:
            hl_mode_started_ = millis();
            playSound({num : 4, type : TrackTypes::advert, flags : TrackFlags::stopBackground}); // Starting map area recording
            playSound({num : 5, type : TrackTypes::advert, pauseAfter : 1500});                  // Waiting for RTK GPS signal
            break;

        case MODE_AUTONOMOUS:
            hl_mode_started_ = millis();
            playSound({num : 12, type : TrackTypes::advert, flags : TrackFlags::stopBackground, pauseAfter : 1500}); // Stay back, autonomous robot mower in use
            playSound({num : 5, type : TrackTypes::advert, pauseAfter : 1500});                                      // Waiting for RTK GPS signal
            break;

        default:
            hl_mode_started_ = 0;
            break;
        }
    }
    last_hl_state_.current_mode = t_hl_state.current_mode;

    // GPS quality ping during area recording
    if (t_hl_state.gps_quality != last_hl_state_.gps_quality && t_hl_state.current_mode == MODE_RECORDING)
    {
        if (millis() >= next_gps_sound_cycle_)
        {
            if (t_hl_state.gps_quality < 50)
            {
                playSound({num : 20, type : TrackTypes::background}); // GPS poor ping
            }
            else if (t_hl_state.gps_quality < 75)
            {
                playSound({num : 21, type : TrackTypes::background}); // GPS acceptable ping
            }
            else
            {
                playSound({num : 22, type : TrackTypes::background}); // GPS good ping
            }
        }
        else
        {
            next_gps_sound_cycle_ = millis() + GPS_SOUND_CYCLETIME;
        }
    }
    last_hl_state_.gps_quality = t_hl_state.gps_quality;

    // Process sound queue
    int n = active_sounds_.size();
    if (n == 0)
        return;

    DfMp3_Status status = myMP3.getStatus();
    uint16_t current_track = myMP3.getCurrentTrack();
    // DEBUG_PRINTF("Status %#04x, track %d\n", status.state, current_track);

    // Do not interrupt advert sound if still playing
    if (!current_playing_is_background_ &&
        (status.state == DfMp3_StatusState_Playing || status.state == DfMp3_StatusState_Shuffling))
        return;

    // Cosmetic pause after advert sound
    if (advert_track_def_.pauseAfter && last_advert_end_ + advert_track_def_.pauseAfter > millis())
    {
        return;
    }

    // Play next in queue
    TrackDef track_def = active_sounds_.back();
    DEBUG_PRINTF("Next (num %d, type %d, flags " PRINTF_BINARY_PATTERN_INT8 ")\n", track_def.num, track_def.type, PRINTF_BYTE_TO_BINARY_INT8(track_def.flags));
    playSoundAdHoc(track_def);
    active_sounds_.pop_back();
}

void MP3Sound::OnError(DfMp3 &mp3, uint16_t errorCode)
{
    DEBUG_PRINTF("DFPlayer error: %#06x\n", errorCode);

    MP3Sound *snd = MP3Sound::GetInstance();
    snd->last_error_code_ = errorCode;
}

void MP3Sound::OnPlayFinished(DfMp3 &mp3, DfMp3_PlaySources source, uint16_t track)
{
    DEBUG_PRINTF("DFPlayer finished track %d (%lu ms)\n", track, millis());
    MP3Sound *snd = MP3Sound::GetInstance();

    // Redundant CB call?
    if (track == snd->last_finished_cb_track_ && millis() < snd->last_finished_cb_call_ + DFP_REDUNDANT_ONPLAYFINISH_CB_MAX)
    {
        DEBUG_PRINTF("DFPlayer redundant OnPLayFinish() call\n");
        return;
    }
    snd->last_finished_cb_track_ = track;
    snd->last_finished_cb_call_ = millis();

    if (!snd->current_playing_is_background_)
    {
        snd->last_advert_end_ = millis();
    }
    if (snd->background_track_def_.num && snd->background_track_def_.flags & TrackFlags::repeat)
    {
        snd->playSoundAdHoc(snd->background_track_def_);
    }
}

void MP3Sound::OnPlaySourceOnline(DfMp3 &mp3, DfMp3_PlaySources source)
{
    DEBUG_PRINTF("DFPlayer source online %#04x\n", source);
}

void MP3Sound::OnPlaySourceInserted(DfMp3 &mp3, DfMp3_PlaySources source)
{
    DEBUG_PRINTF("DFPlayer source inserted %#04x\n", source);
}

void MP3Sound::OnPlaySourceRemoved(DfMp3 &mp3, DfMp3_PlaySources source)
{
    DEBUG_PRINTF("DFPlayer source removed %#04x\n", source);
}

MP3Sound *MP3Sound::GetInstance()
{
    static MP3Sound *instance_;

    /**
     * This is a safer way to create an instance. instance = new Singleton is
     * dangerous in case two instance threads wants to access at the same time
     */
    if (instance_ == nullptr)
    {
        instance_ = new MP3Sound();
    }
    return instance_;
}