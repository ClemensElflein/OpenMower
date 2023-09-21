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
#include <DFMiniMp3.h>

#include "datatypes.h"

#define DFP_ONLINE_TIMEOUT 5000
#define DFP_REDUNDANT_ONPLAYFINISH_CB_MAX 300 // Max. ms to detect a recurring OnPlayFinish() CB as redundant

#ifdef DEBUG_PREFIX
#undef DEBUG_PREFIX
#define DEBUG_PREFIX "[DFP] "
#endif

#define BUFFERSIZE 100
#define PROCESS_CYCLETIME 500

#define GPS_SOUND_CYCLETIME 3000
#define MOW_SOUND_INITIAL_FIX_DELAY 120000 // Play random mow sounds earliest 3 minutes after first fix (max. expected time to navigate to mowing area)
#define MOW_SOUND_MIN_PAUSE_AFTER 60000    // Minimum pause before a new randomized mow sounds get played
#define MOW_SOUND_CHANCE 50                // % change to play a new sound within the next minute after MOW_SOUND_MIN_PAUSE_AFTER
#define ROS_RUNNING_BEFORE_EMERGENCY 10000 // Min. millis of running ROS before emergencies get handled
#define VOLUME_DEFAULT 100
#define VOLUME_STEPS 5                     // Amount of volume setps for volumeUp() and volumeDown()

// For better reading, let's use track names which point to tracks[] indexes
#define SOUND_TRACK_BGD_OM_BOOT 0 // Heartbeat during OM LowLevel startup
#define SOUND_TRACK_ADV_HI_IM_STEVE 1
#define SOUND_TRACK_ADV_IMU_INIT_FAILED 2
#define SOUND_TRACK_BGD_OM_ALARM 3
#define SOUND_TRACK_ADV_EMERGENCY_STOP 4
#define SOUND_TRACK_ADV_EMERGENCY_LIFT 5
#define SOUND_TRACK_ADV_EMERGENCY_ROS 19
#define SOUND_TRACK_ADV_EMERGENCY_CLEARED 20
#define SOUND_TRACK_BGD_EMERGENCY_ALARM 6
#define SOUND_TRACK_ADV_OM_STARTUP_SUCCESS 7
#define SOUND_TRACK_ADV_ROS_INIT 8
#define SOUND_TRACK_BGD_ROS_BOOT 9 // Heartbeat during ROS HighLevel startup
#define SOUND_TRACK_ADV_ROS_STARTUP_SUCCESS 10
#define SOUND_TRACK_ADV_MAP_RECORD_START 11
#define SOUND_TRACK_ADV_AUTONOMOUS_START 12
#define SOUND_TRACK_ADV_RAIN 13
#define SOUND_TRACK_ADV_RTKGPS_WAIT 14
#define SOUND_TRACK_ADV_RTKGPS_POOR 15
#define SOUND_TRACK_ADV_RTKGPS_MODERATE 16
#define SOUND_TRACK_ADV_RTKGPS_GOOD 17
#define SOUND_TRACK_BGD_MUSIC_PINK_PANTHER 18
#define SOUND_TRACK_ADV_UP 21
#define SOUND_TRACK_ADV_DOWN 22
#define SOUND_TRACK_ADV_LANGUAGE 23

namespace soundSystem
{
    enum TrackTypes : uint8_t
    {
        background = 1, // Background tracks are stored in folder mp3 and get interrupted/aborted by higher priority sound like advert
        advert,         // Advert tracks are stored in language specific folder, i.e. "01" US or "49" German, and interrupt/stop background sounds
        advertRaw,      // Raw-Advert tracks are stored in folder 'advert' and interrupt/stop background or advert sounds.
                        // Due to DFPlayer incompatibilities, advert_raw should only be used if you know their drawbacks!
    };
    enum TrackFlags : uint8_t
    {
        repeat = 0x01,         // Repeat this track. This flag is limited to background sounds!
        stopBackground = 0x02, // Stop replaying of a current running background track after this sound got played
    };
    struct TrackDef
    {
        uint16_t num; // Source (SD-Card) track number
        TrackTypes type;
        uint8_t flags = 0;               // See TrackFlags
        unsigned long pauseAfter = 0;    // Cosmetic pause in ms, after advert track got played, before the next sound get processed from queue.
        int32_t repeatDuration = 180000; // How long (ms) to repeat a background sound. Default to 180 sec. noise pollution (i.e. VdS 2300)
    };
    const uint8_t languages[] = {1, 49}; // Available advert language sounds. 1 = English(US), 49 = German

    // For easier reading and simpler code, let's have a list of predefined tracks and its (default) settings
    const TrackDef tracks[] = {
        {2, background, repeat},                          // 0 = OM boot-up background
        {1, advert, pauseAfter : 1500},                   // 1 = Hi I'm steve
        {19, advert, stopBackground, pauseAfter : 500},   // 2 = IMU initialization failed
        {15, background, repeat, repeatDuration : 20000}, // 3 = Alarm02
        {8, advert, pauseAfter : 500},                    // 4 = Stop button triggered
        {9, advert, pauseAfter : 500},                    // 5 = Emergency wheel lift sensor triggered
        {9, background},                                  // 6 = "Bee daa, Bee daa" Minion fire alarm
        {2, advert, pauseAfter : 1500},                   // 7 = OM startup successful
        {3, advert, stopBackground},                      // 8 = Initializing ROS
        {5, background, repeat},                          // 9 = ROS boot-up background
        {16, advert, stopBackground},                     // 10 = ROS startup successful
        {4, advert, stopBackground},                      // 11 = Starting map area recording
        {12, advert, stopBackground, pauseAfter : 1500},  // 12 = Stay back, autonomous robot mower in use
        {10, advert, stopBackground},                     // 13 = Rain detected, heading back to base
        {5, advert, pauseAfter : 1500},                   // 14 = Waiting for RTK GPS signal
        {20, background},                                 // 15 = GPS poor ping
        {21, background},                                 // 16 = GPS moderate/acceptable ping
        {22, background},                                 // 17 = GPS good ping
        {12, background},                                 // 18 = Stalking "Pink Panther"
        {24, advert, pauseAfter : 500},                   // 19 = Emergency triggered by ROS
        {23, advert, stopBackground, pauseAfter : 500},   // 20 = Emergency cleared
        {21, advert, pauseAfter : 100},                   // 21 = Volume "up"
        {20, advert, pauseAfter : 100},                   // 22 = Volume "down"
        {22, advert, pauseAfter : 100},                   // 23 = Switched to (Englisch) language
    };

    bool begin(); // Init serial stream, soundmodule and sound_available_

    void playSound(TrackDef t_track_def);      // Play sound trackDef. This method writes sound trackDef in a list, the method processSounds() (has to run in loop)
                                               // will play the sounds according to the list
    void playSoundAdHoc(TrackDef t_track_def); // Play sound track number immediately without waiting until the end of sound

    void setNextLanguage();        // Select next (available) language
    void setVolume(uint8_t t_vol); // Scales loudness from 0 to 100 %
    void setVolumeUp();            // Scale volume up by VOLUME_STEPS
    void setVolumeDown();          // Scale volume down by VOLUME_STEPS

    void processSounds(ll_status t_ll_state, bool t_ros_running, ll_high_level_state t_hl_state); // This method has to be called cyclic, e.g. every second.
}
#endif // _SOUND_SYSTEM_H_  HEADER_FILE
