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

#define DFP_ADVERT_FOLDER 1U
#define DFP_ONLINE_TIMEOUT 5000
#define BUFFERSIZE 100
#define PROCESS_CYCLETIME 500
#define GPS_SOUND_CYCLETIME 3000

class MP3Sound;                               // forward declaration ...
typedef DFMiniMp3<SerialPIO, MP3Sound> DfMp3; // ... for a more readable/shorter DfMp3 typedef
// typedef DFMiniMp3<SerialPIO, MP3Sound, Mp3ChipMH2024K16SS> DfMp3; // Need to be tested

// Non thread safe singleton MP3Sound class
class MP3Sound
{
protected:
    MP3Sound(){}; // Singleton constructors always should be private to prevent direct construction via 'new'

public:
    enum TrackTypes : uint8_t
    {
        background = 1, // Background tracks are stored in folder mp3 and might be interrupted/aborted by higher priority sounds
        advert,         // Advert tracks are stored in language specific folder, i.e. "01" US or "49" German, and interrupt/stop background sounds
        advertRaw,      // Raw-Advert tracks are stored in folder advert and interrupt/stop background or advert sounds.
                        // Due to DFPlayer incompatibilities, advert_raw should only be used if you know their drawbacks!
    };
    enum TrackFlags : uint8_t
    {
        repeat = 0x01,         // Repeat this track. This flag is limited to background sounds!
        stopBackground = 0x02, // Stop replaying of a current running background track after this sound got played
    };
    struct TrackDef
    {
        uint16_t num;
        TrackTypes type;
        uint8_t flags = 0;            // See TrackFlags
        unsigned long pauseAfter = 0; // Cosmetic pause in ms, after advert track got played, before the next sound get processed from queue.
    };

    bool playing;

    static MP3Sound *GetInstance();
    MP3Sound(MP3Sound &other) = delete;        // Singletons should not be cloneable
    void operator=(const MP3Sound &) = delete; // Singletons should not be assignable

    bool begin();                                                             // Init serial stream, soundmodule and sound_available_
    void playSound(TrackDef t_track_def);                                     // Play sound track number. This method writes sound track nr in a list, the method processSounds() (has to run in loop)
                                                                              // will play the sounds according to the list
    void playSoundAdHoc(TrackDef t_track_def);                                // Play sound track number immediately without waiting until the end of sound
    void setVolume(uint8_t t_vol);                                            // Scales loudness from 0 to 100 %
    void processSounds(ll_status t_ll_state, ll_high_level_state t_hl_state); // This method has to be called cyclic, e.g. every second.

    // DFMiniMP3 specific notification methods
    static void OnError(DfMp3 &mp3, uint16_t errorCode);
    static void OnPlayFinished(DfMp3 &mp3, DfMp3_PlaySources source, uint16_t track);
    static void OnPlaySourceOnline(DfMp3 &mp3, DfMp3_PlaySources source);
    static void OnPlaySourceInserted(DfMp3 &mp3, DfMp3_PlaySources source);
    static void OnPlaySourceRemoved(DfMp3 &mp3, DfMp3_PlaySources source);

private:
    std::list<TrackDef> active_sounds_;
    bool sound_available_ = false;                  // Sound module available as well as SD-Card with some kind of files
    uint16_t last_error_code_ = 0;                  // Last DFPlayer error code
    ll_status last_ll_state_ = {0};                 // Last processed low-level state
    ll_high_level_state last_hl_state_ = {0};       // Last processed high-level state
    TrackDef background_track_def_ = {0};           // Current/last background track
    TrackDef advert_track_def_ = {0};               // Current/last playing advert track
    bool current_playing_is_background_;            // Current/last playing sound is a background sound
    unsigned long hl_mode_started_;                 // Millis when the last high-level mode started. 0 if idle.
    unsigned long next_process_cycle_ = millis();   // Next cycle for sound processing
    unsigned long next_gps_sound_cycle_ = millis(); // Next cycle when a GPS ping sound got played
    unsigned long last_advert_end_;                 // Millis when the last played advert sound ended. Used for pauseAfter calculation
};

#endif // _SOUND_SYSTEM_H_  HEADER_FILE
