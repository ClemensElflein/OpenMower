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
 * playAdvertisement()      |      no       |             |     yes      |              |   yes   |
 * setRepeatPlayAllInRoot() |   Plays ALL   |             |     yes      |              |         |
 * -------------------------------------------------------------------------------------------------------
 * *1 = Autoplay (unasked) all (at least) root files.
 * *2 = Highly depends on SD-Card and content
 */

#include <Arduino.h>
#include "debug.h"
#include "pins.h"
#include "soundsystem.h"

SerialPIO soundSerial(PIN_SOUND_TX, PIN_SOUND_RX, 250);
DfMp3 myMP3(soundSerial);

MP3Sound::MP3Sound()
{
    this->playing = false;
    this->sound_available_ = false;
}

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
    myMP3.playGlobalTrack(1); // DFROBOT LISP3 only advert sounds if there's a current running background sound
    return this->sound_available_;
}

void MP3Sound::setVolume(uint8_t t_vol) // scales from 0 to 100 %
{
    // value of 30 is max equivalent to 100 %
    uint8_t val = (uint8_t)(30.0 / 100.0 * (double)t_vol);
    DEBUG_PRINTF("Set volume %d\n", val);
    myMP3.setVolume(val);
}

void MP3Sound::playSoundAdHoc(uint16_t t_track)
{
    if (!this->sound_available_)
        return;

    myMP3.playAdvertisement(t_track);
}

void MP3Sound::playSound(int soundNr)
{
    if (!this->sound_available_ || (active_sounds_.size() == BUFFERSIZE))
        return;

    active_sounds_.push_front(soundNr);
}

int MP3Sound::sounds2play()
{
    return active_sounds_.size();
}

int MP3Sound::processSounds()
{
    int n = active_sounds_.size();
    if (n == 0)
        return (n);
    //    if (myMP3.isPlaying())
    //        return (n);

    int file2play = active_sounds_.back();
    //    myMP3.play(file2play);
    active_sounds_.pop_back();

    return active_sounds_.size();
}

void MP3Sound::OnError(DfMp3 &mp3, uint16_t errorCode)
{
    DEBUG_PRINTF("DFPlayer error: %#06x\n", errorCode);

    MP3Sound *snd = MP3Sound::GetInstance();
    snd->last_error_code_ = errorCode;
}

void MP3Sound::OnPlayFinished(DfMp3 &mp3, DfMp3_PlaySources source, uint16_t track)
{
#ifdef USB_DEBUG
    DEBUG_SERIAL.print("DFPlayer finished track ");
    DEBUG_SERIAL.println(track);
#endif
}

void MP3Sound::OnPlaySourceOnline(DfMp3 &mp3, DfMp3_PlaySources source)
{
#ifdef USB_DEBUG
    DEBUG_SERIAL.print("DFPlayer source online ");
    DEBUG_SERIAL.println(source);
#endif
}

void MP3Sound::OnPlaySourceInserted(DfMp3 &mp3, DfMp3_PlaySources source)
{
#ifdef USB_DEBUG
    DEBUG_SERIAL.print("DFPlayer source inserted ");
    DEBUG_SERIAL.println(source);
#endif
}

void MP3Sound::OnPlaySourceRemoved(DfMp3 &mp3, DfMp3_PlaySources source)
{
#ifdef USB_DEBUG
    DEBUG_SERIAL.print("DFPlayer source removed ");
    DEBUG_SERIAL.println(source);
#endif
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