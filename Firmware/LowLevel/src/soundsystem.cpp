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

/*                      | Original      |        Old clones          |        Newer clones    | Unknown
 * DFPlayer Chip        | DFROBOT LISP3 | YX5200-24SS | MH2024K-24SS | MH2024K-16SS | GD3200B | YX6200-16S
 * -------------------------------------------------------------------------------------------------------
 * Tested               |      ok       |             |     ok       |              |   ok    |
 * Throws initial 0x3   |      no       |             |    120ms     |              |   yes   |
 * Autoplay files *1    |      yes      |             |              |              |   no    |
 * Reset->isOnline *2   |     902ms     |             |    901ms     |              |  602ms  |
 * playGlobalTrack()    |      yes      |             |     NO       |              |   yes   |
 * -------------------------------------------------------------------------------------------------------
 * *1 = Autoplays (unasked) all (at least) root files = Output spam
 * *2 = Highly depends on SD-Card and content
 *
 */

#include <Arduino.h>
#include "debug.h"
#include "pins.h"
#include "soundsystem.h"

SerialPIO soundSerial(PIN_SOUND_TX, PIN_SOUND_RX, 250);

DfMp3 myMP3(soundSerial);

MP3Sound *MP3Sound::mp3sound_ = nullptr;

MP3Sound::MP3Sound()
{
    this->anzSoundfiles = 0; // number of files stored on the SD-card
    this->playing = false;
    this->sound_available = false;
}

bool MP3Sound::begin()
{
    myMP3.begin();

    // DFPlayer clone chips seem to send an 0x03 (comm error) message once ready to receive commands.
    // Let's use it here to identify when they're ready to receive commands.
    // For sure this is extra penalty of approx. 200ms for the original chip.
    // FIXME: Should we make it optional i.e. with a '#define DFPLAYER_CLONE' switch?
    uint32_t start_ms = millis();
    while (!received_clone_init_error && millis() < start_ms + DFP_CLONE_ERROR_TIMEOUT)
    {
        delay(20);
        myMP3.loop();
    }
    DEBUG_PRINT("DFPlayer-Clone init error delay (");
    DEBUG_PRINT_VAL(millis() - start_ms, DEC);
    DEBUG_PRINTLN("ms)");

    // Some DFPlayer chips better get a reset cmd for a defined 'ready' state, which is more precise than some kind of static (and large) 'delay(3000)'
    DEBUG_PRINTLN("Reset DFPlayer...");
    myMP3.reset();
    start_ms = millis();
    while (!myMP3.isOnline())
    {
        delay(50);
        myMP3.loop();
        if (millis() >= start_ms + DFP_ONLINE_TIMEOUT)
        {
            DEBUG_PRINTLN("DFPlayer timed out!");
            return false;
        }
    }
    DEBUG_PRINT("DFPlayer online (");
    DEBUG_PRINT_VAL(millis() - start_ms, DEC);
    DEBUG_PRINTLN("ms)");

    this->anzSoundfiles = myMP3.getTotalTrackCount(DfMp3_PlaySource_Sd);
    DEBUG_PRINT_VAL(this->anzSoundfiles, DEC);
    DEBUG_PRINTLN(" sound files");
    return this->anzSoundfiles > 0;
}

void MP3Sound::setVolume(uint8_t vol) // scales from 0 to 100 %
{
    // value of 30 is max equivalent to 100 %
    uint8_t val = (uint8_t)(30.0 / 100.0 * (double)vol);
    DEBUG_PRINT("Set volume ");
    DEBUG_PRINTLN_VAL(val, DEC);
    myMP3.setVolume(val);
}

void MP3Sound::playSoundAdHoc(int soundNr)
{
    if (soundNr > anzSoundfiles)
        return;

    myMP3.playGlobalTrack(soundNr);
}

void MP3Sound::playSound(int soundNr)
{
    if ((soundNr > anzSoundfiles) || (active_sounds.size() == BUFFERSIZE))
        return;

    active_sounds.push_front(soundNr);
}

int MP3Sound::sounds2play()
{

    return active_sounds.size();
}

int MP3Sound::processSounds()
{
    int n = active_sounds.size();
    if (n == 0)
        return (n);
    //    if (myMP3.isPlaying())
    //        return (n);

    int file2play = active_sounds.back();
    //    myMP3.play(file2play);
    active_sounds.pop_back();

    return active_sounds.size();
}

void MP3Sound::OnError(DfMp3 &mp3, uint16_t errorCode)
{
#ifdef USB_DEBUG
    DEBUG_SERIAL.print("DFPlayer error: 0x");
    DEBUG_SERIAL.println(errorCode, HEX);
#endif

    MP3Sound *snd = MP3Sound::GetInstance();
    if (!snd->reset_sent)
    {
        snd->received_clone_init_error = true;
    }
}

MP3Sound *MP3Sound::GetInstance()
{
    /**
     * This is a safer way to create an instance. instance = new Singleton is
     * dangeruous in case two instance threads wants to access at the same time
     */
    if (mp3sound_ == nullptr)
    {
        mp3sound_ = new MP3Sound();
    }
    return mp3sound_;
}