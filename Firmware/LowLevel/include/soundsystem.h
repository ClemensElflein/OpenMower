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

#include <stdint.h>
#include <list>
#include <DFMiniMp3.h>

#define BUFFERSIZE 100
#define DFP_ONLINE_TIMEOUT 6000
#define DFP_CLONE_ERROR_TIMEOUT 200

// BTW: While readinging on, take into account that c/cpp is an ancient programming language from the last century ...

class MP3Sound;                               // forward declaration ...
typedef DFMiniMp3<SerialPIO, MP3Sound> DfMp3; // ... for a more readable/shorter DfMp3 typedef

// Non thread safe singleton MP3Sound class
class MP3Sound
{
protected:
    MP3Sound(); // Singleton constructors always should be private to prevent direct construction via 'new'

    static MP3Sound *mp3sound_;

public:
    int16_t anzSoundfiles; // number of files stored on the SD-card
    bool playing;

    static MP3Sound *GetInstance();
    MP3Sound(MP3Sound &other) = delete;        // Singletons should not be cloneable
    void operator=(const MP3Sound &) = delete; // Singletons should not be assignable

    bool begin(); // init serial stream and soundmodule, anzsoundOnSD : maximum number of available soundfiles on the SD-card

    void playSound(int soundNr); // play soundfile number. This method writes soundfile nr in a list, the method processSounds() (has to run in loop) will play
                                 // the sounds according to the list

    void playSoundAdHoc(int soundNr); // play soundfile number immediately whithout waiting until the end of sound

    void setVolume(uint8_t vol); // scales loudness from 0 to 100 %

    int sounds2play(); // returns the number if sounds to play in the list

    int processSounds(); // play all sounds from the list. This method has to be calles cyclic, e.g. every second.

    // DFMiniMP3 specific notification methods
    static void OnError(DfMp3 &mp3, uint16_t errorCode);
    static void OnPlayFinished(DfMp3 &mp3, DfMp3_PlaySources source, uint16_t track){};
    static void OnPlaySourceOnline(DfMp3 &mp3, DfMp3_PlaySources source){};
    static void OnPlaySourceInserted(DfMp3 &mp3, DfMp3_PlaySources source){};
    static void OnPlaySourceRemoved(DfMp3 &mp3, DfMp3_PlaySources source){};

private:
    std::list<int> active_sounds;
    bool sound_available;
    bool reset_sent = false, received_clone_init_error = false;
};

#endif // _SOUND_SYSTEM_H_  HEADER_FILE
