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

//#include <Arduino.h>
#include <stdint.h>
#include <list>

#include <pins.h>
#include <DFPlayerMini_Fast.h>
#include <soundsystem.h>


#define BUFFERSIZE 100


class MP3Sound
{

   
   

    public:

                int16_t anzSoundfiles;          // number of files stored on the SD-card
                bool     playing;

                MP3Sound();

                bool begin();      // init serial stream and soundmodule, anzsoundOnSD : maximum number of available soundfiles on the SD-card
                
                void playSound(int soundNr);        // play soundfile number. This method writes soundfile nr in a list, the method processSounds() (has to run in loop) will play 
                                                    // the sounds according to the list

                void playSoundAdHoc(int soundNr);   // play soundfile number immediately whithout waiting until the end of sound

                void setvolume(int vol);            // scales loudness from 0 to 100 %

                int sounds2play();                  // returns the number if sounds to play in the list

                int processSounds();                // play all sounds from the list. This method has to be calles cyclic, e.g. every second.

                
    private:
                std::list <int> active_sounds;
                bool sound_available;
                

   

};


#endif // _SOUND_SYSTEM_H_  HEADER_FILE
