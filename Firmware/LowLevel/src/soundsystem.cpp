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

#include <soundsystem.h>


SerialPIO soundSerial(PIN_SOUND_TX, PIN_SOUND_RX);
DFPlayerMini_Fast myMP3;


MP3Sound::MP3Sound()
{

    this->anzSoundfiles =   0;          // number of files stored on the SD-card
    this->playing =         false;
    this->sound_available = false;

}



bool MP3Sound::begin(int anzsoundsOnSD)
              
{

    // serial stream init for soundmodule
    soundSerial.begin(9600);
    while (soundSerial.available())
        soundSerial.read();
    // init soundmodule
    sound_available = myMP3.begin(soundSerial,true);    
    delay(1000);
    return (sound_available);
}


void MP3Sound::setvolume(int vol)  // scales from 0 to 100 %
{

     // value of 30 is max equivalent to 100 %
     int val = (int) (30 / 100.0 * vol);  
     myMP3.volume(val);
     delay(300);

}



void MP3Sound::playSoundAdHoc(int soundNr)
{
    if(soundNr > NR_SOUNDFILES) return;

    myMP3.play(soundNr);
    delay(1000);
    
}




void MP3Sound::playSound(int soundNr)
{
    if((soundNr > NR_SOUNDFILES)  || (active_sounds.size() == BUFFERSIZE) ) return;

    active_sounds.push_front(soundNr);
    
}


int MP3Sound::sounds2play()
{


   return active_sounds.size();


}



int MP3Sound::processSounds()
{
   int n = active_sounds.size(); 
   if (n == 0) return(n);
   if (myMP3.isPlaying()) return(n);
   
   int file2play = active_sounds.back();
   myMP3.play(file2play);
   active_sounds.pop_back();

   return active_sounds.size();

}