# Sound via "DFPlayer Mini"

> **Warning**
> Consider about switching your DFPlayer's VCC from 3.3V to 5V
> (via solder jumper JP1 on your OpenMower MainBoard)

**Explanation:**<br>
I'm with sound since a couple of months.<br>
In the first month, I lost 2 Picos because his tiny Buck-Boost Converter "RT6150" cracked.<br>
After a short correspondence with Clemens, he pointed me to the DFPlayer
as a possible reason.<br>
Because: By OM default design, the DFPlayer's VCC is 3.3V (via JP1),
which is provided by the small Buck-Boost Converter on the Pico.<br>
After I switched my DFPlayer's VCC to 5V (via JP1), I didn't lost any Pico anymore.<br>
It's not confirmed yet if that really was the reason for my cracked Pico's,
and you're invited to validate the assumption by leaving your DFPlayer on 3.3V. **But be warned**, even if the Pico is cheap, it's awful to replace it! 

### Update 10/13/2023

As I'm still in doubt if it's really necessary to switch DFPlayer's VCC to 5V,
I did some measuring today:

- Placed an 0.2Ω resistor (4W) within Pico's 3V3 output line
- Measured with an oscilloscope the occuring voltage over the resistor:<br>
  0.028 V<sub>AVG</sub> => divided by 0.2Ω = 140mA => looks fine<br>
  0.195 V<sub>PP</sub> => divided by 0.2Ω = 975mA => hugh :-/ but this is V<sub>PP</sub>!
- Within the Pico Datasheet it's written (somewhere), thats allowed to draw up to 300mA
- The specs of the Buck-Boost Converter "RT6150" (used on the Pico) say:<br>
  "*Up to 800mA Continuous Output Current*", as well as<br>
  "*... current
limit.*"

Conclusion: I'm still unsure :-/

## Sound Buttons

Button | Function
------ | --------
<kbd>Mon</kbd> | Volume up
<kbd>Tue</kbd> | Volume down
<kbd>Wed</kbd> | Language switch (English, German)

## DFPlayer Module / Clones

Beside the original [DFPlayer by DFRobot](https://www.dfrobot.com/product-1121.html) module, there are a couple of "DFPlayer-Mini" clones in the wild.

If you order a "DFPlayer-Mini" by Amazon or the like, there's a >90% chance that you get one of these clones, instead of the original one.

That's why I tried to adapt the code in that way, that some of these clones get also supported.

Check the larger chip on the backside of the module, to verify if your DFPlayer is supported. The following DFPlayer chips are supported/tested at the moment:

- `DFROBOT LISP3` is the one on my original [DFPlayer by DFRobot](https://www.dfrobot.com/product-1121.html) module
- `GD3200B` is one of the newer clones
- `MH2024K-24SS` which is one of the older clones

The used DFPlayer library also indicate support for `YX5200-24SS` and `MH2024K-16SS`, but couldn't test these by myself. If you've one of these, give it a try.

## SD-Card

Look like all DFPlayer support a SD-Card size up to 32GB.
As we don't have a lot soundfiles (<10MB), choose the smallest one you can find, format it with a FAT32 file system (or if it's already formatted, remove all existing files) and copy all these [files and folder](./soundfiles/) to your SD-Card.

### Folder/Track Structure

Due to some incompatibilities with the clone chips, we unfortunately can't use DFPlayer's internal "advert" functionality for our usage.

So we had to trick a little bit with the folder structure:

- OM's old original sounds are now in subfolder 'advert' as they're typically spoken adverts, which may interrupt background sounds like noises or music. But they're here mainly for future usage, for the case that the internal "advert" limitations could be fixed or worked around at some time.
- Folder 'mp3' contain background sounds (noises, music), which are hardcoded in the sources.
- Due to the case that some DFPlayer don't play 'adverts' without a current running background sound, as well as someone might prefer no background sounds at all, the 'advert' sounds got also copied to a `playFolderTrack()` folder (`playFolderTrack16()` had some bugs with some modules).
  At the moment only folder '01' (US English) and '49' (German) exist.
- The single sound in the root folder, is used/required for those DFPlayer types (i.e. DFRobot),
  which start autoplaying directly after power-on, or after reset() during initialize.
