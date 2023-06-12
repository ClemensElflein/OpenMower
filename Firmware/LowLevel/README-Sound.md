# Sound via "DFPlayer Mini"

## DFPlayer Module / Clones

Beside the original [DFPlayer by DFRobot](https://www.dfrobot.com/product-1121.html) module, there are a couple of "DFPlayer-Mini" clones in the wild.

If you order a "DFPlayer-Mini" by Amazon or the like, there's a >90% chance that you get one of these clones, instead of the original one.

That's why I tried to adapt the code in that way, that some of these clones get also supported.

Check the the larger chip on the backside of the module, to verify if your DFPlayer is supported. The following DFPlayer chips are supported/tested at the moment:

- `DFROBOT LISP3` is the one on my original [DFPlayer by DFRobot](https://www.dfrobot.com/product-1121.html) module
- `GD3200B` is one of the newer clones
- `MH2024K-24SS` which is one of the older clones

The used library also indicate supporting `YX5200-24SS` and `MH2024K-16SS`, but couldn't test these by myself. If you've one of these, give it a try.

## SD-Card

Look like all DFPlayer support a SD-Card size up to 32GB.
As we don't have a lot soundfiles (<10MB), choose the smallest one you can find, format it with a FAT32 file system and copy all these [files and folder](./soundfiles/) to your SD-Card.

### Folder/Track Structure

Due to some incompatibilities with the clone chips, we unfortunately can't use DFPlayer internal "advert" functionality for our usage.

So we had to trick a little bit with the folder structure:

- OM's old original sounds are now in subfolder 'advert' as they're typically spoken adverts, which may interrupt background sounds like noises or music. But they're here mainly for future usage, for the case that the internal "advert" limitations could be fixed or worked around some time
- Folder 'mp3/000?' contains background sounds (noises, music), which are hardcoded within the sources
- Folder 'mp3/02??' contains background sounds (noises, music), which get played randomly
  during HighLevelMode-Autonomous. If you add sounds here, you also need to change `#define ???`
- Due to the case that some DFPlayer don't play 'adverts' without a current running background sound, as well as someone might prefer no background sounds at all, the 'advert' sounds got also copied to a `playFolderTrack16()`.
  At the moment only folder '01' (US English), but a folder like '49' (German) might follow some day, or get added by you when customizing
- The single sound in the root folder, is used/required for those
  DFPlayer types (i.e. DFRobot), which start autoplaying directly after power on, or by reset() during initialize.
