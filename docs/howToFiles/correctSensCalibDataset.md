## Easily correct the extra _0001 coming from the outdated bug version of sensors calib inertial ft branch.
Follow these steps:

- create a folder "experiment type" where to put the `icub.sensorsTestDataAcquisition#.seq#` folder.
- go in the folder: `cd "experiment name"`
- use : `prename -v 's/stateExt/toErase1/' */*/*stateExt`
- use : `prename -v 's/analog/toErase2/' */*/*analog`
- use : `prename -v 's/measures/toErase3/' */*/*measures`
- use :   `prename -v 's/_00001//' */*/*_00001/`
- use :  `rm -r  */*/toErase*`


This should have renamed all non `*_0001` folders to `toErase2` and then erase them to allow the renaming of all other folders to remove the _00001

