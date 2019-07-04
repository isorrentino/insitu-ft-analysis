# How to rename multiple files
Sometimes when using the sens calib inertial or when connecting multiple times using yarpdatadumper we might end up with multiple folders with *_000# in the same icub folder.
Since we assume each icub folder is a separate experiment is necesary to separate them.
It might be also useful to rename them so that the use the standard naming format in the params.m file.

To change multiple folders we can use the following command.

`prename  -v 's/_00001//' */*_00001/`


-v is for showing which files were renamed

's/_00001//' is a perl regular expression to replace '_00001' for nothing. So effectively removing it 

*/*_00001/ indicates which files to replace. In here it looks one level down and then replaces whatever has at the end the _00001
