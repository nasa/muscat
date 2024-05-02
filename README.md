# Multi-Spacecraft Concept and Autonomy Tool (MuSCAT) Matlab

## Step 1

Git clone the repo into your desired folder. Lets say it is `XYZ >`

Then the `muscat` folder is accessed using `XYZ > muscat`


## Step 2

Ensure that `MuSCAT_Supporting_Files` is in the same parent folder. You can download it from here https://www.dropbox.com/s/qokkcj6sn802n7p/MuSCAT_Supporting_Files.zip?dl=0 

Then the `MuSCAT_Supporting_Files` folder is accessed using `XYZ > MuSCAT_Supporting_Files`

Ensure that you see the following folders inside `MuSCAT_Supporting_Files` folder: `SC_data`, `SB_data`, `Science_points`, `SPICE`, `time`, `Stars`

Download `mice` into the `SPICE` folder from here https://naif.jpl.nasa.gov/naif/toolkit_MATLAB.html


## Step 3

Now we are ready to run the mission files. 

First, ensure that Matlab base directory is `XYZ > muscat`

Second, open in Matlab any of the mission files in `XYZ > muscat > Mission` 

Thrid, click `Run`. DO NOT `Change Folder`. Instead click `Add to Path`. 

Finally, the code should run out of the box!  

## Debug Log
If bugs are encountered, please report in the issues section. For information on resolved bug, please look at the closed issues. 
