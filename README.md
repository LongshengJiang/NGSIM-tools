# NGSIM-tools (Under construction. Don't use!)
This repository collects a set of tools for extracting, visualizing, and simulating Next Generation Simulation (NGSIM) dataset. The tools are originally developed by Stanford Intelligent Systems Laboratory (SISL).  
Please check their github repositories on https://github.com/sisl

If you are working on your research and plan to publish papers, please check the original places of these packages and cite their work there. Thank you. (disclaimer: I am not affiliate with SISL.)


## Demo: What these tools can do?
![Single agent on highway](media/single_agent_gail.gif)

## Why do we need this recollection?
Successful running the tools for dealing with NGSIM requires packages which are mutually dependent but scattered in different repositories. Since each repository undergoes active development, directly cloning the master branch of the repositories results in version conflicts, which are, trust me, frustrating.

## What is in this repository?
While do not claim any originality, in this repository we make sure you can find all you need to get up and running the tools for dealing with NGSIM dataset. Specifically, the following are included.
1. The packages you will need:
    - Julia v0.6.2 (source file not included here)
    - Records v0.6.0 (branch 0.6_legacy)
    - Vec v0.6.0 (branch 0_6)
    - AutomotiveDriving v0.6.0
    - AutoViz v0.6.0
    - NGSIM v1.1.0
    
    The dependency between the packages, base on my understanding, looks like the following diagram. 
    
      ![package-dependency](media/package_dependency.svg)
      
2. The installation procedure

3. The collection of tutorials on how to using the tools.

However, you do need to download the following files from other repositories. This is because they are too large, I cannot include them here. They are
- Julia v0.6.2
- NGSIM data
I will let you know how to clone or download these files.

## Installation procedure:

### First, install julia v0.6.2
If we want to install julia using command line, using the following code. I use Ubuntu 16.04.6. 
```bash
# Let us install julia in home directory.
cd ~
# Now we install julia-0.6.2
wget https://julialang-s3.julialang.org/bin/linux/x64/0.6/julia-0.6.2-linux-x86_64.tar.gz
tar -xf julia-0.6.2-linux-x86_64.tar.gz
# Clean the installation file.
rm julia-0.6.2-linux-x86_64.tar.gz
# Add the path to the system PATH variable
echo "export PATH=$(pwd)/julia-d386e40c17/bin:\$PATH" >> ~/.bashrc
# Rerun .bashrc to update the changes
source ~/.bashrc
```
### Second, install packages of NGSIM-tools
Install some packages to ubuntu 16.04 in case they are not there already.
```bash
sudo apt-get install libavcodec-ffmpeg56
sudo apt-get install libavformat-ffmpeg56
sudo apt-get install libavfilter-ffmpeg5
sudo apt-get install libavdevice-ffmpeg56
```
Open julia by typing `julia` in your terminal. Make sure the version of julia should be v0.6.2.
```julia
Pkg.add("StaticArrays")
Pkg.add("Reexport")
Pkg.add("Colors")
Pkg.add("DataFrames")
```
These commands will automatically create a new path: `~\.julia\v0.6\` This directory is important. It is where all your packages are saved. 
