# NGSIM-sim
This repository collects a set of tools for extracting, visualizing, and simulating Next Generation Simulation (NGSIM) dataset. The tools are originally developed by Stanford Intelligent Systems Laboratory (SISL).  
Please check their github repositories on https://github.com/sisl

## Demo: What these tools can do?
![Single agent on highway](media/single_agent_gail.gif)

## Why do we need this recollection?
Successful running the tools for dealing with NGSIM requires packages which are mutually dependent but scattered in different repositories. Since each repository undergoes active development, directly cloning the master branch of the repositories results in version conflicts, which are, trust me, frustrating.

## What is in this repository?
While do not claim any originality, in this repository we make sure you can find all you need to get up and running the tools for dealing with NGSIM dataset. Specifically, the following are included.
1. All the packages you will need:
    - Julia v0.6.2
    - Records v0.6.0 (branch 0.6_legacy)
    - Vec v0.6.0 (branch 0_6)
    - AutomotiveDriving v0.6.0
    - AutoViz v0.6.0
    - NGSIM v1.1.0
    - A portion of the NGSIM dataset
    The dependency between the packages, base on my understanding, looks like the following diagram. 
    
      ![package-dependency](media/package_dependency.svg)
2. The installation procedure
3. The collection of tutorials on how to using the tools.
If you are working on your research and plan to publish papers, please check the original places of these packages and cite their work there. Thank you. (disclaimer: I am not affiliate with SISL.)



