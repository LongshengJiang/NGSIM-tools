### download NGSIM data
```bash
##Get the data
cd ~/.julia/packages/NGSIM/9OYUa/data
wget https://github.com/sisl/NGSIM.jl/releases/download/v1.0.0/data.zip
unzip data.zip
# Answer yes to any that ask to be replaced.

##Create trajectories from the data
julia
  >> using NGSIM
  >> convert_raw_ngsim_to_trajdatas()
  >> quit()
# NOTE: my attempt got killed here and i have no idea why. No error messages or anything.
```
