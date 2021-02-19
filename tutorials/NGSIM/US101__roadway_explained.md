
In this document, I will explain the segments labeling and lane labeling of US 101. This information is important if you want to investigate the lane changing behaviors of the vehicles, thus needing the lane information. 

NGSIM_tools transfer the raw road information in NGSIM dataset to a customized data structure, hence the lanes are relabeled. First, the road was divided into 5 segmented as follows.

![](./images/US101_segment.png)

Don't believe it? It's okay. Here is how I figure it out. Try it out by yourselves. We first use the `NGSIM` module
```julia
julia> using NGSIM
```
This module export the us_101 as a roadway object. The roadway type is defined in `AutomotiveDrivingModels` module. This object's name is `ROADWAY_101`. We can always use `fieldnames(ROADWAY_101)` to check its fields.
```julia
julia> fieldnames(ROADWAY_101)
1-element Array{Symbol,1}:
 :segments
```
We can keep using `fieldnames` to go deeper.
```julia
julia> fieldnames(ROADWAY_101.segments)
0-element Array{Symbol,1}
```
Here we see `:segments` is an object with no fields. We can check it size
```julia
julia> size(ROADWAY_101.segments)
(5,)
```
It has 5-elements. Hence, `ROADWAY_101` has 5 segment. We check the first segment.
```julia
julia> fieldnames(ROADWAY_101.segments[1])
2-element Array{Symbol,1}:
 :id   
 :lanes
```
Go deeper.
```julia
julia> fieldnames(ROADWAY_101.segments[1].id)
0-element Array{Symbol,1}

julia> fieldnames(ROADWAY_101.segments[1].lanes)
0-element Array{Symbol,1}

julia> size(ROADWAY_101.segments[1].id)
()

julia> size(ROADWAY_101.segments[1].lanes)
(6,)
```
