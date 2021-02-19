
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
We can check the id of this segment by typing
```julia
julia> ROADWAY_101.segments[1].id
3
```
We know this is the segment 3. We can further go deeper to explore these fields.
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
Hence, we know the first segment of `RAODWAY_101` has 6 lanes. This segment (segment 3) must be part of the middle section of the road in the above picture. We want to see what the fields of the lanes are.
```julia
julia> fieldnames(ROADWAY_101.segments[1].lanes[1])
8-element Array{Symbol,1}:
 :tag           
 :curve         
 :width         
 :speed_limit   
 :boundary_left 
 :boundary_right
 :exits         
 :entrances   
 ```
 Here we can find the tag of the lane. Let's see what a tag is.
 ```julia
julia> fieldnames(ROADWAY_101.segments[1].lanes[1].tag)
2-element Array{Symbol,1}:
 :segment
 :lane  
 ```
 A tag tells us the segment number and the lane number. 
 Let's see what `:exits` and `:entrances` tell us.
 ```julia
 julia> fieldnames(ROADWAY_101.segments[1].lanes[1].exits[1])
3-element Array{Symbol,1}:
 :downstream
 :mylane    
 :target    

julia> fieldnames(ROADWAY_101.segments[1].lanes[1].exits[1].target)
2-element Array{Symbol,1}:
 :ind
 :tag
 
 julia> fieldnames(ROADWAY_101.segments[1].lanes[1].entrances[1])
3-element Array{Symbol,1}:
 :downstream
 :mylane    
 :target    

julia> fieldnames(ROADWAY_101.segments[1].lanes[1].entrances[1].target)
2-element Array{Symbol,1}:
 :ind
 :tag
 ```
 We can see `:exits` and `:entrances` tells us the tags of the lanes connecting to the exit and entrance of this lane. Let's see an example.
 ```julia
julia> ROADWAY_101.segments[1].lanes[1].tag
LaneTag(3, 1)

julia> ROADWAY_101.segments[1].lanes[1].exits
1-element Array{AutomotiveDrivingModels.LaneConnection,1}:
 LaneConnection(D, CurveIndex(825, 1.000), RoadIndex({1, 0.000000}, {2, 1})

julia> ROADWAY_101.segments[1].lanes[1].entrances
1-element Array{AutomotiveDrivingModels.LaneConnection,1}:
 LaneConnection(U, CurveIndex(1, 0.000), RoadIndex({227, 1.000000}, {5, 1})
 ```
 We see the current lane is (segment 3, lane 1). At its downstream is (segment 2, lane 1). At its upstream is (segment 5, lane 1). So where is segment 2 and 5 in the picture?
 
We find here 
```julia
julia> fieldnames(ROADWAY_101.segments[1].lanes[1])
8-element Array{Symbol,1}:
 :tag           
 :curve         
 :width         
 :speed_limit   
 :boundary_left 
 :boundary_right
 :exits         
 :entrances   
 ```
 The lane has information about its left and right boundary. Let us see what they are.
 
 
