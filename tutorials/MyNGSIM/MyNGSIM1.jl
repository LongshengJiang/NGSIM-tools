__precompile__(true)
module  MyNGSIM1

# Using packages
using DataFrames
using Reexport
@reexport using AutomotiveDrivingModels
@reexport using AutoViz
@reexport using NGSIM

export
    # functions
    get_scene_id,
    get_neighbor_fore_along_lane_NGSIM,
    get_neighbor_fore_along_left_lane_NGSIM,
    get_neighbor_fore_along_right_lane_NGSIM,
    get_neighbor_rear_along_lane_NGSIM,
    get_neighbor_rear_along_left_lane_NGSIM,
    get_neighbor_rear_along_right_lane_NGSIM,
    # Raw trajectory data in Type NGSIMTrajectorydata
    tdraw_my,
    # Processed trajectory data
    trajdata_my,
    # My road way
    ROADWAY_my,
    # Customized data types
    LaneChangeInfo,
    RefinedLaneChangeInfo,
    LaneKeepInfo,
    RefinedLaneKeepInfo

# =============================================================================
# Data Type
# =============================================================================
# Define a structure for storing lane change information of vehicles.
struct LaneChangeInfo
    changto::AbstractString         # Changed to which direction 'left' or 'right'.
    from_to::Array{Int32, 1}       # [laneID_1, laneID_2] From laneID_1 to laneID_2
    cam_frames::Array{Int, 1}       # The camera frames during which the change happened.
    df_frames::Array{Int,1}         # The dataframe frames during which the change happened.
end

struct RefinedLaneChangeInfo
    changto::AbstractString         # Changed to which direction 'left' or 'right'.
    from_to::Array{Int32, 1}       # [laneID_1, laneID_2] From laneID_1 to laneID_2
    cam_frame::Int                 # The exact camera frame where the vehicle crossed the lane.
    df_frame::Int                  # The exact dataframe frame where the vehicle crossed the lane.
end

# Define a structure for storing lane change information of vehicles.
struct LaneKeepInfo
    cam_frames::Array{Int, 1}       # The camera frames during which the change happened.
    df_frames::Array{Int,1}         # The dataframe frames during which the change happened.
end

struct RefinedLaneKeepInfo
    cam_frame::Int                  # The exact camera frame where the vehicle started braking.
    df_frame::Int                   # The exact dataframe frame where the vehicle started braking.
end


# =============================================================================
# Variables
# =============================================================================
# We define the reference points on the vehicle body. For example, when we need
# to measure the distance between two vehicles, the distance can be from the center point
# of vehcile 1 to the center point of vehicle 2, with reference point VEHICLE_TARGET_POINT_CENTER;
# or the distance can be from the front of vehicle 1 to the back of vehicle 2,
# with reference points VEHICLE_TARGET_POINT_FRONT, VEHICLE_TARGET_POINT_REAR, respectively.
const VEHICLE_TARGET_POINT_CENTER = VehicleTargetPointCenter()
const VEHICLE_TARGET_POINT_FRONT = VehicleTargetPointFront()
const VEHICLE_TARGET_POINT_REAR = VehicleTargetPointRear()

# Load the traffic data of us_101, the units are meters, m/s
# 1. "i101_trajectories-0750am-0805am.txt"
# 2. "i101_trajectories-0805am-0820am.txt"
# 3. "i101_trajectories-0820am-0835am.txt"
# 4. "i80_trajectories-0400-0415.txt",
# 5. "i80_trajectories-0500-0515.txt",
# 6. "i80_trajectories-0515-0530.txt"
# Indicate the proper dataset here.
dataset_num = 1
#
if dataset_num == 1
    trajdata_my = load_trajdata(1) # "i101_trajectories-0750am-0805am.txt"
    road_filename = "i101_trajectories-0750am-0805am.txt"
    ROADWAY_my = ROADWAY_101
elseif dataset_num == 2
    trajdata_my = load_trajdata(2) # "i101_trajectories-0805am-0820am.txt"
    road_filename = "i101_trajectories-0805am-0820am.txt"
    ROADWAY_my = ROADWAY_101
elseif dataset_num == 3
    trajdata_my = load_trajdata(3) # "i101_trajectories-0820am-0835am.txt"
    road_filename = "i101_trajectories-0820am-0835am.txt"
    ROADWAY_my = ROADWAY_101
elseif dataset_num == 4
    trajdata_my = load_trajdata(4) # "i80_trajectories-0400-0415.txt",
    road_filename = "i80_trajectories-0400-0415.txt"
    ROADWAY_my = ROADWAY_80
elseif dataset_num == 5
    trajdata_my = load_trajdata(5) # "i80_trajectories-0500-0515.txt"
    road_filename = "i80_trajectories-0500-0515.txt"
    ROADWAY_my = ROADWAY_80
elseif dataset_num == 6
    trajdata_my = load_trajdata(6) # "i80_trajectories-0515-0530.txt"
    road_filename = "i80_trajectories-0515-0530.txt"
    ROADWAY_my = ROADWAY_80
else
    error("not appropriate dataset number")
end
road_filepath = Pkg.dir("NGSIM", "data", road_filename)
# --Load the un-converted data of us_101. This data is in DataFrammes structure,
# The units are feet, foot/s
tdraw_my = NGSIM.load_ngsim_trajdata(road_filepath)

# =============================================================================
# Functions of MyNGSIM
# =============================================================================
# We define a function to find vehicle scene id from its NGSIM id.
function get_scene_id{S<:VehicleState,D<:Union{VehicleDef, BicycleModel},I}(
    scene::EntityFrame{S,D,I},
    NGSIM_vehicle_index::Int
    )
    # Use NGSIM_vehicle_index to search for the scene vehicel index.
    # --Get the total number of the vehicles in the current scene.
    total_in_scene = scene.n
    # Note: Be careful, in Julia, if you want to return a value from a for-loop,
    # you should define the corresponding variable before the for-loop. For instance,
    # vehicle_index in the following; otherwise, nothing is returned from the for-loop.
    vehicle_index = 0
    for scene_ind = 1:total_in_scene
        # If the NGSIM_vehicle_index is the same in the current vehicle, this vehicle
        # is what we are looking for.
        if scene[scene_ind].id == NGSIM_vehicle_index
            vehicle_index = scene_ind
            break
        # Otherwise, if the current scene_ind is the last one, a.k.a, we have searched
        # all the vehicles in the scene, we return an error saying no finding.
        elseif scene_ind == total_in_scene
            error("This wanted vehicle is not in the current scene.")
        end
        # If neither case happens, this for-loop goes to the next iteration.
    end
    vehicle_index # return the vehicle' index in the scene
end

# Add a new method to function get_neigbor_fore_along_lane.
# This function returns the struct NeighborLongitudinalResult{id, distance}.
# The id is the scene_id and the distance is intervehicular distance.
function get_neighbor_fore_along_lane_NGSIM{S<:VehicleState,D<:Union{VehicleDef, BicycleModel},I}(
    scene::EntityFrame{S,D,I},
    NGSIM_vehicle_index::Int,
    roadway::Roadway,
    max_distance_fore::Float64 = 50.0 # max distance to search forward [m]
    )
    # Use NGSIM_vehicle_index to search for the scene vehicle index.
    vehicle_index = get_scene_id(scene, NGSIM_vehicle_index)
    # Call the function get_neighbor_fore_along_lane() from the package AutomotiveDrivingModels
    get_neighbor_fore_along_lane(scene, vehicle_index, roadway,
        VEHICLE_TARGET_POINT_FRONT, VEHICLE_TARGET_POINT_REAR,
        VEHICLE_TARGET_POINT_FRONT, max_distance_fore=max_distance_fore)
end

# Add a new method to function get_neigbor_fore_along_left_lane.
# This function returns the struct NeighborLongitudinalResult{id, distance}.
# The id is the scene_id and the distance is intervehicular distance.
function get_neighbor_fore_along_left_lane_NGSIM{S<:VehicleState,D<:Union{VehicleDef, BicycleModel},I}(
    scene::EntityFrame{S,D,I},
    NGSIM_vehicle_index::Int,
    roadway::Roadway,
    max_distance_fore::Float64 = 50.0 # max distance to search forward [m]
    )
    # Use NGSIM_vehicle_index to search for the scene vehicle index.
    vehicle_index = get_scene_id(scene, NGSIM_vehicle_index)
    # Call the function get_neighbor_fore_along_left_lane() from the package AutomotiveDrivingModels
    get_neighbor_fore_along_left_lane(scene, vehicle_index, roadway,
        VEHICLE_TARGET_POINT_FRONT, VEHICLE_TARGET_POINT_REAR,
        VEHICLE_TARGET_POINT_FRONT, max_distance_fore=max_distance_fore)
end

# Add a new method to function get_neigbor_fore_along_right_lane.
# This function returns the struct NeighborLongitudinalResult{id, distance}.
# The id is the scene_id and the distance is intervehicular distance.
function get_neighbor_fore_along_right_lane_NGSIM{S<:VehicleState,D<:Union{VehicleDef, BicycleModel},I}(
    scene::EntityFrame{S,D,I},
    NGSIM_vehicle_index::Int,
    roadway::Roadway,
    max_distance_fore::Float64 = 50.0 # max distance to search forward [m]
    )
    # Use NGSIM_vehicle_index to search for the scene vehicle index.
    vehicle_index = get_scene_id(scene, NGSIM_vehicle_index)
    # Call the function get_neighbor_fore_along_right_lane() from the package AutomotiveDrivingModels
    get_neighbor_fore_along_right_lane(scene, vehicle_index, roadway,
        VEHICLE_TARGET_POINT_FRONT, VEHICLE_TARGET_POINT_REAR,
        VEHICLE_TARGET_POINT_FRONT, max_distance_fore=max_distance_fore)
end

# Add a new method to function get_neigbor_rear_along_lane.
# This function returns the struct NeighborLongitudinalResult{id, distance}.
# The id is the scene_id and the distance is intervehicular distance.
function get_neighbor_rear_along_lane_NGSIM{S<:VehicleState,D<:Union{VehicleDef, BicycleModel},I}(
    scene::EntityFrame{S,D,I},
    NGSIM_vehicle_index::Int,
    roadway::Roadway,
    max_distance_rear::Float64 = 50.0 # max distance to search forward [m]
    )
    # Use NGSIM_vehicle_index to search for the scene vehicle index.
    vehicle_index = get_scene_id(scene, NGSIM_vehicle_index)
    # Call the function get_neighbor_rear_along_lane() from the package AutomotiveDrivingModels
    get_neighbor_rear_along_lane(scene, vehicle_index, roadway,
        VEHICLE_TARGET_POINT_REAR, VEHICLE_TARGET_POINT_FRONT,
        VEHICLE_TARGET_POINT_REAR, max_distance_rear=max_distance_rear)
end

# Add a new method to function get_neigbor_rear_along_left_lane.
# This function returns the struct NeighborLongitudinalResult{id, distance}.
# The id is the scene_id and the distance is intervehicular distance.
function get_neighbor_rear_along_left_lane_NGSIM{S<:VehicleState,D<:Union{VehicleDef, BicycleModel},I}(
    scene::EntityFrame{S,D,I},
    NGSIM_vehicle_index::Int,
    roadway::Roadway,
    max_distance_rear::Float64 = 50.0 # max distance to search forward [m]
    )
    # Use NGSIM_vehicle_index to search for the scene vehicle index.
    vehicle_index = get_scene_id(scene, NGSIM_vehicle_index)
    # Call the function get_neighbor_rear_along_left_lane() from the package AutomotiveDrivingModels
    get_neighbor_rear_along_left_lane(scene, vehicle_index, roadway,
        VEHICLE_TARGET_POINT_REAR, VEHICLE_TARGET_POINT_FRONT,
        VEHICLE_TARGET_POINT_REAR, max_distance_rear=max_distance_rear)
end

# Add a new method to function get_neigbor_rear_along_right_lane.
# This function returns the struct NeighborLongitudinalResult{id, distance}.
# The id is the scene_id and the distance is intervehicular distance.
function get_neighbor_rear_along_right_lane_NGSIM{S<:VehicleState,D<:Union{VehicleDef, BicycleModel},I}(
    scene::EntityFrame{S,D,I},
    NGSIM_vehicle_index::Int,
    roadway::Roadway,
    max_distance_rear::Float64 = 50.0 # max distance to search forward [m]
    )
    # Use NGSIM_vehicle_index to search for the scene vehicle index.
    vehicle_index = get_scene_id(scene, NGSIM_vehicle_index)
    # Call the function get_neighbor_rear_along_left_lane() from the package AutomotiveDrivingModels
    get_neighbor_rear_along_right_lane(scene, vehicle_index, roadway,
        VEHICLE_TARGET_POINT_REAR, VEHICLE_TARGET_POINT_FRONT,
        VEHICLE_TARGET_POINT_REAR, max_distance_rear=max_distance_rear)
end

end # end of module
