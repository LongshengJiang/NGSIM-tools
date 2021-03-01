# The labeling of vehicles is in the following diagram.
#-------------------------------------------------------------------------------
#             ==> veh 3                                  ==> veh 2
#-------------------------------------------------------------------------------
#  ==> veh 4                ==> veh 0 (ego)               ==> veh 1
#-------------------------------------------------------------------------------
#          ==> veh 32                                  ==> veh 22
#-------------------------------------------------------------------------------
# Pseduo code of this program
#
# For a vehicle in all the vehicles:
#   While (this vehicle is between entrance and exit):
#        Move to the next data frame
#        If (veh 1 is slower than veh 0) and (veh 2 or veh 2' is faster than veh 1):
#               While (veh 0 is in the current lane):
#                       If (|speed of veh 1 - speed of veh 0| )
#                               Extract the data frames of the next 5s;
#                               Extract the data frames of the preceding 5s;
#                               Move to the data frame after 5s;
#                               Break;
#                       Else:
#                               Move to the next data frame;
#

# To run this program, the prerequsite module is MyNGSIM. Type in REPL
# using MyNGSIM

# In this program, I define the following functions.
#   get_neighbor_speed
#   traffic_info_NGSIM
#
#
# The function dependence is
#   extract_epoch
#               |_ traffic_info_NGSIM
#                                    |_ get_neighbor_speed

# In this program, I define the following data structures.
#   EpochDuration
#

# The duration of an epoch
struct EpochDuration
    egoid::Int16                  # The id of the ego vehicle
    startframe::Int16             # The frame at which an epoch starts
    after_duration::Int16         # The duration after the start frame
    before_duration::Int16        # The duration before the start frame
end
#
# The information after a triggering event.
struct InfoAfterTrigger
    duration::Int16                 # The duration after event triggering
    egoid::Int16                    # The id of the ego vehicle
    triggerframe::Int16             # The frame ath which event-triggering happens
    laneids::Array{Int16, 1}         # The list of lane ids after the triggering.
    fore_rel_speed::Array{Float64, 1}   # The list of relative speed of the front vehicle
end

# In this program, I define the following useful constants:
#
# A vehicle appears in the dataset in [start frame, end frame]. We add a margin
# to this, hence we only consider [start_frame + margin, end_frame - margin] for
# this vehicle.
const VEHICLE_IN_VIEW_MARGIN = 50   # unit: frames
#
# When the speed difference is between the following threshold, we think the human
# cannot tell the difference.
const SPEED_DIFF_THRESHOLD = 1.0      # unit m/s

# From here I start to define function.
# ------------------------------------------------------------------------------
# """
# get_neighbor_speed gets the speed of the neighbor in the scene.
# """
function get_neighbor_speed(scene::Records.Frame{Records.Entity{AutomotiveDrivingModels.VehicleState,AutomotiveDrivingModels.VehicleDef,Int64}},
                            neighbor::AutomotiveDrivingModels.NeighborLongitudinalResult,
                            alternative_speed::Float64 = 25.0)
    # We now compute the speed. If there is a neighbor
    if neighbor.ind != 0    # index in scene of the neighbor
        # Obtain the lead vehicle velocity [m/s]
        speed = scene[neighbor.ind].state.v
    # otherwise, set the speed to be the speed limit 55 mph == 25 m/s
    else
        # Obtain the lead vehicle velocity [m/s]
        speed = alternative_speed
    end
    speed
end

# ------------------------------------------------------------------------------
# """
# traffic_info_NGSIM gets the relative distance and absolute velocity of the ego or one of its neighbors.
# """
function traffic_info_NGSIM(trajdata::Records.ListRecord{AutomotiveDrivingModels.VehicleState,AutomotiveDrivingModels.VehicleDef,Int64},
                            roadway::AutomotiveDrivingModels.Roadway,
                            carid::Int,
                            cameraframe::Int;
                            longitudinal_direc::String = "self",
                            lateral_direc::String = "middle",
                            sample_duration::Int = 1,
                            aggregate_method::String = "mean")
    # Define a dictionary for traffic info
    traffic_info = Dict{Symbol, Union{Float32, VehicleDef}}()
    # Define a speed cache and a distance chance here
    speed_vec = Vector{Float32}(sample_duration)
    speed = 0.0
    distance_vec = Vector{Float32}(sample_duration)
    distance = 0.0
    # This record the type and dimension of the vehicle.
    vehicle_def = AutomotiveDrivingModels.VehicleDef()

    # If the vehicle to be observed is the ego self,
    if longitudinal_direc == "self"
        for i = 1:sample_duration
            scene = get!(Scene(500), trajdata, cameraframe +1 - i);
            # We get the index of the vehicle (carid) in the scene
            ego_ind = get_scene_id(scene, carid)
            # Let the distance to self be 0
            distance_vec[i] = 0
            # We get the velocity of the ego vehicle in unit [m/s].
            speed_vec[i] = scene[ego_ind].state.v
            # The vehicle definition.
            if i == 1
                vehicle_def = scene[ego_ind].def
            end
        end
    # If the vehicle to be observed is in front,
    elseif longitudinal_direc == "fore"
        # further if the vehicle to be observed shares the lane with the ego
        if lateral_direc == "middle"
            # We only want to record the information of the first neighbor when i=1
            original_neighbor_id = 0;
            for i = 1:(sample_duration)
                scene = get!(Scene(500), trajdata, cameraframe + 1 - i);
                neighbor = get_neighbor_fore_along_lane_NGSIM(scene, carid, roadway)
                # If there is one neighbor exist,
                if neighbor.ind != 0
                    # We retrieve the id of the first neighbor.
                    if i == 1
                        original_neighbor_id = scene[neighbor.ind].id;
                        # The vehicle definition.
                        vehicle_def = scene[neighbor.ind].def
                    end
                    # If the neighbor is the original neighbor
                    if scene[neighbor.ind].id == original_neighbor_id  # when i = 1, this condition is guaranteed to be true!
                        # We retrieve the distance between the neighbor and the ego and the speed of the neighbor.
                        distance_vec[i] = neighbor.Δs
                        speed_vec[i] = get_neighbor_speed(scene, neighbor)
                    # If the neighbor is not the original neighbor, we copy the distance and speed at i-1
                    else
                        distance_vec[i] = distance_vec[i-1]
                        speed_vec[i] = speed_vec[i-1]
                    end
                end
            end
        # If the vehicle to be observed is in the left,
        elseif lateral_direc == "left"
            # We only want to record the information of the first neighbor when i=1
            original_neighbor_id = 0;
            for i = 1:sample_duration
                scene = get!(Scene(500), trajdata, cameraframe + 1  - i);
                neighbor = get_neighbor_fore_along_left_lane_NGSIM(scene, carid, roadway)
                # If there is one neighbor exist,
                if neighbor.ind != 0
                    # We retrieve the id of the first neighbor.
                    if i == 1
                        original_neighbor_id = scene[neighbor.ind].id;
                        # The vehicle definition.
                        vehicle_def = scene[neighbor.ind].def
                    end
                    # If the neighbor is the original neighbor
                    if scene[neighbor.ind].id == original_neighbor_id  # when i = 1, this condition is guaranteed to be true!
                        # We retrieve the distance between the neighbor and the ego and the speed of the neighbor.
                        distance_vec[i] = neighbor.Δs
                        speed_vec[i] = get_neighbor_speed(scene, neighbor)
                    # If the neighbor is not the original neighbor, we copy the distance and speed at i-1
                    else
                        distance_vec[i] = distance_vec[i-1]
                        speed_vec[i] = speed_vec[i-1]
                    end
                end
            end
        # If the vehicle to be observed is on right,
        elseif lateral_direc == "right"
            # We only want to record the information of the first neighbor when i=1
            original_neighbor_id = 0;
            for i = 1:sample_duration
                scene = get!(Scene(500), trajdata, cameraframe + 1 - i);
                neighbor = get_neighbor_fore_along_right_lane_NGSIM(scene, carid, roadway)
                if neighbor.ind != 0
                    # We retrieve the id of the first neighbor.
                    if i == 1
                        original_neighbor_id = scene[neighbor.ind].id;
                        # The vehicle definition.
                        vehicle_def = scene[neighbor.ind].def
                    end
                    # If the neighbor is the original neighbor
                    if scene[neighbor.ind].id == original_neighbor_id  # when i = 1, this condition is guaranteed to be true!
                        # We retrieve the distance between the neighbor and the ego and the speed of the neighbor.
                        distance_vec[i] = neighbor.Δs
                        speed_vec[i] = get_neighbor_speed(scene, neighbor)
                    # If the neighbor is not the original neighbor, we copy the distance and speed at i-1
                    else
                        distance_vec[i] = distance_vec[i-1]
                        speed_vec[i] = speed_vec[i-1]
                    end
                end
            end
        else
            error("When getting front neighbor, the lateral direction is not valid.")
        end
    # If the vehicle to be observed is at rear,
    elseif longitudinal_direc == "rear"
        # further if the vehicle to be observed is in the middle
        if lateral_direc == "middle"
            # We only want to record the information of the first neighbor when i=1
            original_neighbor_id = 0;
            for i = 1:sample_duration
                scene = get!(Scene(500), trajdata, cameraframe + 1 - i);
                neighbor = get_neighbor_rear_along_lane_NGSIM(scene, carid, roadway)
                if neighbor.ind != 0
                    # We retrieve the id of the first neighbor.
                    if i == 1
                        original_neighbor_id = scene[neighbor.ind].id;
                        # The vehicle definition.
                        vehicle_def = scene[neighbor.ind].def
                    end
                    # If the neighbor is the original neighbor
                    if scene[neighbor.ind].id == original_neighbor_id  # when i = 1, this condition is guaranteed to be true!
                        # We retrieve the distance between the neighbor and the ego and the speed of the neighbor.
                        distance_vec[i] = neighbor.Δs
                        speed_vec[i] = get_neighbor_speed(scene, neighbor)
                    # If the neighbor is not the original neighbor, we copy the distance and speed at i-1
                    else
                        distance_vec[i] = distance_vec[i-1]
                        speed_vec[i] = speed_vec[i-1]
                    end
                end
            end
        # If the vehicle to be observed is on left
        elseif lateral_direc == "left"
            # We only want to record the information of the first neighbor when i=1
            original_neighbor_id = 0;
            for i = 1:sample_duration
                scene = get!(Scene(500), trajdata, cameraframe + 1 - i);
                neighbor = get_neighbor_rear_along_left_lane_NGSIM(scene, carid, roadway)
                if neighbor.ind != 0
                    # We retrieve the id of the first neighbor.
                    if i == 1
                        original_neighbor_id = scene[neighbor.ind].id;
                        # The vehicle definition.
                        vehicle_def = scene[neighbor.ind].def
                    end
                    # If the neighbor is the original neighbor
                    if scene[neighbor.ind].id == original_neighbor_id  # when i = 1, this condition is guaranteed to be true!
                        # We retrieve the distance between the neighbor and the ego and the speed of the neighbor.
                        distance_vec[i] = neighbor.Δs
                        speed_vec[i] = get_neighbor_speed(scene, neighbor)
                    # If the neighbor is not the original neighbor, we copy the distance and speed at i-1
                    else
                        distance_vec[i] = distance_vec[i-1]
                        speed_vec[i] = speed_vec[i-1]
                    end
                end
            end
        # If the vehicle to be observed is on the right,
        elseif lateral_direc == "right"
            # We only want to record the information of the first neighbor when i=1
            original_neighbor_id = 0;
            for i = 1:sample_duration
                scene = get!(Scene(500), trajdata, cameraframe + 1 - i);
                neighbor = get_neighbor_rear_along_right_lane_NGSIM(scene, carid, roadway)
                if neighbor.ind != 0
                    # We retrieve the id of the first neighbor.
                    if i == 1
                        original_neighbor_id = scene[neighbor.ind].id;
                        # The vehicle definition.
                        vehicle_def = scene[neighbor.ind].def
                    end
                    # If the neighbor is the original neighbor
                    if scene[neighbor.ind].id == original_neighbor_id  # when i = 1, this condition is guaranteed to be true!
                        # We retrieve the distance between the neighbor and the ego and the speed of the neighbor.
                        distance_vec[i] = neighbor.Δs
                        speed_vec[i] = get_neighbor_speed(scene, neighbor)
                    # If the neighbor is not the original neighbor, we copy the distance and speed at i-1
                    else
                        distance_vec[i] = distance_vec[i-1]
                        speed_vec[i] = speed_vec[i-1]
                    end
                end
            end
        # If the vehicle to be observed is on right,
        else
            error("When getting rear neighbor, the lateral direction is not valid.")
        end # if end
    else
        error("The longitudinal direction is not valid.")
    end # if end
    # Depending on the aggregation method, we use min or mean.
    if aggregate_method == "min"
        # Get minimum values
        speed = minimum(speed_vec)
        distance = minimum(distance_vec)
    else
        # Getting mean values
        speed = mean(speed_vec)
        distance = mean(distance_vec)
    end
    traffic_info[:speed] = speed
    traffic_info[:distance] = distance
    traffic_info[:vehicle_def] = vehicle_def
    traffic_info
end # function end

# """
# Get the total number of lane changes in lane 1--5
# """
function lanechanges_stats()
    # Start recording the lane changes
    n_lanechange = 0;
    n_car_lanekeep = 0;
    n_aborted = 0;
    # We define some containers
    # A dictionary of aborted lane changes
    aborted_lanechanges = Dict{Int, Vector{Int}}()
    #For a vehicle in all the vehicles:
    for egoid in keys(tdraw_my.car2start)  # tdraw_my is loaded by using MyNGSIM
        # Get the starting row of the ego vehicle in tdraw_my
        ego_start_row = tdraw_my.car2start[egoid];
        # Get the total number of frames of the ego vehicle in tdraw_my
        total_n_frames = tdraw_my.df[ego_start_row, :n_frames_in_dataset];
        # Compute the ending row of the ego vehicle in tdraw_my
        ego_end_row = ego_start_row + total_n_frames - 1; # df is organized by clustering same vehicle together.
        # Get the lane  of the ego
        ego_laneids = tdraw_my.df[ego_start_row:ego_end_row, :lane];
        # Get the current lane id
        ego_current_laneid = ego_laneids[1];
        # Let us get the starting frame of the ego
        ego_frame = tdraw_my.df[ ego_start_row, :frame ]
        # Also, we include an immediately last lane for checking aborted lane-changing
        ego_immlast_laneid = ego_current_laneid;
        # Let us assume the last lane change happens more than 2s ago.
        ego_lastchange_frame = ego_frame - 21;
        # Get the rightmost lane of the ego on the road
        ego_rightmost_laneid = maximum(ego_laneids);
        # Get the class of the ego
        # We use trajdata_my which is loaded by using MyNGSIM
        ego_class = trajdata_my.defs[egoid].class;
        # We also loaded the AgentClass module by using MyNGSIM. AgentClass has MOTORCYCLE, CAR, TRUCK, and PEDESTRIAN.
        # We only want to investigate CAR in lane 1--5 (the main road)
        if ego_class == AgentClass.CAR && ego_rightmost_laneid <= 5
            # Check the lane id frame by frame.
            for (i, laneid) in enumerate(ego_laneids)
                # If the lane id is not the same as the ego_old_laneid,
                if laneid != ego_current_laneid
                    # Get the frame in which the lane change happens.
                    ego_frame = tdraw_my.df[ ego_start_row + i - 1 ,:frame ]
                    # We check if this is aborting the onging lane-changing maneuver
                    # by checking if the ego goes back to the immediately last lane within 2s.
                    if laneid == ego_immlast_laneid && ego_frame - ego_lastchange_frame <= 20
                        n_aborted += 1;
                        n_lanechange -= 1;
                        println(("ego id", egoid, "abort change at frame", ego_frame));
                        # If the ego is not recorded yet, we record it for the first time
                        if !haskey(aborted_lanechanges, egoid)
                            aborted_lanechanges[egoid] = [ego_frame]
                        # else, the ego is already recorded, we add a new frame instance.
                        else
                            push!(aborted_lanechanges[egoid], ego_frame)
                        end
                    else
                        println(("ego id", egoid, "change at frame", ego_frame));
                        n_lanechange += 1
                        # The current lane becomes immediately last lane.
                        ego_immlast_laneid = ego_current_laneid;
                        # We also record when this happens
                        ego_lastchange_frame = ego_frame;
                    end
                    # Update the current lane id
                    ego_current_laneid = laneid;
                # Else: the lane id is always the same
            elseif laneid == ego_laneids[1]
                    # If it is the last recorded frame the ego,
                    if i == length(ego_laneids)
                        n_lanekeep += 1;
                    end
                end
            end
        end
    end
    # Print the total number of lane changes
    println(("total number of lane-changing", n_lanechange));
    println(("total number of cars always lane-keeping", n_car_lanekeep));
    println(("total number of aborted lane change", n_aborted));
    aborted_lanechanges
end

# """
# Get the number of lane changes in lane 1--5 due to slow lead vehicle
# """
function lanechanges_dueto_slowlead()
    # Start a timer
    tic();
    # Record the following numbers
    n_searched = 0
    n_triggers = 0
    n_lanechanges = 0
    n_lanekeeps = 0
    n_abortedchanges = 0
    # Assign constant parameters.
    SAMPLE_DURATION = 5
    EVENT_SPEPARATION = 50
    GOBACK_INTERVAL = 20
    LANECHANGE_SEARCH_INTERVEL = 50
    #For a vehicle in all the vehicles:
    for egoid in keys(tdraw_my.car2start)  # tdraw_my is loaded by using MyNGSIM
        # Get the starting row of the ego vehicle in tdraw_my
        ego_start_row = tdraw_my.car2start[egoid];
        # Get the total number of frames of the ego vehicle in tdraw_my
        total_n_frames = tdraw_my.df[ego_start_row, :n_frames_in_dataset];
        # Compute the ending row of the ego vehicle in tdraw_my
        ego_end_row = ego_start_row + total_n_frames - 1; # df is organized by clustering same vehicle together.
        # Get the lanes of the ego
        ego_laneids = tdraw_my.df[ego_start_row:ego_end_row, :lane];
        # Get the rightmost lane of the ego on the road
        ego_rightmost_laneid = maximum(ego_laneids);
        # Get the class of the ego
        # We use trajdata_my which is loaded by using MyNGSIM
        ego_class = trajdata_my.defs[egoid].class;
        # We also loaded the AgentClass module by using MyNGSIM. AgentClass has MOTORCYCLE, CAR, TRUCK, and PEDESTRIAN.
        # We only want to investigate CAR or TRUCK in lane 1--5 (the main road)
        if ego_class == AgentClass.CAR && ego_rightmost_laneid <= 5
            # Add a margin to the appearing duration of the ego.
            ego_start_row_m = ego_start_row + VEHICLE_IN_VIEW_MARGIN;
            ego_end_row_m = ego_end_row - VEHICLE_IN_VIEW_MARGIN;
            # Define an row index for the ego.
            ego_row_ind = ego_start_row_m;
            # Define the immediately last event frame for measuring event-separation.
            # Only this measure is greater than EVENT_SPEPARATION, can we declare a new event.
            ego_lastevent_frame = tdraw_my.df[ego_start_row_m, :frame]
            # While (this vehicle is in view):
            while ego_row_ind <= ego_end_row_m
                # Record frame searched
                n_searched += 1;
                # Move to the next data frame (the next row)
                ego_row_ind += 1;
                # Retrieve the frame in which the ego appears.
                ego_frame = tdraw_my.df[ego_row_ind, :frame];
                # Since the lane-changing intention is triggered by a slow lead vehicle,
                # we want to define this event (slow lead vehicle).
                # The event is 1) the lead vehicle (veh 1) is slower than the ego, and
                #              2) the adjacent lead vehicle (veh 2, 2') is faster than the lead.
                # Let us get the average speed of the ego over 3 frames up to the current frame.
                ego_traffic_info = traffic_info_NGSIM(trajdata_my, ROADWAY_my, egoid,
                                                ego_frame,
                                                longitudinal_direc="self",
                                                lateral_direc = "middle",
                                                sample_duration = SAMPLE_DURATION,
                                                aggregate_method = "mean" );
                ego_speed = ego_traffic_info[:speed];
                # Let us get the average speed of veh 1 over 3 frames up to the current frame.
                veh1_traffic_info = traffic_info_NGSIM(trajdata_my, ROADWAY_my, egoid,
                                                ego_frame,
                                                longitudinal_direc="fore",
                                                lateral_direc = "middle",
                                                sample_duration = SAMPLE_DURATION,
                                                aggregate_method = "mean" );
                veh1_speed = veh1_traffic_info[:speed];
                # Let us get the average speed of veh 2 over 3 frames up to the current frame.
                # First, check if the ego is in the leftmost lane,
                ego_current_laneid = tdraw_my.df[ego_row_ind, :lane]
                if ego_current_laneid == 1      # In orignial NGSIM, lane 1 is the leftmost lane.
                    veh2_speed = 0.0;
                # if the ego is not in lane 1, then get the neighbor's speed.
                else
                    veh2_traffic_info = traffic_info_NGSIM(trajdata_my, ROADWAY_my, egoid,
                                                    ego_frame,
                                                    longitudinal_direc="fore",
                                                    lateral_direc = "left",
                                                    sample_duration = SAMPLE_DURATION,
                                                    aggregate_method = "mean" );
                    veh2_speed = veh2_traffic_info[:speed];
                end
                # Let us get the average speed of veh 22 over 3 frames up to the current frame.
                # First, check if the ego is in the rightmost lane,
                if ego_current_laneid == 5      # In orignial NGSIM, lane 5 is the rightmost lane.
                    veh22_speed = 0.0;
                else
                    # if the ego is not in lane 5, then get the neighbor's speed.
                    veh22_traffic_info = traffic_info_NGSIM(trajdata_my, ROADWAY_my, egoid,
                                                    ego_frame,
                                                    longitudinal_direc="fore",
                                                    lateral_direc = "right",
                                                    sample_duration = SAMPLE_DURATION,
                                                    aggregate_method = "mean" );
                    veh22_speed = veh22_traffic_info[:speed];
                end
                # Here we model the event-triggering with the if-statement.
                # If (veh 1 is slower than veh 0) and (veh 2 or veh 22 is faster than veh 1):
                if ego_speed - veh1_speed >= SPEED_DIFF_THRESHOLD && ( veh2_speed - veh1_speed >= SPEED_DIFF_THRESHOLD || veh22_speed - veh1_speed >= SPEED_DIFF_THRESHOLD )
                    # Only if this the triggering frame is far enough from the previous event-triggering frame, can
                    # we claim a new event.
                    # If the separation is enough, we have a new event.
                    if ego_frame - ego_lastevent_frame > EVENT_SPEPARATION
                        # Add one event
                        n_triggers += 1
                        # We want to search in the next period of time if a lane-change happens.
                        for i = 0:LANECHANGE_SEARCH_INTERVEL
                            # Check if the current lane id is the same as the immdiately previous lane id.
                            ego_searching_ind =  ego_row_ind + i;
                            if tdraw_my.df[ego_searching_ind, :lane] != ego_current_laneid  # this means a lane change
                                # Further if the current lane id is the same as the to the previous lane ids in 2s,
                                # we know this is an aborted lane change.
                                goingback = false
                                for j = 1:GOBACK_INTERVAL
                                    ego_lookback_ind = ego_searching_ind -j;
                                    goingback = goingback || (tdraw_my.df[ego_searching_ind, :lane] == tdraw_my.df[ego_lookback_ind, :lane])
                                end
                                if goingback    # this means an aborted lane change
                                    n_abortedchanges += 1
                                    n_lanechanges -= 1
                                else            # this means this is a lane change
                                    n_lanechanges += 1
                                end
                                # We break the for-loop (for i = 0:LANECHANGE_SEARCH_INTERVEL).
                                break;
                            else  # this means the ego stays in the lane
                                    n_lanekeeps += 1;
                            end
                        end
                    end
                    # We update the event triggering frame.
                    ego_lastevent_frame = ego_frame;
                end
            end
        end
    end
    # test code {
    # Print the total number of lane changes
    println(("total searched frames:", n_searched));
    println(("total triggering frames:", n_triggers));
    println(("total number of lane-changing", n_lanechanges));
    println(("total number of lane-keeping", n_lanekeeps));
    println(("total number of aborted lane change", n_abortedchanges));
    # test code }
    # Print the elapsed time.
    toc();
end


# ------------------------------------------------------------------------------
# """
# Extract the epoch after a lane-changing intention is triggered by a slow leading
# vehicle
# """
function extract_epoch()
    # Start a timer
    tic();
    # test code {
    total_triggers = 0
    total_lanechanges = 0
    total_lanekeeps = 0
    # } test code
    #For a vehicle in all the vehicles:
    for egoid in keys(tdraw_my.car2start)  # tdraw_my is loaded by using MyNGSIM
        # Get the starting row of the ego vehicle in tdraw_my
        ego_start_row = tdraw_my.car2start[egoid];
        # Get the total number of frames of the ego vehicle in tdraw_my
        total_n_frames = tdraw_my.df[ego_start_row, :n_frames_in_dataset];
        # Compute the ending row of the ego vehicle in tdraw_my
        ego_end_row = ego_start_row + total_n_frames - 1; # df is organized by clustering same vehicle together.
        # Get the lane  of the ego
        ego_laneids = tdraw_my.df[ego_start_row:ego_end_row, :lane];
        # Get the rightmost lane of the ego on the road
        ego_rightmost_laneid = maximum(ego_laneids);
        # Get the class of the ego
        # We use trajdata_my which is loaded by using MyNGSIM
        ego_class = trajdata_my.defs[egoid].class;
        # We also loaded the AgentClass module by using MyNGSIM. AgentClass has MOTORCYCLE, CAR, TRUCK, and PEDESTRIAN.
        # We only want to investigate CAR or TRUCK in lane 1--5 (the main road)
        if ego_class == AgentClass.CAR && ego_rightmost_laneid <= 5
            # Add a margin to the appearing duration of the ego.
            ego_start_row_m = ego_start_row + VEHICLE_IN_VIEW_MARGIN;
            ego_end_row_m = ego_end_row - VEHICLE_IN_VIEW_MARGIN;
            # Define an row index for the ego.
            ego_row_ind = ego_start_row_m;
            # While (this vehicle is in view):
            while ego_row_ind <= ego_end_row_m
                # Move to the next data frame (the next row)
                ego_row_ind += 1;
                # Retrieve the frame in which the ego appears.
                ego_frame = tdraw_my.df[ego_row_ind, :frame];
                # Since the lane-changing intention is triggered by a slow lead vehicle,
                # we want to define this event (slow lead vehicle).
                # The event is 1) the lead vehicle (veh 1) is slower than the ego, and
                #              2) the adjacent lead vehicle (veh 2, 2') is faster than the lead.
                # Let us get the average speed of the ego over 3 frames up to the current frame.
                ego_traffic_info = traffic_info_NGSIM(trajdata_my, ROADWAY_my, egoid,
                                                ego_frame,
                                                longitudinal_direc="self",
                                                lateral_direc = "middle",
                                                sample_duration = 3,
                                                aggregate_method = "mean" );
                ego_speed = ego_traffic_info[:speed];
                # Let us get the average speed of veh 1 over 3 frames up to the current frame.
                veh1_traffic_info = traffic_info_NGSIM(trajdata_my, ROADWAY_my, egoid,
                                                ego_frame,
                                                longitudinal_direc="fore",
                                                lateral_direc = "middle",
                                                sample_duration = 3,
                                                aggregate_method = "mean" );
                veh1_speed = veh1_traffic_info[:speed];
                # Let us get the average speed of veh 2 over 3 frames up to the current frame.
                # First, check if the ego is in the leftmost lane,
                ego_current_laneid = tdraw_my.df[ego_row_ind, :lane]
                if ego_current_laneid == 1      # In orignial NGSIM, lane 1 is the leftmost lane.
                    veh2_speed = 0.0;
                # if the ego is not in lane 1, then get the neighbor's speed.
                else
                    veh2_traffic_info = traffic_info_NGSIM(trajdata_my, ROADWAY_my, egoid,
                                                    ego_frame,
                                                    longitudinal_direc="fore",
                                                    lateral_direc = "left",
                                                    sample_duration = 3,
                                                    aggregate_method = "mean" );
                    veh2_speed = veh2_traffic_info[:speed];
                end
                # Let us get the average speed of veh 22 over 3 frames up to the current frame.
                # First, check if the ego is in the rightmost lane,
                if ego_current_laneid == 5      # In orignial NGSIM, lane 5 is the rightmost lane.
                    veh22_speed = 0.0;
                else
                    # if the ego is not in lane 5, then get the neighbor's speed.
                    veh22_traffic_info = traffic_info_NGSIM(trajdata_my, ROADWAY_my, egoid,
                                                    ego_frame,
                                                    longitudinal_direc="fore",
                                                    lateral_direc = "right",
                                                    sample_duration = 3,
                                                    aggregate_method = "mean" );
                    veh22_speed = veh22_traffic_info[:speed];
                end
                # Here we model the event-triggering with the if-statement.
                # If (veh 1 is slower than veh 0) and (veh 2 or veh 22 is faster than veh 1):
                if ego_speed - veh1_speed >= SPEED_DIFF_THRESHOLD && ( veh2_speed - veh1_speed >= SPEED_DIFF_THRESHOLD || veh22_speed - veh1_speed >= SPEED_DIFF_THRESHOLD )
                    # test code {
                    total_triggers += 1
                    #} test code
                    # The current lane may become obsolete, since the ego may change lanes before slow down.
                    ego_old_laneid = ego_current_laneid;
                    # While (veh 0 is in the current lane):
                    while ego_current_laneid == ego_old_laneid
                        # If (|speed of veh 1 - speed of veh 0| )
                        if ego_speed - veh1_speed <= (SPEED_DIFF_THRESHOLD / 2)
                            # We make sure we the row indices do not exceed ego_end_row.
                            if ego_row_ind + 50 < ego_end_row
                                ego_rowind_after = ego_row_ind + 50;
                            else
                                ego_rowind_after = ego_end_row;
                            end
                            ego_laneids_after = tdraw_my.df[ego_row_ind:ego_rowind_after, :lane];
                            # Check if the ego made a lane change in the trajectory after matching speed.
                            for (i, laneid) in enumerate(ego_laneids_after)
                                # If the lane id is not the same as the ego_old_laneid,
                                if laneid != ego_old_laneid
                                    # the ego made a lane change at i-th frame after the matching speed.
                                    # test code {
                                    println(("change at", i));
                                    cases += 1
                                    # }test code
                                    # Break the for-loop
                                    break;
                                # Else the lane id is the same as the ego_old_laneid
                                else
                                    # If all the trajectory is examed,
                                    if i == length(ego_laneids_after)
                                        # the ego
                                        # test code {
                                        # println(("lane keep"))
                                        # }test code
                                    end
                                end


                            end
                            # Extract the data frames of the next 5s;
                            # if ego_row_ind + 50 < ego_end_row
                            # else
                            # end
                            # Extract the data frames of the preceding 5s;
                            # Move to the data frame after 5s;
                            ego_row_ind += 50
                            # Break;
                            break; # break while-loop (ego_current_laneid == ego_old_laneid)
                        #  Else:
                        else
                            # Move to the next data frame;
                            ego_row_ind += 1;
                            # If this the row index goes beyond the end row with margin
                            if ego_row_ind > ego_end_row_m
                                break; # break while-loop (ego_current_laneid == ego_old_laneid)
                            # Else:
                            else
                                # Get new current lane id
                                ego_current_laneid = tdraw_my.df[ego_row_ind, :lane];
                                # Get new speeds
                                # Retrieve the frame in which the ego appears.
                                ego_frame = tdraw_my.df[ego_row_ind, :frame];
                                # Let us get the average speed of the ego over 3 frames up to the current frame.
                                ego_traffic_info = traffic_info_NGSIM(trajdata_my, ROADWAY_my, egoid,
                                                                ego_frame,
                                                                longitudinal_direc="self",
                                                                lateral_direc = "middle",
                                                                sample_duration = 3,
                                                                aggregate_method = "mean" );
                                ego_speed = ego_traffic_info[:speed];
                                # Let us get the average speed of veh 1 over 3 frames up to the current frame.
                                veh1_traffic_info = traffic_info_NGSIM(trajdata_my, ROADWAY_my, egoid,
                                                                ego_frame,
                                                                longitudinal_direc="fore",
                                                                lateral_direc = "middle",
                                                                sample_duration = 3,
                                                                aggregate_method = "mean" );
                                veh1_speed = veh1_traffic_info[:speed];
                            end
                        end
                    end
                end
            end
        end
    end
    # test code {
    println(cases)
    # test code }
    # Print the elapsed time.
    toc();
end

lanechanges_dueto_slowlead()
