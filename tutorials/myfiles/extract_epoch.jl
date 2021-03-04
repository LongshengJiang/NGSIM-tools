# The labeling of vehicles is in the following diagram.
#-------------------------------------------------------------------------------
#             ==> veh 3                                  ==> veh 2
#-------------------------------------------------------------------------------
#  ==> veh 4                ==> veh 0 (ego)               ==> veh 1
#-------------------------------------------------------------------------------
#          ==> veh 32                                  ==> veh 22
#-------------------------------------------------------------------------------


# To run this program, the prerequsite modules are
# using MyNGSIM1
using JLD
using Reel
using AutoViz
using Plots

#
# The information after a triggering event.
mutable struct InfoAfterTrigger
        egoid::Int64                    # The id of the ego vehicle
        triggerframe::Int64             # The frame ath which event-triggering happens
        trigger_ind::Int64              # The n-th trigger
        triggered::Bool                 # If the event is at least triggered once
        laneids::Array{Int64, 1}        # The list of lane ids after the triggering.
        ego_v::Array{Float64, 1}        # The list of speed of the ego vehicle
        fore_v::Array{Float64, 1}       # The list of speed of the front vehicle
        leadids::Array{Int64, 1}        # The list of ids of the lead vehicles
        duration::Int64                 # The duration after event triggering
        InfoAfterTrigger(;egoid::Int64 = 0,
                        triggerframe::Int64 = 0,
                        trigger_ind::Int64 = 0,
                        triggered::Bool = false,
                        laneids::Array{Int64, 1} = Array{Int64, 1}(),
                        ego_v::Array{Float64, 1} = Array{Float64, 1}(),
                        fore_v::Array{Float64, 1} = Array{Float64, 1}(),
                        leadids::Array{Int64, 1} = Array{Int64, 1}(),
                        duration::Int64 = 50) = new(egoid, triggerframe, trigger_ind, triggered, laneids, ego_v, fore_v, leadids, duration)
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

function get_neighbor_id_from_scene(scene::Records.Frame{Records.Entity{AutomotiveDrivingModels.VehicleState,AutomotiveDrivingModels.VehicleDef,Int64}},
                                    neighbor::AutomotiveDrivingModels.NeighborLongitudinalResult,
                                    alternative_id::Int = 0)
    # If there is a neighbor in the scene, we get its id in NGSIM.
    if neighbor.ind != 0
        neighbor_id = scene[neighbor.ind].id;
    # If there is no neighbor, we assume the id is 0.
    else
        neighbor_id = alternative_id;
    end
    neighbor_id
end

# a_sample_neighbor_v_Δs_def is a subfunction of samples_neighbor_v_Δs_def
function a_sample_neighbor_v_Δs_def!( speed_vec::Vector{Float64},
                                      distance_vec::Vector{Float64},
                                      scene::Records.Frame{Records.Entity{AutomotiveDrivingModels.VehicleState,AutomotiveDrivingModels.VehicleDef,Int64}},
                                      neighbor::AutomotiveDrivingModels.NeighborLongitudinalResult,
                                      original_neighborid::Int)

    # Get the neighbor's id. The function get_neighbor_id_from_scene is defined in this file.
    neighbor_id = get_neighbor_id_from_scene(scene, neighbor);
    # If the neighbor is the original neighbor
    if neighbor_id == original_neighborid
        # We retrieve the distance between the neighbor and the ego and the speed of the neighbor.
        push!(distance_vec, neighbor.Δs);
        push!(speed_vec, get_neighbor_speed(scene, neighbor));
    # If the neighbor is not the original neighbor, we copy the distance and speed of the original neighbor.
    else
        push!(distance_vec, distance_vec[end]);
        push!(speed_vec, speed_vec[end]);
    end

    # The vehicle definition.
    if neighbor.ind != 0
        vehicle_def = scene[neighbor.ind].def;
    else
        vehicle_def = VehicleDef(0, 0, 0);
    end
    # We output the following variables
    (speed_vec, distance_vec, vehicle_def)
end


function samples_neighbor_v_Δs_def(get_neighbor_in_scene::Function,
                                   trajdata::Records.ListRecord{AutomotiveDrivingModels.VehicleState,AutomotiveDrivingModels.VehicleDef,Int64},
                                   roadway::AutomotiveDrivingModels.Roadway,
                                   carid::Int,
                                   cameraframe::Int;
                                   sample_duration::Int = 1)
    # Define an empty speed cache and distance cache here
    speed_vec = Vector{Float64}()
    distance_vec = Vector{Float64}()
    # Record the type and dimension of the vehicle.
    vehicle_def = AutomotiveDrivingModels.VehicleDef()

    # We first want to record the information of the neighbor right at cameraframe.
    scene = get!(Scene(500), trajdata, cameraframe);
    neighbor = get_neighbor_in_scene(scene, carid, roadway, 50.0);  # 50.0 is the maximum search distance.
    # We want to get this neigbor's id.
    original_neighbor_id = get_neighbor_id_from_scene(scene, neighbor);
    # We get new samples of speed and distance.
    (speed_vec, distance_vec, vehicle_def) = a_sample_neighbor_v_Δs_def!(speed_vec, distance_vec, scene, neighbor, original_neighbor_id);
    # If the sample duration is longer than 1, we keep recording.
    if sample_duration > 1
        for i = 1:(sample_duration)
            # Retrieve a neighbor from the scene.
            scene = get!(Scene(500), trajdata, cameraframe - i);
            neighbor = get_neighbor_in_scene(scene, carid, roadway, 50.0);  # 50.0 is the maximum search distance.
            # We get new samples of speed and distance.
            (speed_vec, distance_vec, vehicle_def) = a_sample_neighbor_v_Δs_def!(speed_vec, distance_vec, scene, neighbor, original_neighbor_id);
        end
    end
    (speed_vec, distance_vec, vehicle_def, original_neighbor_id)
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
    traffic_info = Dict{Symbol, Union{Float64, Int, VehicleDef}}()
    # Initialize a container for storing the relative heading of the ego.
    ego_rel_heading_vec = Vector{Float64}();
    # If the vehicle to be observed is the ego self,
    if longitudinal_direc == "self"
        # Define a speed cache and a distance chance here
        speed_vec = Vector{Float64}()
        distance_vec = Vector{Float64}()
        # This record the type and dimension of the vehicle.
        vehicle_def = AutomotiveDrivingModels.VehicleDef()
        vehicle_id = carid
        for i = 1:sample_duration
            scene = get!(Scene(500), trajdata, cameraframe + 1 - i);
            # We get the index of the vehicle (carid) in the scene
            ego_ind = get_scene_id(scene, carid)
            # Let the distance to self be 0
            push!(distance_vec, 0);
            # We get the velocity of the ego vehicle in unit [m/s].
            push!(speed_vec, scene[ego_ind].state.v);
            # We get the relative heading of the ego in unit rad.
            push!(ego_rel_heading_vec, scene[ego_ind].state.posF.ϕ);
            # The vehicle definition and id
            if i == 1
                vehicle_def = scene[ego_ind].def
            end
        end
    # If the vehicle to be observed is in front,
    elseif longitudinal_direc == "fore"
        # further if the vehicle to be observed shares the lane with the ego
        if lateral_direc == "middle"
            # Get samples of speed, distance, and vehicle type of the neighbor in this direction.
            (speed_vec, distance_vec, vehicle_def, vehicle_id) = samples_neighbor_v_Δs_def(get_neighbor_fore_along_lane_NGSIM, trajdata, roadway, carid, cameraframe, sample_duration = sample_duration);

        # If the vehicle to be observed is in the left,
        elseif lateral_direc == "left"
            # Get samples of speed, distance, and vehicle type of the neighbor in this direction.
            (speed_vec, distance_vec, vehicle_def, vehicle_id) = samples_neighbor_v_Δs_def(get_neighbor_fore_along_left_lane_NGSIM, trajdata, roadway, carid, cameraframe, sample_duration = sample_duration);

        # If the vehicle to be observed is on right,
        elseif lateral_direc == "right"
            # Get samples of speed, distance, and vehicle type of the neighbor in this direction.
            (speed_vec, distance_vec, vehicle_def, vehicle_id) = samples_neighbor_v_Δs_def(get_neighbor_fore_along_right_lane_NGSIM, trajdata, roadway, carid, cameraframe, sample_duration = sample_duration);

        else
            error("When getting front neighbor, the lateral direction is not valid.")
        end
    # If the vehicle to be observed is at rear,
    elseif longitudinal_direc == "rear"
        # further if the vehicle to be observed is in the middle
        if lateral_direc == "middle"
            # Get samples of speed, distance, and vehicle type of the neighbor in this direction.
            (speed_vec, distance_vec, vehicle_def, vehicle_id) = samples_neighbor_v_Δs_def(get_neighbor_rear_along_lane_NGSIM, trajdata, roadway, carid, cameraframe, sample_duration = sample_duration);

        # If the vehicle to be observed is on left
        elseif lateral_direc == "left"
            # Get samples of speed, distance, and vehicle type of the neighbor in this direction.
            (speed_vec, distance_vec, vehicle_def, vehicle_id) = samples_neighbor_v_Δs_def(get_neighbor_rear_along_left_lane_NGSIM, trajdata, roadway, carid, cameraframe, sample_duration = sample_duration);

        # If the vehicle to be observed is on the right,
        elseif lateral_direc == "right"
            # Get samples of speed, distance, and vehicle type of the neighbor in this direction.
            (speed_vec, distance_vec, vehicle_def, vehicle_id) = samples_neighbor_v_Δs_def(get_neighbor_rear_along_right_lane_NGSIM, trajdata, roadway, carid, cameraframe, sample_duration = sample_duration);

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
        # If the vehicle to be observed is the ego self, we get its relative heading.
        if longitudinal_direc == "self"
            ego_rel_heading = minimum(ego_rel_heading_vec)
        else    # Otherwise, we simply let the ego_rel_heading be 0.
            ego_rel_heading = 0.0
        end
    else
        # Getting mean values
        speed = mean(speed_vec)
        distance = mean(distance_vec)
        # If the vehicle to be observed is the ego self, we get its relative heading.
        if longitudinal_direc == "self"
            ego_rel_heading = mean(ego_rel_heading_vec)
        else    # Otherwise, we simply let the ego_rel_heading be 0.
            ego_rel_heading = 0.0
        end
    end
    traffic_info[:speed] = speed
    traffic_info[:distance] = distance
    traffic_info[:vehicle_def] = vehicle_def
    traffic_info[:vehicle_id] = vehicle_id
    traffic_info[:ego_rel_heading] = ego_rel_heading
    traffic_info
end # function end


function add_lanedecision_cases!(lanedecision_cases::Dict{Int64,Array{Int64,1}}, egoid::Int, triggerframe::Int)::Dict{Int64,Array{Int64,1}}
    # If the ego is not recorded yet, we record it for the first time
    if !haskey(lanedecision_cases, egoid)
        lanedecision_cases[egoid] = [triggerframe];
    # else, if the ego is already recorded and if the frame is not added, we add the new frame instance.
elseif lanedecision_cases[egoid][end] != triggerframe
        push!(lanedecision_cases[egoid], triggerframe);
    else    # Otherwise, we don't do anything.
        nothing;
    end
    lanedecision_cases
end

# """
# Determine lane change or lane keep
# """
function lane_change_or_keep!(lanedecision_cases::Tuple{Dict{Int64,Array{Int64,1}},Dict{Int64,Array{Int64,1}},Dict{Int64,Array{Int64,1}}},
                              info_after_trigger::InfoAfterTrigger)
    # Initialize some counters
    Δn_lanechanges = 0
    Δn_lanekeeps = 0
    Δn_abortedchanges = 0
    Δn_notchange = 0
    Δn_slow_lead = 0
    # Unpack the lane decision cases.
    (lanechange_cases, lanekeep_cases, abortchange_cases) = lanedecision_cases;

    # We determine if a lane change happens. A lane change is charaterized by
    # 1) the ego moved to a new lane
    # 2) the ego did not go back to the original lane quickly (abort the lane change)
    #
    # Scan the lane ids to see if they are the same.
    if !isempty(info_after_trigger.laneids)
        aftrg_laneids_len = length(info_after_trigger.laneids);
        aftrg_current_laneid =  info_after_trigger.laneids[1];
        for i = 2:aftrg_laneids_len
            # If we find the lane id is not the same as the first one, that may be a lane change.
            if info_after_trigger.laneids[i] != aftrg_current_laneid
                # We further check if this lane change is aborted
                goingback = false;
                for j = i:minimum([i + 20, aftrg_laneids_len])
                    # If the vehicle goes back to the previous lane, that is an aborted change.
                    goingback = goingback || (info_after_trigger.laneids[j] == aftrg_current_laneid);
                end
                # If the ego goes back to the current lane, the lane change is aborted.
                if goingback
                    # Add one lane-change abortion
                    old_abortchange_cases = deepcopy(abortchange_cases);
                    abortchange_cases = add_lanedecision_cases!(abortchange_cases, info_after_trigger.egoid, info_after_trigger.triggerframe);
                    # If the abortchange_cases is updated, we count one more.
                    if abortchange_cases != old_abortchange_cases
                        Δn_abortedchanges += 1;
                    end
                else
                    # Add one lane-change
                    Δn_lanechanges += 1
                    lanechange_cases = add_lanedecision_cases!(lanechange_cases, info_after_trigger.egoid, info_after_trigger.triggerframe);
                    # Break the loop for (i = 2:aftrg_laneids_len)
                    break
                end
            end
        end
        #
        # We determine if a lane-keeping happens. A lane-keeping is charaterized by
        # 1) The ego did not change lane and the lead is the same.
        # 2) The ego matched the speed of the slow lead vehicle.
        # It is possible that the lead vehicle may speed up quickly, hence the ego
        # aborts the lane change quickly. We do NOT consider this case.
        #
        changinglane = false;
        samelead = true;
        for i = 2:aftrg_laneids_len
            changinglane = changinglane || (info_after_trigger.laneids[i] != info_after_trigger.laneids[1]);
            samelead = samelead && (info_after_trigger.leadids[i] == info_after_trigger.leadids[1]);
        end

        # If the ego did not change lane and if the lead is the same
        if !changinglane && samelead
            Δn_notchange += 1;
            # If the lead vehicle is slowing down,
            # We want to do some averaging.
            avg_range = maximum([1, length(info_after_trigger.fore_v) ÷ 10]);
            fore_v_head = mean(info_after_trigger.fore_v[1:avg_range]);
            fore_v_end = mean(info_after_trigger.fore_v[(end+1-avg_range):end]);
            if fore_v_head - fore_v_end > SPEED_DIFF_THRESHOLD
                Δn_slow_lead += 1;
                # If the relative speed of the lead to the ego is small enough, it is a lane-keeping.
                rel_speed_fore = info_after_trigger.fore_v - info_after_trigger.ego_v;
                if minimum( abs.(rel_speed_fore) ) <= (SPEED_DIFF_THRESHOLD / 2)
                    Δn_lanekeeps += 1;
                    lanekeep_cases = add_lanedecision_cases!(lanekeep_cases, info_after_trigger.egoid, info_after_trigger.triggerframe);

                end
            end
        end
        Δn_lanedecisions= [Δn_lanechanges, Δn_lanekeeps, Δn_abortedchanges, Δn_notchange, Δn_slow_lead]
        # Pack the lane decisions before return.
        lanedecision_cases = (lanechange_cases, lanekeep_cases, abortchange_cases);
        return (lanedecision_cases, Δn_lanedecisions)
    else
        error("The laneids vector is empty.");
    end
end

function ask_use_which_dataset()::String
    # Get to know which dataset is used.
    println("Which is the dataset used? 1 = MyNGSIM1, 2 = MyNGSIM2, or 3 = MyNGSIM3");
    # Let us get user input.
    myngsim = "";
    while true
        myngsim_number = readline();
        if myngsim_number == "1"
            myngsim = "i101_0750_0805";
            break
        elseif myngsim_number == "2"
            myngsim = "i101_0805_0820";
            break
        elseif myngsim_number == "3"
            myngsim = "i101_0820_0835";
            break
        else
            println("Please use integer: 1, 2, or 3.")
        end
    end
    myngsim
end

# """
# Get the number of lane changes in lane 1--5 due to slow lead vehicle
# """
function lanedecisions_dueto_slowlead()
    # We first ask the use to specify which dataset is used.
    myngsim = ask_use_which_dataset()

    # From here we start the real meat of this function.
    # Start a timer
    tic();
    # Record the following numbers
    n_searched = 0
    n_triggers = 0
    n_lanechanges = 0
    n_lanekeeps = 0
    n_notchange = 0
    n_slow_lead = 0
    n_abortedchanges = 0
    n_laneids_notempty = 0
    # Assign constant parameters.
    SAMPLE_DURATION = 1
    EVENT_SPEPARATION = 50
    LANEDECISION_DURATION = 50
    LOOK_AHEAD = 20
    # Initialize the containers for storing the lane-changing and lane-keeping and aborted-changing cases.
    lanechange_cases = Dict{Int, Vector{Int}}();    # Dict: egoid ==> [triggerframe1, triggerframe2, ...]
    lanekeep_cases = Dict{Int, Vector{Int}}();
    abortchange_cases = Dict{Int, Vector{Int}}();
    # Pack these variables up.
    lanedecision_cases = (lanechange_cases, lanekeep_cases, abortchange_cases);

    #For a vehicle in all the vehicles:
    for egoid in keys(tdraw_my.car2start)  # tdraw_my is loaded by using MyNGSIM
        # Define info_after_trigger to contain the related information after an event-triggering
        # InfoAfterTrigger is defined in this file
        info_after_trigger = InfoAfterTrigger(egoid = egoid, duration = LANEDECISION_DURATION);
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
            while ego_row_ind < ego_end_row_m
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
                # Let us get the average speed of the ego over some frames up to the current frame.
                ego_traffic_info = traffic_info_NGSIM(trajdata_my, ROADWAY_my, egoid, ego_frame,
                                                      longitudinal_direc="self",
                                                      lateral_direc = "middle", sample_duration = SAMPLE_DURATION,
                                                      aggregate_method = "mean" );
                ego_speed = ego_traffic_info[:speed];
                # Let us get the average speed of veh 1 over 3 frames up to the current frame.
                veh1_traffic_info = traffic_info_NGSIM(trajdata_my, ROADWAY_my, egoid, ego_frame,
                                                       longitudinal_direc="fore",
                                                       lateral_direc = "middle", sample_duration = SAMPLE_DURATION,
                                                       aggregate_method = "mean" );
                veh1_speed = veh1_traffic_info[:speed];
                veh1_id = veh1_traffic_info[:vehicle_id];
                # Let us get the average speed of veh 2 over some frames up to the current frame.
                # First, check if the ego is in the leftmost lane,
                ego_current_laneid = tdraw_my.df[ego_row_ind, :lane]
                if ego_current_laneid == 1      # In orignial NGSIM, lane 1 is the leftmost lane.
                    veh2_speed = 0.0;
                # if the ego is not in lane 1, then get the neighbor's speed.
                else
                    veh2_traffic_info = traffic_info_NGSIM(trajdata_my, ROADWAY_my, egoid, ego_frame,
                                                           longitudinal_direc="fore",
                                                           lateral_direc = "left", sample_duration = SAMPLE_DURATION,
                                                           aggregate_method = "mean" );
                    veh2_speed = veh2_traffic_info[:speed];
                end
                # Let us get the average speed of veh 22 over some frames up to the current frame.
                # First, check if the ego is in the rightmost lane,
                if ego_current_laneid == 5      # In orignial NGSIM, lane 5 is the rightmost lane.
                    veh22_speed = 0.0;
                else
                    # if the ego is not in lane 5, then get the neighbor's speed.
                    veh22_traffic_info = traffic_info_NGSIM(trajdata_my, ROADWAY_my, egoid, ego_frame,
                                                            longitudinal_direc="fore",
                                                            lateral_direc = "right", sample_duration = SAMPLE_DURATION,
                                                            aggregate_method = "mean" );
                    veh22_speed = veh22_traffic_info[:speed];
                end
                # We check if the event has been triggered (n_triggers >= 1).
                # If the event has been triggered as least once, we start recording
                # the after triggering information. We make sure to at most fill the duration.
                if info_after_trigger.triggered && length(info_after_trigger.laneids) < info_after_trigger.duration
                    # Push in the current lane id.
                    push!(info_after_trigger.laneids, ego_current_laneid);
                    # Push in the current ego speed.
                    push!(info_after_trigger.ego_v, ego_speed);
                    # Push in the current lead speed.
                    push!(info_after_trigger.fore_v, veh1_speed);
                    # Push in the id of the lead vehicle
                    push!(info_after_trigger.leadids, veh1_id);
                end
                # ================================================================
                # Here we model the event-triggering as the following if-statement.
                # ================================================================
                # If (veh 1 is slower than veh 0) and (veh 2 or veh 22 is faster than veh 1):
                if ego_speed - veh1_speed >= SPEED_DIFF_THRESHOLD && ( veh2_speed - veh1_speed >= SPEED_DIFF_THRESHOLD || veh22_speed - veh1_speed >= SPEED_DIFF_THRESHOLD )
                    # Only if this the triggering frame is far enough from the previous event-triggering frame, can
                    # we claim a new event.
                    # If the separation is enough, we have a new event.
                    if ego_frame - ego_lastevent_frame > EVENT_SPEPARATION
                        n_triggers += 1;

                        # We first process the information in info_after_trigger for the previous recording
                        if !isempty(info_after_trigger.laneids)
                            n_laneids_notempty += 1;
                            # Checking if a lane change or a lane keep was made
                            (lanedecision_cases, Δn_lanedecisions) = lane_change_or_keep!(lanedecision_cases, info_after_trigger);
                            # Update the counters
                            n_lanechanges += Δn_lanedecisions[1];
                            n_lanekeeps += Δn_lanedecisions[2];
                            n_abortedchanges += Δn_lanedecisions[3];
                            n_notchange += Δn_lanedecisions[4];
                            n_slow_lead += Δn_lanedecisions[5];
                        end
                        # We then reinitialize the info_after_trigger
                        info_after_trigger = InfoAfterTrigger(egoid = egoid,
                                                              triggerframe = ego_frame,
                                                              trigger_ind = n_triggers,
                                                              triggered = true,
                                                              duration = LANEDECISION_DURATION);
                    end
                    # We update the event triggering frame whenever the speed criteria as follows are triggered.
                    # ego_speed - veh1_speed >= SPEED_DIFF_THRESHOLD && ( veh2_speed - veh1_speed >= SPEED_DIFF_THRESHOLD || veh22_speed - veh1_speed >= SPEED_DIFF_THRESHOLD )
                    ego_lastevent_frame = ego_frame;
                end

                # When the while-loop is about to end, we need to check the lane decisions from the last event trigger.
                # We run some clean up steps.
                if ego_row_ind == ego_end_row_m
                    if !isempty(info_after_trigger.laneids)
                        n_laneids_notempty += 1;
                        # Checking if a lane change or a lane keep was made
                        (lanedecision_cases, Δn_lanedecisions) = lane_change_or_keep!(lanedecision_cases, info_after_trigger);
                        # Update the counters
                        n_lanechanges += Δn_lanedecisions[1];
                        n_lanekeeps += Δn_lanedecisions[2];
                        n_abortedchanges += Δn_lanedecisions[3];
                        n_notchange += Δn_lanedecisions[4];
                        n_slow_lead += Δn_lanedecisions[5];
                    end
                end
            end
        end
    end
    # Print the total number of lane changes
    println(("total searched frames:", n_searched));
    println(("total triggering events:", n_triggers));
    println(("n_laneids_notempty", n_laneids_notempty));
    println(("total number of lane-changing", n_lanechanges));
    println(("total number of lane-keeping", n_lanekeeps));
    println(("total number of aborted lane change", n_abortedchanges));
    println(("number of slow lead:", n_slow_lead));
    println(("number of not change:", n_notchange));
    # Print the elapsed time.
    toc();
    tic();
    # A final touch on saving the lane decision cases.
    println("Save the results...");
    JLD.save("./Data/lanedecision_cases_"*myngsim*".jld", "lanedecision_cases",lanedecision_cases);
    toc();
end

# """
# We analyze in detail the extracted lane change cases.
# """
function lanechange_detailed_analysis()
    # Define the range of frames to analyze.
    AFTER_MARGIN = 50
    BEFORE_MARGIN = 50
    SAMPLE_DURATION = 1
    range_margins = (BEFORE_MARGIN, AFTER_MARGIN);

    # Loading the saved data. This requires package JLD.
    whichdataset = ask_use_which_dataset();
    filepath = "./Data/lanedecision_cases_"*whichdataset*".jld"
    (change_cases, keep_cases, abort_cases) = JLD.load(filepath, "lanedecision_cases");

    # Let us create a container to store the trajectories of the relative heading trajectories
    # of each recorded vehicle and each of its recorded frame.
    ego_rel_heading_dict = Dict{ String, Tuple{ Array{Int, 1}, Array{Float64, 1} } }();
    # The above data structure looks complex. It is
    #  ego_rel_heading_dict
    #           |_ (egoid, recorded_frame) => ( frames, relative headings)

    # We do the following analysis for each recorded vehicle,
    for egoid in keys(change_cases) # change_cases is of type Dict{Int, Vector{Int}}
        # and for each recorded frame.
        for recorded_frame in change_cases[egoid]
            # We want to skip the following cases, because the manual review found they are not proper.
            if (egoid, recorded_frame) in [(928, 2328), (704, 1956), (645, 1818), (2056, 6187)]
                nothing
            else
                # Extract the relative heading of the ego in the analyzing range.
                # Generate the analyzing range.
                (b_margin, a_margin) = proper_frame_range(range_margins, egoid, recorded_frame);
                ana_framerange = collect( recorded_frame - b_margin: recorded_frame + a_margin );
                # We want to get the relative heading of the ego for each frame in the analyzing range.
                # Initialize a container to save the retrieved ego relative heading.
                ego_rel_heading_vec = Vector{Float64}();
                # Initialize container to save relative speeds
                rel_speed_1_0_vec = Vector{Float64}();
                rel_speed_2_1_vec = Vector{Float64}();
                rel_speed_22_1_vec = Vector{Float64}();
                for ego_frame in ana_framerange
                    # Get traffic info of the ego.
                    ego_traffic_info = traffic_info_NGSIM(trajdata_my, ROADWAY_my, egoid, ego_frame,
                                                          longitudinal_direc="self",
                                                          lateral_direc = "middle", sample_duration = SAMPLE_DURATION,
                                                          aggregate_method = "mean" );
                    push!(ego_rel_heading_vec, ego_traffic_info[:ego_rel_heading]);

                    # Get the traffic info of the lead vehicle (veh 1)
                    veh1_traffic_info = traffic_info_NGSIM(trajdata_my, ROADWAY_my, egoid, ego_frame,
                                                            longitudinal_direc="fore",
                                                            lateral_direc = "middle", sample_duration = SAMPLE_DURATION,
                                                            aggregate_method = "mean" );
                    # We compute the relative speed between veh 1 and the ego
                    rel_speed_1_0 = veh1_traffic_info[:speed] - ego_traffic_info[:speed];
                    push!(rel_speed_1_0_vec, rel_speed_1_0);

                    # Get the traffic info of the left front vehicle (veh 2)
                    veh2_traffic_info = traffic_info_NGSIM(trajdata_my, ROADWAY_my, egoid, ego_frame,
                                                            longitudinal_direc="fore",
                                                            lateral_direc = "left", sample_duration = SAMPLE_DURATION,
                                                            aggregate_method = "mean" );
                    # We compute the relative speed between veh 2 and veh 1
                    rel_speed_2_1 = veh2_traffic_info[:speed] - veh1_traffic_info[:speed];
                    push!(rel_speed_2_1_vec, rel_speed_2_1);

                    # Get the traffic info of the right front vehicle (veh 22)
                    veh22_traffic_info = traffic_info_NGSIM(trajdata_my, ROADWAY_my, egoid, ego_frame,
                                                            longitudinal_direc="fore",
                                                            lateral_direc = "right", sample_duration = SAMPLE_DURATION,
                                                            aggregate_method = "mean" );
                    # We compute the relative speed between veh 22 and veh 1
                    rel_speed_22_1 = veh22_traffic_info[:speed] - ego_traffic_info[:speed];
                    push!(rel_speed_22_1_vec, rel_speed_22_1);

                end
                # Store the ego relative heading together with the corresponding frames.
                # ego_rel_heading_dict[string(egoid)*"_"*string(recorded_frame)] = ( ana_framerange, ego_rel_heading_vec );
                # plt_ϕ = plot(ana_framerange, ego_rel_heading_vec, label=["rel_heading"]);
                # plt_ϕ = plot!(plt_ϕ, [recorded_frame, recorded_frame], [-0.05, 0.05]);
                # plt_path_ϕ = "./Figure/heading/"*string(egoid)*"_"*string(recorded_frame)*"_relative_heading.png";
                # savefig(plt_ϕ, plt_path_ϕ);
                plt_Δv = plot(ana_framerange, rel_speed_1_0_vec, label=["rel_speed_1_0"]);
                # plt_Δv = plot(ana_framerange, [rel_speed_2_1_vec, rel_speed_22_1_vec], label=["rel_speed_2_1", "rel_speed_22_1"]);
                plt_Δv = plot!(plt_Δv, [recorded_frame, recorded_frame], [-1, 1]);
                plt_path_Δv = "./Figure/relative_speed/"*string(egoid)*"_"*string(recorded_frame)*"_relative_speeds.png";
                savefig(plt_Δv, plt_path_Δv);
            end
        end

    end
end

# """
# Make sure the frame range always contains the vehicle.
# """
function proper_frame_range(range_margins::Tuple{Int, Int}, carid::Int, frame::Int)::Tuple{Int, Int}
    # We make sure the frames do not exceed the frame range of the car.
    # Get the number of the preceding and remaining frames.
    car_start_row = tdraw_my.car2start[carid]
    car_start_frame = tdraw_my.df[car_start_row, :frame]
    n_prec_frames = frame - car_start_frame

    # Get the margins
    (b_margin, a_margin) = range_margins;
    # If the before_margin is greater than the number of preceding frames, replace it
    # with the number of the preceding frames.
    if b_margin > n_prec_frames
        b_margin = n_prec_frames
    end
    n_total_frames = tdraw_my.df[car_start_row, :n_frames_in_dataset]
    remain_frames = n_total_frames - n_prec_frames - 1
    # If the after_margin is larger than the remaining frames, use the remaining frames
    # as the after_margin.
    if a_margin > remain_frames
        a_margin = remain_frames
    end
    # Return the proper before_margin and after_margin.
    (b_margin, a_margin)
end

# """
# Make aninimations
# """
function make_animation()
    # Define the range of frames to analyze.
    AFTER_MARGIN = 50
    BEFORE_MARGIN = 50

    # Loading the saved data. This requires package JLD.
    whichdataset = ask_use_which_dataset();
    filepath = "./Data/lanedecision_cases_"*whichdataset*".jld";
    println("filepath: ", filepath);
    (change_cases, keep_cases, abort_cases) = JLD.load(filepath, "lanedecision_cases");
    for category_name in ["change", "keep", "abort"]
        if category_name == "change"
            making_films = change_cases;
        elseif category_name == "keep"
            making_films = keep_cases;
        elseif category_name == "abort"
            making_films = abort_cases;
        else
            error("Please use the corret category names: change, keep, or abort");
        end
        # making_films = Dict{Int, Vector{Int}}()
        # making_films[100] = [480]

        for carid in keys(making_films)
            range_margins = (BEFORE_MARGIN, AFTER_MARGIN);
            for frame in making_films[carid]
                # We make sure the car is present in the range defined by range_margin and the frame.
                (b_margin, a_margin) = proper_frame_range(range_margins, carid, frame);

                # Start to prepare each slide of the animation.
                filmframes = Frames(MIME("image/png"), fps=10);
                for i = -b_margin:a_margin
                    scene = get!(Scene(500), trajdata_my, frame + i);
                    carcolors = Dict{Int, Colorant}()
                    carcolors[carid] = colorant"green";
                    ego_ind = get_scene_id(scene, carid)
                    # We want to highlight the trajectory after the event-trigginger frame by displaying a red font color.
                    if i >= 0
                        slide = render(scene, ROADWAY_my,
                                      [NeighborsOverlay(carid),
                                      TextOverlay(text=[@sprintf("ego id: %d", carid)], font_size= 20, pos=VecE2(250,30)),
                                      TextOverlay(text=[@sprintf("frame: %d", frame+i)], color = colorant"red", font_size= 20, pos=VecE2(250,60))],
                                      cam=CarFollowCamera{Int}(carid, 8.0, 2.0/9.0*pi),
                                      car_colors=carcolors);
                    else
                        slide = render(scene, ROADWAY_my,
                                      [NeighborsOverlay(carid),
                                      TextOverlay(text=[@sprintf("ego id: %d", carid)], font_size= 20, pos=VecE2(250,30)),
                                      TextOverlay(text=[@sprintf("frame: %d", frame+i)], font_size= 20, pos=VecE2(250,60))],
                                      cam=CarFollowCamera{Int}(carid, 8.0, 2.0/9.0*pi),
                                      car_colors=carcolors);
                    end
                    push!(filmframes, slide)
                end
                filmframes;
                write("./Animation/"*category_name*"_"*string(carid)*"_"*string(frame)*".mp4", filmframes);
            end
        end
    end
end

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

function main()
    # Get to know which dataset is used.
    println("What to do?");
    println("E = Extract lane decision cases due to slow lead");
    println("A = Make animations");
    println("CA = Lane change detailed analysis");
    println("KA = Lane keep detailed analysis");

    while true
        what2do = readline();
        if what2do == "E"
            lanedecisions_dueto_slowlead();
            break
        elseif what2do == "A"
            make_animation();
            break
        elseif what2do == "CA"
            lanechange_detailed_analysis();
            break
        elseif what2do == "KA"
            lanekeep_detailed_analysis();
            break
        else
            println("Please use right keyword: E, A, CA, KA.")
        end
    end
end

# If this file is run as the main file, execute the function main defined above.
if abspath(PROGRAM_FILE) == @__FILE__
    main()
end

# make_animation();
# lanedecisions_dueto_slowlead();
lanechange_detailed_analysis()
