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
#                                    |_ sample_neighbor_Δs_and_v!
#                                    |___________________________|_get_neighbor_speed

# In this program, I define the following data structures.
#   EpochDuration
#

# The duration of an epoch
struct EpochDuration
    egoid::Int64                  # The id of the ego vehicle
    startframe::Int64             # The frame at which an epoch starts
    after_duration::Int64         # The duration after the start frame
    before_duration::Int64        # The duration before the start frame
end
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
        duration::Int64                 # The duration after event triggering
        InfoAfterTrigger(;egoid::Int64 = 0,
                        triggerframe::Int64 = 0,
                        trigger_ind::Int64 = 0,
                        triggered::Bool = false,
                        laneids::Array{Int64, 1} = Array{Int64, 1}(),
                        ego_v::Array{Float64, 1} = Array{Float64, 1}(),
                        fore_v::Array{Float64, 1} = Array{Float64, 1}(),
                        duration::Int64 = 50) = new(egoid, triggerframe, trigger_ind,
                                                    triggered, laneids, ego_v,
                                                    fore_v, duration)
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
function a_sample_neighbor_v_Δs_def!( speed_vec::Vector{Float32},
                                      distance_vec::Vector{Float32},
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
    speed_vec = Vector{Float32}()
    distance_vec = Vector{Float32}()
    # Record the type and dimension of the vehicle.
    vehicle_def = AutomotiveDrivingModels.VehicleDef()

    # We first want to record the information of the neighbor right at cameraframe.
    scene = get!(Scene(500), trajdata, cameraframe);
    neighbor = get_neighbor_in_scene(scene, carid, roadway);
    # We want to get this neigbor's id.
    original_neighbor_id = get_neighbor_id_from_scene(scene, neighbor);
    # We get new samples of speed and distance.
    (speed_vec, distance_vec, vehicle_def) = a_sample_neighbor_v_Δs_def!(speed_vec, distance_vec, scene, neighbor, original_neighbor_id);
    # If the sample duration is longer than 1, we keep recording.
    if sample_duration > 1
        for i = 1:(sample_duration)
            # Retrieve a neighbor from the scene.
            scene = get!(Scene(500), trajdata, cameraframe - i);
            neighbor = get_neighbor_in_scene(scene, carid, roadway);
            # We get new samples of speed and distance.
            (speed_vec, distance_vec, vehicle_def) = a_sample_neighbor_v_Δs_def!(speed_vec, distance_vec, scene, neighbor, original_neighbor_id);
        end
    end
    (speed_vec, distance_vec, vehicle_def)
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

    # If the vehicle to be observed is the ego self,
    if longitudinal_direc == "self"
        # Define a speed cache and a distance chance here
        speed_vec = Vector{Float32}()
        speed = 0.0
        distance_vec = Vector{Float32}()
        distance = 0.0
        # This record the type and dimension of the vehicle.
        vehicle_def = AutomotiveDrivingModels.VehicleDef()
        for i = 1:sample_duration
            scene = get!(Scene(500), trajdata, cameraframe +1 - i);
            # We get the index of the vehicle (carid) in the scene
            ego_ind = get_scene_id(scene, carid)
            # Let the distance to self be 0
            push!(distance_vec, 0);
            # We get the velocity of the ego vehicle in unit [m/s].
            push!(speed_vec, scene[ego_ind].state.v);
            # The vehicle definition.
            if i == 1
                vehicle_def = scene[ego_ind].def
            end
        end
    # If the vehicle to be observed is in front,
    elseif longitudinal_direc == "fore"
        # further if the vehicle to be observed shares the lane with the ego
        if lateral_direc == "middle"
            # Get samples of speed, distance, and vehicle type of the neighbor in this direction.
            (speed_vec, distance_vec, vehicle_def) = samples_neighbor_v_Δs_def(get_neighbor_fore_along_lane_NGSIM, trajdata, roadway, carid, cameraframe, sample_duration = sample_duration);

        # If the vehicle to be observed is in the left,
        elseif lateral_direc == "left"
            # Get samples of speed, distance, and vehicle type of the neighbor in this direction.
            (speed_vec, distance_vec, vehicle_def) = samples_neighbor_v_Δs_def(get_neighbor_fore_along_left_lane_NGSIM, trajdata, roadway, carid, cameraframe, sample_duration = sample_duration);

        # If the vehicle to be observed is on right,
        elseif lateral_direc == "right"
            # Get samples of speed, distance, and vehicle type of the neighbor in this direction.
            (speed_vec, distance_vec, vehicle_def) = samples_neighbor_v_Δs_def(get_neighbor_fore_along_right_lane_NGSIM, trajdata, roadway, carid, cameraframe, sample_duration = sample_duration);

        else
            error("When getting front neighbor, the lateral direction is not valid.")
        end
    # If the vehicle to be observed is at rear,
    elseif longitudinal_direc == "rear"
        # further if the vehicle to be observed is in the middle
        if lateral_direc == "middle"
            # Get samples of speed, distance, and vehicle type of the neighbor in this direction.
            (speed_vec, distance_vec, vehicle_def) = samples_neighbor_v_Δs_def(get_neighbor_rear_along_lane_NGSIM, trajdata, roadway, carid, cameraframe, sample_duration = sample_duration);

        # If the vehicle to be observed is on left
        elseif lateral_direc == "left"
            # Get samples of speed, distance, and vehicle type of the neighbor in this direction.
            (speed_vec, distance_vec, vehicle_def) = samples_neighbor_v_Δs_def(get_neighbor_rear_along_left_lane_NGSIM, trajdata, roadway, carid, cameraframe, sample_duration = sample_duration);

        # If the vehicle to be observed is on the right,
        elseif lateral_direc == "right"
            # Get samples of speed, distance, and vehicle type of the neighbor in this direction.
            (speed_vec, distance_vec, vehicle_def) = samples_neighbor_v_Δs_def(get_neighbor_rear_along_right_lane_NGSIM, trajdata, roadway, carid, cameraframe, sample_duration = sample_duration);

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
function lanedecisions_dueto_slowlead()
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
                            Δn_lanedecisions=lane_change_or_keep(info_after_trigger);
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
                    # We update the event triggering frame.
                    ego_lastevent_frame = ego_frame;
                end
                # When the while-loop is about to end, we need to check the lane decisions from the last event trigger.
                if ego_row_ind == ego_end_row_m
                    if !isempty(info_after_trigger.laneids)
                        n_laneids_notempty += 1;
                        # Checking if a lane change or a lane keep was made
                        Δn_lanedecisions=lane_change_or_keep(info_after_trigger);
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
    # test code {
    # Print the total number of lane changes
    println(("total searched frames:", n_searched));
    println(("total triggering events:", n_triggers));
    println(("n_laneids_notempty", n_laneids_notempty));
    println(("total number of lane-changing", n_lanechanges));
    println(("total number of lane-keeping", n_lanekeeps));
    println(("total number of aborted lane change", n_abortedchanges));
    println(("number of slow lead:", n_slow_lead));
    println(("number of not change:", n_notchange));
    # test code }
    # Print the elapsed time.
    toc();
end

# """
# Determine lane change or lane keep
# """
function lane_change_or_keep(info_after_trigger::InfoAfterTrigger)
    # Initialize some counters
    Δn_lanechanges = 0
    Δn_lanekeeps = 0
    Δn_abortedchanges = 0
    Δn_notchange = 0
    Δn_slow_lead = 0
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
                Δn_lanechanges += 1
                # We further check if this lane change is aborted
                goingback = false;
                for j = i:minimum([i + 20, aftrg_laneids_len])
                    # If the vehicle goes back to the previous lane, that is an aborted change.
                    goingback = goingback || (info_after_trigger.laneids[j] == aftrg_current_laneid);
                end
                # If the ego goes back to the current lane, the lane change is aborted.
                if goingback
                    # Add one lane-change abortion
                    Δn_abortedchanges += 1
                    # Abort the lane change
                    Δn_lanechanges -= 1
                else
                    # Update the current lane to be this lane.
                    aftrg_current_laneid = info_after_trigger.laneids[i]
                end
            end
        end
        #
        # We determine if a lane-keeping happens. A lane-keeping is charaterized by
        # 1) The ego did not change lane
        # 2) The ego matched the speed of the slow lead vehicle.
        # It is possible that the lead vehicle may speed up quickly, hence the ego
        # aborts the lane change quickly. We do NOT consider this case.
        #
        changinglane = false
        for i = 2:aftrg_laneids_len
            changinglane = changinglane || (info_after_trigger.laneids[i] != info_after_trigger.laneids[1]);
        end
        # If the ego did not change lane,
        if !changinglane
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
                # test code{
                else
                    #println(("rel_speed", rel_speed_fore))
                    println(("fore_v", info_after_trigger.fore_v[1:10]))
                    # println(("ego_v", info_after_trigger.ego_v))
                    println(("egoid", info_after_trigger.egoid, "triggerframe,", info_after_trigger.triggerframe))
                # test code}
                end
            end
        end
        Δn_lanedecisions= [Δn_lanechanges, Δn_lanekeeps, Δn_abortedchanges, Δn_notchange, Δn_slow_lead]
    else
        error("The laneids vector is empty.");
    end
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

lanedecisions_dueto_slowlead()
