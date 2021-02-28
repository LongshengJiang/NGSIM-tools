# Prerequsite modules
# using Reel
# using AutoViz
using Reel

AFTER_MARGIN = 50
BEFORE_MARGIN = 50

include("extract_epoch.jl")
making_films =  lanechanges_stats()# [vehicle_id, frame_index]
# making_films = Dict{Int, Vector{Int}}()
# making_films[2967] = [9277]

for carid in keys(making_films)
    b_margin = BEFORE_MARGIN
    a_margin = AFTER_MARGIN
    for frame in making_films[carid]
        # Get the number of the preceding and remaining frames.
        car_start_row = tdraw_my.car2start[carid]
        car_start_frame = tdraw_my.df[car_start_row, :frame]
        n_prec_frames = frame - car_start_frame
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
        filmframes = Frames(MIME("image/png"), fps=10);
        for i = -b_margin:a_margin
            scene = get!(Scene(500), trajdata_my, frame + i);
            carcolors = Dict{Int, Colorant}()
            carcolors[carid] = colorant"green";
            ego_ind = get_scene_id(scene, carid)
            slide = render(scene, ROADWAY_my, [NeighborsOverlay(carid)],cam=CarFollowCamera{Int}(carid, 8.0, 2.0/9.0*pi), car_colors=carcolors);
            push!(filmframes, slide)
        end
        filmframes;
        write("./Animation/aborted_lane_change/i101_0820/lane_change_"*string(carid)*"_"*string(frame)*".mp4", filmframes);
        # write("test_"*string(carid)*"_"*string(frame)*".mp4", filmframes);
    end
end
