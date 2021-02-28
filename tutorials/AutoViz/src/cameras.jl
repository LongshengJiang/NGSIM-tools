export
    Camera,
    StaticCamera,
    FitToContentCamera,
    CarFollowCamera,
    SceneFollowCamera

abstract type Camera end
camera_set!{S,D,I,R}(::RenderModel, cam::Camera, ::EntityFrame{S,D,I}, ::R, canvas_width::Int, canvas_height::Int) = error("camera_set! not implemented for Camera $cam")

mutable struct StaticCamera <: Camera
    pos::VecE2
    zoom::Float64 # [pix/meter]
    StaticCamera(pos::VecE2, zoom::Float64=3.0) = new(pos, zoom)
    StaticCamera(x::Float64, y::Float64, zoom::Float64=3.0) = new(VecE2(x,y), zoom)
end
function camera_set!(rendermodel::RenderModel, cam::StaticCamera, canvas_width::Int, canvas_height::Int)

    camera_set_pos!(rendermodel, cam.pos)
    camera_setzoom!(rendermodel, cam.zoom)

    rendermodel
end
camera_set!{S,D,I,R}(rendermodel::RenderModel, cam::StaticCamera, scene::EntityFrame{S,D,I}, roadway::R, canvas_width::Int, canvas_height::Int) = camera_set!(rendermodel, cam, canvas_width, canvas_height)

mutable struct FitToContentCamera <: Camera
    percent_border::Float64
    FitToContentCamera(percent_border::Float64=0.1) = new(percent_border)
end
function camera_set!(rendermodel::RenderModel, cam::FitToContentCamera, canvas_width::Int, canvas_height::Int)
    camera_fit_to_content!(rendermodel, canvas_width, canvas_height, percent_border=cam.percent_border)
    rendermodel
end
camera_set!{S,D,I,R}(rendermodel::RenderModel, cam::FitToContentCamera, scene::EntityFrame{S,D,I}, roadway::R, canvas_width::Int, canvas_height::Int) = camera_set!(rendermodel, cam, canvas_width, canvas_height)

mutable struct CarFollowCamera{I} <: Camera
    targetid::I
    zoom::Float64 # [pix/meter]
    rotation::Float64 # [rad]

    CarFollowCamera{I}(targetid::I, zoom::Float64=3.0, rotation::Float64 = 0.0) where {I} = new(targetid, zoom, rotation)
end

function camera_set!{S<:State1D,D,I,R}(rendermodel::RenderModel, cam::CarFollowCamera{I}, scene::EntityFrame{S,D,I}, roadway::R, canvas_width::Int, canvas_height::Int)

    veh_index = findfirst(scene, cam.targetid)
    if veh_index != 0
        camera_set_pos!(rendermodel, VecE2(scene[veh_index].state.s, 0.0))
        camera_setzoom!(rendermodel, cam.zoom)
        camera_setrotation!(rendermodel, cam.rotation)
    else
        add_instruction!( rendermodel, render_text, (@sprintf("CarFollowCamera did not find id %d", cam.targetid), 10, 15, 15, colorant"white"), incameraframe=false)
        camera_fit_to_content!(rendermodel, canvas_width, canvas_height)
    end

    rendermodel
end
function camera_set!{S<:VehicleState,D,I,R}(rendermodel::RenderModel, cam::CarFollowCamera{I}, scene::EntityFrame{S,D,I}, roadway::R, canvas_width::Int, canvas_height::Int)

    veh_index = findfirst(scene, cam.targetid)
    if veh_index != 0
        camera_set_pos!(rendermodel, scene[veh_index].state.posG)
        camera_setzoom!(rendermodel, cam.zoom)
        camera_setrotation!(rendermodel, cam.rotation)
    else
        add_instruction!( rendermodel, render_text, (@sprintf("CarFollowCamera did not find id %d", cam.targetid), 10, 15, 15, colorant"white"), incameraframe=false)
        camera_fit_to_content!(rendermodel, canvas_width, canvas_height)
    end

    rendermodel
end

mutable struct SceneFollowCamera <: Camera
    zoom::Float64 # [pix/meter]
    SceneFollowCamera(zoom::Float64=3.0) = new(zoom)
end
function camera_set!{S<:State1D,D,I,R}(rendermodel::RenderModel, cam::SceneFollowCamera, scene::EntityFrame{S,D,I}, roadway::R, canvas_width::Int, canvas_height::Int)


    if length(scene) > 0

        # get camera center
        C = 0.0
        for veh in scene
            C += veh.state.s
        end
        C = C / length(scene)

        camera_set_pos!(rendermodel, VecE2(C, 0.0))
        camera_setzoom!(rendermodel, cam.zoom)
    else
        add_instruction!( rendermodel, render_text, ("SceneFollowCamera did not find any vehicles", 10, 15, 15, colorant"white"), incameraframe=false)
        camera_fit_to_content!(rendermodel, canvas_width, canvas_height, 0.1)
    end

    rendermodel
end
function camera_set!{S<:VehicleState,D,I,R}(rendermodel::RenderModel, cam::SceneFollowCamera, scene::EntityFrame{S,D,I}, roadway::R, canvas_width::Int, canvas_height::Int)


    if length(scene) > 0

        # get camera center
        C = VecE2(0.0,0.0)
        for veh in scene
            C += convert(VecE2, veh.state.posG)
        end
        C = C / length(scene)

        camera_set_pos!(rendermodel, C)
        camera_setzoom!(rendermodel, cam.zoom)
    else
        add_instruction!( rendermodel, render_text, ("SceneFollowCamera did not find any vehicles", 10, 15, 15, colorant"white"), incameraframe=false)
        camera_fit_to_content!(rendermodel, canvas_width, canvas_height, 0.1)
    end

    rendermodel
end
