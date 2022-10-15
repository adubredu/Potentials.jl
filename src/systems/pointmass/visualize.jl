function visualize_system!(sys::PointMass) 
    fig = Figure()
    ax = Axis(fig[1,1], aspect=DataAspect(), limits=(-10.,10.,-10.,10.))
    
    obstacle_observables = []
    if length(sys.o) > 0
        for (i, ob) in enumerate(sys.o)
            ox = Observable(SVector{2, Float64}(ob...))
            scatter!(ax, ox; marker=:circle, markersize=sys.or[i], color=:black, markerspace=:data)
            push!(obstacle_observables, ox)
        end
    end

    scatter!(ax, [sys.g[1]], [sys.g[2]]; marker=:rect, markersize=1.0, color=:green, markerspace=:data)

    robot_position_observable = Observable(SVector{2, Float64}(sys.x...))
    scatter!(ax, robot_position_observable; marker=:circle, markersize=sys.r, color=:blue, markerspace=:data)

    tail = nothing
    if sys.show_tail
        tail = CircularBuffer{SVector{2, Float64}}(10000)
        fill!(tail, SVector{2, Float64}(sys.x...))
        tail = Observable(tail)
        c = to_color(:purple)
        tailcol = [RGBA(c.r, c.g, c.b, (i/300)^2) for i in 1:10000]
        lines!(ax, tail; linewidth = 3, color = tailcol)
    end
    
    sys.obs_x = robot_position_observable
    sys.obs_tail = tail
    sys.obs_o = obstacle_observables

    # hidedecorations!(ax)
    display(fig)
    
    return ax, fig
end