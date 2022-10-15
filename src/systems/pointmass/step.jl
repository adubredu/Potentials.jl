function step!(ẍ::Vector{Float64}, prob::Problem)
    # x = sys.x + sys.ẋ * sys.Δt
    # ẋ = sys.ẋ + ẍ * sys.Δt 
    # ẋ = clamp.(ẋ, -15.0, 15.0)
    sys = prob.sys
    x = sys.x + ẍ*sys.Δt
    
    sys.obs_x[] = x 

    if sys.show_tail
        push!(sys.obs_tail[], SVector(x...))
        sys.obs_tail[] = sys.obs_tail[]
    end

    

    sys.x = x 
    # sys.ẋ = ẋ
end

function step!(ẍ::Vector{Float64}, sys::PointMass) 
    x = sys.x + ẍ*sys.Δt 
    sys.obs_x[] = x 
    if sys.show_tail
        push!(sys.obs_tail[], SVector(x...))
        sys.obs_tail[] = sys.obs_tail[]
    end  
    sys.x = x  
end


function move_obstacles!(sys::PointMass) 
    for ob in sys.obs_o
        val = collect(ob.val)
        val[2] -= sys.obstacle_speed
        ob[] = SVector(val...)
        if ob[][2] < -10.0
            val[2] = 10
            ob[] = SVector(val...)
        end
    end 
end