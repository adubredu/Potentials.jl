function attractor_force(θ, θ̇,  sys)
    g = sys.g
    K = sys.M
    ϕ(x) = 0.5*(x-g)'*K*(x-g)
    δθ = FiniteDiff.finite_difference_gradient(ϕ, θ)
    θ̈ = -δθ 
    return θ̈
end

function repeller_force(θ, θ̇, sys) 
    forces = [[0.0, 0.0]]
    d_range = sys.max_range
    # d_range = 10.0
    K = sys.M
    for i in eachindex(sys.or)
        o = sys.o[i]; r = sys.or[i]
        d(x) = get_closest_points(x, o, r)  
        distance(x) = norm(d(x)[1] - d(x)[2])        
        s = distance(θ) > d_range ? 0.0 : 1.0 
        # s=1
        ϕ(x) = (K/2)*s*((d_range - distance(x)))/(d_range*distance(x))^2
        # ϕ(x) = s*K/(2*distance(x)^2)
        δθ = FiniteDiff.finite_difference_gradient(ϕ, θ)
        push!(forces, -δθ)
    end
    θ̈ = sum(forces)
    return θ̈
end

function potential_force(θ, θ̇, sys)
    f1 = attractor_force(θ, θ̇, sys)
    f2 = repeller_force(θ, θ̇, sys)
    resultant_force = f1 + f2 
    return resultant_force
end