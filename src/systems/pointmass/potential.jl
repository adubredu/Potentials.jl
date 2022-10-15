function recede_goal(θ, sys; ∇=[0.0, -1.5])
    g = copy(sys.g)
    Δrobot = (g[2]-θ[2])/(g[1]-θ[1])
    Δobs = []
    for i in eachindex(sys.or)
        o = sys.o[i] 
        Δ = (g[2]-o[2])/(g[1]-o[1]) 
        push!(Δobs, Δ)    
    end
    if any([abs(Δrobot-Δ)<0.5 for Δ in Δobs]) 
        g+=∇ 
    end
    return g
end

### Task maps
function attractor_task_map(θ, θ̇, prob::Problem)
    sys = prob.sys
    g = recede_goal(θ, sys)
    res = θ-g 
    return res
end

function repeller_task_map(θ, θ̇, prob::Problem)
    res = Float64[]
    sys = prob.sys
    for i in eachindex(sys.or)
        o = sys.o[i]; r = sys.or[i]
        d(x) = get_closest_points(x, o, r)  
        distance(x) = norm(d(x)[1] - d(x)[2])     
        push!(res, distance(θ))    
    end
    return res
end

### Potentials 
function attractor_potential(x, ẋ, prob::Problem)
    sys = prob.sys
    K = sys.M
    ϕ(x) = 0.5*x'*K*x
    δₓ = FiniteDiff.finite_difference_gradient(ϕ, x)
    f = -K*δₓ
    return f
end

function repeller_potential(x, ẋ, prob::Problem)
    sys = prob.sys 
    K = sys.M 
    s = [v > sys.max_range ? 0.0 : 1.0 for v in x]
    ϕ(σ) = vec((K/2) .* s .* (sys.max_range .- σ)/(sys.max_range .* σ).^2)
    δₓ = FiniteDiff.finite_difference_jacobian(ϕ, x) 
    f =-K*δₓ
    return f
end

### Solve
function potential_eval(x, ẋ, name::Symbol, prob::Problem)
    ϕ = eval(Symbol(name, :_potential))
    f = ϕ(x, ẋ, prob)
    return f
end

function potential_solve(θ, θ̇, prob::Problem, sys::PointMass)
    Js = []; fs = []
    for t in prob.tasks 
        ψ = eval(Symbol(t, :_task_map))
        x = ψ(θ, θ̇, prob)
        J = FiniteDiff.finite_difference_jacobian(σ->ψ(σ, θ̇, prob), θ)
        ẋ = J*θ̇
        f = potential_eval(x, ẋ, t, prob)
        push!(Js, J); push!(fs, f)
    end
    τ = vec(sum([J'*f for (J, f) in zip(Js, fs)]))  
    return τ
end