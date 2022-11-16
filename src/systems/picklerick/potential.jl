### Task maps
function posture_task_map(θ, θ̇, prob::Problem)
    sys = prob.sys
    g = sys.θᵣ
    return θ - g
end

function lefthand_attractor_task_map(θ, θ̇, prob::Problem)
    sys = prob.sys
    x₉ = sys.g
    xh = left_hand_pose(θ, sys)
    return xh - x₉
end

function righthand_attractor_task_map(θ, θ̇, prob::Problem)
    sys = prob.sys
    x₉ = sys.g
    xh = right_hand_pose(θ, sys)
    return xh - x₉
end

function head_attractor_task_map(θ, θ̇, prob::Problem)
    sys = prob.sys
    x₉ = sys.g[1] 
    xh = head_pose(θ, sys)
    return xh - x₉
end

function waist_attractor_task_map(θ, θ̇, prob::Problem)
    sys = prob.sys
    x₉ = sys.g[3] 
    xh = head_pose(θ, sys)
    return xh - x₉
end

function dodge_task_map(θ, θ̇, prob::Problem)
    sys = prob.sys
    os = sys.obstacle_observables
    rs = sys.r
    chains = link_poses(θ, sys)
    xs = Float64[]; o = os[1]; r = 2*rs[1]
    lh = chains[5][end]; rh = chains[4][end]
    lf = chains[1][end]; rf = chains[2][end]
    hd = chains[3][end]; nk = chains[6][1]
    kp = [lh, rh, hd, nk]#[hd, lh, rh, lf, rf]
    for k in kp 
        Δ = (norm(k - o.val)/r)[1] - 1.0
        push!(xs, Δ)
    end 
    # @show xs
    return xs
end

### Potentials 
function posture_potential(x, ẋ, prob::Problem)
    sys = prob.sys
    K = sys.M/3
    ϕ(x) = 0.5*x'*K*x
    δₓ = FiniteDiff.finite_difference_gradient(ϕ, x)
    f = -K*δₓ
    return f
end

function lefthand_attractor_potential(x, ẋ, prob::Problem)
    sys = prob.sys
    K = 2*sys.M
    ϕ(x) = 0.5*x'*K*x
    δₓ = FiniteDiff.finite_difference_gradient(ϕ, x)
    f = -K*δₓ
    return f
end

function righthand_attractor_potential(x, ẋ, prob::Problem)
    sys = prob.sys
    K = 2*sys.M
    ϕ(x) = 0.5*x'*K*x
    δₓ = FiniteDiff.finite_difference_gradient(ϕ, x)
    f = -K*δₓ
    return f
end

function head_attractor_potential(x, ẋ, prob::Problem)
    sys = prob.sys
    K = 3*sys.M
    ϕ(x) = 0.5*x'*K*x
    δₓ = FiniteDiff.finite_difference_gradient(ϕ, x)
    f = -K*δₓ
    return f
end

function waist_attractor_potential(x, ẋ, prob::Problem)
    sys = prob.sys
    K = 3*sys.M
    ϕ(x) = 0.5*x'*K*x
    δₓ = FiniteDiff.finite_difference_gradient(ϕ, x)
    f = -K*δₓ
    return f
end

function dodge_potential(x, ẋ, prob::Problem)
    sys = prob.sys 
    K = sys.M 
    s = [v > sys.max_range ? 0.0 : 1.0 for v in x]  
    ϕ(σ) = (K/2) .* s .* (sys.max_range .- σ)./(sys.max_range .* σ).^2
    δₓ = FiniteDiff.finite_difference_jacobian(ϕ, x) 
    f =-K*diag(δₓ)  
    return f
end

### Solve 
function potential_solve(θ, θ̇, prob::Problem, sys::PickleRick)
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