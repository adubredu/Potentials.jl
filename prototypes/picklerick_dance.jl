using Revise
using Potentials

θ = [6.0, 0.0]
θ̇ = [0.0, 0.0]
Obstacles = [[-4.0, 3.5]]
Obstacle_radii = [1.0]
goal = [[-1.36, 3.2], [1.36, 3.2], [0.0, 3.5]]
goals = [[[-1.36, 3.2], [1.36, 3.2], [0.0,3.5]], [[1.36, 3.2], [-1.36, 3.2], [0.0, 2.5]]]
init_joint_positions = [π/2, 2π/3, π/6, 2π/3, π/6, π/2, π/3, 7π/12, 2π/3, 5π/12]

sys = PickleRick(Obstacles, 0.5 .* Obstacle_radii,
                init_joint_positions,
                zero(init_joint_positions),
                goal)
 
sys.show_contacts = false
sys.dynamic = false
sys.show_obstacle = false  
sys.show_goal = false
ax, fig = visualize_system!(sys)

tasks = [:waist_attractor, :lefthand_attractor, :righthand_attractor]
        # ,:head_attractor]
prob = Problem(tasks, sys)
horizon = 20000

θ = init_joint_positions
θ̇ = zero(init_joint_positions)
id = 1
for i=1:horizon 
    global θ, θ̇ , id
    if i%400 == 0
        ind = id == 1 ? 1 : 2
        sys.g[1:2] = goals[ind][1:2]
        id *= -1
    end
    if i%200 == 0
        ind = id == 1 ? 1 : 2
        sys.g[3] = goals[ind][3]
        id *= -1
    end  
    τ = potential_solve(θ, θ̇ , prob, sys)
    step!(τ, prob, sys)
    θ = prob.sys.θ; θ̇ = prob.sys.θ̇
    sleep(prob.sys.Δt/horizon)
end