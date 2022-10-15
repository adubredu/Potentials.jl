using Revise
using Potentials

θ = [6.0, 0.0]
θ̇ = [0.0, 0.0]
Obstacles = [[-2.0, 0.0]]
Obstacle_radii = [2.8]
goal = [-5.0, 0.0]
robot_radius = 0.5


sys = PointMass(θ, θ̇, robot_radius, 
                Obstacles, Obstacle_radii, goal)
sys.show_tail = true
sys.dynamic = false
sys.obstacle_speed = 0.01
ax, fig = visualize_system!(sys)

tasks = [:attractor, :repeller]
prob = Problem(tasks, sys)
horizon = 500

for i=1:horizon 
    global θ, θ̇
    if prob.sys.dynamic move_obstacles!(prob.sys) end
    τ = potential_solve(θ, θ̇ , prob)
    step!(τ, prob)
    θ = prob.sys.x; θ̇ = prob.sys.ẋ
    sleep(prob.sys.Δt/horizon)
end