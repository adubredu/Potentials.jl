using Revise
using Potentials

θ = [5.5, -1]
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
horizon = 1000

for i=1:horizon 
    global θ, θ̇
    if sys.dynamic move_obstacles!(sys) end
    θ̈ = potential_force(θ, θ̇ , sys)
    step!(θ̈, sys)
    θ = sys.x; θ̇ = sys.ẋ
    sleep(sys.Δt/horizon)
end