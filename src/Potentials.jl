module Potentials

using GLMakie
using DataStructures: CircularBuffer
using FiniteDiff 
using LinearAlgebra
using StaticArrays
using Colors

include("utils.jl")

include("systems/pointmass/types.jl")
include("systems/pointmass/step.jl")
include("systems/pointmass/visualize.jl")
include("systems/pointmass/potential.jl")

export visualize_system!,
       step!

export PointMass,
       attractor_force,
       move_obstacles!,
       potential_force

end
