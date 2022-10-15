module Potentials

using GLMakie
using DataStructures: CircularBuffer
using FiniteDiff 
using LinearAlgebra
using StaticArrays
using Colors

include("utils.jl")
include("types.jl")

include("systems/pointmass/types.jl")
include("systems/pointmass/step.jl")
include("systems/pointmass/visualize.jl")
include("systems/pointmass/potential.jl")
include("systems/pointmass/old_potential.jl")

include("systems/picklerick/types.jl")
include("systems/picklerick/step.jl")
include("systems/picklerick/visualize.jl")
include("systems/picklerick/potential.jl") 

export visualize_system!,
       step!,
       Problem

export PointMass, 
       move_obstacles!,
       potential_solve,
       potential_force

export PickleRick

end
