function get_keypoints(c::Vector{Float64}, r::Float64; N = 16)
    Δθ = 2π/N
    θs = 0: Δθ : 2π
    keypoints = [[c[1]+r*cos(θ), c[2]+r*sin(θ)] for θ in θs]
    return keypoints
end

function get_closest_points(x, o, r)
    opoints = get_keypoints(o, r)
    pts = [[a,x] for a in opoints]
    ind = argmin([norm(σ[1]-σ[2] for σ in pts)]) 
    return pts[ind]
end