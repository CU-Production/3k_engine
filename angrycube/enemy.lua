-- Angry Cube Enemy Script
-- Tracks damage and destruction

local health = 100
local hit_count = 0
local destroyed = false

return {
    update = function(dt)
        if destroyed then
            return
        end
        
        -- Simple collision detection based on velocity
        -- In a real implementation, this would use collision callbacks
        -- For now, we just track if the enemy is still alive
        
        -- Check if fallen off the platform (y < -350)
        -- This would need access to transform component
        -- Placeholder: enemy gets destroyed after some time in play mode
        
        if hit_count > 0 then
            log("Enemy hit " .. hit_count .. " times! Health: " .. health)
        end
    end
}
