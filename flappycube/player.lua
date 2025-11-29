-- Flappy Cube Player Script
-- Controls the yellow bird cube

local flap_force = 400
local alive = true
local score = 0

return {
    update = function(dt)
        if not alive then
            return
        end
        
        -- SPACE to flap
        if get_key_down(32) then
            -- Apply upward impulse
            -- Note: In real implementation, we'd need access to rigidbody
            -- to apply b2Body_ApplyLinearImpulseToCenter
            log("FLAP!")
        end
        
        -- Check if hit ground/ceiling (y < -280 or y > 280)
        -- This would need transform component access
        -- Placeholder for collision detection
        
        -- Display score periodically
        local time = math.floor(dt * 100) % 100
        if time == 0 and score > 0 then
            log("Score: " .. score)
        end
    end
}
