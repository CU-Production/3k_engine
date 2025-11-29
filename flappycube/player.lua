-- Flappy Cube Player Script
-- Controls the yellow bird cube

local flap_force = 300
local max_velocity = 400
local dead = false

return {
    init = function()
        log("Flappy Cube Started! Press SPACE to flap!")
        game_over = false
        game_score = 0
    end,
    
    update = function(dt)
        if dead or game_over then
            return
        end
        
        -- Get current velocity
        local vel = get_velocity(entity_id, entity_generation)
        -- log("entity_id : "..tostring(entity_id))
        -- log("entity_id : "..tostring(vel))
        if not vel then return end
        
        -- SPACE to flap
        if get_key(32) then
            -- Set upward velocity instead of impulse for more responsive control
            set_velocity(entity_id, entity_generation, vel.x, flap_force)
            log("FLAP!")
        end
        
        -- Clamp velocity
        if vel.y > max_velocity then
            set_velocity(entity_id, entity_generation, vel.x, max_velocity)
        elseif vel.y < -max_velocity then
            set_velocity(entity_id, entity_generation, vel.x, -max_velocity)
        end
        
        -- Check collision with ground/ceiling
        local pos = get_transform(entity_id, entity_generation)
        if pos then
            if pos.y < -280 or pos.y > 280 then
                dead = true
                game_over = true
                log("GAME OVER! Final Score: " .. game_score)
                log("Press R to restart")
            end
        end
        
        -- R to restart
        if get_key_down(82) and game_over then
            game_over = false
            game_score = 0
            dead = false
            set_transform(entity_id, entity_generation, -200, 0)
            set_velocity(entity_id, entity_generation, 0, 0)
            log("Game Restarted!")
        end
    end
}
