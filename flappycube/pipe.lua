-- Flappy Cube Pipe Script
-- Scrolls pipes from right to left

local scroll_speed = -150  -- pixels per second
local reset_x = 700        -- reset position when off screen
local off_screen_x = -350  -- off screen threshold
local has_scored = false
local player_id = 0        -- Player entity ID

return {
    init = function()
        player_id = 0  -- Player is always entity 0 in our scene
    end,
    
    update = function(dt)
        if game_over then
            return
        end
        
        -- Get current position
        local pos = get_transform(entity_id, entity_generation)
        if not pos then return end
        
        -- Move pipe left
        local new_x = pos.x + scroll_speed * dt
        set_transform(entity_id, entity_generation, new_x, pos.y)
        
        -- When pipe goes off screen left, reset to right
        if new_x < off_screen_x then
            set_transform(entity_id, entity_generation, reset_x, pos.y)
            has_scored = false
        end
        
        -- Track if player passed this pipe for scoring
        -- Only score once per pipe, and only for bottom pipes (to avoid double scoring)
        local player_pos = get_transform(player_id, 0)
        if player_pos and not has_scored and player_pos.x > pos.x and pos.y < 0 then
            has_scored = true
            game_score = game_score + 1
            log("Score: " .. game_score)
        end
    end
}
