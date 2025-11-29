-- Flappy Cube Pipe Script
-- Scrolls pipes from right to left

local scroll_speed = -100  -- pixels per second
local reset_x = 700        -- reset position when off screen
local off_screen_x = -300  -- off screen threshold
local has_passed = false

return {
    update = function(dt)
        -- Move pipe left
        -- In real implementation, we'd update transform.position.x
        -- This is a placeholder showing the intended behavior
        
        -- Placeholder: log movement every second
        local time = math.floor(dt * 50) % 50
        if time == 0 then
            log("Pipe scrolling...")
        end
        
        -- When pipe goes off screen left, reset to right
        -- if current_x < off_screen_x then
        --     current_x = reset_x
        --     has_passed = false
        -- end
        
        -- Track if player passed this pipe for scoring
        -- if not has_passed and player_x > current_x then
        --     has_passed = true
        --     score = score + 1
        -- end
    end
}
