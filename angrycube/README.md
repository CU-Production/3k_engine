# Angry Cube - Physics Puzzle Game

An Angry Birds-style physics game built with the custom 2D engine.

## How to Play

1. **Load the Scene**:
   - In the editor: File → Load Scene
   - Select `angrycube/game.txt`

2. **Controls**:
   - **SPACE**: Hold to charge launch power (watch console)
   - **Release SPACE**: Launch the red cube
   - **R**: Reset the projectile
   - **F5**: Toggle Play/Stop mode

3. **Objective**:
   - Launch the red cube to knock down the green enemy
   - Destroy the wooden structure
   - Use physics to your advantage!

## Game Elements

- **Red Cube**: Projectile (you control this)
- **Green Cube**: Enemy target
- **Brown Blocks**: Destructible structure
- **Gray Walls**: Level boundaries
- **Brown Base**: Ground

## Tips

- Charge longer for more power (max 2 seconds)
- Aim for structural weak points
- Watch the console for launch power feedback
- The structure will collapse realistically with Box2D physics

## Scene Structure

- Ground: Static body (immovable)
- Walls: Kinematic body (can be moved by code)
- Projectile: Dynamic with restitution (bouncy)
- Structure & Enemy: Dynamic bodies
- All objects have proper density, friction, and restitution

Enjoy the physics mayhem!
