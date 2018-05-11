# Boids

This project tries to simulate Starling murmuration. We simulated them using boids in 2D plane. The basic rules it follows are:

   Separation: steer to avoid crowding local flockmates
   Alignment: steer towards the average heading of local flockmates
   Cohesion: steer to move toward the average position (center of mass) of local flockmates

## Running the simulation
We are using SFML Library for the graphic part. We can generate blue-tiangular boids and red circular obstacles. 

To run the simulation, `cd` into the `src` folder and run `make` to produce an executable file called `flock`.

```bash
./flock
```

You can add Boids and Obstacles. To add a boid, click the left mouse button. To add an obstacle, press the right mouse button.
To Exit press <kbd>esc</kbd>.

## Issues and Improvements
- Fine tuning of velocity and force variables, along with line of sight, cohesion and steering factors to get most natural behavior. In the current version, the behavior is not very close to the natural behavior of real starling flocks.
- Need to implement 3D simulation to get closer to natural flocks

# starling-cop290
