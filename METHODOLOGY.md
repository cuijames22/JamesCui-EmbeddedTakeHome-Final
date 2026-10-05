# Methodology

Replace this template with your own. Keep it short: write what a teammate would need to run your work and trust it.

## 1. How to run it

To run the program, use the line: python sim/launch.py 
To add a visual simulation through matplotlib --visualize to make: 
python sim/launch.py --visualize
To remove remove faults like duplicate inputs, use --no-faults just like --visualize
Use --no-node to remove the odometry code and test simulation only
To use a specific seed (same noise everytime) use --seed # such as --seed 42

## 2. Thought process

- My original approach was to use the GPS to calculated the headings and the encoders to determine distance.
- Additionally, the robot would correct its position based off a weighting toward the GPS everytime a recording was published
- To determine heading, I originally planned a polynomial regression based off of 10 points from the GPS that proved to not be as efficient nor accurate as a simple linear regression
- I ended up settling on a linear regression with 5 of the most recent points from the GPS to calculate the heading
- I take the derivative of x position in respect to time and the derivative of y in respect to time then arctan dy/dx to find the angle. 
- I handle duplicate entries, skipped entries and innacurate readings with the fusion system to a decent degree. 

## 3. Known limitations

The rover heavily relies on the gps reading much more than the encoder readings. Whenever a GPS reading is provided, the robot trusts that much more than the estimate with the heading. Additionally, there is often a huge mismatch of heading when the input is noisy that causes error up to 2-3m. If provided more time and the estimation is extremely important to the job, I would most likely create a smooth curve generator that would give a variable heading like the original polynomial regression method but use higher power and non-elementary curves. 
