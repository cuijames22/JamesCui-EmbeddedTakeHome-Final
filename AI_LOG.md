# AI Usage Log

Replace this template with your own entries. Add one entry per significant use of an AI tool.


## 1. <State Variable above subscriptions>
**What I asked:** Why odometry_node has no attribute called self.last_tick_count and pasted code

**What I kept vs. rewrote, and why:** Kept the general idea that I had to move the variables above and copy and pasted

**What the AI got wrong that I had to catch:** Nothing

**How I verified it ran correctly (not just that it compiled):** It compiled and didn't give the same error as previously given


## 2. <MatPlotLib Installation>
**What I asked:** Took a screenshot of a warning and asked if it had anything to do with Matplotlib not working. Then asked why I am not seeing a visualization

**What I kept vs. rewrote, and why:** instead of using python sim/launch.py --visualize I was using sim/launch.py --visualize which did not include python and also the reason it did not launch

**What the AI got wrong that I had to catch:** Nothing

**How I verified it ran correctly (not just that it compiled):** I saw a Matplotlib visualization afterward


## 3. <Strings in Python>
**What I asked:** How do you print floats in Python and integrate and concatenate strings

**What I kept vs. rewrote, and why:** I used the stuff to print the floats in the format f"{variable:.2f}" for the monitoring_callback function

**What the AI got wrong that I had to catch:** Nothing

**How I verified it ran correctly (not just that it compiled):** It printed correctly in the terminal after I ran the simulation


## 4. <Issues with heading>
**What I asked:** Pasted code and asked why heading was being wonky. Additionally asked how to fix the bug where a negative slope would cause the heading to be the wrong direction. 

**What I kept vs. rewrote, and why:** Found error that I had swaped sin and cosine in the code. Also recommended using the arctan of the derivatives instead of the arctan of just one slope

**What the AI got wrong that I had to catch:** Nothing

**How I verified it ran correctly (not just that it compiled):** The robot began to move better into the direcions that I wish instead of moving 90 degrees perpendicularly


When searching up syntax and small bug issues on google, google ai automatically popped up and summarized the answers from the web which I did not report