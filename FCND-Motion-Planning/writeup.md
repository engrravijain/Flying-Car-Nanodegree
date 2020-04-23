## Project: 3D Motion Planning
![Quad Image](./misc/enroute.png)

---

## [Rubric](https://review.udacity.com/#!/rubrics/1534/view) Points
### Here I will consider the rubric points individually and describe how I addressed each point in my implementation.  

---
### Writeup / README

#### 1. Provide a Writeup / README that includes all the rubric points and how you addressed each one.  You can submit your writeup as markdown or pdf.  

You're reading it! Below I describe how I addressed each rubric point and where in my code each point is handled.

### Explain the Starter Code

#### 1. Explain the functionality of what's provided in `motion_planning.py` and `planning_utils.py`

These scripts contain a basic planning implemantation that include basic heuristic and search algorithm (a* search) which are used to plan path for the drone and fly 10eters and land. Below I'll explain in more detail.    

[motion_planning.py](./motion_planning.py): This file contains the base implementation for the motion planning project. This has been further modified to perform the following actions.
 
1. global home location is read from [colliders.csv](./colliders.csv)
2. Functionality to set the goal location via command line arguments. (--goal_lat, --goal_lon, --goal_alt).
3. prune the calculated path by a_star using a collinearity function to eliminate unnecessary waypoints.
    
[planning_utils.py](./planning_utils.py): This file contains the implementation of A* Search Algorithms and other utility functions. This has been further modified to

1. Extend the A* Search algorithm to include diagonal actions.
2. The collinearity function used in motion planning is defined here in this file.

### Implementing Your Path Planning Algorithm

#### 1. Set your global home position

at [line 126](./motion_planning.py#126) I am reading the first line of colliders.csv and then extract the lat0 and lon0 values. After this I set this as the global home position at [line 135](./motion_planning.py#135).

#### 2. Set your current local position
Here as long as you successfully determine your local position relative to global home you'll be all set. Explain briefly how you accomplished this in your code.

current global position is tuple of three values (latitude, longitude & altitude)
I have used the global_to_local() function here to get my current local position by passing current global position calculated in above step and self.global_home. 

#### 3. Set grid start position from local position
This is another step in adding flexibility to the start location. As long as it works you're good to go!

This is how I have calculated the grid start position
```python
grid, north_offset, east_offset = create_grid(data, TARGET_ALTITUDE, SAFETY_DISTANCE)
grid_start = (int(local_position[0]) - north_offset, int(local_position[1]) - east_offset)
```

#### 4. Set grid goal position from geodetic coords
This step is to add flexibility to the desired goal location. Should be able to choose any (lat, lon) within the map and have it rendered to a goal location on the grid.

I modified the main function to allow passing the goal latitude, longitude and altitude via arguments at the runtime. Please check the (run.sh)[./run.sh] file. 

```python
parser.add_argument('--goal_lon', type=str, help="Goal longitude")
parser.add_argument('--goal_lat', type=str, help="Goal latitude")
parser.add_argument('--goal_alt', type=str, help="Goal altitude")
```

the below command should be used to run the motion_planning.py file.

```shell script
python motion_planning.py --goal_lat 37.79360 --goal_lon -122.39759 --goal_alt -0.147
```

#### 5. Modify A* to include diagonal motion (or replace A* altogether)
Minimal requirement here is to modify the code in planning_utils() to update the A* implementation to include diagonal motions on the grid that have a cost of sqrt(2), but more creative solutions are welcome. Explain the code you used to accomplish this step.

This is how I added the diagonal motions that have a cost of sqrt(2)
```python
# Diagonal actions
NORTH_WEST = (-1, -1, np.sqrt(2))
NORTH_EAST = (-1, 1, np.sqrt(2))
SOUTH_WEST = (1, -1, np.sqrt(2))
SOUTH_EAST = (1, 1, np.sqrt(2))
```

And to check if our diagonal movement is colliding with an obstacle or off the grid
```python
 # filter diagonal action feasibility
if x - 1 < 0 or y - 1 < 0 or grid[x - 1, y - 1] == 1:
    valid_actions.remove(Action.NORTH_WEST)
if x + 1 > n or y - 1 < 0 or grid[x + 1, y - 1] == 1:
    valid_actions.remove(Action.SOUTH_WEST)
if x - 1 < 0 or y + 1 > m or grid[x - 1, y + 1] == 1:
    valid_actions.remove(Action.NORTH_EAST)
if x + 1 > n or y + 1 > m or grid[x + 1, y + 1] == 1:
    valid_actions.remove(Action.SOUTH_EAST)
```

#### 6. Cull waypoints 
For this step you can use a collinearity test or ray tracing method like Bresenham. The idea is simply to prune your path of unnecessary waypoints. Explain the code you used to accomplish this step.

Used (collinearity_prune)[./planning_utils.py#161] function to eliminate unnecessary waypoints. By this way I get rid of a lot of additional waypoints on straight line. 


