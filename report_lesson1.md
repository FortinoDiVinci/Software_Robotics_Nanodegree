![udacity logo](./misc/udacity_logo.png)
# Project: Search and Sample Return
### Vincent FORTINEAU, R&D Engineer at ALPhANOV, Bordeaux, France

#### Project due : 24th April 2018
---


## Goals and steps of the project

### Training / Calibration  

* Download the simulator and take data in "Training Mode"
* Test out the functions in the Jupyter Notebook provided
* Add functions to detect obstacles and samples of interest (golden rocks)
* Fill in the `process_image()` function with the appropriate image processing steps (perspective transform, color threshold etc.) to get from raw images to a map.  The `output_image` you create in this step should demonstrate that your mapping pipeline works.
* Use `moviepy` to process the images in your saved dataset with the `process_image()` function.  Include the video you produce as part of your submission.

### Autonomous Navigation / Mapping

* Fill in the `perception_step()` function within the `perception.py` script with the appropriate image processing functions to create a map and update `Rover()` data (similar to what you did with `process_image()` in the notebook). 
* Fill in the `decision_step()` function within the `decision.py` script with conditional statements that take into consideration the outputs of the `perception_step()` in deciding how to issue throttle, brake and steering commands. 
* Iterate on your perception and decision function until your rover does a reasonable (need to define metric) job of navigating and mapping.  

[//]: # (Image References)

[rgb_hsv_rock_detect]: ./misc/rock_detection_RGB_HSV.png
[rgb_obstacle_detect]: ./misc/obstacle_detection_RGB.png
[roll_pitch]: ./misc/mapping_pitch_roll_vs.jpg
[distance_thresh]: ./misc/mapping_pitch_vs_distance.jpg
[split ways]: ./misc/rover_front_obstacle.png
[gaussian]: ./misc/gaussian.png
[circle hell]: ./misc/circle_hell.gif
[best run 1]: ./misc/best_run_lucky_pick_up.png
[best run 2]: ./misc/best_run_luckiest_pick_up.png
[image3]: ./misc/example_rock1.jpg

#### [Evaluation criteria](https://review.udacity.com/#!/rubrics/916/view) 

## Table of Contents

* [Notebook Analysis](#part1)
	* [1. Color selection](#1-1)
		* [Rock Detection](#1-1-a)
		* [Obstacle Detection](#1-1-b)
	* [2. Terrain identification ](#1-2)
		* [Data world map update](#1-2-a)
* [Autonomous Navigation and Mapping](#part2)
	* [1. Perception](#2-1)
		* [Data worldmap update](#2-1-a)
		* [Polar coordinates](#2-1-b)
		* [Rock position](#2-1-c)
		* [Split way detection](#2-1-d)
	* [2. Autonomous decision making](#2-2)
		* [Stuck Scenarios](#2-2-a)
		* [Obstacle avoidance](#2-2-b)
		* [Wall crawler](#2-2-c)
		* [Smoothing the steering](#2-2-d)
		* [Golden rock search and pick up](#2-2-e)
* [Results](#part3)

## Notebook Analysis <a name="part1"></a>
#### 1. Color selection of obstacles and rock samples <a name="1-1"></a>
##### Rock Detection <a name="1-1-a"></a>

The subject suggest to use [**HSV color space**](http://opencv-python-tutroals.readthedocs.io/en/latest/py_tutorials/py_imgproc/py_colorspaces/py_colorspaces.html) to spot more easily the golden rocks - _nuggets_ - using both `cv2.cvtColor(img, cv2.COLOR_BGR2HSV)` and `cv2.inRange(hsv_img, lower_yellow, upper_yellow)`.

Staying in the **RGB color space**, seemed to bring good results with the function `color_thresh(img, rgb_thresh=(160, 160, 160))` to detect the navigable space. For a better detection of golden rocks, the idea is to modify the function a little bit, by adding a **second threshold**.

Here are the values used as lower and upper thresholds for both methods :

<center><table>
<tr><th>RGB color space </th><th>HSV color space</th></tr>
<tr><td>
lower=(150, 100, 0)
</td><td>
lower=(85,50,50)
</td></tr>

</td><td>
upper=(100, 255, 100)
</td><td>
upper=(100, 255, 255)
</td></tr> 

</table></center>

For a first test, I will use the calibration image given as follow :

![image3]

Here are the differents results using RGB space - _on the left_, and HSV color space - _on the right_. As we can see, the first method **barely** succeded to detect the golden rock, whereas, the second one did it with more success.

![RGB detection][rgb_hsv_rock_detect]

At this point, I stopped digging into the rgb space color, for rock detection. However, it is really likely, that there are several better rgb threshold that should bring up better results.

**Warning** : It is important to underline the type returned by openCV function `cv2.inRange()`. To avoid undesired behaviours, it should be converted into numpy array, to match the output of the provided threshold function. It can be done with the numpy function `asarray(array_like, dtype=None, order=None)`.

**Update** : After some mapping trials, it was obvious that the golden rock detection function was spoting too many false positive. This was corrected using a more demanding lower threshold `low_hsv_thresh=(85,100,100)`.

##### Obstacle Detection <a name="1-1-b"></a>

For this second objective, we could considered the inverse of the navigable terrain as a quick solution. By doing so, we would have rock samples and the sky considered as obstacles but it would still work.

Since there is an easy anwser to that issue, and we have already done most of the work, when we modify the `color_thresh()`, we will avoid the quick solution. Once we have inverted the threshold to capture evreything but the ground, a reduction of the **red upper threshold** is enough to get rid of both the sky and the rocks.

Here are some results presented bellow, still using the same sample picture. On the right the quick method, and on the left, the answer to this little issue. We can see how both the sky and the rock are taken out of the picture. 

![RGB detection][rgb_obstacle_detect]

The following table gives the lower and upper threshold used for obstacle detection.

<center>

| R  | G  | B  |
|:--:|:--:|:--:|
| 0  | 0  | 0  |
| 100| 160| 160|

</center>

**Warning** : The given `color_thresh()` function has its boundaries stricly defined. Since the obstacle are dark, some parts are black `rgb=(0,0,0)`, to make sure we get them, the original code should be corrected using `<=` to use equal or superior. It can also be done for the other threshold.

#### 2. Identification of navigable terrain, obstacles and rock samples <a name="1-2"></a>

Once we are able to detect the ground, the rocks and the obstacles, the next goal is to map all those information into one single map. Which is what does the `process_image():` function.

The first interesting thing to notice, is that I chose to apply **color thresholding first**, even before the perspective transform.

Then I followed the basic given states :

* Perspective transform
* Rover centric conversion
* World coordinates conversion
* Data world map update
* Display

##### Data world map update <a name="1-2-a"></a>

Here I am only going to describe the data world map update, since the other states where pretty much straight forward.

As suggested in [_Optimizing Map Fidelity Tip_](https://classroom.udacity.com/nanodegrees/nd209/parts/c199593e-1e9a-4830-8e29-2c86f70f489e/modules/0573f9ff-2e23-48cd-87d0-af24d758348a/lessons/ef841d31-8c53-49b3-8da3-a9d1523adef0/concepts/ed33ab6f-2b67-4991-b372-ab915538b734), the map is only updated if both **pitch** and **roll** are under 0.8° or above 359.2°. This helps with the accuracy. Bellow are two differents mapping, the one on the left is obtained without taking care of the pitch and roll angle.

![roll and pitch mapping][roll_pitch]

We can see that there is not that much data lost appart from the obstacles - _which are not that relevant_ and that the naviguable terrain is more accurate.

As said in [RoboND Rover Project Livestream](https://www.youtube.com/watch?v=-L0Fz4UnlC8) the **further** data is captured from the rover, the less accurate it is. Only points that are within a short range are therefore updated. Bellow are two differents mapping with the distance threshold, the one on the left is obtained without taking care of the pitch and roll angle.

![distance thresholding][distance_thresh]

We can observe that the distance threshold helps a lot with the accuracy, yet, **relevant data was lost** at that price.

## Autonomous Navigation and Mapping <a name="part2"></a>

#### 1. Perception <a name="2-1"></a>

The function `perception_step(Rover)` in _perception.py_ is pretty similar to the one in the jupyter notebook. Some modifications can be noted.

* Rover vision update
* Data **worldmap** update
* Conversion to **polar coordinates**
* **Rock position**
* Detect **split** way

##### Data worldmap update <a name="2-1-a"></a>

For a better accuracy, the map update is **weighted**. The obstacle value was chose a third of the navigable terrain, so that to overwrite a navigable terrain pixel, it will need to be spotted at least 4 times more than navigable pixels. This change was made possible thanks to the following block in the `update_rover()` function :

```python
      # Create a scaled map for plotting and clean up obs/nav pixels a bit
      if np.max(Rover.worldmap[:,:,2]) > 0:
            nav_pix = Rover.worldmap[:,:,2] > 0
            navigable = Rover.worldmap[:,:,2] * (255 / np.mean(Rover.worldmap[nav_pix, 2]))
      else: 
            navigable = Rover.worldmap[:,:,2]
      if np.max(Rover.worldmap[:,:,0]) > 0:
            obs_pix = Rover.worldmap[:,:,0] > 0
            obstacle = Rover.worldmap[:,:,0] * (255 / np.mean(Rover.worldmap[obs_pix, 0]))
      else:
            obstacle = Rover.worldmap[:,:,0]

      likely_nav = navigable >= obstacle
      obstacle[likely_nav] = 0
```

##### Polar coordinates <a name="2-1-b"></a>

To guide the rover through the map, it needs an angle and an acceleration/speed amplitude. This can be embody with a direction vector. The function defined during the class remained unchanged. Here is a way of defining the direction vector :

* **Steer angle** : _mean polar angle_ of navigable terrain
* **Max velocity** : proportional to _mean polar distance_ of navigable terrain

The same strategy will be used to search golden rocks.

##### Rock position <a name="2-1-c"></a>

Because the perspective transform mostly accurate for the ground, the golden rocks are distorted on the map. The polar angle stays pretty precise, but the mean distance is false. Only the **closest position** should be considered. This also helps to deal with the situation when 2 rocks are spotted at the same time.

```python
Rover.rck_dists = np.amin(Rover.rck_dists)
```

##### Split way detection <a name="2-1-d"></a>

When the rover face a small obstacle, the mean angle could leads him straight to the obstacle, as shown bellow. 

![split ways]

If we sum up all the columns to get a 1D array, in a normal situation we get a [**gaussian distribution**](http://mathworld.wolfram.com/NormalDistribution.html) - _blue curve bellow_ . So if we are able to get the center of the estimated gaussian, we can then check is the value is greater than the mean value. If not, it obvioulsy means that we are not at a maximum as expected. The _red curve_ shows what we could expect in a split way situation.


![gaussian]

The algorithm chosen to detect this scenario roughly estimates the center of the gaussian, using threshold.

```python
    for val in nav_2d:
        # value threshold to avoid undesired small values
        if(val > 25):  
            gaussian_detect += 1
            gaussian_size += 1
            first_detec = True
        # try to find the offset to the gaussian center
        elif(gaussian_detect == 0 and (not first_detec)):
            nav_counter += 1
        # end of the gaussian
        elif(gaussian_detect > 0):
            gaussian_detect -= 10 # decrement faster
            gaussian_size += 1
            
    # Truncate to only get the gaussian
    nav_2d_truc = nav_2d [(nav_counter):(nav_counter + gaussian_size)] 
	# Mean value of the gaussian
    nav_mean = np.mean(nav_2d_truc)
	# Estimated center of the gaussian
    gaussian_center = (gaussian_size/2) + (nav_counter)
```

The threshold value should be estimated more precisely, but after a single trial, it seemed to bring good results.

**Upgrade** As an unexpected behaviour, this little algorithm, seems to helps following left wall by making the rover turns right when facing a wall - _sometimes at least_.

#### 2. Autonomous decision making <a name="2-2"></a>

The function `decision_step(Rover)` in _decision.py_  acts quite like a state machine, to help guide the rover through its way.

* **Stuck** scenario
* Obstacle avoidance
* Wall **crawling**
* **Smoothing** the steering
* Rock **search & collect**

##### Stuck Scenarios <a name="2-2-a"></a>

One of the first thing we realize while in autonomous mode, is that the rover get stuck all the time. To try to answer those _uncomfortable scenarios_, the **watchdog timer** seems a good solution. At every update of the rover, the watchdog check if a determined time has elapsed since last checking. If true, it compares the last checked position with the current position. If the difference is within a close radius, therefore, it enter 'stuck mode'.

```python
def stuck_watchdog(Rover, radius=0.20, elapsed_time=5.0):
    # check if enough time has elapsed since last check
    if (time.time() - Rover.time_counter) > elapsed_time:
        
        # Update counter
        Rover.time_counter = time.time()
            
            # Check if rover has moved since last check
        if (np.absolute(Rover.pos[0] - Rover.stuck_pos[0]) < radius) and (np.absolute(Rover.pos[1] - Rover.stuck_pos[1]) < radius):
                
            # Update rover yaw stuck position
            Rover.stuck_yaw = Rover.yaw
            Rover.mode = 'stuck'
    
        # Update rover stuck position
        Rover.stuck_pos = Rover.pos

    return
```

Once we know the rover is stuck, we cannot send him back to the classic '_stop_' case, the rover would not be stuck if this case could help.

The '_stuck_' mode is almost the same as the stop mode, the main difference is defined by how the rover gets out of the state. Instead of looking to navigable spaces, it **compares** its current **yaw**, with the yaw it had when it got stuck. When the rover has steered enough, it tries to go forward.

##### Obstacle avoidance <a name="2-2-b"></a>

In the '_forward_ ' mode, if the mean navigable distance is low, - _which mean that there is probably a wall ahead_ - the rover switch to '_stop_ ' mode.

```python
# Check for front obstacles
if np.mean(Rover.nav_dists) < 7.0 and Rover.vel > 0.2:
    Rover.mode = 'stop'        
```

##### Wall crawler, _lefty I guess_... <a name="2-2-c"></a>

In [Optimizing for Finding All Rocks Tip](https://classroom.udacity.com/nanodegrees/nd209/parts/c199593e-1e9a-4830-8e29-2c86f70f489e/modules/0573f9ff-2e23-48cd-87d0-af24d758348a/lessons/ef841d31-8c53-49b3-8da3-a9d1523adef0/concepts/ed33ab6f-2b67-4991-b372-ab915538b734), it was suggested to make the rover a wall [crawler](https://youtu.be/G0-0ekfyP0A?t=1m31s), and in the [RoboND Rover Project Livestream](https://www.youtube.com/watch?v=-L0Fz4UnlC8), since the left polls where higher, I chose to make it a left wall crawler.

To do so, if there is a **wide area**, the rover will turn left until it meets a wall, then it will enter the '_stop_ ' mode and will make a little right turn. 

To force a left turn, **a positive offset** is added to the steering computation as shown bellow.

```python
# If there is a wide area the rover is going to look for a left wall
if len(Rover.nav_angles) > 1300 and Rover.vel > 0.2:
# Set steering to average angle clipped to the range +/- 15 
# with an offset to make it a wall crawler
    val = np.clip(np.clip(np.mean(Rover.nav_angles * 180/np.pi), \
    -15, 15) + 13.0, -15, 15)
    Rover.steer = queue_mean(val, Rover.steer_queue, Rover.steer_queue_max_size)
                
else:
# Set steering to average angle clipped to the range +/- 15 
# with a smaller offset
    val = np.clip(np.mean(Rover.nav_angles * 180/np.pi + 5), -15, 15)
    Rover.steer = queue_mean(val, Rover.steer_queue, Rover.steer_queue_max_size)
```

If the rover is in a corridor or close to a wall, there will be less navigable space, and it will apply a smaller offset that is within the `np.clip(val + l_offset, -15, 15)` function, the rover will still be able to make right turn if necessary - _to avoid wall collision_.


##### Smoothing the steering wheel, _not the baby driver yet_... <a name="2-2-d"></a>

Since the rover driving algorithm cannot yet compete with [_Ansel Elgort_](https://www.youtube.com/watch?v=l9JG2MCQGJY), while in forward mode, the steering is smoothed with a running average. The size of the running average can be set to any value - _the greater the smoother_ - and the value are dealt with like in a FIFO.

```python
def queue_mean(val, ar, size):
    if len(ar) < size:        
        ar.append(val)
        return np.mean(ar)
                
    else:
        # Replace older elem
        ar[0] = val
        # Left shift
        tmp = ar[1:] + ar[:1]
        for i in range(len(tmp)):
            ar[i] = tmp[i]
        
        return np.mean(ar)
```

##### Golden rock search and pick up <a name="2-2-e"></a>

The main goal to achieved this project was to map the simulated area with the greatest fidelity, so I did not wanted the rover to go for a golden nugget that was too far - _with some luck they will meet again_ - and get lost in the process.

The first thing I did was to make it go for the rock only in some conditions :

```python
if Rover.rck_dists < 30 and \
np.absolute(np.mean(Rover.rck_angles) - np.mean(Rover.nav_angles)) < 60:
```

If the conditions are met, the rover brakes, and enter the '_gld rock_' mode. Therefore it tries to follow the golden rock at slow speed with the following process :

```python
if Rover.mode == 'gld rock':            
    Rover.debug = 'gld rock'
    # Is rock still on sight?
    if Rover.rck_dists != None:
        if Rover.vel > 0.5:
            Rover.debug = 'gld rock too fast'
            Rover.throttle = 0
            Rover.brake = Rover.brake_set
            Rover.steer = np.clip(np.mean(Rover.rck_angles * 180/np.pi), -15, 15)
            
        elif Rover.vel < 0.2 and not Rover.near_sample:
            Rover.debug = 'gld rock too slow'
            Rover.throttle = 0.4
            Rover.brake = 0
            Rover.steer = np.clip(np.mean(Rover.rck_angles * 180/np.pi), -15, 15)
                
        elif Rover.vel < 0.2 and Rover.near_sample and not Rover.picking_up:
            Rover.debug = 'gld rock picking yeah'
            Rover.send_pickup = True
                
        else:
            Rover.throttle = 0.1
            Rover.brake = 0
            Rover.steer = np.clip(np.mean(Rover.rck_angles * 180/np.pi), -15, 15)
    
        return Rover
     # No rock on sight anymore :'(
    else:                 
        Rover.mode = 'forward'
        return Rover
```

If at any point it loses sight of the rock, the rover goes back to its normal behaviour.

In a desperate try to increase the chances to pick up a rock, I put the pick up block as the first test of the `decision_step(Rover)`function.

```python
# If in a state where want to pickup a rock send pickup command
if Rover.near_sample and Rover.vel == 0 and \ 
  not Rover.picking_up:
    Rover.send_pickup = True
```

##### Stuck Scenarios _the return, the endless loop hell_

This _unbearable scenario_ has not yet been dealt with...

![circle hell]

## Results <a name="part3"></a>

Most of the time, the rover will be able to get a **single or two rocks within 600s**. With an average FPS between 2 and 3, I guess the rover is not reactive enough, and my rock search algorithm is not really efficient.

As said before, I concentrated myself into mapping. If the rover does not get into the loop hell, he is able to map at least 45% of the map with a **fidelity between 70% and 75%** most of the time.

I did not implant a path planning algorithm to avoid returning to places already visited. This could obviously upgrade the percentage of the map the rover is able to visit.

The split way solution, also brings undesired behaviour. Sometimes, the rover is really close to getting a rock and he runs away from it.

After some time, the rover seems to have a not negligeable lag. Its behaviour is then umpredictable.

Sometimes the '_stuck_ ' mode is not enough, an other watchdog or a counter should be implemented to try an other solution if the rover is blocked.

Making the rover a wall crawler, made it bump on the wall regularly. I was not able to find a more efficient offset, after several trials and errors, _13.0_ seems a good choice.

_There is lot of space for improvements..._ but since it happened once, I'll show the most amazing run the rover suceeded to do ! It might have been touched by [Sébastien Loeb](https://en.wikipedia.org/wiki/Sébastien_Loeb) this time !

![best run 1]

![best run 2]

_Then it becames less of a pilot..._ kept going back inside the northen corridor.
