Autonomous-driving car software programming project successfully done!
![IMG_2967](https://user-images.githubusercontent.com/108254705/187071300-166fe58c-7126-4919-89d9-7ab65852131e.gif)



# Hardware : LIMO robot
### There are depth camera, LiDAR, IMU sensor on LIMO platform.

![LIMO](https://user-images.githubusercontent.com/108254705/186352378-ec45f209-4c7a-4a15-9984-02d90ca82962.jpg)

***
<br/>
<br/>
<br/>
<br/>

## What is LiDAR??

![LiDAR](https://user-images.githubusercontent.com/108254705/186352724-907a4902-391f-4c5c-98fe-27b1494014a3.jpg)

### :arrow_right: LiDAR sensor keeps rotating rapidly and detect obstacle nearby.

***
<br/>
<br/>
<br/>
<br/>

![LIMO2](https://user-images.githubusercontent.com/108254705/186352596-25e4ec85-e3af-46cd-965d-0c6f2d76ec1c.jpg)

### :arrow_right: You need some abilities to understand this document.

***
<br/>
<br/>
<br/>
<br/>

![perpective_transform](https://user-images.githubusercontent.com/108254705/186352856-d07b4a86-cada-423a-a38a-f4155460f272.jpg)

## :arrow_right: perspective transform : It let us see the road above like *bird view*

<br/>
<br/>
<br/>
<br/>

![HSV_filter](https://user-images.githubusercontent.com/108254705/186353246-de4b5383-a892-4940-b705-bfe8b616d96c.jpg)

## :arrow_right: HSV filter : We will detect *yellow line* only, so we can adjust *Saturation* through trackbar on screen

<br/>
<br/>
<br/>
<br/>

![ROI](https://user-images.githubusercontent.com/108254705/186353513-5bb1d591-f6b7-4895-a7b4-8296422451ea.jpg)

## :arrow_right: ROI : It stands for 'Region Of Interest', we do not care about noise that has yellow color. ex) yellow car and pedestrian

<br/>
<br/>
<br/>
<br/>

![Sliding_Window](https://user-images.githubusercontent.com/108254705/186353763-0c4a3e3f-d847-4db1-9f2c-ee8a2c40b2b8.jpg)

## :arrow_right: Sliding Window : After all those data processing, now we will detect the lane gradient from 'Sliding Window'. It makes histogram and make small box on that axe, so that we can calculate gradient between those boxes.

<br/>
<br/>
<br/>
<br/>
<br/>

Any question, contact : tim4688@naver.com
