![wego](https://user-images.githubusercontent.com/108254705/186351817-5becc3a2-193c-42ba-bfb1-c69a68793806.jpg)

# this is self-driving project using LIMO platform by ros

![LIMO](https://user-images.githubusercontent.com/108254705/186352378-ec45f209-4c7a-4a15-9984-02d90ca82962.jpg)

### There are depth camera, LiDAR, IMU sensor on LIMO platform.

***

![LiDAR](https://user-images.githubusercontent.com/108254705/186352724-907a4902-391f-4c5c-98fe-27b1494014a3.jpg)

### LiDAR sensor keeps rotating rapidly and detect obstacle nearby.

***

![LIMO2](https://user-images.githubusercontent.com/108254705/186352596-25e4ec85-e3af-46cd-965d-0c6f2d76ec1c.jpg)

### You need some abilities to understand this document.

***

![perpective_transform](https://user-images.githubusercontent.com/108254705/186352856-d07b4a86-cada-423a-a38a-f4155460f272.jpg)

## perspective transform : It let us see the road above like *bird view*

![HSV_filter](https://user-images.githubusercontent.com/108254705/186353246-de4b5383-a892-4940-b705-bfe8b616d96c.jpg)

## HSV filter : We will detect *yellow line* only, so we can adjust *Saturation* through trackbar on screen

![ROI](https://user-images.githubusercontent.com/108254705/186353513-5bb1d591-f6b7-4895-a7b4-8296422451ea.jpg)

## ROI : It stands for 'Region Of Interest', we do not care about noise that has yellow color. ex) yellow car and pedestrian

![Sliding_Window](https://user-images.githubusercontent.com/108254705/186353763-0c4a3e3f-d847-4db1-9f2c-ee8a2c40b2b8.jpg)

## Sliding Window : After all those data processing, now we will detect the lane gradient from 'Sliding Window'. It makes histogram and make small box on that axe, so that we can calculate gradient between those boxes.

Any question, contact : tim4688@naver.com
