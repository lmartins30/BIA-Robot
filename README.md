# BIA-Robot
TCC project for Control and Automation Engineering. Bibliotecary IA but this project is just about it's navigation method.

The .py archive is basically its Fuzzy Logic for the distance perception.
The robot has 3 sensors( left, center and right). Sesq is the left one, Scen is the center and the Sdir is the right.
And it had 2 motors, one for each tire( Mesq is the left and Mdir is the right)
The sensors Fuzzy Logic levels were defined as 4 (too close, close, far, too far) and the motors were 5( High Positive, Low Positive, Zero, Low Negative and High Negative).

Some rules were created for control and a main function to publish to the arduino and finally move the robot.

Many tests needed to be done because of the Fuzzyfication and the degree of membership. For example, too close degree of membership were set by the readings 0.01,0.01,0.3,0.6). Tests were made for both sensors and motors, sensors adjusting the distance and motors adjusting the speed of the axis rotation.

Since was an Arduino controlling the wheels had to make a function to publish writing down the speed to the motors and reading the sensors.

Any questions, feel free to ask ( they be can better answered in PT-br)
