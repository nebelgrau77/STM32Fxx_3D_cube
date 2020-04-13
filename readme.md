# rotating 3D cube - WORK IN PROGRESS 

Trying to port it to Rust from this example: 

https://www.twobitarcade.net/article/3d-rotating-cube-micropython-oled/

It's not working correctly: something is off with the projection, as the cube converges to a single line (X or Y axis) 
or a single point (Z-axis). 

Corrected the front-to-back edges from the example (they were diagonals of the top and bottom side instead of edges).

Code needs major refactoring which I currently don't know how to do:

- store the vertices as an array or Point3D elements: this works, but gets complicated with accessing and modifying them because of the borrowing/references
- create the edges with a function that takes two vertices and a style as arguments, and returns a line (problems with some parts in the 
embedded-graphics being private)
- ideally the cube would be a struct, this way I could easily create multiple cubes at once

To do:
- fix the rotation problems
- step 1: connect the X and Y axis rotation to a joystick
- step 2: connect the rotation to the MCU6050 sensor
- 


//! MCU: STM32F051
//! 
//! cube rotation
//! ported to Rust from this example:
//! https://www.twobitarcade.net/article/3d-rotating-cube-micropython-oled/
//! (edges connecting the front and back are wrong in the example above, corrected here)
//! 
//! DOESN'T WORK RIGHT YET:
//! something is off with the projection: the cube converges into a single point (Z-axis) 
//! or a single line (X-axis or Y-axis)
//! 
//! needs refactoring:
//! - vertices stored in an array
//! - edges created with a function, with two vertices as args
//! 