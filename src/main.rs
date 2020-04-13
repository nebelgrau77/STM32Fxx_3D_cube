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

#![no_std]
#![no_main]

extern crate cortex_m;
extern crate cortex_m_rt as rt;
//extern crate panic_halt;
extern crate panic_semihosting;
extern crate stm32f0xx_hal as hal;

use cortex_m_semihosting::hprintln;

use cortex_m_rt::entry;

use ssd1306::{prelude::*, Builder as SSD1306Builder};

use micromath::F32Ext;

use crate::hal::{
    prelude::*,
    stm32,
    delay::Delay,
    i2c::I2c,
};

use embedded_graphics::{
    pixelcolor::BinaryColor,
    prelude::*,
    style::{PrimitiveStyle,PrimitiveStyleBuilder},
    primitives::Line,
    geometry::Point,
};

const BOOT_DELAY_MS: u16 = 200;

// for the 3D animation:

struct Point3D {
    x: f32,
    y: f32,
    z: f32,
}


const SCREENWIDTH: u8 = 128;
const SCREENHEIGHT: u8 = 64;
const FIELD_OF_VIEW: u8 = 64;
const VIEWER_DISTANCE: u8 = 4;

#[entry]

fn main() -> ! {
    
    let mut p = stm32::Peripherals::take().unwrap();
    let mut cp = cortex_m::peripheral::Peripherals::take().unwrap();

    cortex_m::interrupt::free(move |cs| {

        let mut rcc = p.RCC.configure().sysclk(48.mhz()).freeze(&mut p.FLASH);

        let mut delay = Delay::new(cp.SYST, &rcc);

        delay.delay_ms(BOOT_DELAY_MS);

        let gpiob = p.GPIOB.split(&mut rcc);
        let scl = gpiob.pb8.into_alternate_af1(cs);
        let sda = gpiob.pb7.into_alternate_af1(cs);
        let i2c = I2c::i2c1(p.I2C1, (scl, sda), 400.khz(), &mut rcc);


        let mut disp: GraphicsMode<_> = SSD1306Builder::new().size(DisplaySize::Display128x64).connect_i2c(i2c).into();

        disp.init().unwrap();

        let style_on = PrimitiveStyleBuilder::new().stroke_width(1).stroke_color(BinaryColor::On).build();
        let style_off = PrimitiveStyleBuilder::new().stroke_width(1).stroke_color(BinaryColor::Off).build();


        /*
        let mut vertices: [Point3D;8] = [
            Point3D{x:-1,y:1,z:-1},
            Point3D{x:1,y:1,z:-1},
            Point3D{x:1,y:-1,z:-1},
            Point3D{x:-1,y:-1,z:-1},
            Point3D{x:-1,y:1,z:1},
            Point3D{x:1,y:1,z:1},
            Point3D{x:1,y:-1,z:1},
            Point3D{x:-1,y:-1,z:1}
            ];
        */

        let mut pointA = Point3D{x:-1.0, y: 1.0, z:-1.0};
        let mut pointB = Point3D{x: 1.0, y: 1.0, z:-1.0};
        let mut pointC = Point3D{x: 1.0, y:-1.0, z:-1.0};        
        let mut pointD = Point3D{x:-1.0, y:-1.0, z:-1.0};
        let mut pointE = Point3D{x:-1.0, y: 1.0, z: 1.0};
        let mut pointF = Point3D{x: 1.0, y: 1.0, z: 1.0};
        let mut pointG = Point3D{x: 1.0, y:-1.0, z: 1.0};
        let mut pointH = Point3D{x:-1.0, y:-1.0, z: 1.0}; 

        let mut vertexA = projection(&pointA, SCREENWIDTH, SCREENHEIGHT, FIELD_OF_VIEW, VIEWER_DISTANCE);
        let mut vertexB = projection(&pointB, SCREENWIDTH, SCREENHEIGHT, FIELD_OF_VIEW, VIEWER_DISTANCE);
        let mut vertexC = projection(&pointC, SCREENWIDTH, SCREENHEIGHT, FIELD_OF_VIEW, VIEWER_DISTANCE);
        let mut vertexD = projection(&pointD, SCREENWIDTH, SCREENHEIGHT, FIELD_OF_VIEW, VIEWER_DISTANCE);
        let mut vertexE = projection(&pointE, SCREENWIDTH, SCREENHEIGHT, FIELD_OF_VIEW, VIEWER_DISTANCE);
        let mut vertexF = projection(&pointF, SCREENWIDTH, SCREENHEIGHT, FIELD_OF_VIEW, VIEWER_DISTANCE);
        let mut vertexG = projection(&pointG, SCREENWIDTH, SCREENHEIGHT, FIELD_OF_VIEW, VIEWER_DISTANCE);
        let mut vertexH = projection(&pointH, SCREENWIDTH, SCREENHEIGHT, FIELD_OF_VIEW, VIEWER_DISTANCE);


        loop {            

            //SOMETHING IS WRONG

            

                //back
                Line::new(Point::new(vertexA.x as i32, vertexA.y as i32),Point::new(vertexB.x as i32, vertexB.y as i32)).into_styled(style_off).draw(&mut disp).unwrap();
                Line::new(Point::new(vertexB.x as i32, vertexB.y as i32),Point::new(vertexC.x as i32, vertexC.y as i32)).into_styled(style_off).draw(&mut disp).unwrap();
                Line::new(Point::new(vertexC.x as i32, vertexC.y as i32),Point::new(vertexD.x as i32, vertexD.y as i32)).into_styled(style_off).draw(&mut disp).unwrap();
                Line::new(Point::new(vertexD.x as i32, vertexD.y as i32),Point::new(vertexA.x as i32, vertexA.y as i32)).into_styled(style_off).draw(&mut disp).unwrap();
                    
                //front
                Line::new(Point::new(vertexE.x as i32, vertexE.y as i32),Point::new(vertexF.x as i32, vertexF.y as i32)).into_styled(style_off).draw(&mut disp).unwrap();
                Line::new(Point::new(vertexF.x as i32, vertexF.y as i32),Point::new(vertexG.x as i32, vertexG.y as i32)).into_styled(style_off).draw(&mut disp).unwrap();
                Line::new(Point::new(vertexG.x as i32, vertexG.y as i32),Point::new(vertexH.x as i32, vertexH.y as i32)).into_styled(style_off).draw(&mut disp).unwrap();
                Line::new(Point::new(vertexH.x as i32, vertexH.y as i32),Point::new(vertexE.x as i32, vertexE.y as i32)).into_styled(style_off).draw(&mut disp).unwrap();
        
                //front to back
                Line::new(Point::new(vertexA.x as i32, vertexA.y as i32),Point::new(vertexE.x as i32, vertexE.y as i32)).into_styled(style_off).draw(&mut disp).unwrap();
                Line::new(Point::new(vertexB.x as i32, vertexB.y as i32),Point::new(vertexF.x as i32, vertexF.y as i32)).into_styled(style_off).draw(&mut disp).unwrap();
                Line::new(Point::new(vertexC.x as i32, vertexC.y as i32),Point::new(vertexG.x as i32, vertexG.y as i32)).into_styled(style_off).draw(&mut disp).unwrap();
                Line::new(Point::new(vertexD.x as i32, vertexD.y as i32),Point::new(vertexH.x as i32, vertexH.y as i32)).into_styled(style_off).draw(&mut disp).unwrap();

/*
                pointA = rotateX(&pointA, 5);
                pointB = rotateX(&pointB, 5);
                pointC = rotateX(&pointC, 5);
                pointD = rotateX(&pointD, 5);
                pointE = rotateX(&pointE, 5);
                pointF = rotateX(&pointF, 5);
                pointG = rotateX(&pointG, 5);
                pointH = rotateX(&pointH, 5);

                pointA = rotateY(&pointA, 5);
                pointB = rotateY(&pointB, 5);
                pointC = rotateY(&pointC, 5);
                pointD = rotateY(&pointD, 5);
                pointE = rotateY(&pointE, 5);
                pointF = rotateY(&pointF, 5);
                pointG = rotateY(&pointG, 5);
                pointH = rotateY(&pointH, 5);
*/
                pointA = rotateZ(&pointA, 5);
                pointB = rotateZ(&pointB, 5);
                pointC = rotateZ(&pointC, 5);
                pointD = rotateZ(&pointD, 5);
                pointE = rotateZ(&pointE, 5);
                pointF = rotateZ(&pointF, 5);
                pointG = rotateZ(&pointG, 5);
                pointH = rotateZ(&pointH, 5);

                

                vertexA = projection(&pointA, SCREENWIDTH, SCREENHEIGHT, FIELD_OF_VIEW, VIEWER_DISTANCE);
                vertexB = projection(&pointB, SCREENWIDTH, SCREENHEIGHT, FIELD_OF_VIEW, VIEWER_DISTANCE);
                vertexC = projection(&pointC, SCREENWIDTH, SCREENHEIGHT, FIELD_OF_VIEW, VIEWER_DISTANCE);
                vertexD = projection(&pointD, SCREENWIDTH, SCREENHEIGHT, FIELD_OF_VIEW, VIEWER_DISTANCE);
                vertexE = projection(&pointE, SCREENWIDTH, SCREENHEIGHT, FIELD_OF_VIEW, VIEWER_DISTANCE);
                vertexF = projection(&pointF, SCREENWIDTH, SCREENHEIGHT, FIELD_OF_VIEW, VIEWER_DISTANCE);
                vertexG = projection(&pointG, SCREENWIDTH, SCREENHEIGHT, FIELD_OF_VIEW, VIEWER_DISTANCE);
                vertexH = projection(&pointH, SCREENWIDTH, SCREENHEIGHT, FIELD_OF_VIEW, VIEWER_DISTANCE);
                
                //back
                Line::new(Point::new(vertexA.x as i32, vertexA.y as i32),Point::new(vertexB.x as i32, vertexB.y as i32)).into_styled(style_on).draw(&mut disp).unwrap();
                Line::new(Point::new(vertexB.x as i32, vertexB.y as i32),Point::new(vertexC.x as i32, vertexC.y as i32)).into_styled(style_on).draw(&mut disp).unwrap();
                Line::new(Point::new(vertexC.x as i32, vertexC.y as i32),Point::new(vertexD.x as i32, vertexD.y as i32)).into_styled(style_on).draw(&mut disp).unwrap();
                Line::new(Point::new(vertexD.x as i32, vertexD.y as i32),Point::new(vertexA.x as i32, vertexA.y as i32)).into_styled(style_on).draw(&mut disp).unwrap();
                    
                //front
                Line::new(Point::new(vertexE.x as i32, vertexE.y as i32),Point::new(vertexF.x as i32, vertexF.y as i32)).into_styled(style_on).draw(&mut disp).unwrap();
                Line::new(Point::new(vertexF.x as i32, vertexF.y as i32),Point::new(vertexG.x as i32, vertexG.y as i32)).into_styled(style_on).draw(&mut disp).unwrap();
                Line::new(Point::new(vertexG.x as i32, vertexG.y as i32),Point::new(vertexH.x as i32, vertexH.y as i32)).into_styled(style_on).draw(&mut disp).unwrap();
                Line::new(Point::new(vertexH.x as i32, vertexH.y as i32),Point::new(vertexE.x as i32, vertexE.y as i32)).into_styled(style_on).draw(&mut disp).unwrap();
        
                //front to back
                Line::new(Point::new(vertexA.x as i32, vertexA.y as i32),Point::new(vertexE.x as i32, vertexE.y as i32)).into_styled(style_on).draw(&mut disp).unwrap();
                Line::new(Point::new(vertexB.x as i32, vertexB.y as i32),Point::new(vertexF.x as i32, vertexF.y as i32)).into_styled(style_on).draw(&mut disp).unwrap();
                Line::new(Point::new(vertexC.x as i32, vertexC.y as i32),Point::new(vertexG.x as i32, vertexG.y as i32)).into_styled(style_on).draw(&mut disp).unwrap();
                Line::new(Point::new(vertexD.x as i32, vertexD.y as i32),Point::new(vertexH.x as i32, vertexH.y as i32)).into_styled(style_on).draw(&mut disp).unwrap();
        
    
                disp.flush().unwrap();

                //delay.delay_ms(50_u16);
    






        }


        

    });


    loop {continue;}

}

// On each iteration we apply the rotational transformations to each point, then project it onto our 2D surface.


fn trig_func(angle: i16) -> (f32, f32) {
    // helper function for calculation of the sinus and cosinus of an angle
    let rad: f32 = angle as f32 * core::f32::consts::PI / 180.0;
    let sin_a = rad.sin();
    let cos_a = rad.cos();
    return (sin_a, cos_a);
}

fn rotateX(point: &Point3D, x_angle: i16) -> Point3D {

    //rotates a point around X, Y, Z axis by given angles
    //returns it projected on a 2D plane

    let mut new_point: Point3D = Point3D {x: point.x, y:point.y, z: point.z};
        
    //rotation around the X axis
    let (sin_xa, cos_xa) = trig_func(x_angle);
    new_point.y = (new_point.y as f32) * cos_xa - (new_point.z as f32) * sin_xa;
    new_point.z = (new_point.y as f32) * sin_xa + (new_point.z as f32) * cos_xa;

    return new_point;

}

fn rotateY(point: &Point3D, y_angle: i16) -> Point3D {

    //rotates a point around X, Y, Z axis by given angles
    //returns it projected on a 2D plane

    let mut new_point: Point3D = Point3D {x: point.x, y:point.y, z: point.z};
        
    //rotation around the Y axis
    let (sin_ya, cos_ya) = trig_func(y_angle);
    new_point.x = (new_point.z as f32) * sin_ya + (new_point.x as f32) * cos_ya;
    new_point.z = (new_point.z as f32) * cos_ya - (new_point.x as f32) * sin_ya;

    return new_point;

}

fn rotateZ(point: &Point3D, z_angle: i16) -> Point3D {

    //rotates a point around X, Y, Z axis by given angles
    //returns it projected on a 2D plane

    let mut new_point: Point3D = Point3D {x: point.x, y:point.y, z: point.z};
        
    //rotation around the Z axis
    let (sin_za, cos_za) = trig_func(z_angle);
    new_point.x = (new_point.x as f32) * cos_za - (new_point.y as f32) * sin_za;
    new_point.y = (new_point.x as f32) * sin_za + (new_point.y as f32) * cos_za;

    return new_point;

}




fn projection(point: &Point3D, width: u8, height: u8, fov: u8, distance: u8) -> Point3D {
        
    //returns it projected on a 2D plane
    
    let mut new_point: Point3D = Point3D {x: point.x, y:point.y, z: point.z};
        
    let factor: f32 = fov as f32 / (distance as f32 + point.z as f32);
    new_point.x = new_point.x as f32 * factor + width as f32 / 2.0;
    new_point.y = -new_point.y as f32 * factor + height as f32 / 2.0;
    
    return new_point;
    
    }
    