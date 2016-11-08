/// @description Insert description here
// You can write your code in this editor

new_x = mouse_x-500
new_y = mouse_y-500

angle = darctan2(y-new_y, new_x-x) - theta

if (abs(angle) > 180){
            if (angle > 0)
                angle = -(360 - angle)
            else
                angle = 360 + angle
}
theta += angle

//move forwards

alarm[0] = 30
