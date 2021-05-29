/*
    This program is a simulator for building robots.
    Copyright (C) 2021 Robert Lowe <rlowe@semo.edu>

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */ 
/**
 * @file gradbot.js
 * @copyright Robert Lowe 2021
 * @license GPL 3.0 
 */

/**
 * The Part object is the base of all robot parts. 
 * @param {*} parent  - Parent container of the part.
 * @param {*} x - X coordinate of the part.
 * @param {*} y  - Y coordinate of the part.
 * @param {*} heading  - Angle (0-2*Pi) of the part. 
 * @param {*} width  - Width of the part.
 * @param {*} height - Height of the part.
 */
function Part(parent, x, y, heading, width, height) 
{
    // populate the fields
    this.parent = parent;
    this.x = x != undefined ? x : 0;
    this.y = y != undefined ? y : 0;
    this.heading = heading != undefined ? heading : 0;
    this.width = width != undefined ? width : 0;
    this.height = height != undefined ? height : 0;

    // correct the heading range
    var twoPi = 2 * Math.PI;
    this.head %= twoPi;
    if(this.heading < 0) {
        this.heading += twoPi;
    }

    //set up the functions By default, they do nothing

    /**
    * Send data to the part.
     * @param {*} data - The data to send.
     */
    this.send = function(data){ };

    /**
     * Receive data from the part.
     * @returns The data from the part.
     */
    this.receive = function(){ };

    /**
     * The power level of the part.
     */
    this.power = 0;

    /**
     * Set the power level of the part. Power level is a number between
     * 0 and 100 inclusive.
     * @param {*} power - The power level.
     */
    this.setPower = function(power) {
        this.power = power;
    };


    /**
     * Update the part's state.
     */
    this.update = function() { };
}


function Motor(x, y, heading)
{
    //construct the part
    Part.call(this, x, y, heading, 5, 2);

    // handle speed of the motor
    this.speed = 0;  // motor speed in radians per second
    this.update = function() {
        //we are basing this on the sparkfun hobby motors which spin at 65 RPM (max)
        //This maximum speed is roughly 6.81 radians per second
        this.speed = 6.81 * this.power / 100;
    }
}


function Chassis(x, y, heading) 
{
    Part.call(this, x, y, 14, 20)

    //handle the subparts of the chassis
    this.parts = Array();
    this.addPart = function(p) {
        this.parts.push(p);
    };

    // create the left and right motors
    this.left = new Motor(-2, 15);
    this.right = new Motor(14, 15);

    this.update = function() 
    {
        //update all the sub parts
        for(var p in this.parts) {
            p.update();
        }

        //update the motors
        this.left.update();
        this.right.update();

        //compute our forward translation and yaw speeds
        var r = 0.065; // 65mm diameter wheels
        var l = 0.238; // 238mm axel length
        var fwd = r/2 * (this.left.speed + this.right.speed);
        var yaw = r/l * (this.right.speed - this.left.speed);

        //populate the last update (if needed)
        if(this.lastUpdate == undefined) {
            this.lastUpdate = Date.now();
        }

        //compute elapsed time
        var cur = Date.now();
        var elapsed = (cur - this.lastUpdate) / 1000;
        this.lastUpdate = cur;

        //perform translation
        this.x += fwd * Math.cos(this.heading) * elapsed;
        this.y += fwd * Math.sin(this.heading) * elapsed;
        this.heading += yaw * elapsed;
    };
}


function VectorView(x, y, heading, scale, points) {
    // set up the fields
    this.x = x != undefined ? x : 0;
    this.y = y != undefined ? y : 0;
    this.heading = heading != undefined ? heading : 0;
    this.scale = scale != undefined ? scale : 1;
    this.points = points != undefined ? points : {};
    this.outline = undefined;
    this.fill = undefined;

    // draw the shape
    this.draw = function(canvas, context) {
        var x;
        var y;
        var started = false;

        // skip the blank shapes
        if(this.points.length == 0) {
            return;
        }

        //compute rotation coeffecients
        var sin_th = Math.sin(this.heading);
        var cos_th = Math.cos(this.heading);

        context.beginPath();
        for(var i in points) {
            var p = points[i];

            // get the raw point and scale
            x = p.x * this.scale;
            y = p.y * this.scale;

            // rotate
            var rx, ry;
            rx = x * cos_th - y * sin_th;
            ry = x * sin_th + y * cos_th;
            x = rx;
            y = ry;

            // translate
            x += this.x;
            y += this.y;

            if(started) {
                context.lineTo(x, y);
            } else {
                context.moveTo(x, y);
                started = true;
            }
        }
        context.closePath();

        // set the colors, if needed
        if(this.outline) {
            context.strokeStyle = this.outline;
        } 
        if(this.fill) {
            context.fillStyle = this.fill;
        }
        
        //draw the path
        context.fill();
        context.stroke();
    }
}
