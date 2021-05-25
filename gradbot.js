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
 * @param {*} heading  - Angle (0-360) of the part. 
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
    this.head %= 360;
    if(this.heading < 0) {
        this.heading += 360;
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
     * Send power to the part's positive port. 
     * @param {*} level - The amount of power applied to the positive terminal. 
     */
    this.pos = function(level){ };


    /**
     * Send power to the part's negative port. 
     * @param {*} level - The amount of power applied to the negative terminal. 
     */
    this.neg = function(level){ };


    /**
     * Update the part's state.
     */
    this.update = function() { };
}


function Motor(x, y, heading)
{
    //construct the part
    Part.call(this, x, y, heading, 5, 2);

    //set up the power settings
    this.plevel = 0;
    this.nlevel = 0;
    this.pos = function(level) {
        this.plevel = level;
    };
    this.neg = function(level) {
        this.nlevel = level;
    };

    // handle speed of the motor
    this.speed = 0;
    this.update = function() {
        var ps = 0;
        if(this.nlevel < 0 && this.plevel > 0) {
            ps = this.plevel;
        } else if(this.nlevel > 0 && this.plevel < 0) {
            ps = this.plevel;
        } 

        this.speed = 180 * ps / 100;
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
    };
}