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
 * @file userbot.js
 * @copyright Robert Lowe 2021
 * @license GPL 3.0 
 */

/******************************************
 * User Model of Robot Parts 
 ******************************************/

/**
 * Create a local model of a part from the given source.
 * @param {*} source 
 */
function Part(source) {
    this.name = source.name;
    this.type = source.type;

    this.toJSON = function () {
        result = {};
        for (var attr in this) {
            //skip parents and functions
            if (attr == "parent" || typeof this[attr] == "function") {
                continue;
            }
            result[attr] = this[attr];
        }
        return result;
    };

    /**
     * Return a sendable (postable to a worker) version of this object.
     */
    this.sendable = function() {
        //by default, just use the toJSON function
        return this.toJSON();
    };
}


/**
 * Local model of a motor.
 * @param source{*} The source object.
 */
function Motor(source) {
    Part.call(this, source);
    this.power = source.power;

    this.setPower = function(power) {
        this.power = power;
        postMessage(this.sendable());
    }
}


function Chassis(source) {
    Part.call(this, source);
    this.parts = [];
    this.code = source.code;

    //capture the parts
    for(var i = 0; i < source.parts.length; i++) {
        this.parts.push(new Motor(source.parts[i]));
    }
}


/**
 * Construct a local model object from the source.
 * This selects the appropriate constructor and invokes it.
 * @param {*} source 
 * @returns The user model object
 */
function constructPart(source) {
    if(source.type == "Chassis") {
        return new Chassis(source);
    } else if(source.type == "Motor") {
        return new Motor(source);
    }

    // this is an unknown part!
    return undefined;
}


/******************************************
 * Message Handler 
 ******************************************/
onmessage = function(message) {
    // handle the type of the message
    if(message.data.type == "start") {
        runRobot(message.data.robot);
    }
}


/******************************************
 * Robot Running
 ******************************************/

/**
 * Run the user robot's code.
 * @param {*} source 
 */
function runRobot(source) {
    var robot = new Chassis(source);
    var robotFun = getRobotFunction(robot);

    robotFun(robot);
}


/**
 * Get the robot's code function.
 */
function getRobotFunction(robot) {
    var preamble = "";    

    //reference all the robot variables
    for(var i=0; i<robot.parts.length; i++) {
        preamble += "var " + robot.parts[i].name + " = r.parts["+i+"];\n";
    }

    //undefine the r parameter
    preamble += "r = undefined;\n";
    preamble += "async function userFunction() {\n";

    return new Function("r", preamble + robot.code + "}\n  userFunction();");
}



/****************************************** 
 * Utility Functions 
 ******************************************/

/**
 * Delay for the specified number of milliseconds.
 * @param {*} ms - milliseonds to sleep
 */
function delay(ms) {
  return new Promise(resolve => setTimeout(resolve, ms));
}