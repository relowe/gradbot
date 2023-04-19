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


/****************************************** 
 * Utility Functions and Objects
 ******************************************/

//!!!!!!!! Sam Elfrink Additions !!!!!!!!!!
var simulationMode = 'toroidal';
var addListTrue = 0; 
var loadRobotTrue = 0;
var wheelSize = .065 // original default wheel size
var opponentClicked = 0; // check to see if the opponent as been clicked
let blackList = ["Part","Motor","Marker","Chassis","chassis","Light","LightSensor","RangeSensor","Laser","constructPart","delay","left","right","runRobot","onmessage","getRobotFunction","updateRobot"]; // blacklist of part names
let newPartList = []; // An array for the new parts
var selectPartName; // The name of the current select part
var cancelAdd = 0; // determines whether the name should be added to newPartList
//!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

/**
 * This function takes an angle in radians and reduces it so its range
 * is always within [0, 2*PI]
 * @param {number} a - The angle to reduce
 * @returns The reduced angle.
 */
function reduceAngle(a) {
    var tau = 2 * Math.PI;

    a %= tau;
    if(a<0) {
        a+= tau;
    }

    return a;
}


/**
 * Compute the minimum distance from point e to the line segment
 * ab. Each point is expected to be an object with fields x, y
 * for the coordinates. This function also computes the angle to
 * that closest point.
 * @param {object} a - Endpoint a
 * @param {object} b - Endpoint b
 * @param {object} e - Endpoint e
 * @returns {object} - an object {distance:, angle:}
 */
function minLineDist(a, b, e) {
    //vectors
    var ab = {x: b.x - a.x, y: b.y - a.y};
    var be = {x: e.x - b.x, y: e.y - b.y};
    var ae = {x: e.x - a.x, y: e.y - a.y};

    //dot products
    ab_be = ab.x * be.x + ab.y * be.y;
    ab_ae = ab.x * ae.x + ab.y * ae.y;
    
    //Minimum distance from e to the line segment
    var result = {distance: Infinity, angle: 0};

    if(ab_be > 0) {
        // Case 1 - Point b is the closest
        var y = b.y - e.y;
        var x = b.x - e.x;
        result.distance = Math.sqrt(x * x + y * y);
        result.angle = Math.atan2(y, x);
    } else if(ab_ae < 0) {
        // Case 2 - Point a is the closest
        var y = b.y - e.y;
        var x = b.x - e.x;
        result.distance = Math.sqrt(x * x + y * y);
        result.angle = Math.atan2(y, x);
    } else {
        // Case 3 - Perpendicular distance
        var x1 = ab.x;
        var y1 = ab.y;
        var x2 = ae.x;
        var y2 = ae.y;
        var mod = Math.sqrt(x1 * x1 + y1 * y1);
        result.distance = Math.abs(x1 * y2 - y1 * x2) / mod;
        result.angle = Math.atan2(ab.y, ab.x) + Math.PI/2;
    }

    return result;
}



/**
 * Find the minimum distance between point p and polygon poly.
 * It also computes the angle to the nearest point.
 * @param {object} p - The point as an x, y object.
 * @param {object} poly - A list of points in the polygon
 * @returns {object} - an object {distance:, angle:}
 */
function minPolyDist(p, poly) {
    var result = {distance: Infinity, angle: 0};
    var dist;

    //handle single point polygons 
    if(poly.length == 1) {
        var dx = poly[0].x - p.x;
        var dy = poly[0].y - p.y;
        result.distance = Math.sqrt(dx*dx + dy*dy);
        result.angle = Math.atan2(dy, dx);
        return result;
    }

    //close the path
    poly = poly.concat([poly[0]]);

    // check all the line segments
    for(var i=0; i<poly.length - 1; i++) {
        a = {x: poly[i].x, y: poly[i].y};
        b = {x: poly[i+1].x, y: poly[i+1].y};
        dist = minLineDist(a, b, p);
        if(dist.distance < result.distance) {
            result = dist;
        }
    }

    return result;
}


/**
 * Collision detection.
 * @param {object} view1 - First vector view.
 * @param {object} view2 - Second vector view.
 * @returns True if the views are in collision, false otherwise.
 */
function collision(view1, view2) {
    // Check for overlap in the x-axis
    if (view1.minx < view2.maxx && view1.maxx > view2.minx) {
      // Check for overlap in the y-axis
      if (view1.miny < view2.maxy && view1.maxy > view2.miny) {
        // The views overlap
        return true;
      }
    }
  
    // The views do not overlap
    return false;
  }

  

/**
 * This is a prototype for a positionable object. It handles all the 
 * intricacies of moving an object around.
 * @param {number} x - x coordinate 
 * @param {number} y - y coordinate
 * @param {number} heading  - the direction in which the object is facing
 */
function Positionable(x, y, heading) {
    this.x = x != undefined ? x : 0;
    this.y = y != undefined ? y : 0;
    this.heading = heading != undefined ? heading : 0;
    this.heading = reduceAngle(this.heading);

    /**
     * Rotate the positionable.
     * @param {number} a - The angle to rotate by (in radians)
     */
    this.rotate = function(a) {
        this.heading = reduceAngle(this.heading + a);
    };


    /**
     * Turn the positionable to face along the given heading.
     * @param {number} a - The new heading to face along
     */
    this.face = function(a) {
        this.heading = reduceAngle(a);
    }


    /**
     * Move the positionable to (x,y)
     * @param {*} x 
     * @param {*} y 
     */
    this.moveTo = function(x, y) {
        this.x = x;
        this.y = y;
    }
}



/****************************************** 
 * Simulation Objects
 ******************************************/

/**
 * A document for a part.
 * Functions are an array of dictionaries:
 *   { name: name of the function
 *     doc: explanation of the function
 *     params: Array( {
 *        name : name of parameter
 *        doc : document of parameter
 *     }
 *  }
 * Vars are an array of dictionaries:
 *  { name: name of var, doc: document of var }
 * @param {*} name - name of the part
 */
function PartDoc() {
    this.functions = Array();
    this.vars = Array();
    this.showName = true;

    this.display = function(parent, name) {
        var ul = document.createElement("ul");
        parent.appendChild(ul);

        // process the functions
        for(var i=0; i<this.functions.length; i++) {
            var f = this.functions[i];

            // create the list elements
            var li = document.createElement("li");
            var code = document.createElement("span");
            var funDoc = document.createElement("span");
            var parameterList = document.createElement("ul");

            //create the function document
            funDoc.innerHTML = f.doc;

            //start off the function call
            code.classList.add('code');
            code.innerHTML = "";
            if(this.showName) {
                code.innerHTML = name + ".";
            }
            code.innerHTML += f.name + "(";

            // process the parameters
            for(var j=0; j < f.params.length; j++) {
                var p = f.params[j];
                if(j != 0) { code.innerHTML += ","; }
                code.innerHTML += p.name;

                var pli = document.createElement("li");
                pli.innerHTML = "<span class=\"code\">" + p.name + "</span> - " + p.doc;
                parameterList.appendChild(pli);

            }

            code.innerHTML += ")";
            li.appendChild(code);
            li.appendChild(document.createElement("br"));
            li.appendChild(funDoc);
            li.appendChild(parameterList);
            ul.appendChild(li);
        }

        // process the vars
        for(var i=0; i<this.vars.length; i++) {
            var v = this.vars[i];
            var li = document.createElement("li");
            var code = document.createElement("span");
            var varDoc = document.createElement("span");
            code.classList.add('code');

            code.innerHTML = "";
            if(this.showName) {
                code.innerHTML = name + ".";
            }
            code.innerHTML += v.name;
            varDoc.innerHTML = " - " + v.doc;
            li.appendChild(code);
            li.appendChild(varDoc);
            ul.appendChild(li);
        }
    };
}
// !!!!!!!!!!!!!!!!!!!!!! Sam Elfrink Addition !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
/**
 * A document for a part.
 * Functions are an array of dictionaries:
 *   { name: name of the function
 *     doc: explanation of the function
 *     params: Array( {
 *        name : name of parameter
 *        doc : document of parameter
 *     }
 *  }
 * Vars are an array of dictionaries:
 *  { name: name of var, doc: document of var }
 * @param {*} name - name of the part
 */
function MovementDoc() {
    this.functions = Array();
    this.vars = Array();
    this.showName = true;

    this.display = function(parent, name) {
        var ul = document.createElement("ul");
        parent.appendChild(ul);

        // process the functions
        for(var i=0; i<this.functions.length; i++) {
            var f = this.functions[i];

            // create the list elements
            var li = document.createElement("li");
            var code = document.createElement("span");
            var funDoc = document.createElement("span");
            var parameterList = document.createElement("ul");

            //create the function document
            funDoc.innerHTML = f.doc;

            //start off the function call
            code.classList.add('code');
            code.innerHTML = "";
            if(this.showName) {
                code.innerHTML = name + ".";
            }
            code.innerHTML += f.name + ":";

            // process the parameters
            for(var j=0; j < f.params.length; j++) {
                var p = f.params[j];
                if(j != 0) { code.innerHTML += ","; }
                code.innerHTML += p.name;

                var pli = document.createElement("li");
                pli.innerHTML = "<span class=\"code\">" + p.name + "</span> - " + p.doc;
                parameterList.appendChild(pli);

            }

            li.appendChild(code);
            li.appendChild(document.createElement("br"));
            li.appendChild(funDoc);
            li.appendChild(parameterList);
            ul.appendChild(li);
        }

        // process the vars
        for(var i=0; i<this.vars.length; i++) {
            var v = this.vars[i];
            var li = document.createElement("li");
            var code = document.createElement("span");
            var varDoc = document.createElement("span");
            code.classList.add('code');

            code.innerHTML = "";
            if(this.showName) {
                code.innerHTML = name + ".";
            }
            code.innerHTML += v.name;
            varDoc.innerHTML = " - " + v.doc;
            li.appendChild(code);
            li.appendChild(varDoc);
            ul.appendChild(li);
        }
    };
}
// !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

/**
 * The Part object is the base of all robot parts. 
 * @param {*} parent  - Parent container of the part.
 * @param {*} x - X coordinate of the part.
 * @param {*} y  - Y coordinate of the part.
 * @param {*} heading  - Angle (0-2*Pi) of the part. 
 */
var partCount = 0;
function Part(parent, x, y, heading, name) 
{
    partCount++;
    // populate the fields
    this.parent = parent;
    Positionable.call(this, x, y, heading);
    this.type = "part";

    this.name = name != undefined ? name : ("part" + partCount);
    this.doc = new PartDoc();

    // !!!!!!!! Sam Elfrink Addition !!!!!!!
    // add the new part name to the drop-down list
   
    if (loadRobotTrue == 1) {
        //do nothing
        //console.log("loadRobotTrue = 1");
        //console.log("Don't add list");
    }
    else {
        //console.log("loadRobotTrue = 0");
        //console.log("Addlist Part function called");
        //console.log(this.name);
        addList(this.name);
    }
    // !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

    //position in world coordinates
    this.worldx = 0;
    this.worldy = 0;

    // the outline and fill color of the part
    this.outline = "black";
    this.fill = "white";


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
        
        //!!!!!!!!!!!!!!!!!! Sam Elfrink Addition !!!!!!!!!!!!!!!
        // NOTE: the userbot side of set power will trigger first
        //console.log("setPower gradbot side");
        // check to make sure that the setPower() function isn't empty
        if(power == undefined) {
            alert("Error: You left your power level blank. You must enter a power value between 1-100 in your code ( ex: left.setPower(70) )");
            
            // set power to 0
            power = 0;
        }
        /*
        console.log(typeof(power));
        console.log(typeof(this.power));
        // if the power value isn't a number, set power to 0 and do nothing
        if(typeof(power) != 'number') {
            alert("Error: You put a non-number character for your power level. You must enter a power value between 1-100 in your code ( ex: left.setPower(70) )");
            power = 0;
        }
        // !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
        */
        
        //limit the power setting's range
        if(power > 100) {
            power = 100;
        } else if(power < -100) {
            power = -100;
        }
        this.power = power;
    };


    /**
     * Update the part's state.
     */
    this.update = function() { };


    /**
     * Serialize
     */
    this.toJSON = function () {
        result = {};
        for (var attr in this) {
            //skip parents and functions
            if (attr == "parent" || typeof this[attr] == "function" || attr == "doc") {
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


    /**
     * Receive a message from the user thread
     * @param {*} message 
     */
    this.receiveUser = function(message) {
        //by default, just ignore the message!
    }
}




function Motor(parent, x, y, heading, name)
{
    //construct the part
    Part.call(this, parent, x, y, heading, name);
    this.type = "Motor";

    //document the motor
    this.doc.functions = Array(
        { name: 'setPower',
          doc: 'This sets the power of the motor.',
          params: Array( { name: 'p', doc: 'This is the power setting. Its value can range from -100 to 100.'})
        }
    );
    this.doc.vars = Array(
        {name: 'power', doc: 'This represents the current power setting of the motor.'}
    );


     //GAVIN added multiplier
    // handle speed of the motor
    this.speed = 0;  // motor speed in radians per second

    //GAVIN'S UPDATED CODE STARTS HERE
    this.update = function() {
        var multi = getSpeedMult();
        //we are basing this on the sparkfun hobby motors which spin at 65 RPM (max)
        //This maximum speed is roughly 6.81 radians per second
        this.speed = (6.81 * this.power / 100) * multi;
    }
    //GAVIN'S UPDATED CODE ENDS HERE


    /**
     * Receive a message from the user thread
     * @param {*} message 
     */
    this.receiveUser = function(message) {
        //copy the power setting from the user model
        this.setPower(message.power);
    }
}


function Marker(parent, x, y, name) {
    //construct the part
    Part.call(this, parent, x, y, 0, name);
    this.type = "Marker";

    // document the part
    this.doc.functions = Array(
        { name: 'penDown', doc: 'This begins drawing.', params: Array()},
        { name: 'penUp', doc: 'This stops drawing.', params: Array()},
        { name: 'setColor', doc: 'This changes the drawing color.', params: Array(
            {name: 'c', doc: 'This is color (ex: blue)'}
        )}
    );
    this.doc.vars = Array(
        { name: 'color', doc: 'The current color of the marker.' },
        { name: 'penDrawing', doc: 'True if the pen is drawing.'}
    );

    //by default we are coloring black
    this.color = "black";

    //by default the pen is up
    this.penDrawing = false;

    //set the marker color
    this.setColor = function(color) {
        this.color = color;
    }

    //lower the pen
    this.penDown = function() {
        this.penDrawing = true;
    }

    //raise the pen
    this.penUp = function() {
        this.penDrawing = false;
    }


    /**
     * Receive a message from the user thread
     * @param {*} message 
     */
    this.receiveUser = function(message) {
        this.color = message.color;
        this.penDrawing = message.penDrawing;
    }
}


function Chassis(x, y, heading, name) 
{
    Part.call(this, null, x, y, heading, name);
    this.type = "Chassis";
    
    //no thread at first
    this.thread = null;

    //handle the subparts of the chassis
    this.parts = Array();
    this.addPart = function(p) {
        this.parts.push(p);
    };

    // create the left and right motors
    this.left = new Motor(this, -7, -7, 0, "left");
    this.right = new Motor(this, -7, 7, Math.PI, "right");

    // !!!!!! Sam Elfrink Addition !!!!!!!!!!!!!!!!!!!
    // adding a wheel size variable to the chassis
    console.log("Chassis called");
    this.chassisWheelSize = document.getElementById("wheelSize").value;
    
    // update function to preserve wheel size and update it to the robot
    this.updateWheel = function() {
        this.chassisWheelSize = document.getElementById("wheelSize").value;
    }
    //!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

    //we start unexploded and healthy!
    this.hp = 3;
    this.blowedUp = false;
    this.explosionVelocities = [];

    // create the robot code
    this.code = "";

    // set up the laser battery
    this.laserBattery = 50;

    // deal with explosions
    this.explode = function() {
        //terminate the robot thread
        if(this.thread){
            this.thread.terminate();
        }
        this.blowedUp = true;
        this.explosionVelocities = [];

        //put all the parts in the world with new velocities
        this.parts = this.parts.concat([this.left, this.right, this]);
        var speed = 10;
        for(var i=0; i<this.parts.length; i++) {
            //worldify it!
            this.parts[i].x = this.parts[i].worldx;
            this.parts[i].y = this.parts[i].worldy;
            this.parts[i].parent = null;

            //compute the new velocities
            var heading = Math.random() * Math.PI * 2;
            var velocity = { dx: speed * Math.cos(heading),
                             dy: speed * Math.sin(heading) };
            this.explosionVelocities.push(velocity);
        }

    }


    //explosion frame
    this.explosionUpdate = function() {
        for(var i=0; i < this.parts.length; i++) {
            var p = this.parts[i];

            //tumble
            p.rotate(0.3);

            //travel
            p.x += this.explosionVelocities[i].dx;
            p.y += this.explosionVelocities[i].dy;
        }
    }


    this.update = function() 
    {
        //handle exploding
        if(this.blowedUp) {
            this.explosionUpdate();
            return;
        }

        //has the end come?
        if(this.hp <= 0) {
          this.explode();
          return;
        }

        // !!!!! Sam Elfrink Addition !!!!!!!!!
        this.updateWheel();
        // !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

        //update all the sub parts
        for(var i in this.parts) {
            var p = this.parts[i];
            p.update();
        }

        //update the motors
        this.left.update();
        this.right.update();

        //compute our forward translation and yaw speeds
        // !!!!!!!!!!!!!!!! Sam Elfrink Addition !!!!!!!!!!!!!!!!!!
        var r = wheelSize; // adjustable wheel size, 65mm diameter wheels by default
        // !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
        //var r = .065; // 65mm diameter wheels
        var l = 0.238; // 238mm axel length
        var fwd = r/2 * (this.left.speed + this.right.speed) * 60;
        var yaw = r/l * (this.left.speed - this.right.speed);

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

        // Update the position of the object based on the current simulation mode
        switch(simulationMode) {
            case 'toroidal':
                var wxmax = simState.width + 40;
                var wymax = simState.height + 40;
                if(this.x <= -40) {
                    this.x = wxmax-1;
                }
                if(this.y <= -40) {
                    this.y = wymax-1;
                }
                if(this.x >= wxmax) {
                    this.x -= wxmax;
                }
                if(this.y >= wymax) {
                    this.y -= wymax;
                }
                // check if the marker is out of the simstate
                if(this.x <= -30 || this.x >= simState.width + 30 || this.y <= -30 || this.y >= simState.height + 30) {
                    // if so, stop drawing
                    penDrawing = false;
                } else {
                    // if not, resume drawing
                    penDrawing = true;

                }
                break;
            case 'infinite':
                var wxmax = simState.width;
                var wymax = simState.height;
                var removeFlag = false;

                if(this.x < -40 || this.x > wxmax + 40 || this.y < -40 || this.y > wymax + 40) {
                    removeFlag = true;
                }
                break;
        }
    };


    /**
     * Return a sendable (postable to a worker) version of this object.
     */
    this.sendable = function() {
        var result = {};
        result.name = this.name;
        result.type = this.type;
        result.code = this.code;
        result.parts = [];

        //push the motors onto the parts list
        result.parts.push(this.left.sendable());
        result.parts.push(this.right.sendable());

        //push all the parts
        for(var i=0; i<this.parts.length; i++) {
            result.parts.push(this.parts[i].sendable());
        }

        return result;
    };

    /**
     * Return the part with the given name
     * @param {*} name 
     */
    this.getPartByName = function(name) {
        //try the motors
        if(this.left.name == name) {
            return this.left;
        }
        if(this.right.name == name) {
            return this.right;
        }

        //try the parts
        for(var i=0; i<this.parts.length; i++) {
            if(this.parts[i].name == name) {
                return this.parts[i];
            }
        }

        //not found!
        return null;
    }


    // reset the laser battery
    this.resetLaserBattery = function() {
        this.laserBattery = 50;
    }

    // deplete the laser battery by a certain amount
    this.depleteLaserBattery = function(amount) {
        
        //!!!!!!! Sam Elfrink Addition!!!!!!!!!!!!!!!!
        // clear the current HUD
        drawPlayerHUDClear(); 
        if(opponentClicked == 1) {
            drawOpponentHUDClear();
        }
        //!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

        this.laserBattery -= amount;

        //!!!!!!! Sam Elfrink Addition!!!!!!!!!!!!!!!!
        // display the new HUD with the current battery count
        drawPlayerHUD(); 
        if(opponentClicked == 1) {
            drawOpponentHUD(); 
        }
        //!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
    }
}


/**
 * A light. It glows!
 * @param {*} parent 
 * @param {*} x 
 * @param {*} y 
 */
function Light(parent, x, y) {
    Part.call(this, parent, x, y);
    this.type = "Light";
    this.radius = 3;
    this.fill = "yellow"       //GAVIN ADDED FOR PACMAN EASE
    this.moveable = true;      //Added by Gavin 03/21/2023
    this.doc.functions = Array(
        { name: 'setColor', doc: 'This changes the light color.', params: Array(
            {name: 'c', doc: 'The color value (ex: blue)'}
        )}
    );


    /**
     * Receive a message from the user thread
     * @param {*} message 
     */
    this.receiveUser = function(message) {
        //copy the power setting from the user model
        this.fill = message.fill;
    }
}


/**
 * A wall, it just is!
 * @param {*} parent 
 * @param {*} x 
 * @param {*} y 
 */
function Wall(parent, x, y) {
    Part.call(this, parent, x, y);
    this.type = "Wall";
    this.outline="blue";    //GAVIN EDITED LIGHTBLUE TO BLUE
    this.fill = "blue";
    this.rotated = false;       //Added by Gavin 03/09/2023
    this.moveable = true;       //Added by Gavinn 03/21/2023
}

/**
 * what's in the fing box!
 * @param {*} parent 
 * @param {*} x 
 * @param {*} y 
 * @param {*} size 
 */
function Box(parent, x, y, size) {
    Part.call(this, parent, x, y);
    this.type = "Box";
    this.outline = "blue";
    this.fill = "lightblue";
    this.size = size;
    this.moveable = true;   //Added by Gavin 03/21/2023
}

/**
 * A sensor which computes the range to an object.
 * @param {*} parent 
 * @param {*} x 
 * @param {*} y 
 */
function RangeSensor(parent, x, y) {
    Part.call(this, parent, x, y);
    this.type = "RangeSensor";
    this.distance = Infinity;
    this.freq = 10;  //frequency in hertz
    this.worldx = 0;
    this.worldy = 0;

    this.doc.vars = Array(
        {name: 'distance', doc: 'The distance to the nearest object.'}
    );


    this.updateSensor = function() {
        var closest = Infinity; 
        var bots = [ simView ];
        if(opponent) {
            bots.push(opponentView);
        }
        var objects = bots.concat(simState.worldObjects);

        //find the closest visible light source
        for(var i in objects) {
            var part = objects[i].part;

            //ADDED BY GAVIN 03/24/2023
            //Do not lights in pacman, this will break the game
            if(simState.pacmanWorldLoaded == true && part.type == "Light"){
                continue;
            }
            //END OF ADDED BY GAVIN 03/24/2023
            //skip our parent part
            if(this.parent === part) {
                continue;
            }
    
            //calculate displacement to the object
            var dist = minPolyDist({x: this.worldx, y: this.worldy}, objects[i].view.polygon);
            var angle = reduceAngle(dist.angle);
            angle = reduceAngle(this.parent.heading - angle);

            //skip the lights outside of our field of view
            if(angle > 0.52 && angle < 5.76) { 
                continue;
            }

            //check for being the closest
            if(dist.distance < closest) {
                closest = dist.distance;
            }
        }

        //calculate the distance
        this.distance = closest / 60;
        if(isNaN(this.distance) || this.distance > 5) {
            this.distance = Infinity;
        }


        //pass the update into the web worker
        if(this.parent.thread) {
            this.parent.thread.postMessage({type: "update", update: {name: this.name, distance: this.distance}});
        }
    };


    this.update = function() {
        //populate the last update (if needed)
        if(this.lastUpdate == undefined) {
            this.lastUpdate = Date.now();
        }

        //compute elapsed time
        var cur = Date.now();
        var elapsed = (cur - this.lastUpdate) / 1000;

        // trigger the sensor
        if(elapsed >= 1 / this.freq) {
            this.updateSensor();
            this.lastUpdate = cur;
        }
    };
}



/**
 * A sensor which determines the intensity of a light source.
 * It can also be filtered, which makes it only see one color of light.
 * @param {*} parent 
 * @param {*} x 
 * @param {*} y 
 */
function LightSensor(parent, x, y) {

    Part.call(this, parent, x, y);
    this.type = "LightSensor"
    this.worldx = 0;
    this.worldy = 0;
    this.intensity = 0;
    this.freq = 10;  //frequency in hertz

    this.doc.vars = Array(
        {name: 'intensity', doc: 'The intensity of the sensed light.'}
    );

    this.getRobotLights = function(r) {
        var lights = [];

        for(var i=0; i<r.parts.length; i++) {
            if(r.parts[i].type == "Light") {
                lights.push(r.parts[i]);
            }
        }

        return lights;
    }


    this.getWorldLights = function() {
        var lights = [];
        for(var i=0; i<simState.worldObjects.length; i++) {
            var part = simState.worldObjects[i].part;

            if(part.type=="Light") {
                lights.push(part);
            }
        }
        return lights;
    }



    this.updateSensor = function() {
        var closest = Infinity; 
        var lights = this.getWorldLights();
        lights = lights.concat(this.getRobotLights(robot));
        if(opponent) {
            lights = lights.concat(this.getRobotLights(opponent));
        }

        //find the closest visible light source
        for(var i in lights) {
            var part = lights[i];
    
            //we only sense lights
            if(part.type != "Light") { 
                continue; 
            }

            //filter the lighty by our fill color
            if(this.fill != "white" && part.fill != this.fill) {
                continue;
            }

            //calculate displacement to the light
            var dx = part.worldx - this.worldx;
            var dy = part.worldy - this.worldy;

            //calculate the angle to the light
            var angle = reduceAngle(Math.atan2(dy, dx));
            angle = reduceAngle(this.parent.heading - angle);

            //skip the lights outside of our field of view
            if(angle > 0.52 && angle < 5.76) {
                continue;
            }

            //calculate the square distance
            var dist = dx*dx + dy*dy;
            if(dist < closest) {
                closest = dist;
            }
        }

        //calculate the intensity
        closest = closest / 400; // 20px per meter => 400px per meter^2

        // 100% intensity at 1 m so...
        this.intensity = 100 / closest;
        if(isNaN(this.intensity)) {
            this.intensity = 0;
        }
        if(this.intensity > 100) {
            this.intensity=100;
        }

        //pass the update into the web worker
        if(this.parent.thread) {
            this.parent.thread.postMessage({type: "update", update: {name: this.name, intensity: this.intensity}});
        }
    };


    this.update = function() {
        //populate the last update (if needed)
        if(this.lastUpdate == undefined) {
            this.lastUpdate = Date.now();
        }

        //compute elapsed time
        var cur = Date.now();
        var elapsed = (cur - this.lastUpdate) / 1000;

        // trigger the sensor
        if(elapsed >= 1 / this.freq) {
            this.updateSensor();
            this.lastUpdate = cur;
        }
    };

}



/**
 * A blast, from a laser. What else would it be?
 */
function LaserBlast(x, y, heading, firedBy) {
    Part.call(this, null, x, y, heading);
    this.type = "LaserBlast";

    this.outline="red";
    this.fill="red";

    var speed = 20;     //Changed from 30 GAVIN 03/30/2023

    this.dx = speed * Math.cos(heading);
    this.dy = speed * Math.sin(heading);
    this.firedBy = firedBy;

    this.vanish = function() {
        //take ourselves out of the world objects
        var toRemove = -1;
        for(var i = 0; i< simState.worldObjects.length; i++) {
            if(simState.worldObjects[i].part === this) {
                toRemove = i;
                break;
            }
        }
        if(i >= 0) {
            simState.worldObjects.splice(toRemove, 1);
        }
    }


    this.update = function() {
        this.x += this.dx;
        this.y += this.dy;

        //TODO, dont' hardcode the sizes
        if(this.x < 0 || this.x > 800 || this.y < 0 || this.y > 600) {
            this.vanish();
        }
    }
}


/**
 * A frickin' laser beam.
 */
function Laser(parent, x, y, heading) {
    Part.call(this, parent, x, y, heading);
    this.type = "Laser";
    this.charged = true;
    this.lastUpdate = undefined;
    this.chargeTime = 500;

    this.doc.functions = Array(
        {name: 'fire', doc: 'This fires the laser beam once.', params: Array()}
    );

    //fire the laser beam
    this.fire = function() {
        // no charge, no pew
        if(!this.charged) { return; }

        // check that we have ample battery power
        if(this.parent.laserBattery <= 0) {
            return;
        }

        //no more power.
        this.charged = false;
        this.lastUpdate = Date.now();

        //fire!
        this.parent.depleteLaserBattery(1);
        var lb = new LaserBlast(this.worldx, this.worldy, this.parent.heading, this.parent);
        simState.worldObjects.push(constructView(lb));
    }

    //update the laser (charge it)
    this.update = function() {
        //already charged?
        if(this.charged) { 
            return;
        }

        var elapsed = Date.now() - this.lastUpdate;
        if(elapsed > this.chargeTime) {
            this.charged = true;
            this.lastUpdate = undefined;
        }
    }


    //fire the laser when we receive a message
    this.receiveUser = function(obj) {
        this.fire();
    }
}


/**
 * Document the system functions.
 */
function SystemFunctions() {
    this.doc = new PartDoc();
    this.doc.showName = false;
    this.name = 'System Functions';
    this.type = '';

    this.doc.functions = Array(
        { name: 'await delay',
          doc: 'Wait for a period of time to pass.',
          params: Array({name: 'ms', doc: 'The amount of delay in milliseconds'})} 
    );

}

// !!!!!!!!!!!!!!!!!!!!!!!!!!!!! Sam Elfrink Addition !!!!!!!!!!!!!!!!!!!!!!!!!!!!!
/**
 * Document sample functions.
 */
function RobotMovement() {
    this.doc = new MovementDoc();
    this.doc.showName = false;
    this.name = 'Robot Movement';
    this.type = '';

    this.doc.functions = Array(
        { name: 'Forward movement', doc: 'left.setPower(100) \n right.setPower(100) \n', 
            params: Array()},
        { name: 'Backward movement', doc: 'left.setPower(-100) \n right.setPower(-100) \n', 
            params: Array()},
        { name: 'Left Turn', doc: 'left.setPower(50) \n right.setPower(100) \n', 
            params: Array()},
        { name: 'Right Turn', doc: 'left.setPower(100) \n right.setPower(50) \n', 
            params: Array()},
    );
}
// !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!


/****************************************** 
 * Utility Functions and Objects
 ******************************************/


function VectorView(x, y, heading, scale, points) {
    // set up the fields
    Positionable.call(this, x, y, heading);
    this.scale = scale != undefined ? scale : 1;
    this.points = points != undefined ? points : {};
    this.outline = undefined;
    this.fill = undefined;
    this.polygon = [];
    

    /**
     * Reset the minx, miny, maxx, and maxy to Infinity and -Infinity
     */
    this.resetExtents = function() {
        //extents of the vector view
        this.minx = Infinity;
        this.miny = Infinity;
        this.maxx = -Infinity;
        this.maxy = -Infinity;
    };
    this.resetExtents();


    // draw the shape
    this.draw = function(canvas, context) {
        var x;
        var y;
        var started = false;
        this.polygon = [];

        //reset the extents
        this.resetExtents();

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

            // add to the polygon
            this.polygon.push({x: x, y: y});

            //add the line or start the shape
            if(started) {
                context.lineTo(x, y);
            } else {
                context.moveTo(x, y);
                started = true;
            }

            //track extents
            if(x < this.minx) { this.minx = x; }
            if(x > this.maxx) { this.maxx = x; }
            if(y < this.miny) { this.miny = y; }
            if(y > this.maxy) { this.maxy = y; }
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
    };


    /**
     * Determines if the point (x,y) is within the extents of the view.
     * @param {*} x - x coordinate 
     * @param {*} y - y coordinate
     * @returns true if (x,y) is inside this view, false otherwise.
     */
    this.encloses = function(x, y) {
        return x >= this.minx && x <= this.maxx &&
               y >= this.miny && y <= this.maxy;
    }
}


/**
 * Construct a view for the given part. It works by calling the correct
 * constructor for the part.
 * @param {*} part 
 * @returns The newly constructed part. Returns undefined for parts with no known view.
 */
function constructView(part) {
    if(part.type == "Chassis") {
        return new ChassisView(part);
    } else if(part.type == "Motor") {
        return new MotorView(part);
    } else if(part.type == "Marker") {
        return new MarkerView(part);
    } else if(part.type == "Light") {
        return new LightView(part);
    } else if(part.type == "LightSensor") {
        return new LightSensorView(part);
    } else if(part.type == "RangeSensor") {
        return new RangeSensorView(part);
    } else if(part.type == "Wall") {
        return new WallView(part);
    } else if(part.type == "Box") {
        return new BoxView(part); 
    } else if(part.type == "LaserBlast") {
        return new LaserBlastView(part);
    } else if(part.type == "Laser") {
        return new LaserView(part);
    }
    // we don't know how to show this part.
    return undefined;
}


function PartView(part) {
    // Construct the positionable parts of ourselves
    Positionable.call(this, part.x, part.y, part.heading);

    // Every PartView has a vector view
    this.view = null;

    // Scale the view
    this.scale = 1;

    // We also need a list of sub views 
    this.subviews = [];
    this.addSubview = function(view) {
        view.reOrigin();
        this.subviews.push(view);
    }

    // remember the part we are viewing!
    this.part = part;

    /**
     * Draw the part along with all of its subparts.
     * @param {*} canvas - The canvas to draw on.
     * @param {*} context - The context to draw on.
     */
    this.draw = function(canvas, context) {
        // unparented parts are in the world (but not necessarily of
        // it!)
        if(!this.part.parent) {
            this.x = this.part.x;
            this.y = this.part.y;
            this.heading = this.part.heading;
        }

        // set the color
        this.view.outline = this.part.outline;
        this.view.fill = this.part.fill;

        // draw the base view (if it exists)
        if(this.view) {
            this.view.x = this.x;
            this.view.y = this.y;
            this.view.heading = this.heading;
            this.view.scale = this.scale;
            this.view.draw(canvas, context);
            part.worldx = (this.view.minx + this.view.maxx)/2;
            part.worldy = (this.view.miny + this.view.maxy)/2;
        }

        // draw each subviews offset to this view's pose
        for(var i = 0; i < this.subviews.length; i++) {
            var v = this.subviews[i];
            v.x = this.x;
            v.y = this.y;
            v.face(this.heading);
            v.scale = this.scale;
            v.draw(canvas, context);
        }
    };

    
    /**
     * Shift the part's origin to its x,y and set its position to 0,0.
     */
    this.reOrigin = function() {
        //compute rotation coeffecients
        var sin_th = Math.sin(this.heading);
        var cos_th = Math.cos(this.heading);

        for(var i=0; i < this.view.points.length; i++) {
            var p = this.view.points[i];
            var rx, ry;
            rx = p.x * cos_th - p.y * sin_th;
            ry = p.x * sin_th + p.y * cos_th;
            p.x = rx + this.x;
            p.y = ry + this.y;
        }

        this.heading = 0;
        this.view.heading = 0;
        this.view.x = 0;
        this.view.y = 0;
        this.x = 0;
        this.y = 0;
    };


    /**
     * Move the part to x, y (in part coordinates)
     * @param {*} x 
     * @param {*} y 
     */
    this.moveTo = function(x, y) {
        var dx = x - this.part.x;
        var dy = y - this.part.y;

        // Move all the points in the view
        for(var i=0; i < this.view.points.length; i++) {
            var p = this.view.points[i];
            p.x += dx;
            p.y += dy;
        }

        // move the part position
        this.part.x = x;
        this.part.y = y;
    }


    /**
     * Convert a global coordinate to a part view coordinate. 
     * @param {*} x 
     * @param {*} y 
     * @returns Location object {x:, y:}
     */
    this.globalToPartLoc = function(x, y) {
        var sin_th = Math.sin(-this.heading);
        var cos_th = Math.cos(-this.heading);
        var result = {};

        x /= this.scale;
        y /= this.scale;
        x -= this.x / this.scale;
        y -= this.y / this.scale;

        result.x = x * cos_th - y * sin_th;
        result.y = x * sin_th + y * cos_th;

        return result;
    };
}


/**
 * Constructor for the chassis view object. This visualizes an entire 
 * robot. 
 * @param {*} part - The chassis part
 */
function ChassisView(part) {
    // initialize the partview
    PartView.call(this, part);

    // add the motors to the subview list.
    this.addSubview(constructView(part.left));
    this.addSubview(constructView(part.right));

    //add the other parts to the view
    for(var i=0; i<part.parts.length; i++) {
        this.addSubview(constructView(part.parts[i]));
    }

    //create my vector view
    var points = [
        {x: -10, y: -6},
        {x: 10, y: -6},
        {x: 10, y: 6},
        {x: -10, y: 6},
    ];
    this.view = new VectorView(part.x, part.y, part.heading, 1.0, points);
    this.view.fill = "white";
    this.view.stroke = "black"


    //remember the base version
    //this.partDraw = this.draw;

    /**
     * Update for the movement of the model and then draw.
     * @param {*} canvas 
     * @param {*} context 
     */
    /*this.draw = function(canvas, context) {
        //copy the chassis pose
        this.x = this.part.x;
        this.y = this.part.y;
        this.heading = this.part.heading;

        //draw like normal
        this.partDraw(canvas, context); 
    }*/


    /**
     * Add a part to the view
     * @param {*} part 
     */
    this.addPart = function(part) {
        this.addSubview(constructView(part));
    }
}


function ChassisBuildView(part) {
    ChassisView.call(this, part);

    // This view has a fixed position and heading
    this.x = 400;
    this.y = 300;
    this.scale=30; //it's also big!
    this.heading = -Math.PI/2;

    this.partDraw = this.draw;
    this.draw = function(canvas, context) {
        // remember the real location
        var x = this.part.x;
        var y = this.part.y;
        var heading = this.part.heading;

        // shift the robot
        this.part.x = this.x;
        this.part.y = this.y;
        this.part.heading = this.heading;

        //draw the robot
        this.partDraw(canvas, context);

        // restore the robot position
        this.part.x = x;
        this.part.y = y;
        this.part.heading = heading;
    }
}


/**
 * constructor for the motor view object. This visualizes a motor.
 * @param {*} part - The motor part 
 */
function MotorView(part) {
    //initialize the part view
    PartView.call(this, part);


    //create my vector view
    var points = [
        {x: -3, y: -1},
        {x: 3, y: -1},
        {x: 3, y: 1},
        {x: -3, y: 1},
    ];
    this.view = new VectorView(part.x, part.y, part.heading, 1.0, points);
    this.view.fill = "white";
    this.view.stroke = "black"
}


/**
 * constructor for the marker view object. This visualizes a marker.
 * @param {*} part - The motor part 
 */
function MarkerView(part) {
    //initialize the part view
    PartView.call(this, part);

    //create my vector view
    var points = [
        {x: -1, y: -1.5},
        {x: 1, y: -1.5},
        {x: -1, y: 1.5},
        {x: 1, y: 1.5},
    ];
    this.view = new VectorView(part.x, part.y, part.heading, 1.0, points);
    this.view.fill = "white";
    this.view.stroke = "black"

    //store the original draw
    this.drawPart = this.draw;

    this.draw = function(canvas, context) {
        if(part.penDrawing) {
            part.fill = part.color;
        } else {
            part.fill = "#00000000";
        }
        part.outline = part.color;
        this.drawPart(canvas, context);

        //draw a line if the pen is down and we have two endpoints
        this.updateLoc();
        if(this.part.penDrawing && this.loc && this.prevLoc && simState.running) {
            var canvas = document.getElementById("simbg");
            var context = canvas.getContext("2d");
            context.beginPath();
            context.moveTo(this.prevLoc.x, this.prevLoc.y);
            context.lineTo(this.loc.x, this.loc.y);
            context.strokeStyle = this.part.color;
            context.stroke();
        }
    }

    this.loc = null;
    this.prevLoc = null;
    this.updateLoc = function() {
        this.prevLoc = this.loc;
        this.loc = {};
        this.loc.x = (this.view.maxx + this.view.minx) / 2;
        this.loc.y = (this.view.maxy + this.view.miny) / 2;
    }
}


/**
 * Construct a light view.
 * @param {*} part 
 */
function LightView(part) {
    //initialize the part view
    PartView.call(this, part);


    //create the actual drawing part
    var points = [ {x: 0, y: 0} ];
    this.view = new VectorView(part.x, part.y, part.heading, 1.0, points);
    this.view.fill = "white";
    this.view.outline = "black";
    this.view.radius=part.radius;
    this.view.draw = function(canvas, context) {
        var sin_th = Math.sin(this.heading);
        var cos_th = Math.cos(this.heading);
        var x = this.points[0].x * this.scale;
        var y = this.points[0].y * this.scale;
        var rx, ry;
        rx = x * cos_th - y * sin_th;
        ry = x * sin_th + y * cos_th;
        x = rx + this.x;
        y = ry + this.y;
        this.polygon = [{x: x, y: y}];

        //set the extents
        this.minx = x - this.radius * this.scale;
        this.maxx = x + this.radius * this.scale;
        this.miny = y - this.radius * this.scale;
        this.maxy = y + this.radius * this.scale;

        //draw the arch
        context.beginPath();
        context.arc(x, y, this.radius * this.scale, 0, 2 * Math.PI);
        context.fillStyle = this.fill;
        context.strokeStyle = this.outline;
        context.fill();
        context.stroke();
    }

    //handle world-centric drawing
    if(!part.parent) {
        this.partDraw = this.draw;
        this.draw = function(canvas, context) {
            this.x = part.x;
            this.y = part.y;
            this.partDraw(canvas, context);
        };
    }

}


function LightSensorView(part) {
    PartView.call(this, part);

    //points for the light sensor
    var points = [ {x:0.5, y:-1},
                   {x:-0.5, y:-1},
                   {x:-0.5, y:1},
                   {x:0.5, y:1} ];
    this.view = new VectorView(this.x, this.y, this.heading, this.scale, points);

    // a little custom drawing action
    this.partDraw = this.draw;
    this.draw = function(canvas, context) {
        var outline = part.outline;
        part.outline = part.fill;
        part.worldx = (this.view.minx + this.view.maxx)/2;
        part.worldy = (this.view.miny + this.view.maxy)/2;

        //draw the filter
        this.partDraw(canvas, context);

        part.outline = outline;

        //compute rotation coeffecients
        var sin_th = Math.sin(this.heading);
        var cos_th = Math.cos(this.heading);

        //draw the actual part outline.
        context.beginPath();
        var started = false;
        for(var i=0; i<this.view.points.length; i++ ) {
            var p = this.view.points[i];
            var x = p.x * this.scale;
            var y = p.y * this.scale;

            // rotate
            var rx, ry;
            rx = x * cos_th - y * sin_th;
            ry = x * sin_th + y * cos_th;
            x = rx;
            y = ry;

            // translate
            x += this.x;
            y += this.y;

            //plot the point
            if(!started) {
                context.moveTo(x, y);
                started = true;
            } else {
                context.lineTo(x, y);
            }
        }

        context.strokeStyle = this.part.outline;
        context.stroke();
    }
}



/**
 * constructor for the range sensor view object. This visualizes a marker.
 * @param {*} part - The motor part 
 */
function RangeSensorView(part) {
    //initialize the part view
    PartView.call(this, part);

    //create my vector view
    var points = [
        {x: -0.5, y: -2.5},
        {x:  0, y: -2.5},
        {x:  0, y: -1.5},
        {x:  1, y: -1.5},
        {x:  1, y: -0.5},
        {x:  0, y: -0.5},
        {x:  0, y:  0.5},
        {x:  1, y:  0.5},
        {x:  1, y:  1.5},
        {x:  0, y:  1.5},
        {x:  0, y:  2.5},
        {x: -0.5, y:  2.5}
    ];
    this.view = new VectorView(part.x, part.y, part.heading, 1.0, points);
    this.view.fill = "white";
    this.view.stroke = "black"


    // a little custom drawing action
    this.partDraw = this.draw;
    this.draw = function(canvas, context) {
        //report world coordinates
        part.worldx = (this.view.minx + this.view.maxx)/2;
        part.worldy = (this.view.miny + this.view.maxy)/2;

        //draw the filter
        this.partDraw(canvas, context);
    }
}


/**
 * Display the wall in all of its rectangular glory!
 * @param {*} part 
 */

//UPDATED AND EDITED BY GAVIN 03/08/2023
function WallView(part) {
    if (!part.resizeFactor){
        part.resizeFactor = 1;
    }
    if (!part.resizeFactorHeight){
        part.resizeFactorHeight = 1;
    }
    if (!part.resizeFactorWidth){
        part.resizeFactorWidth = 1;
    }
    PartView.call(this, part);

    //create my vector view
    if((simState.dragMode == DRAG_ROTATE90 && part.rotated == true) || (simState.dragMode != DRAG_ROTATE90 && part.rotated == true)){
        var points = [
            {x: -10 * part.resizeFactor, y: -5 * part.resizeFactor},
            {x: 10 * part.resizeFactorHeight, y: -5 * part.resizeFactor},
            {x: 10 * part.resizeFactorHeight, y: 5 * part.resizeFactorWidth},
            {x: -10 * part.resizeFactor, y: 5 * part.resizeFactorWidth},
        ];
        //console.log("Rotated Height Resize: ", part.resizeFactorHeight);
    }
    if((simState.dragMode == DRAG_ROTATE90 && part.rotated == false) || (simState.dragMode != DRAG_ROTATE90 && part.rotated == false)){
        var points = [
            {x: -5 * part.resizeFactor, y: -10 * part.resizeFactorHeight},
            {x: 5 * part.resizeFactorWidth, y: -10 * part.resizeFactorHeight},
            {x: 5 * part.resizeFactorWidth, y: 10 * part.resizeFactor},
            {x: -5 * part.resizeFactor, y: 10 * part.resizeFactor},
        ];
        //console.log("Upright Height Resize: ", part.resizeFactorHeight);
    }
    this.view = new VectorView(part.x, part.y, part.heading, 1.0, points);
    this.view.fill = "white";
    this.view.stroke = "black"
}
//END OF UPDATED AND EDITED BY GAVIN 03/08/2023



/**
 * Display the box!
 * @param {*} part 
 */
function BoxView(part) {
    if (!part.resizeFactor){
        part.resizeFactor = 1;
    }
    PartView.call(this, part);

     //create my vector view
    var points = [
        {x: -8 * part.resizeFactor, y: -8 * part.resizeFactor},
        {x: 8 * part.resizeFactor, y: -8 * part.resizeFactor},
        {x: 8 * part.resizeFactor, y: 8 * part.resizeFactor},
        {x: -8 * part.resizeFactor, y: 8 * part.resizeFactor},
    ];
    this.view = new VectorView(part.x, part.y, part.heading, 1.0, points);
    this.view.fill = "white";
    this.view.stroke = "black"
}


/**
 * Display the laser blast
 * @param {*} part 
 */
function LaserBlastView(part) {
    PartView.call(this, part);

    //create my vector view
    var points = [
        {x: -10, y:-1},
        {x: 0, y: -1}, //GAVIN CHANGED x:10 to 0 03/30/2023
        {x: 0, y: 1},  //GAVIN CHANGED x:10 to 0 03/30/2023
        {x: -10, y:1},
    ];
    this.view = new VectorView(part.x, part.y, part.heading, 1.0, points);
}


/**
 * Display the laser.
 * @param {*} part
 */
function LaserView(part) {
    PartView.call(this, part);

    //create my vector view
    var points = [
        {x: -3, y: 1.5 },
        {x: 1.0, y: 1.5 },
        {x: 1.0, y: 0.5 },
        {x: 3, y: 0.5 },
        {x: 3, y: -0.5 },
        {x: 1.0, y:-0.5},
        {x: 1.0, y: -1.5},
        {x: -3, y: -1.5}
    ];
    this.view = new VectorView(part.x, part.y, part.heading, 1.0, points);
}


/****************************************** 
 * USER INTERFACE
 ******************************************/
var world;
var robot;
var opponent;
var opponentView;
var simView;
var buildView;
var flask;

// dragmodes
const DRAG_NONE= 0;
const DRAG_MOVE=1;
const DRAG_ROTATE=2;
const DRAG_RESIZEHEIGHT=3;      //Updated By Gavin 03/08/2023
const DRAG_RESIZEWIDTH=4;       //Added By Gavin 03/08/2023
const DRAG_ROTATE90=5;          //Added By Gavin 03/08/2023

//state of the simulator ui
var simState = {
    prefix: "sim",
    dragMode: DRAG_NONE,
    dragTarget: null,
    lastX: 0,
    lastY: 0,
    robotStartX: 100,
    robotStartY: 100,
    //ADDED BY GAVIN 03/24/2023
    //robot start location for maze map
    robotMazeStartX: 60,        
    robotMazeStartY: 60,
    //robot start location for combat map
    robotCombatStartX: 60,
    robotCombatStartY: 300,
    //robot start location for pacman map
    robotPacmanStartX: 400,
    robotPacmanStartY: 300,
    //END OF ADDED BY GAVIN 03/24/2023
    robotStartHeading: 0,
    //faces the robot upward for pacman
    pacmanStartHeading: 4.7123889804,    //ADDED BY GAVIN
    //opponent start location
    opponentStartX: 700,
    opponentStartY: 500,
    //location for opponent start on combat
    opponentCombatStartX: 740,      //ADDED BY GAVIN 04/03/2023
    opponentCombatStartY: 300,      //ADDED BY GAVIN 04/03/2023
    opponentStartHeading: Math.PI,
    timer: null,
    prevTab: null,
    robotThread: null,
    opponentThread: null,
    worldObjects: [],
    editTarget: null,
    editOriginalOutline: null,
    running : false,
    width: 0,
    height: 0,
    rotated: false,         //Added By Gavin 03/08/2023
    //ADDED BY GAVIN 03/20/2023
    //sets the maps currently loaded
    mazeWorldLoaded: false,
    combatWorldLoaded: false,
    pacmanWorldLoaded: false,
    pacmanPoints: 0,
    //END OF ADDED BY GAVIN 03/20/2023
};


//state of the build ui
var buildState = {
    prefix: "build",
    dragMode: DRAG_NONE,
    dragTarget: null,
    lastX: 0,
    lastY: 0,
    editTarget: null,
    editOriginalOutline: null
};


/**
 * Open a UI tab.
 * This is based on a code tutorial found at: https://www.w3schools.com/howto/howto_js_tabs.asp 
 * @param {*} evt 
 * @param {*} tabId
 */
function openTab(evt, tabId) {
    var i, tabcontent, tablinks;
    tabcontent = document.getElementsByClassName("tabcontent");
    for (i = 0; i < tabcontent.length; i++) {
        tabcontent[i].style.display = "none";
    }
    tablinks = document.getElementsByClassName("tablinks");
    for (i = 0; i < tablinks.length; i++) {
        tablinks[i].className = tablinks[i].className.replace(" active", "");
    }
    document.getElementById(tabId).style.display = "block";
    evt.currentTarget.className += " active";


    //handle switching to specific tabs
    if(tabId == "Simulate") {
        deselectPart(buildState);
        drawSim();
    } else if(tabId == "Build") {
        drawBuild();
    } else if(tabId == "Code") {
        var partList = document.getElementById("codePartList");
        partList.innerHTML="";
        addPartToPartList(partList, new SystemFunctions());

        // !!!!!!!! Sam elfrink addition !!!!!!!!
        addPartToPartList(partList, new RobotMovement());
        // !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

        addPartToPartList(partList, robot.left);
        addPartToPartList(partList, robot.right);
        for(var i=0; i < robot.parts.length; i++) {
            addPartToPartList(partList, robot.parts[i]);
        }
        flask.updateCode(robot.code);
    }

    //handle previous tab transitions
    if(simState.prevTab == "Simulate") {
        //stop the simulation
        simulationStop();
    } else if(simState.prevTab == "Build") {
        //reconstruct the chassis view after build
        simView = new ChassisView(robot);
        drawSim();
        graphPaperFill("simbg");
    } else if(simState.prevTab == "Code") {
        robot.code = flask.getCode();
    } 

    //save robot in local store
    saveRobot(robot);

    // remember previous tab
    simState.prevTab = tabId;
}

// !!!!!!!!! Sam Elfrink Addition !!!!!!!!!
/**
 * 
 * @param {*} event 
 */
function changeWheelSize(state) {
    event.preventDefault()
    console.log("changeWheelSize() called");
    var size = document.getElementById("wheelSize").value;
    console.log("Size Variable:");
    console.log(size);
    //this.chassisWheelSize = size;
    robot.update();
    wheelSize = size;
    console.log("wheelSize Variable:");
    console.log(wheelSize);
}
// !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

/**
 * Add part to the partList element
 * @param {*} partList 
 * @param {*} part 
 */
function addPartToPartList(partList, part) {
    var li = document.createElement('li');
    li.innerHTML = part.name + ": <i>" + part.type + "</i>";
    li.onclick = function() {displayPartDoc(part); };
    partList.appendChild(li);
}


/**
 * Display part documentation.
 * @param {*} part
 */
function displayPartDoc(part) {
    var doc = document.getElementById('codePartDoc');
    doc.innerHTML = '';
    var e = document.createElement('div');
    e.classList.add('toolboxHead');
    e.innerHTML = part.name;
    doc.appendChild(e);
    part.doc.display(doc, part.name);
}


/**
 * Draw the simulator canvas.
 */
function drawSim() {
    var canvas = document.getElementById('simfg');
    var context = canvas.getContext("2d");

    //scale the view
    simView.scale=2;

    // clear the frame
    context.clearRect(0, 0, canvas.width, canvas.height);

    //draw the world objects
    for(i in simState.worldObjects) {
        simState.worldObjects[i].scale = 2;
        simState.worldObjects[i].draw(canvas, context);
    }

    //draw the robot
    simView.draw(canvas, context);

    //draw the opponent (if there is one)
    if(opponent) {
        opponentView.scale=2;
        opponentView.draw(canvas, context);
    }
}


/**
 * Draws the build canvas.
 */
function drawBuild() {
    var canvas = document.getElementById("buildCanvas");
    var context = canvas.getContext("2d");
    
    //draw the build window
    graphPaperFill("buildCanvas");
    buildView.draw(canvas, context);
}

/**
     * Draw the part along with all of its subparts.
     * @param {*} canvas - The canvas to draw on.
     * @param {*} context - The context to draw on.
*/
function DrawFunction() {
    console.log("Drawfunction called");
    
    const canvas = document.getElementById('simfg');
    const canvasDraw = document.getElementById('simdg');
    const contextDraw = canvasDraw.getContext('2d');
    

    let isPainting = false;
    let lineWidth = 1;
    let startX;
    let startY;

    const draw = (e) => {
        console.log("drawing in draw function");
        if(!isPainting) {
            return;
        }

        const rect = canvas.getBoundingClientRect();
        const x = e.clientX - rect.left;
        const y = e.clientY - rect.top;
        contextDraw.lineWidth + lineWidth;
        contextDraw.lineCap = 'round';
        contextDraw.lineTo(x, y);
        contextDraw.stroke();
    }

    canvas.addEventListener('mousedown', (e) => {
        console.log("mouse down");
        if(document.getElementById('dragDraw').checked) {
            console.log("zzzzzzzz");
            isPainting = true;
            startX = e.clientX;
            startY = e.clientY;

        }
    });

    canvas.addEventListener('mouseup', e => {
        isPainting = false;
        contextDraw.stroke();
        contextDraw.beginPath();
    });

    canvas.addEventListener('mousemove', draw);
    
}


//!!!!!!! Sam Elfrink Addition !!!!!!!!!!!!!
/**
 * Draws the hud on the canvas.
 */
function drawPlayerHUD() {
    
    var simCanvas = document.getElementById("simbg"); 
    var context = simCanvas.getContext("2d");
    //drawSim();
    //context.clearRect(0,0,40000,40000);

    // SetUp player and opponent titles
    
    context.font = "20px Trebuchet MS"; 
    textAlign = "center";
    context.fillStyle = "black"; 
    context.color = "black";
    context.fillText("Player 1",535,38);
   
    
    // set up lives and power level
    context.font = "15px Trebuchet MS"; 
    textAlign = "center";
    //context.fillStyle = "black"; 
    //context.color = "black";

    // player 1 info
    context.fillText("Lives:",535,58); // x,y
    //context.fillText(playerHealthPoints,595,40);
    context.fillText(robot.hp,580,58);
    context.fillText("Laser Power:", 535, 78);
    context.fillText(robot.laserBattery,630,78);
}
// !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

//!!!!!!! Sam Elfrink Addition !!!!!!!!!!!!!
/**
 * Draws the hud on the canvas.
 */
function drawOpponentHUD() {
    
    var simCanvas = document.getElementById("simbg"); 
    var context = simCanvas.getContext("2d");
    
    // SetUp player and opponent titles
    context.font = "20px Trebuchet MS"; 
    textAlign = "center";
    context.fillStyle = "black"; 
    context.color = "black";
    context.fillText("Player 2",665,38);
    
    // set up lives and power level
    context.font = "15px Trebuchet MS"; 
    textAlign = "center";

    context.fillStyle = "black"; 
    context.color = "black";
    
    //player 2 info
    context.fillText("Lives:",665,58); // x,y
    context.fillText(opponent.hp,710,58);
    context.fillText("Laser Power:", 665,78);
    context.fillText(opponent.laserBattery,760,78);
}
// !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

//!!!!!!! Sam Elfrink Addition !!!!!!!!!!!!!
/**
 *  Draws a white overlay over the player HUD text to clear it
 */
function drawPlayerHUDClear() {
    var simCanvas = document.getElementById("simbg"); 
    var context = simCanvas.getContext("2d");

    // SetUp player and opponent titles
    
    context.font = "20px Trebuchet MS"; 
    textAlign = "center";
    context.fillStyle = "white"; 
    context.color = "white";
    context.fillText("Player 1",535,38);
   
    
    // set up lives and power level
    context.font = "15px Trebuchet MS"; 
    textAlign = "center";

    // player 1 info
    context.fillText("Lives:",535,58); // x,y
    //context.fillText(playerHealthPoints,595,40);
    context.fillText(robot.hp,580,58);
    context.fillText("Laser Power:", 535,78);
    context.fillText(robot.laserBattery,630,78);
    
}
// !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

//!!!!!!! Sam Elfrink Addition !!!!!!!!!!!!!
/**
 * Draws a white overlay over the opponent HUD text to clear it
 */
function drawOpponentHUDClear() {
        var simCanvas = document.getElementById("simbg"); 
        var context = simCanvas.getContext("2d");
        
        // SetUp player and opponent titles
        context.font = "20px Trebuchet MS"; 
        textAlign = "center";
        context.fillStyle = "white"; 
        context.color = "white";
        context.fillText("Player 2",665,38);
        
        // set up lives and power level
        context.font = "15px Trebuchet MS"; 
        textAlign = "center";
        
        //player 2 info
        context.fillText("Lives:",665,58); // x,y
        context.fillText(opponent.hp,710,58);
        context.fillText("Laser Power:", 665,78);
        context.fillText(opponent.laserBattery,760,78);
}
//!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

/**
 * Fill the canvas with the given ID with a graph paper like pattern.
 * @param {*} - id
 */
function graphPaperFill(id) {
    var back = document.getElementById(id);
    var context = back.getContext("2d");
    context.fillStyle = "white";
    context.fillRect(0, 0, back.width, back.height);

    context.beginPath();
    for (var lx = 0; lx < back.width; lx += 20) {
        context.moveTo(lx, 0);
        context.lineTo(lx, back.height);
    }
    for (var ly = 0; ly < back.height; ly += 20) {
        context.moveTo(0, ly);
        context.lineTo(back.width, ly);
    }
    context.strokeStyle = "lightblue";
    context.stroke();

    context.strokeRect(0, 0, back.width, back.height);
}


/*
 * Simulation Events
 */

/**
 * Handler for mouse down events on the sim canvas.
 */
function simMouseDown(event) {

    // get the target of the click
    simState.dragTarget = null;
    if(simView.view.encloses(event.offsetX, event.offsetY)) {
        simState.dragTarget = simView;
    } else {
        for(i in simState.worldObjects) {
            var obj = simState.worldObjects[i];
            //ADDED BY GAVIN 03/21/2023
            //EDITED BY GAVIN 04/05/2023
            if ((obj.part.type == "Light" && obj.part.moveable == false) || (obj.part.type == "Wall" && obj.part.moveable == false) || (obj.part.type == "Box" && obj.part.moveable == false)){
                console.log("HEY THERE");
                break;
            }
            //END OF ADDED BY GAVIN 03/21/2023
            if(obj.view.encloses(event.offsetX, event.offsetY)) {
                simState.dragTarget = obj;
                console.log(simState.dragTarget);
                break;
            }
        }
    }

    if(opponent){
        if(opponentView.view.encloses(event.offsetX, event.offsetY)){
            simState.dragTarget = opponentView;
        }
    }
    
    
    if(!simState.dragTarget) {
        // !!!!!!!!!!!!! Sam Elfrink addition !!!!!!!!!!!!!
        if(document.getElementById('dragDraw').checked) {
            console.log("checked");
            //dragDraw = true;
            DrawFunction();
        }
        return false;
    }

    //record this position
    simState.lastX = event.offsetX;
    simState.lastY = event.offsetY;

    //get the mode
    if(document.getElementById('dragRotate').checked) {
        simState.dragMode = DRAG_ROTATE;
    } else if(document.getElementById('dragMove').checked) {
        simState.dragMode = DRAG_MOVE;
        simState.dragTarget.part.moveTo(event.offsetX, event.offsetY);

    //UPDATED AND ADDED BY GAVIN 03/08/2023
    } else if(document.getElementById('dragResizeHeight').checked) {
        simState.dragMode = DRAG_RESIZEHEIGHT;  //Updated by Gavin 03/08/2023
    }else if(document.getElementById('dragResizeWidth').checked) {
        simState.dragMode = DRAG_RESIZEWIDTH;  //Updated by Gavin 03/08/2023
    } else if(document.getElementById('dragRotate90').checked) {
        simState.dragMode = DRAG_ROTATE90;  //Updated by Gavin 03/08/2023
    } 
    // !!!!!!!!!!!!! Sam Elfrink addition !!!!!!!!!!!!!
    //else if(document.getElementById('dragDraw').checked) {
    //    console.log("checked");
    //    dragDraw = true;
    //    DrawFunction();
    //}
    // !!!!!!!!!!!!!
    //END OF UPDATED AND ADDED BY GAVIN 03/08/2023
    else {
        simState.dragMode = DRAG_NONE;
    }

    //refresh the canvas
    drawSim();

    return true;
}


/**
 * Handler for mouse up events on the sim canvas.
 */
function simMouseUp(event) {
    console.log("mouse up");
    // one last move (if that is what we are up to!)
    if(simState.dragMode == DRAG_MOVE) {
        simState.dragTarget.part.moveTo(event.offsetX, event.offsetY);
    } 
    //Updated and Added by Gavin 03/08/2023
    else if (simState.dragMode == DRAG_RESIZEHEIGHT && simState.dragTarget.part.type == 'Wall'){
        if (simState.dragTarget.part.resizeFactorHeight < 100){
            simState.dragTarget.part.resizeFactorHeight = simState.dragTarget.part.resizeFactorHeight + 1;
        }else{
            simState.dragTarget.part.resizeFactorHeight = 1;
        }
        for (var i=0; simState.worldObjects.length; i++){
            if (simState.worldObjects[i].part.name == simState.dragTarget.part.name){
                simState.worldObjects.splice(i, 1);
                break;
            };
        }
        simState.worldObjects.push(new WallView(simState.dragTarget.part));
    }

    else if (simState.dragMode == DRAG_RESIZEWIDTH && simState.dragTarget.part.type == 'Wall'){
        if (simState.dragTarget.part.resizeFactorWidth < 100){
            simState.dragTarget.part.resizeFactorWidth = simState.dragTarget.part.resizeFactorWidth + 1;
        }else{
            simState.dragTarget.part.resizeFactorWidth = 1;
        }
        for (var i=0; simState.worldObjects.length; i++){
            if (simState.worldObjects[i].part.name == simState.dragTarget.part.name){
                simState.worldObjects.splice(i, 1);
                break;
            };
        }
        simState.worldObjects.push(new WallView(simState.dragTarget.part));
    }

    else if (simState.dragMode == DRAG_ROTATE90 && simState.dragTarget.part.type == 'Wall'){
       
        simState.dragTarget.part.rotated ^= true;
        console.log(simState.dragTarget.part.rotated);
        var part = simState.dragTarget.part;
        deselectPart(simState);

        var partArray = [];

        for(var i=0; i<simState.worldObjects.length; i++) {
            partArray[i] = simState.worldObjects[i].part;
        }

        // remove it from the robot parts list
        simState.worldObjects.splice(partArray.indexOf(part), 1);

        // redraw the sim
        drawSim();
        simState.worldObjects.push(new WallView(simState.dragTarget.part));

    }
    //End of Updated and Added by Gavin 03/08/2023

    // end the drag mode
    simState.dragMode = DRAG_NONE;

    //if this is not the robot, select it
    if(simState.dragTarget && simState.dragTarget.part.type != "Chassis") {
        selectPart(simState.dragTarget, simState);
    }

    //refresh the canvas
    //!!!!!!!!!!!!!!!! Sam Elfrink Addition !!!!!!!!!!!!!!!!
    if(document.getElementById('dragDraw').checked) {
        // don't draw sim, drawing won't stay otherwise
        console.log("drawDraw checked");
        return true;
    }
    else {
        console.log("drawDraw not checked");
        drawSim(); 
    }

    //!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
    //drawSim(); // zzz Sam Elfrink Addition: drawing won't stay otherwise
    return true;
}


/**
 * Handler for mouse move events on the sim canvas.
 */
function simMouseMove(event) {
    // if we have no drag mode, do nothing
    if(simState.dragMode == DRAG_NONE) {
        return false;
    }

    // process movement
    if(simState.dragMode == DRAG_MOVE) {
        simState.dragTarget.part.moveTo(event.offsetX, event.offsetY);
    }

    //process rotation
    if(simState.dragMode == DRAG_ROTATE) {
        simState.dragTarget.part.rotate((event.offsetY-simState.lastY) * .01); //.01 actual
    }

    //record this position
    simState.lastX = event.offsetX;
    simState.lastY = event.offsetY;

    // refresh the canvas
    drawSim();

    return true;
}


/**
 * Add a light to the simulator.
 * @param {*} event 
 */
function simAddLight(event) {
    //ADDED BY GAVIN 04/05/2023
    //Don't create light in premade world
    if(simState.combatWorldLoaded || simState.mazeWorldLoaded || simState.pacmanWorldLoaded){
        notAvailablePopup();
        return;
    }
    //END OF ADDED BY GAVIN 04/05/2023
    var canvas = document.getElementById("simfg");
    var light = new Light(null, canvas.width/2, canvas.height/2);

    simState.worldObjects.push(constructView(light));
    drawSim();
}


/**
 * Add a wall to the simulator.
 * @param {*} event 
 */
function simAddWall(event) {
    //ADDED BY GAVIN 04/05/2023
    //Don't create wall in premade world
    if(simState.combatWorldLoaded || simState.mazeWorldLoaded || simState.pacmanWorldLoaded){
        notAvailablePopup();
        return;
    }
    //END OF ADDED BY GAVIN 04/05/2023
    var canvas = document.getElementById("simfg");
    simState.rotated = false;   //Added by Gavin 03/09/2023
    var wall = new Wall(null, canvas.width/2, canvas.height/2);

    simState.worldObjects.push(constructView(wall));
    //console.log(wall);
    drawSim();
}


/**
 * Add a box to the simulator.
 * @param {*} event 
 */
function simAddBox(event) {
    //ADDED BY GAVIN 04/05/2023
    //Don't create box in premade world
    if(simState.combatWorldLoaded || simState.mazeWorldLoaded || simState.pacmanWorldLoaded){
        notAvailablePopup();
        return;
    }
    //END OF ADDED BY GAVIN 04/05/2023
    var canvas = document.getElementById("simfg");
    var box = new Box(null, canvas.width/2, canvas.height/2, 50);

    simState.worldObjects.push(constructView(box));
    drawSim();
}


/*
 * Build Events
 */

/**
 * Show the editor for the part specified by "view".
 * @param {*} view 
 */
function showPartEditor(view, state) {
    //populate the type and name
    document.getElementById(state.prefix +"PartType").innerHTML = view.part.type;
    document.getElementById(state.prefix +"PartName").value = view.part.name;

    //get the colors populated
    document.getElementById(state.prefix + "PartOutlineColor").value = view.part.outline;
    document.getElementById(state.prefix + "PartFillColor").value = view.part.fill;


    //show the editor pane
    document.getElementById(state.prefix + "PartEditor").style.display="block";

    //!!!!!!!!!!!! Addition by Sam Elfrink !!!!!!!!!!!!!!!!!
    // If a part is click on the grid, the drop-down menu auto selects it
    var options = document.getElementById("partDropDown").options;
    for (var i = 0; i < options.length; i++) {
        if (options[i].text == view.part.name) {
            options[i].selected = true;
            break;
        }
    }
    //!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
}

// !!!!!!!!!! Addition Sam Elfrink !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
// Same as the showPartEditor, but removed the auto select addition
/**
 * Show the editor for the part specified by "view".
 * @param {*} view 
 */
function dropDownClick(view, state) {
    
    selectPartName = view.part.name;
    console.log(selectPartName);
    //populate the type and name
    document.getElementById(state.prefix +"PartType").innerHTML = view.part.type;
    document.getElementById(state.prefix +"PartName").value = view.part.name;

    //get the colors populated
    document.getElementById(state.prefix + "PartOutlineColor").value = view.part.outline;
    document.getElementById(state.prefix + "PartFillColor").value = view.part.fill;


    //show the editor pane
    document.getElementById(state.prefix + "PartEditor").style.display="block";
    
}
// !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

/**
 * Hide the part editor.
 */
function hidePartEditor(state) {
    //hide the editor pane
    document.getElementById(state.prefix + "PartEditor").style.display="none";
}


/**
 * Select the partview in the editor.
 * @param {*} view 
 */
function selectPart(view, state) {

    // Sam Elfrink Addition zzzz
    selectPartName = view.part.name;
    console.log(selectPartName);
    // end of addition
    
    // deselect (if needed) 
    if(state.editTarget != null) {
        deselectPart(state);
    }

    // grab the original color 
    state.editOriginalOutline = view.part.outline;

    // set up the target
    state.editTarget = view;

    //show the editor
    showPartEditor(view, state);

    // color it coral
    view.part.outline = "coral";


    // redraw the canvas
    if(state.prefix == "build") {
        drawBuild();
    } else if(state.prefix == "sim") {
        drawSim();
    }
}

/**
 * !!!!! Addition by Sam Elfrink: Drop Down menu for part editor !!!!!!
 * Very similar to the selectPart function, just triggered on the drop down click
 * Select the partview in the editor.
 * @param {*} event 
 */
function dropDownPartSelect(event) {
    // get part name from the drop-down menu
    var dropDownMenu = document.getElementById("partDropDown");
    var dropPartName = dropDownMenu.options[dropDownMenu.selectedIndex].text;
  
    // check for clicking on a robot subpart
    for(var i=0; i < buildView.subviews.length; i++) {
        var partView = buildView.subviews[i]; //left = [0] right [1], parts ... 
       
        // if the part selected matches the part current partview, select it
        //NOTE: THERE IS NO CHASSISS IN THE buildView.subviews ARRAY, so chassiss select doesn't work
        if(partView.part.name == dropPartName) {
            buildState.dragTarget = partView;
            break;
        }
    }

    // deselect (if needed) 
    if(buildState.editTarget != null) {
        deselectPart(buildState);
    }

    // grab the original color 
    buildState.editOriginalOutline = buildState.dragTarget.part.outline;
    
    // set up the target
    buildState.editTarget = buildState.dragTarget;

    //show the editor
    dropDownClick(buildState.dragTarget, buildState);
    //showPartEditor(buildState.dragTarget, buildState);

    // color it coral
    buildState.dragTarget.part.outline = "coral";


    // redraw the canvas
    if(buildState.prefix == "build") {
        drawBuild();
    } else if(buildState.prefix == "sim") {
        drawSim();
    }
}


/**
 * Deselect the selected partview (if there is one)
 */
function deselectPart(state) {
    // do nothing if there is no selected part
    if(state.editTarget == null) {
        return;
    }

    // retore the original color
    state.editTarget.part.outline = state.editOriginalOutline;

    // remove the selection
    state.editTarget = null;
    hidePartEditor(state);

    // redraw the canvas
    if(state.prefix == "build") {
        drawBuild();
    } else if(state.prefix == "sim") {
        drawSim();
    }
}

//!!!!!!!!!!!! Addition by Sam Elfrink !!!!!!!!!!!!!!!!!
// check to see if a string input is a valid javascript variable name
function isVariableName(string) {
    // Check to make sure the input is a string
    if(typeof string !== 'string') {
        return false;
    }
    // Check for whitepace before or after
    if(string.trim() !== string) {
        return false;
    }
    // try the value as a variable
    try {
        new Function(string, 'var ' + string);
    } catch (_) {
        return false;
    }
    // return true if it passes the test
    return true;
}
//!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

/**
 * Apply the editor to the given state.
 */
function applyEditor(state) {
    //get the color from the editor
    var fill = document.getElementById(state.prefix + "PartFillColor").value;
    var outline = document.getElementById(state.prefix + "PartOutlineColor").value;
    
    //get the part name
    var name = document.getElementById(state.prefix + "PartName").value;

    // !!!!! Sam Elfrink Addition !!!!!!!!!
    // check to make sure the new part name is a valid javascript variable
    if(isVariableName(name) == false) {
        alert("Part name must be a valid JavaScript variable name");
        return;
    }
    // !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
    
    // !!!!! Sam Elfrink Addition !!!!!!!!!
    // Check to make sure new part name is not one 
    // of the function names in userbot
    let i = 0;
    while (i < blackList.length) {
        if(name == blackList[i]) {
            alert("Part name cannot be a default gradbot name.");
            return;
        }
        i++;
    }
    // !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

    // !!!!! Sam Elfrink Addition !!!!!!!!!
    // check to make sure the part name was changed
    if(name != selectPartName) {
        // Check to make sure new part name already used
        let j = 0;
        while (j < newPartList.length) {
            if(name == newPartList[j]) {
                alert("Part name must be unique.");
                return;
            }
            j++;
        }
        // remove the old name from the list
        var index = newPartList.indexOf(selectPartName);
        newPartList.splice(index, 1);
    }
    else {
        cancelAdd = 1;
    }
    // !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

    //get the part we are editing
    var part = state.editTarget.part;

    //deselect the part
    deselectPart(state);

    //set the fields
    part.fill = fill;
    part.outline = outline;
    part.name = name;

    // !!!!!!!!!!!!!!!!! Addition by Sam Elfrink !!!!!!!!!!!!!!!!
    // When a part is selected, the editor removes the name from the drop-down list
    var dropDownElement = document.getElementById("partDropDown");
    dropDownElement.remove(dropDownElement.selectedIndex);
    // The new name is add to the drop-down list
    //console.log("apply editor");
    //console.log("Addlist ApplyEditor function called");
    addListTrue = 1;
    addList(part.name);
    addListTrue = 0;
    //!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

    // redraw the canvas
    if(state.prefix == "build") {
        drawBuild();
    } else if(state.prefix == "sim") {
        drawSim();
    }
}

//Aayush 
function DropMarker(event) {
    var marker = new Marker(robot);
    robot.addPart(marker);
    buildView.addPart(marker);
    drawBuild();
}

    //For Dragging part editor for build : Aayush

    var dragMarkerButton = document.getElementById("DragMarker");
    var dragLightButton = document.getElementById("DragLight");
    var dragLightSensorButton = document.getElementById("DragLightSensor");
    var dragRangeFinderButton = document.getElementById("DragRangeFinder");
    var dragLaserButton = document.getElementById("DragLaser");

    // Add event listeners for dragstart, dragmove, and dragend events to all buttons
    [dragMarkerButton, dragLightButton, dragLightSensorButton, dragRangeFinderButton, dragLaserButton].forEach(function(button) {
        // Add event listener for dragstart event
        button.addEventListener("dragstart", function(event) {
            console.log("dragstart");
            event.dataTransfer.setData("text/plain", "This is a test drag and drop.");
        });

        // Add event listener for dragmove event
        button.addEventListener("dragmove", function(event) {
            console.log("dragmove");
        });

        // Add event listener for dragend event
        button.addEventListener("dragend", function(event) {
            console.log("dragend");
            var data = event.dataTransfer.getData("text/plain");
            if (button == dragMarkerButton) {
                DropMarker(event);
            } else if (button == dragLightButton) {
                buildAddLight();
            } else if (button == dragLightSensorButton) {
                buildAddLightSensor();
            } else if (button == dragRangeFinderButton) {
                buildAddRangeSensor();
            } else if (button == dragLaserButton) {
                buildAddLaser();
            }
        });
    });


    //Drag for Buttons : Aayush
    var buildAddMarkerButton = document.getElementById("buildAddMarker");
        // Add event listener for dragstart event
	buildAddMarkerButton.addEventListener("dragstart", function(event) {
		console.log("dragstart");
		event.dataTransfer.setData("text/plain", "This is a test drag and drop.");
	});

	// Add event listener for dragmove event
	buildAddMarkerButton.addEventListener("dragmove", function(event) {
		console.log("dragmove");
	});

	// Add event listener for dragend event
	buildAddMarkerButton.addEventListener("dragend", function(event) {
		console.log("dragend");
        DropMarker(event);
	});

	// Add event listener for drop event
	document.body.addEventListener("drop", function(event) {
		console.log("drop");
		event.preventDefault();
		var data = event.dataTransfer.getData("text/plain");
		console.log("Dropped data: " + data);
	});

	// Add event listener for dragover event
	document.body.addEventListener("dragover", function(event) {
		console.log("dragover");
		event.preventDefault();
	});

    var buildAddLightButton = document.getElementById("buildAddLight");
    // Add event listener for dragend event
    buildAddLightButton.addEventListener("dragend", function(event) {
        buildAddLight(event);
    });           
        
    var buildAddLightSensorButton = document.getElementById("buildAddLightSensor");
    // Add event listener for dragend event
    buildAddLightSensorButton.addEventListener("dragend", function(event) {
        buildAddLightSensor(event);
    });

    var buildAddRangeSensorButton = document.getElementById("buildAddRangeSensor");
    // Add event listener for dragend event
    buildAddRangeSensorButton.addEventListener("dragend", function(event) {
        buildAddRangeSensor(event);
    });         
        
    var buildAddLaserButton = document.getElementById("buildAddLaser");
    // Add event listener for dragend event
    buildAddLaserButton.addEventListener("dragend", function(event) {
        buildAddLaser(event);
    });



/**
 * Handle build canvas mouse down 
 * @param {*} event 
 */
function buildMouseDown(event) {
    var x = event.offsetX;
    var y = event.offsetY;

    // assume we have found nothing
    buildState.dragTarget = null;
    buildState.dragMode = DRAG_NONE;

    // check for clicking on a robot subpart
    for(var i=0; i < buildView.subviews.length; i++) {
        var partView = buildView.subviews[i];
        if(partView.view.encloses(x, y)) {
            buildState.dragTarget = partView;
            if(buildState.dragTarget.part.type != "Motor") {
                buildState.dragMode = DRAG_MOVE;
            }
            break;
        }
    }

    // if no subpart is selected, see if we have selected the chassis body.
    if(buildState.dragTarget == null && buildView.view.encloses(x,y)) {
        buildState.dragTarget = buildView;
    }

    // if we still lack a subpart, return
    if(!buildState.dragTarget) {
        return;
    }

    //record last x and last y
    buildState.lastX = x;
    buildState.lastY = y;
}


/**
 * Handle build canvas mouse move 
 * @param {*} event 
 */
function buildMouseMove(event) {
    //if there is no return target, stop!
    if(!buildState.dragTarget) { return; }

    var x = event.offsetX;
    var y = event.offsetY;

    //move the part (if that is our mode)
    if(buildState.dragMode == DRAG_MOVE) {
        if(buildView.view.encloses(x, y)) {
            var p = buildState.dragTarget.globalToPartLoc(x, y);
            buildState.dragTarget.moveTo(p.x, p.y);
            drawBuild();
        }
    }

    //record last x and last y
    buildState.lastX = x;
    buildState.lastY = y;
}


/**
 * Handle build canvas mouse up 
 * @param {*} event 
 */
function buildMouseUp(event) {
    // if there was a drag target, it is now the selected object
    if(buildState.dragTarget) {
        selectPart(buildState.dragTarget, buildState);
    }

    // clear the drag target
    buildState.dragTarget = null;
    buildState.dragMode = DRAG_NONE;
}

/**
 * Handle the apply button for the build part editor.
 * @param {*} event 
 */
function buildApply(event) {
    applyEditor(buildState);
}

//!!!!!!!! Sam Elfrink Addition !!!!!!!!
/**
 * 
 * @param {*} event 
 */
function wheelApply(event){
    changeWheelSize(buildState);
}
//!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

/**
 * Handle the apply button for the sim part editor.
 * @param {*} event 
 */
function simApply(event) {
    applyEditor(simState);
}


/**
 * Handle the build cancel button
 * @param {*} event 
 */
function buildCancel(event) {
    deselectPart(buildState);
}


/**
 * Handle the sim cancel button
 * @param {*} event 
 */
function simCancel(event) {
    deselectPart(simState);
}



/**
 * Handle the build delete button.
 * @param {*} event
 */
function buildDeletePart(event) {
    var part = buildState.editTarget.part;

    // Sam Elfrink Addition 
    console.log(newPartList);
    var index = newPartList.indexOf(part.name);
    console.log(part.name);
    console.log(index);
    newPartList.splice(index, 1);
    // end Sam Elfrink Addition

    if(part.type == "Motor" || part.type == "Chassis") {
        alert("You cannot remove this part.");
        return;
    }

    if(!confirm("Are you sure you want to remove " + part.name + "?")) {
        return;
    }

    deselectPart(buildState);

    // remove it from the robot parts list
    robot.parts.splice(robot.parts.indexOf(part), 1);

    //!!!!!! Addition Sam Elfrink !!!!!!!!
    // delete the part from the drop-down list
    var options = document.getElementById("partDropDown").options;
     for (var i = 0; i < options.length; i++) {
        if (options[i].text == part.name) {
            options[i].selected = true;
            break;
        }
    }
    var dropDownElement = document.getElementById("partDropDown");
    dropDownElement.remove(dropDownElement.selectedIndex);
    //!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

    // rebuild the robot
    buildView = new ChassisBuildView(robot);
    drawBuild();
}


/**
 * Handle the sim delete button.
 * @param {*} event
 */
function simDeletePart(event) {
    var part = simState.editTarget.part;

    if(part.type == "Motor" || part.type == "Chassis") {
        alert("You cannot remove this part.");
        return;
    }

    if(!confirm("Are you sure you want to remove " + part.name + "?")) {
        return;
    }

    deselectPart(simState);

    var partArray = [];
    for(var i=0; i<simState.worldObjects.length; i++) {
        partArray[i] = simState.worldObjects[i].part;
    }

    // remove it from the robot parts list
    simState.worldObjects.splice(partArray.indexOf(part), 1);

    // redraw the sim
    drawSim();
}

/**
 * !!!!!!! Addition by Sam Elfrink !!!!!!
 * Added function to add an item to the parts drop down list
 * @param {*} event 
 */
function addList(name) {

    // !!!!!!!! Sam Elfrink Addition !!!!!!!
    // add the part name to the list, unless it didn't change
    if(cancelAdd != 1) {
        newPartList.push(name);
        //console.log(newPartList);
    }
    cancelAdd = 0;
    //!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

    var partDropDown = document.getElementById("partDropDown");
    var option = document.createElement("OPTION");
    //console.log(addListTrue);
    if(addListTrue == 1) {
          option.innerHTML = name;
          //option.value = document.getElementById("txtValue").value;
          partDropDown.options.add(option);
    }
}

/**
 * Handle adding a marker. 
 * @param {*} event 
 */
function buildAddMarker(event) {
    addListTrue = 1; //Sam Elfrink zzzz
    var marker = new Marker(robot);
    robot.addPart(marker);
    buildView.addPart(marker);
    drawBuild();
    addListTrue = 0; //Sam Elfrink zzzz
}


/**
 * Handle adding a light.
 * @param {*} event
 */
function buildAddLight(event) {
    addListTrue = 1; //Sam Elfrink zzzz
    var light = new Light(robot);
    light.radius = 1;
    robot.addPart(light);
    buildView.addPart(light);
    drawBuild();
    addListTrue = 0; //Sam Elfrink zzzz
}


/**
 * Handle adding a light sensor.
 * @param {*} event
 */
function buildAddLightSensor(event) {
    addListTrue = 1; //Sam Elfrink zzzz
    var sensor = new LightSensor(robot);
    robot.addPart(sensor);
    buildView.addPart(sensor);
    drawBuild();
    addListTrue = 0; //Sam Elfrink zzzz
}


/**
 * Handle adding a range sensor.
 * @param {*} event
 */
function buildAddRangeSensor(event) {
    addListTrue = 1; //Sam Elfrink zzzz
    var sensor = new RangeSensor(robot);
    robot.addPart(sensor);
    buildView.addPart(sensor);
    drawBuild();
    addListTrue = 0; //Sam Elfrink zzzz
}


/**
 * Handle adding a laser.
 * @param {*} event
 */
function buildAddLaser(event) {
    addListTrue = 1; //Sam Elfrink zzzz
    var laser = new Laser(robot);
    robot.addPart(laser);
    buildView.addPart(laser);
    drawBuild();
    addListTrue = 0; //Sam Elfrink zzzz
}

// !!!!!!!!!!! Sam Elfrink Addition !!!!!!!!!!!!!!
// handle the PNG background upload
/**
 * Handle the simulation go button.
 * @param {*} event 
 */
function backgroundPhotoDraw(event) {
    var reader = new FileReader();
    reader.onload = function(event) {
        var canvas = document.getElementById("simbg");
        var context = canvas.getContext("2d");
        console.log("yo");
        const img = new Image();
        img.onload = () => {
            console.log("loop");
            context.drawImage(img, 0, 0, 800, 600);
        };
        img.src = event.target.result;
    }
    reader.readAsDataURL(event.target.files[0]);
}
// !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

/**
 * Handle the simulation go button.
 * @param {*} event 
 */
function simulationGo(event) {

    //!!!!! Sam Elfrink Addition !!!!!!!!!
    drawPlayerHUD();
    if(opponentClicked == 1){
        drawOpponentHUD();
    }
    // !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

    var text = event.target.innerHTML;

    if(text == "Start") {
        event.target.innerHTML = "Pause";
        
        simulationStart();
    } else if(text == "Pause") {
        event.target.innerHTML = "Resume";
        var elapsed = stopStopwatch(); //ADDED BY GAVIN 04/06/2023
        simulationStop();
    } else if(text == "Resume") {
        event.target.innerHTML = "Pause";
        simulationStart();
    }

    if (robot.blowedUp == true && event.target.innerHTML == "Resume"){
        robot.thread = null;
        if(simState.robotThread) {
            simState.robotThread.terminate();
        }
        alert("Oops, looks like your robot has been destroyed. Hit reset.");
        return;
    }

    //preserve the starting pose of the robot
    simState.robotStartX = robot.x;
    simState.robotStartY = robot.y;
    simState.robotStartHeading = robot.heading;

    
    if(opponent){
        //preserve the starting pose of the opponent
        simState.opponentStartX = opponent.x;
        simState.opponentStartY = opponent.y;
        simState.opponentStartHeading = opponent.heading;
        
        //allows the player to resume without the bot
        if (opponent.blowedUp == true && event.target.innerHTML == "Resume"){
            opponentClicked = 0;
            opponent = null;
            if(simView.opponentThread) {
                simView.opponentThread.terminate();
            }

            simView.opponentThread = null;
            opponentView = null;
            simState.opponentThread = null;
        }
    }       
}

function simulationReset(event) {
    document.getElementById("simGo").innerHTML = "Start";
    var canvas = document.getElementById("simfg");
    // Sam Elfrink Addition
    // prompts the user to reupload their picture
    //document.getElementById("pictureUpload").click();

    robotStartingLocation();  //ADDED BY GAVIN 04/05/2023

    // move the robot to the normal start position.
    robot.x = 100;
    robot.y = 100;
    robot.heading = 0;

    // reset the opponent
    if(opponent){
        opponent.x = 700;
        opponent.y = 500;
        opponent.heading = Math.PI;
    }

    //refuel and powered up
    robot.left.setPower(0);
    robot.right.setPower(0);
    robot.resetLaserBattery();
    robot.hp = 3;

    //MAYDAY!
    if (robot.blowedUp == true){
        console.log('blowedUp')
        robot.thread = null;
        if(simState.robotThread) {
            simState.robotThread.terminate();
        }
        
        //Six Million Dollar Bot
        robot = new Chassis();
        robot.x = 100;
        robot.y = 100;
        robot.heading = 0;
        
        addListTrue = 1; // Sam Elfrink

        loadRobot(robot);
        
        addListTrue = 0; // Sam Elfrink

        simView = new ChassisView(robot); 
        buildView = new ChassisBuildView(robot);
        simState.robotThread = new Worker("userbot.js");
        simState.robotThread.onerror = gradbotError;
        simState.robotThread.onmessage = simulationReceiveMessage;
        simState.robotThread.postMessage({type: "start", robot: robot.sendable()});
        robot.thread = simState.robotThread;

        //refreshed stats
        robot.left.setPower(0);
        robot.right.setPower(0);
        robot.resetLaserBattery();
        robot.hp = 3;
    }  

    if(opponent){

        //ADDED BY GAVIN 04/03/2023
        if(simState.combatWorldLoaded == true){
            console.log("COMBAT WORLD LOADED")
            opponent.moveTo(simState.opponentCombatStartX,simState.opponentCombatStartY);
            opponent.face(simState.opponentStartHeading);   //ADDED BY GAVIN 04/06/2023
        }
        else{
            console.log("COMBAT WORLD NOT LOADED")
            opponent.moveTo(simState.opponentStartX, simState.opponentStartY);
            opponent.face(simState.opponentStartHeading);
        }
        //END OF ADDED BY GAVIN 

        //put the opponent back in its original position and heading
        //opponent.moveTo(simState.opponentStartX, simState.opponentStartY);    //GAVIN COMMENTED OUT 04/06/2023
        //opponent.face(simState.opponentStartHeading);

        //Tis but a scratch 
        opponent.left.setPower(0);
        opponent.right.setPower(0);
        opponent.resetLaserBattery();
        opponent.hp = 3;
    
        //checks if opponent exploded and removes the mess after reset
        if (opponent.blowedUp == true){
            console.log('blowedUp')
            opponent = null;
            if(simView.opponentThread) {
                simView.opponentThread.terminate();
            }
            simView.opponentThread = null;
            opponentView = null;
            simState.opponentThread.terminate();
            simState.opponentThread = null;
    
            // Respawn the correct opponent
            switch(opponentType) {
                case "rover":
                    loadRoverOpponent();
                    break;
                case "circler":
                    loadCirclerOpponent();
                    break;
                case "spinner":
                    loadSpinnerOpponent();
                    break;
                case "custom":
                    document.getElementById("simUpload").value = "";
                    document.getElementById('simUpload').click();          
                    break;
            }
        }
    }

    //clear the background
    graphPaperFill("simbg");

    simulationStop();

    //draw the simulator
    drawSim();
}

/**
 * Handle simulation clear.
 * @param {*} event
 */
function simulationClear(event) {
    document.getElementById("simReset").click();

    // move the robot to the normal start position.
    robot.x = 100;
    robot.y = 100;
    robot.heading = 0;

    // reset the opponent
    if(opponent){
        opponent.x = 700;
        opponent.y = 500;
        opponent.heading = Math.PI;
    }
    
    removePacmanPoints();   //Added By GAVIN 04/06/2023
    //ADDED BY GAVIN 03/25/2023
    simState.combatWorldLoaded = false;
    simState.mazeWorldLoaded = false;
    simState.pacmanWorldLoaded = false;
    //END OF ADDED BY GAVIN 03/25/2023

    //clear lights and walls
    if(simState.worldObjects.length != 0) {
        if(confirm("Would you like to remove the world objects?")) {
            simState.worldObjects= [];
        }
    }

    //redraw 
    var canvas = document.getElementById('simdg');
    var context = canvas.getContext("2d");
    context.clearRect(0, 0, canvas.width, canvas.height);
    graphPaperFill("simbg");
    drawSim();
}


/**
 * Gradbot Error Handler
 * @param {*} event 
 */
function gradbotError(message) {
    document.getElementById("simReset").click();
    alert(message);
}


/**
 * Gradbot Error Handler
 * @param {*} event 
 */
function gradbotError(message) {
    document.getElementById("simReset").click();
    alert(message);
}




/**
 * Initialize the gradbot interface.
 */
function gradbotInit() {
    
    // Sam Elfrink Addition
    loadRobotTrue = 1;

    console.log("gradbotInit called");
    //fill the simulation background with graph paper
    graphPaperFill('simbg');

    //create the robot
    robot = new Chassis(100, 100, 0, "chassis");
    loadRobot(robot);
    simState.robotStartX = robot.x;
    simState.robotStartY = robot.y;
    simState.robotStartHeading = robot.heading;

    //!!!!!!!!!!!!!!!Sam Elfrink Addition !!!!!!!!!!!!!!!!!!!!!
     // Sam Elfrink Addition
     loadRobotTrue = 0;
    // remove motors and chassis from the drop-down list
    //var dropDownElementInit = document.getElementById("partDropDown");
    //var firstListName = dropDownElementInit.options[0].text;
    //var firstListName = dropDownElementInit.options[dropDownElementInit.selectedIndex].value;
    //console.log(firstListName);
    //if(firstListName == 'chassis'){
    //    document.getElementById("partDropDown").options.length = 0;
    //}
    //document.getElementById("partDropDown").options.length = 0;
    //!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

    //put the robot on the foreground of the simulator
    simView = new ChassisView(robot);
    drawSim();

    //build the robot builder view
    buildView = new ChassisBuildView(robot);
    drawBuild();

    //set up the sim mouse events
    var canvas = document.getElementById('simfg');
    canvas.onmousedown = simMouseDown;
    canvas.onmouseup = simMouseUp;
    canvas.onmousemove = simMouseMove;

    //!!!!!!!!! Sam Elfrink Addition !!!!!!!!!!!!!!
    const backgroundPhotoInput = document.getElementById("pictureUpload");
    document.getElementById("backgroundChange").onclick = function() {
        backgroundPhotoInput.value = "";
        backgroundPhotoInput.click();
        
    };
    backgroundPhotoInput.onchange = backgroundPhotoDraw;
    //!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

    //set up the sim buttons
    document.getElementById('simGo').onclick = simulationGo;
    document.getElementById('simReset').onclick = simulationReset;
    document.getElementById('simClear').onclick = simulationClear;

    //set up the object buttons
    document.getElementById('simAddLight').onclick = simAddLight;
    document.getElementById('simAddWall').onclick = simAddWall;
    document.getElementById('simAddBox').onclick = simAddBox;

    //set up the build mouse events
    canvas = document.getElementById("buildCanvas");
    canvas.onmousedown = buildMouseDown;
    canvas.onmouseup = buildMouseUp;
    canvas.onmousemove = buildMouseMove;

    // Gavin Added 02/22/2023
    //set up world handlers
    document.getElementById("worldOpen").onclick = function() {
        deselectPart(simState); //Might need to fix
        document.getElementById("worldUpload").value = ""; 
        document.getElementById("worldUpload").click();
    };
    document.getElementById("worldSave").onclick = saveWorldFile;
    document.getElementById("worldUpload").onchange = openWorldFile ;
    //End of Gavin Added

    //!!!!!!!!! Sam Elfrink Addition !!!!!!!!!!!!!
    document.getElementById("partDropDown").onchange = dropDownPartSelect;

    //!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

    // !!!!!!!!!!!!!!!!!!!!! Sam Elfrink Addition !!!!!!!!!!!!!!!!!!!!!
    // Apply the text fields for the part editor in the build tab when the enter key is pressed
    // Get the input field for buildPartName
    var partNameBuild = document.getElementById("buildPartName");

    // If the user presses the "Enter" key on the keyboard, apply the input field
    partNameBuild.addEventListener("keypress", function(event) {
    if (event.key === "Enter") {
        // Cancel the default action
        event.preventDefault();
        // Trigger the buildpartApply button
        document.getElementById("buildPartApply").click();
    }
    });    

    // Get the input field buildPartOutlineColor
    var partOutlineColorBuild = document.getElementById("buildPartOutlineColor");
    // If the user presses the "Enter" key on the keyboard, apply the input field
    partOutlineColorBuild.addEventListener("keypress", function(event) {
    // If the user presses the "Enter" key on the keyboard
    if (event.key === "Enter") {
         // Cancel the default action
        event.preventDefault();
        // Trigger the buildpartApply button
        document.getElementById("buildPartApply").click();
    }
    });    

    // Get the input field buildPartFillColor
    var partFillColorBuild = document.getElementById("buildPartFillColor");
    // If the user presses the "Enter" key on the keyboard, apply the input field
    partFillColorBuild.addEventListener("keypress", function(event) {
    // If the user presses the "Enter" key on the keyboard
    if (event.key === "Enter") {
         // Cancel the default action
        event.preventDefault();
        // Trigger the buildpartApply button
        document.getElementById("buildPartApply").click();
    }
    });    
    // !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

    //set up the build's form buttons
    document.getElementById("buildPartApply").onclick = buildApply;
    document.getElementById("buildPartCancel").onclick = buildCancel;
    document.getElementById("buildPartDelete").onclick = buildDeletePart;

    // !!!!!!!!!!!!!!!!!!!!! Sam Elfrink Addition !!!!!!!!!!!!!!!!!!!!!
    // Apply the text fields for the part editor in the simulation when the enter key is pressed
    // Get the input field for simPartName
    var partNameSim = document.getElementById("simPartName");

    // If the user presses the "Enter" key on the keyboard, apply the input field
    partNameSim.addEventListener("keypress", function(event) {
    if (event.key === "Enter") {
        // Cancel the default action
        event.preventDefault();
        // Trigger the simPartApply button
        document.getElementById("simPartApply").click();
    }
    });    

    // Get the input field simPartOutlineColor
    var partOutlineColorSim = document.getElementById("simPartOutlineColor");
    // If the user presses the "Enter" key on the keyboard, apply the input field
    partOutlineColorSim.addEventListener("keypress", function(event) {
    // If the user presses the "Enter" key on the keyboard
    if (event.key === "Enter") {
         // Cancel the default action
        event.preventDefault();
        // Trigger the simPartApply button
        document.getElementById("simPartApply").click();
    }
    });    

    // Get the input field simPartOutlineColor
    var partFillColorSim = document.getElementById("simPartFillColor");
    // If the user presses the "Enter" key on the keyboard, apply the input field
    partFillColorSim.addEventListener("keypress", function(event) {
    // If the user presses the "Enter" key on the keyboard
    if (event.key === "Enter") {
         // Cancel the default action
        event.preventDefault();
        // Trigger the simPartApply button
        document.getElementById("simPartApply").click();
    }
    });    
    // !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

    //set up the sim's form buttons
    document.getElementById("simPartApply").onclick = simApply;
    document.getElementById("simPartCancel").onclick = simCancel;
    document.getElementById("simPartDelete").onclick = simDeletePart;

    //select the simulation tab
    document.getElementById('simButton').click();

    //set up file handlers
    document.getElementById("buildOpen").onclick = function() {
        deselectPart(buildState);
        document.getElementById("buildUpload").value = "";
        document.getElementById("buildUpload").click();
    };
    document.getElementById("buildSave").onclick = saveRobotFile;
    document.getElementById("buildUpload").onchange = openRobotFile ;
    document.getElementById("buildNew").onclick = function() {
        if(confirm("Are you sure you want to create a new robot? Any unsaved changes will be lost!")) {
            deselectPart(buildState);
            newRobot();
        }
    }

    // !!!!!!!!!!!!!!!!!!!!! Sam Elfrink Addition !!!!!!!!!!!!!!!!!!!!!
    // Apply the text fields for the wheelsize when the enter key is pressed
    // Get the input field for wheelSize
    var wheelSizeVar = document.getElementById("wheelSize");

    // If the user presses the "Enter" key on the keyboard, apply the input field
    wheelSizeVar.addEventListener("keypress", function(event) {
    if (event.key === "Enter") {
        // Cancel the default action
        event.preventDefault();
        // Trigger the buildpartApply button
        document.getElementById("changeWheelSize").click();
    }
    });  
    //!!!!!!!!! Sam Elfrink Addition !!!!!!!!!!!!!
    document.getElementById("changeWheelSize").onclick = wheelApply;
    //!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

    //set up opponent handlers
    document.getElementById("simUpload").onchange = openOpponentFile;
    document.getElementById("simOpenOpponent").onclick = function() {
        //document.getElementById("simClear").click(); //GAVIN Commented out 03/30/2023
        document.getElementById("simUpload").value = "";
        document.getElementById("simUpload").click();
    };
    document.getElementById("simRemoveOpponent").onclick = function() {
        opponent = null;
        opponentClicked = null; //ADDED BY GAVIN 04/03/2023
        if(simView.opponentThread) {
            simView.opponentThread.terminate();
        }
        simView.opponentThread = null;
        opponentView = null;
        drawSim();
    };
    
    document.getElementById("simRoverOpponent").onclick = loadRoverOpponent;
    document.getElementById("simCirclerOpponent").onclick = loadCirclerOpponent;
    document.getElementById("simSpinnerOpponent").onclick = loadSpinnerOpponent;

    //GAVIN ADDED 04/04/2023
    document.getElementById("Robot1").onclick = loadRobotOne;
    document.getElementById("Robot2").onclick = loadRobotTwo;
    document.getElementById("Robot3").onclick = loadRobotThree;
    //END OF GAVIN ADDED 04/04/2023

    // add part buttons
    document.getElementById("buildAddMarker").onclick = buildAddMarker;
    document.getElementById("buildAddLight").onclick = buildAddLight;
    document.getElementById("buildAddLightSensor").onclick = buildAddLightSensor;
    document.getElementById("buildAddRangeSensor").onclick = buildAddRangeSensor;
    document.getElementById("buildAddLaser").onclick = buildAddLaser;

    // set up code editor
    flask = new CodeFlask('#robotCode', {language: 'js'});


    //activate our error handler
    window.onerror = gradbotError;

    // Change the world state
    var toroidalButton = document.getElementById("toroidal-mode");
    var infiniteButton = document.getElementById("infinite-mode");

    // Add click event listeners to the buttons
    toroidalButton.addEventListener("click", function() {
    simulationMode = "toroidal";
    });

    infiniteButton.addEventListener("click", function() {
    simulationMode = "infinite";
    });

    //GAVIN'S UPDATED CODE STARTS HERE
    //Set up Speed Multipliers
    document.getElementById("x1").onclick = setSpeedMult1;
    document.getElementById("x5").onclick = setSpeedMult5;
    document.getElementById("x10").onclick = setSpeedMult10;
    document.getElementById("x25").onclick = setSpeedMult25;
    //GAVIN'S UPDATED CODE ENDS HERE

    //load world handlers under simulation tabs
    //
    /*
	document.getElementById("worldOpen").onclick = function() {
		deselectPart(buildState);
		document.getElementById("worldUpload").click();
	};
	document.getElementById("worldSave").onclick = saveWorldFile;
	document.getElementById("worldUpload").onchange = openWorldFile ;
	document.getElementById("worldNew").onclick = function() {
		if(confirm("Are you sure you want to create a new World? Any unsaved changes will be lost!")) {
			deselectPart(buildState);
			newWorld();
		}
	}
    */
}


/******************************************
 * Simulator Functions
 ******************************************/


/**
 * Start the simulation.
 */
function simulationStart() {

    // get the simulation extents
    simbg = document.getElementById("simbg");
    simState.width = simbg.width;
    simState.height = simbg.height;

    // mark the simulation as running
    simState.running = true;

    // clear the timer, if there is one
    if(simState.timer) {
        clearInterval(simState.timer);
    }

    //start the robot thread
    simState.robotThread = new Worker("userbot.js");
    simState.robotThread.onerror = gradbotError;
    simState.robotThread.onmessage = simulationReceiveMessage;
    simState.robotThread.postMessage({type: "start", robot: robot.sendable()});
    robot.thread = simState.robotThread;


    //start the opponent thread (if there is one)
    if(opponent){
        if(simState.opponentThread) {
            simState.opponentThread.terminate();
        }
        simState.opponentThread = new Worker("userbot.js");
        simState.opponentThread.onerror = gradbotError;
        simState.opponentThread.onmessage = opponentReceiveMessage;
        simState.opponentThread.postMessage({type: "start", robot: opponent.sendable()});
        opponent.thread = simState.opponentThread;
    }

    //set the timer going!
    simState.timer = setInterval(simulationUpdate, 1000/60); //Gavin changed 1000/30 to 1000/60

    //ADDED BY GAVIN 04/06/2023
    if(simState.mazeWorldLoaded == true){
        startStopwatch();
    }
    //END OF ADDED BY GAVIN 04/06/2023
}


/**
 * Handle messages from the user robot.
 * @param {*} message 
 */
function simulationReceiveMessage(message) {
    robot.getPartByName(message.data.name).receiveUser(message.data);
}


/**
 * Handle messages from the opponent robot.
 * @param {*} message 
 */
function opponentReceiveMessage(message) {
    opponent.getPartByName(message.data.name).receiveUser(message.data);
}


/**
 * Stop the simulation
 */
function simulationStop() {
    // Mark the simulation as not running
    simState.running = false;

    // clear the timer, if there is one
    if(simState.timer) {
        clearInterval(simState.timer);
    }

    // terminate the robot thread
    if(simState.robotThread) {
        simState.robotThread.terminate();
        simState.robotThread = null;
        robot.thread = null;
    }

    //terminate the opponent thread
    if(simState.opponentThread) {
        simState.opponentThread.terminate();
        simState.opponentThread = null;
        opponent.thread = null;
    }

    // remove the robot's abilityt to teleport on resume ^_^
    robot.lastUpdate = undefined;
    if(opponent) {
        opponent.lastUpdate = undefined;
    }

}



/**
 * Update one simulation frame.
 */
function simulationUpdate() {
    var bots = [robot];
    var botViews = [simView];
    if(opponent) {
        bots.push(opponent);
        botViews.push(opponentView);
    }

    for(var i=0; i < bots.length; i++) {
        bots[i].update();
    }

    //Chase new opponent collision
    // BotView collision check
    for (var i = 0; i < botViews.length; i++) {
        for (var j = i + 1; j < botViews.length; j++) {
            if (collision(botViews[i].view, botViews[j].view)) {
                    bots[i].left.setPower(0);
                    bots[i].right.setPower(0);
                    bots[j].left.setPower(0);
                    bots[j].right.setPower(0);
            }
        }
    }

    //update all the world objects
    var toVanish = [];
    for(var i=0; i < simState.worldObjects.length; i++) {
        var obj = simState.worldObjects[i];
        obj.part.update();

        //check for laser blast collisions
        if(obj.part.type == "LaserBlast") {

            //ADDED BY GAVIN 03/30/2023
            for(var k = 0; k < simState.worldObjects.length; k++){
                if(simState.worldObjects[k].part.type != "LaserBlast"){
                    if(collision(obj.view, simState.worldObjects[k].view)){
                        toVanish.push(obj.part);
                    }
                }
            }
            //END OF ADDED BY GAVIN 03/30/2023

            for(var j=0; j < botViews.length; j++) {
                if(bots[j] !== obj.part.firedBy && collision(botViews[j].view, obj.view)) {
                    
                    //!!!!!!! Sam Elfrink Addition!!!!!!!!!!!!!!!!
                    drawPlayerHUDClear();
                    if(opponentClicked == 1) {
                        drawOpponentHUDClear();
                    }
                    // !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

                    bots[j].hp--;

                    //!!!!!!! Sam Elfrink Addition!!!!!!!!!!!!!!!!
                    drawPlayerHUD();
                    if(opponentClicked == 1) {
                        drawOpponentHUD(); // Elfrink
                    }
                    // !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

                    toVanish.push(obj.part);
                }
            }
        }

        // check for wall collisions
        if(obj.part.type == "Wall") {
            for(var j=0; j < botViews.length; j++) {
                if(collision(botViews[j].view, obj.view)) {
                    //bots[j].hp--;
                    bots[j].left.setPower(0);
                    bots[j].right.setPower(0);
                }
            }
        }

       //Chase new box collision
        // keep track of boxes in contact with bot
        var boxesInContact = [];

        // check for box collisions
        if (obj.part.type === "Box") {
            for (var j = 0; j < botViews.length; j++) {
                if (collision(botViews[j].view, obj.view)) {
                    // check if box is already in contact with bot
                    if (!boxesInContact.includes(obj.part)) {
                        // add box to list of boxes in contact with bot
                        boxesInContact.push(obj.part);
                        // set box's data property to store the bot view
                        obj.part.botView = botViews[j].view;
                        
                        //ADDED BY GAVIN 04/06/2023
                        if(simState.mazeWorldLoaded){
                            var endTime = stopStopwatch();
                            window.alert("Congratulations! It took you " + endTime / 1000 + " seconds to complete the maze!");
                            document.getElementById('simGo').click();
                            endTime = 0;
                            robotStartingLocation();
                        }
                        //END OF ADDED BY GAVIN 04/06/2023
                    }
                    // move the box based on the relative position of the bot view and object view
                    if (obj.part.botView.y < obj.part.y) {
                        obj.part.y = obj.part.botView.y + 36; // move box down
                    } else if (obj.part.botView.y > obj.part.y) {
                        obj.part.y = obj.part.botView.y -36; // move box up

                    } else if (obj.part.botView.x < obj.part.x) {
                        obj.part.x = obj.part.botView.x + 36; // move box to the right
                    } else if (obj.part.botView.x > obj.part.x) {
                        obj.part.x = obj.part.botView.x - 36; // move box to the left
                    }

                } else {
                    // remove box from list of boxes in contact with bot
                    const index = boxesInContact.indexOf(obj.part);
                    if (index !== -1) {
                        boxesInContact.splice(index, 1);
                    }
                    // remove box's data property
                    delete obj.part.botView;
                }
            }
        }

        //UPDATED BY GAVIN 04/06/2023
        if(simState.pacmanWorldLoaded == true){
            if(obj.part.type == "Light") {
                for(var j=0; j < botViews.length; j++) {
                    if(collision(botViews[j].view, obj.view)) {
                        var part = obj.part;
                        deselectPart(simState);
                        var partArray = [];
                        for(var i=0; i<simState.worldObjects.length; i++) {
                            partArray[i] = simState.worldObjects[i].part;
                        }
                        simState.worldObjects.splice(partArray.indexOf(part), 1)
                        pacmanEatingSound.play();
                        simState.pacmanPoints += 10;
                        displayPacmanPoints();
                        drawSim();
                    }
                }
            }
        }
        //END OF UPDATED BY GAVIN 04/06/2023

    }
    for(var i=0; i < toVanish.length; i++) {
        toVanish[i].vanish();
    }
    drawSim();
}




/******************************************
 * Storage Functions
 ******************************************/

//World Save Funtions

function newWorld() {
   robot = new Chassis(100, 100, 0);

    //rebuild the robot views
    simView = new ChassisView(robot);
    buildView = new ChassisBuildView(robot);

    //redraw
    graphPaperFill("simbg");
    drawSim();
    drawBuild();
}

//Robot Save Functions
function saveRobot(robot) {
    localStorage.setItem("robot", JSON.stringify(robot));
}


function saveRobotFile() {
    /* !!!!! Addition By Sam Elfrink: Allows users to name their robot file !!!!!!*/
    let text;
    let robotname = prompt("Please enter your robot file name:", "Robot");
    if (robotname == null || robotname == "") {
        /* do nothing */
        return;
    } else {
        var file = new Blob([JSON.stringify(robot)]);
        var a = document.getElementById('buildDownload');
        a.href = URL.createObjectURL(file, {type: "text/plain"});
        a.download = robotname;
        a.click();
        
        URL.revokeObjectURL(a.href);
    }
}


function openRobotFile() {
    var reader = new FileReader();
    reader.onload = function() {
        addListTrue = 1; //zzz Sam Elfrink 
        loadRobot(robot, reader.result);

        //rebuild the robot views
        simView = new ChassisView(robot);
        buildView = new ChassisBuildView(robot);

        //redraw
        graphPaperFill("simbg");
        drawSim();
        drawBuild();
        addListTrue = 0; //zzz Sam Elfrink 
    };

    reader.readAsText(this.files[0]);

}

function openOpponentFile() {
    
    //ADDED BY GAVIN 04/05/2023
    if(simState.pacmanWorldLoaded == true || simState.mazeWorldLoaded == true){
        notAvailablePopup();
        return;
    }
    //END OF ADDED BY GAVIN 04/05/2023 (MOVED FUNCTION PLEASE CHANGE WHEN MERGING)

    loadRobotTrue = 1; // Sam Elfrink Addition
    
    opponentType = "custom";
    var reader = new FileReader();
    reader.onload = function() {
        opponent = new Chassis();
        loadRobotOpp(opponent, reader.result);
        //ADDED BY GAVIN 03/30/2023
        if(simState.combatWorldLoaded == false && simState.mazeWorldLoaded == false && simState.pacmanWorldLoaded == false){
        opponent.x = 700;
        opponent.y = 500;
        opponent.heading = Math.PI;
        }
        else if(simState.combatWorldLoaded == true){
            opponent.x = 740;
            opponent.y = 300;
            opponent.heading = Math.PI;
        }
        //END OF ADDED BY GAVIN 03/30/2023

        //rebuild the robot view
        opponentView = new ChassisView(opponent);

        //redraw
        graphPaperFill("simbg");
        drawSim();
    };
    console.log(this.files[0]);
    reader.readAsText(this.files[0]);

}

function loadRoverOpponent() {
    var rover = '{"x":95.31671683510646,"y":504.2753606734182,"heading":11.038709558823625,"type":"Chassis","name":"part7","worldx":400,"worldy":300,"outline":"black","fill":"silver","power":0,"thread":null,"parts":[{"x":0,"y":0,"heading":0,"type":"Light","name":"part11","worldx":400,"worldy":300,"outline":"black","fill":"red","power":0,"radius":1},{"x":6.433333333333334,"y":-0.10000000000000024,"heading":0,"type":"Laser","name":"laser","worldx":396.9999999999999,"worldy":107.00000000000003,"outline":"black","fill":"black","power":0,"charged":false,"lastUpdate":1655407045989,"chargeTime":500}],"left":{"x":-7,"y":-7,"heading":0,"type":"Motor","name":"left","worldx":190.00000000000006,"worldy":510,"outline":"black","fill":"black","power":90.49773755656109,"speed":6.16289592760181},"right":{"x":-7,"y":7,"heading":3.141592653589793,"type":"Motor","name":"right","worldx":610,"worldy":510,"outline":"black","fill":"black","power":90.49773755656109,"speed":6.16289592760181},"hp":3,"blowedUp":false,"explosionVelocities":[],"code":"function setSpeed(vx, vyaw) {\\n  const r = 0.065;    //wheel radius\\n  const l = 0.238;    //axle length\\n  var sleft;          //left speed\\n  var sright;         //right speed\\n  var lpower;         //left power\\n  var rpower;         //right power\\n  \\n  // Compute lpower and rpower\\n  sright = 1/r * vx - l/(2*r) * vyaw;\\n  sleft = 2/r * vx - sright;\\n  lpower = 100/6.8 * sleft;\\n  rpower = 100/6.8 * sright;\\n  \\n  left.setPower(lpower);\\n  right.setPower(rpower);\\n}\\n\\n// Turtle Graphics\\nconst tspeed=0.4;  //turtle rolling speed\\nconst trot=0.25;    //turtle rotation speed\\n\\nasync function forward(d) \\n{\\n  // calculate move time\\n  var t = d/tspeed;\\n  \\n  // move for the specified time\\n  setSpeed(tspeed, 0);\\n  await delay(t*1000);\\n  setSpeed(0, 0);\\n}\\n\\nasync function back(d) \\n{\\n  // calculate move time\\n  var t = d/tspeed;\\n  \\n  // move for the specified time\\n  setSpeed(-tspeed, 0);\\n  await delay(t*1000);\\n  setSpeed(0, 0);\\n}\\n\\nfunction toRadians(deg) \\n{\\n  return Math.PI * deg / 180.0;\\n}\\n\\n\\nasync function turnLeft(d) \\n{\\n  // calculate move time\\n  var t = toRadians(d) / trot;\\n  \\n  // move for the specified time\\n  setSpeed(0, -trot);\\n  await delay(t*1000);\\n  setSpeed(0, 0);\\n}\\n\\n\\nasync function turnRight(d)\\n{\\n  // calculate move time\\n  var t = toRadians(d) / trot;\\n  \\n  // move for the specified time\\n  setSpeed(0, trot);\\n  await delay(t*1000);\\n  setSpeed(0, 0);\\n}\\n\\n\\n\\n/////////////////////////////////////////////////////\\n\\nwhile(true) {\\n  await forward(10);\\n  laser.fire();\\n  await turnRight(90);\\n  laser.fire();\\n  await forward(7);\\n  laser.fire();\\n  await turnRight(90);\\n  laser.fire();\\n}","laserBattery":36}';
    opponentType = "rover";
    loadSampleOpponent(rover);

}


function loadCirclerOpponent() {
    var circler='{"x":327.5469402567586,"y":167.76097113134486,"heading":13.770062022058815,"type":"Chassis","name":"part7","worldx":400,"worldy":300,"outline":"black","fill":"silver","power":0,"thread":null,"parts":[{"x":0,"y":0,"heading":0,"type":"Light","name":"part11","worldx":400,"worldy":300,"outline":"black","fill":"red","power":0,"radius":1},{"x":6.433333333333334,"y":-0.10000000000000024,"heading":0,"type":"Laser","name":"laser","worldx":396.9999999999999,"worldy":107.00000000000003,"outline":"black","fill":"black","power":0,"charged":true,"chargeTime":500}],"left":{"x":-7,"y":-7,"heading":0,"type":"Motor","name":"left","worldx":190.00000000000006,"worldy":510,"outline":"black","fill":"black","power":59.55253896430367,"speed":4.05552790346908},"right":{"x":-7,"y":7,"heading":3.141592653589793,"type":"Motor","name":"right","worldx":610,"worldy":510,"outline":"black","fill":"black","power":53.56963298139768,"speed":3.6480920060331816},"hp":3,"blowedUp":false,"explosionVelocities":[],"code":"function setSpeed(vx, vyaw) {\\n  const r = 0.065;    //wheel radius\\n  const l = 0.238;    //axle length\\n  var sleft;          //left speed\\n  var sright;         //right speed\\n  var lpower;         //left power\\n  var rpower;         //right power\\n  \\n  // Compute lpower and rpower\\n  sright = 1/r * vx - l/(2*r) * vyaw;\\n  sleft = 2/r * vx - sright;\\n  lpower = 100/6.8 * sleft;\\n  rpower = 100/6.8 * sright;\\n  \\n  left.setPower(lpower);\\n  right.setPower(rpower);\\n}\\n\\n// Turtle Graphics\\nconst tspeed=0.4;  //turtle rolling speed\\nconst trot=0.25;    //turtle rotation speed\\n\\nasync function forward(d) \\n{\\n  // calculate move time\\n  var t = d/tspeed;\\n  \\n  // move for the specified time\\n  setSpeed(tspeed, 0);\\n  await delay(t*1000);\\n  setSpeed(0, 0);\\n}\\n\\nasync function back(d) \\n{\\n  // calculate move time\\n  var t = d/tspeed;\\n  \\n  // move for the specified time\\n  setSpeed(-tspeed, 0);\\n  await delay(t*1000);\\n  setSpeed(0, 0);\\n}\\n\\nfunction toRadians(deg) \\n{\\n  return Math.PI * deg / 180.0;\\n}\\n\\n\\nasync function turnLeft(d) \\n{\\n  // calculate move time\\n  var t = toRadians(d) / trot;\\n  \\n  // move for the specified time\\n  setSpeed(0, -trot);\\n  await delay(t*1000);\\n  setSpeed(0, 0);\\n}\\n\\n\\nasync function turnRight(d)\\n{\\n  // calculate move time\\n  var t = toRadians(d) / trot;\\n  \\n  // move for the specified time\\n  setSpeed(0, trot);\\n  await delay(t*1000);\\n  setSpeed(0, 0);\\n}\\n\\n\\n\\n/////////////////////////////////////////////////////\\nvar count = 0;\\nvar r = 3;\\nvar vx = 0.25;\\nvar dr = -0.25;\\nvar delays;\\n\\nsetSpeed(0.4, 0);\\nfor(var i=0; i<5; i++) {\\n  laser.fire();\\n  await delay(1000);\\n}\\n\\n\\nwhile(true) {\\n  delays = Math.floor(0.628 * r / vx);\\n  vyaw = vx/r;\\n  setSpeed(vx, vyaw);\\n  count = count + 1;\\n  \\n  if(count % 20 ==  0) {\\n    laser.fire();\\n  }\\n  \\n  if(count % delays == 0) {\\n    r += dr;\\n    if(r >= 3 || r <= 0.5) {\\n      dr *= -1;\\n    }\\n  }\\n  await delay(100);\\n}","laserBattery":0}';
    opponentType = "circler";
    loadSampleOpponent(circler);
}


function loadSpinnerOpponent() {
    var spinner='{"x":100,"y":100,"heading":0,"type":"Chassis","name":"part7","worldx":400,"worldy":300,"outline":"black","fill":"silver","power":0,"thread":null,"parts":[{"x":0,"y":0,"heading":0,"type":"Light","name":"part11","worldx":400,"worldy":300,"outline":"black","fill":"red","power":0,"radius":1},{"x":6.433333333333334,"y":-0.10000000000000024,"heading":0,"type":"Laser","name":"laser","worldx":396.9999999999999,"worldy":107.00000000000003,"outline":"black","fill":"black","power":0,"charged":true,"chargeTime":500}],"left":{"x":-7,"y":-7,"heading":0,"type":"Motor","name":"left","worldx":190.00000000000006,"worldy":510,"outline":"black","fill":"black","power":100,"speed":6.81},"right":{"x":-7,"y":7,"heading":3.141592653589793,"type":"Motor","name":"right","worldx":610,"worldy":510,"outline":"black","fill":"black","power":-80,"speed":-5.4479999999999995},"hp":3,"blowedUp":false,"explosionVelocities":[],"code":"function setSpeed(vx, vyaw) {\\n  const r = 0.065;    //wheel radius\\n  const l = 0.238;    //axle length\\n  var sleft;          //left speed\\n  var sright;         //right speed\\n  var lpower;         //left power\\n  var rpower;         //right power\\n  \\n  // Compute lpower and rpower\\n  sright = 1/r * vx - l/(2*r) * vyaw;\\n  sleft = 2/r * vx - sright;\\n  lpower = 100/6.8 * sleft;\\n  rpower = 100/6.8 * sright;\\n  \\n  left.setPower(lpower);\\n  right.setPower(rpower);\\n}\\n\\n// Turtle Graphics\\nconst tspeed=0.4;  //turtle rolling speed\\nconst trot=0.25;    //turtle rotation speed\\n\\nasync function forward(d) \\n{\\n  // calculate move time\\n  var t = d/tspeed;\\n  \\n  // move for the specified time\\n  setSpeed(tspeed, 0);\\n  await delay(t*1000);\\n  setSpeed(0, 0);\\n}\\n\\nasync function back(d) \\n{\\n  // calculate move time\\n  var t = d/tspeed;\\n  \\n  // move for the specified time\\n  setSpeed(-tspeed, 0);\\n  await delay(t*1000);\\n  setSpeed(0, 0);\\n}\\n\\nfunction toRadians(deg) \\n{\\n  return Math.PI * deg / 180.0;\\n}\\n\\n\\nasync function turnLeft(d) \\n{\\n  // calculate move time\\n  var t = toRadians(d) / trot;\\n  \\n  // move for the specified time\\n  setSpeed(0, -trot);\\n  await delay(t*1000);\\n  setSpeed(0, 0);\\n}\\n\\n\\nasync function turnRight(d)\\n{\\n  // calculate move time\\n  var t = toRadians(d) / trot;\\n  \\n  // move for the specified time\\n  setSpeed(0, trot);\\n  await delay(t*1000);\\n  setSpeed(0, 0);\\n}\\n\\n\\n\\n/////////////////////////////////////////////////////\\n\\nleft.setPower(100);\\nright.setPower(-80);\\n\\nwhile(true) {\\n  await delay(1000);\\n  laser.fire();\\n}","laserBattery":38}';
    opponentType = "spinner";
    loadSampleOpponent(spinner);
}

//GAVIN ADDED 04/04/2023
function loadRobotOne() {
    var test1 = '{"x":100,"y":100,"heading":0,"type":"Chassis","name":"part75","worldx":400,"worldy":300,"outline":"black","fill":"red","power":0,"thread":null,"parts":[{"x":-1,"y":-0.033333333333333395,"heading":0,"type":"Laser","name":"part78","worldx":399,"worldy":330,"outline":"black","fill":"blue","power":0,"charged":true,"chargeTime":500},{"x":0,"y":0,"heading":0,"type":"RangeSensor","name":"part79","worldx":400,"worldy":292.5,"outline":"black","fill":"black","power":0,"distance":null,"freq":10},{"x":3.4333333333333336,"y":0.06666666666666705,"heading":0,"type":"Marker","name":"part80","worldx":402,"worldy":197,"outline":"black","fill":"#00000000","power":0,"color":"black","penDrawing":false}],"left":{"x":-7,"y":-7,"heading":0,"type":"Motor","name":"left","worldx":190.00000000000006,"worldy":510,"outline":"black","fill":"black","power":0,"speed":0},"right":{"x":-7,"y":7,"heading":3.141592653589793,"type":"Motor","name":"right","worldx":610,"worldy":510,"outline":"black","fill":"black","power":0,"speed":0},"hp":3,"blowedUp":false,"explosionVelocities":[],"code":"","laserBattery":50}'
    loadPrebuiltUser(test1);
}

function loadRobotTwo() {
    var test2 = '{"x":100,"y":100,"heading":0,"type":"Chassis","name":"part31","worldx":400,"worldy":300,"outline":"black","fill":"hotpink","power":0,"thread":null,"parts":[{"x":1.1999999999999993,"y":2.2043642384652343e-16,"heading":0,"type":"RangeSensor","name":"part8","worldx":400,"worldy":256.5,"outline":"black","fill":"purple","power":0,"distance":null,"freq":10},{"x":0,"y":0,"heading":0,"type":"Light","name":"part9","worldx":400,"worldy":300,"outline":"black","fill":"lilac","power":0,"radius":1,"moveable":true},{"x":1.166666666666666,"y":2.1431318985078668e-16,"heading":0,"type":"Marker","name":"part11","worldx":400,"worldy":265,"outline":"black","fill":"#00000000","power":0,"color":"black","penDrawing":false}],"left":{"x":-7,"y":-7,"heading":0,"type":"Motor","name":"left","worldx":190.00000000000006,"worldy":510,"outline":"coral","fill":"white","power":0,"speed":0},"right":{"x":-7,"y":7,"heading":3.141592653589793,"type":"Motor","name":"right","worldx":610,"worldy":510,"outline":"black","fill":"white","power":0,"speed":0},"chassisWheelSize":"0.065","hp":3,"blowedUp":false,"explosionVelocities":[],"code":"","laserBattery":50}'
    loadPrebuiltUser(test2);
}

function loadRobotThree() {
    var test3 = '{"x":100,"y":100,"heading":0,"type":"Chassis","name":"part28","worldx":400,"worldy":300,"outline":"black","fill":"green","power":0,"thread":null,"parts":[{"x":5.9,"y":0.06666666666666751,"heading":0,"type":"LightSensor","name":"part31","worldx":402,"worldy":123,"outline":"black","fill":"brown","power":0,"intensity":0,"freq":10},{"x":0,"y":0,"heading":0,"type":"Laser","name":"part32","worldx":400,"worldy":300,"outline":"black","fill":"brown","power":0,"charged":true,"chargeTime":500},{"x":0,"y":0,"heading":0,"type":"Marker","name":"part33","worldx":400,"worldy":300,"outline":"black","fill":"#00000000","power":0,"color":"black","penDrawing":false},{"x":0,"y":0,"heading":0,"type":"Light","name":"part34","worldx":400,"worldy":300,"outline":"black","fill":"green","power":0,"radius":1,"moveable":true}],"left":{"x":-7,"y":-7,"heading":0,"type":"Motor","name":"left1","worldx":190.00000000000006,"worldy":510,"outline":"black","fill":"black","power":0,"speed":0},"right":{"x":-7,"y":7,"heading":3.141592653589793,"type":"Motor","name":"right1","worldx":610,"worldy":510,"outline":"black","fill":"black","power":0,"speed":0},"chassisWheelSize":"0.065","hp":3,"blowedUp":false,"explosionVelocities":[],"code":"","laserBattery":50}';
    loadPrebuiltUser(test3);
}

function robotStartingLocation(){
    if (simState.pacmanWorldLoaded){
        robot.heading = simState.pacmanStartHeading;
        robot.moveTo(simState.robotPacmanStartX, simState.robotPacmanStartY);
    }

    else if(simState.mazeWorldLoaded){
        robot.heading = simState.robotStartHeading;
        robot.moveTo(simState.robotMazeStartX, simState.robotMazeStartY);      //Resets Robot to coordinates 60,60
    }

    else if (simState.combatWorldLoaded){
        robot.heading = simState.robotStartHeading;
        robot.moveTo(simState.robotCombatStartX, simState.robotCombatStartY);     //Resets Robot to coordinates 60,300
    }

    else{
        robot.head = simState.robotStartHeading
        robot.moveTo(simState.robotStartX, simState.robotStartY)
    }
}

function loadPrebuiltUser(test){
    deselectPart(buildState);
    addListTrue = 1; //zzz Sam Elfrink 
    loadRobot(robot,test);
    addListTrue = 0;
    //rebuild the robot view
    simView = new ChassisView(robot);
    buildView = new ChassisBuildView(robot);
    robotStartingLocation();
    //redraw
    graphPaperFill("simbg");
    drawSim();
    drawBuild();
}
//END OF GAVIN ADDED 04/04/2023

function loadSampleOpponent(robotString) {
    //EDITED BY GAVIN 04/05/2023
    if(simState.pacmanWorldLoaded == true || simState.mazeWorldLoaded == true){
        notAvailablePopup();
        return;
    }
    opponent = new Chassis();

    loadRobotOpp(opponent, robotString);

    if(simState.combatWorldLoaded == false && simState.mazeWorldLoaded == false && simState.pacmanWorldLoaded == false){
        opponent.x = 700;
        opponent.y = 500;
        opponent.heading = Math.PI;
    }
    else if(simState.combatWorldLoaded == true){
            opponent.x = 740;
            opponent.y = 300;
            opponent.heading = Math.PI;
    }
    //END OF EDITED BY GAVIN 04/05/2023
    //rebuild the robot view
    opponentView = new ChassisView(opponent);

    //redraw
    graphPaperFill("simbg");
    drawSim();
}

//Created a new function so that opponent parts do not show up in Drop Down
function loadRobotOpp(robot, robotString) {
    loadRobotTrue = 1;

    // !!!!!!!!!!!!! Sam Elfrink Addition !!!!!!!!!!!!!
    opponentClicked = 1;
    // !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
    
    if(!robotString) robotString = localStorage.getItem("robot");
    if(!robotString) return;


    var obj = JSON.parse(robotString);

    /* grab the attributes */
    for(var attr in obj) {
        if(attr == "parts") { continue; }
        robot[attr] = obj[attr];
    }

    /* handle the motors */
    robot.left = finishPart(obj.left);
    robot.right = finishPart(obj.right);
    robot.left.parent = robot;
    robot.right.parent = robot;


    robot.parts = [];
    for(var i=0; i<obj.parts.length; i++) {
        robot.addPart(finishPart(obj.parts[i]));
        robot.parts[i].parent = robot;
    }

    //console.log("loadRobotTrue set to 0");
    loadRobotTrue = 0;
}


function loadRobot(robot, robotString) {
    
    if(!robotString) robotString = localStorage.getItem("robot");
    if(!robotString) return;


    var obj = JSON.parse(robotString);

    /* grab the attributes */
    for(var attr in obj) {
        if(attr == "parts") { continue; }
        robot[attr] = obj[attr];
    }

    /* handle the motors */

    // !!!!!!! Sam Elfrink Addition !!!!!!!!
    loadRobotTrue = 1; // prevent unwanted parts from the drop down
    //!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

    robot.left = finishPart(obj.left);
    robot.right = finishPart(obj.right);


    robot.left.parent = robot;
    robot.right.parent = robot;

    // !!!!!! Sam Elfrink Addition !!!!!!!!!
    //handle the wheel size
    wheelSize = robot.chassisWheelSize;
    // set wheelsize of the text file to the wheelsize value on the webpage
    document.getElementById("wheelSize").value = wheelSize;
    //!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

    /* handle the parts */
    
    // !!!!!! Sam Elfrink Addition !!!!!!!!!
    // Remove all elements of the drop-down list except for the first 3
    document.getElementById("partDropDown").options.length = 0;
    //console.log("loadRobotTrue set to 1");
    //loadRobotTrue = 1;
    //!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

    robot.parts = [];
    for(var i=0; i<obj.parts.length; i++) {
        robot.addPart(finishPart(obj.parts[i]));
        robot.parts[i].parent = robot;
        
        // !!!!!!!!!!!!!! Addition by Sam Elfrink !!!!!!!!!!!
        // When a robot is opened, add the part names to the list
        addList(robot.parts[i].name)
        //!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
    }
    //console.log("loadRobotTrue set to 0");
    loadRobotTrue = 0;
}


function finishPart(part) {
    var result;

    // run the part constructor
    if(part.type == "Motor") {
        result = new Motor();
    } else if(part.type == "Marker") {
        result = new Marker();
    } else if(part.type == "Light") {
        result = new Light();
    } else if(part.type == "LightSensor") {
        result = new LightSensor();
    } else if(part.type == "RangeSensor") {
        result = new RangeSensor();
    } else if(part.type == "Wall") {
        result = new Wall();
    } else if(part.type == "Box") {
        result = new Box();
    } else if(part.type == "Laser") {
        result = new Laser();
    } else {
        return undefined;
    }

    for(var attr in part) {
        if(attr == "doc") continue;
        result[attr] = part[attr];
    }

    return result;
}


function newRobot() {
    robot = new Chassis(100, 100, 0);

    // !!!!!! Sam Elfrink Addition !!!!!!!!!
    // Remove all elements of the drop-down list except for the first 3
    document.getElementById("partDropDown").options.length = 0;
    //!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

    //rebuild the robot views
    simView = new ChassisView(robot);
    buildView = new ChassisBuildView(robot);

    robotStartingLocation();    //ADDED BY GAVIN 04/05/2023

    //redraw
    graphPaperFill("simbg");
    drawSim();
    drawBuild();
}

//GAVIN'S UPDATED CODE STARTS HERE
var multiplyer = 1;

function setSpeedMult1(){
    multiplyer = 1;
}

function setSpeedMult5(){
    multiplyer = 5;
}

function setSpeedMult10(){
    multiplyer = 10;
}

function setSpeedMult25(){
    multiplyer = 25;
}
function getSpeedMult(){
    return multiplyer;
}
//GAVIN'S UPDATED CODE ENDS HERE

//Edited by GAVIN 02/22/2023
function openWorldFile() {
    var reader = new FileReader();
    reader.onload = function() {
        loadWorld(reader.result);
        //redraw
        graphPaperFill("simbg");
        drawSim();
        drawBuild();
    };
    reader.readAsText(this.files[0]);
}


function saveWorldFile() {
    let text;
    let worldname = prompt("Please enter your world file name:", "World");
    if (worldname == null || worldname == "") {
        //do nothing 
        return;
    } else {
        var file = new Blob([JSON.stringify(simState)]);
        var a = document.getElementById('worldDownload');
        a.href = URL.createObjectURL(file, {type: "text/plain"});
        a.download = worldname;
        a.click();
        
        URL.revokeObjectURL(a.href);
    }
}


//set up handlers for premade maps
document.getElementById("combatWorld").onclick = loadCombatWorld;
document.getElementById("mazeWorld").onclick = loadMazeWorld;
document.getElementById("pacmanWorld").onclick = loadPacmanWorld;

//Edited by Gavin 03/20/2023
function loadCombatWorld() {
    simState.combatWorldLoaded = true;
    simState.mazeWorldLoaded = false;
    simState.pacmanWorldLoaded = false;
    simState.worldObjects= []; //Clears world objects
    var combatWorld = '{"prefix":"sim","dragMode":0,"dragTarget":{"x":289,"y":421,"heading":0,"view":{"x":289,"y":421,"heading":0,"scale":2,"points":[{"x":-5,"y":-10},{"x":5,"y":-10},{"x":5,"y":10},{"x":-5,"y":10}],"outline":"blue","fill":"blue","polygon":[{"x":279,"y":401},{"x":299,"y":401},{"x":299,"y":441},{"x":279,"y":441}],"minx":279,"miny":401,"maxx":299,"maxy":441,"stroke":"black"},"scale":2,"subviews":[],"part":{"x":289,"y":421,"heading":0,"type":"Wall","name":"part31","worldx":289,"worldy":421,"outline":"blue","fill":"blue","power":0,"rotated":false,"moveable":false,"resizeFactor":1,"resizeFactorHeight":1,"resizeFactorWidth":1}},"lastX":289,"lastY":421,"robotStartX":100,"robotStartY":100,"robotStartHeading":0,"opponentStartX":700,"opponentStartY":500,"opponentStartHeading":3.141592653589793,"timer":null,"prevTab":"Simulate","robotThread":null,"opponentThread":null,"worldObjects":[{"x":21,"y":590,"heading":0,"view":{"x":21,"y":590,"heading":0,"scale":2,"points":[{"x":-10,"y":-5},{"x":390,"y":-5},{"x":390,"y":5},{"x":-10,"y":5}],"outline":"blue","fill":"blue","polygon":[{"x":1,"y":580},{"x":801,"y":580},{"x":801,"y":600},{"x":1,"y":600}],"minx":1,"miny":580,"maxx":801,"maxy":600,"stroke":"black"},"scale":2,"subviews":[],"part":{"x":21,"y":590,"heading":0,"type":"Wall","name":"part6","worldx":401,"worldy":590,"outline":"blue","fill":"blue","power":0,"rotated":1,"moveable":false,"resizeFactor":1,"resizeFactorHeight":39,"resizeFactorWidth":1}},{"x":791,"y":582,"heading":0,"view":{"x":791,"y":582,"heading":0,"scale":2,"points":[{"x":-5,"y":-290},{"x":5,"y":-290},{"x":5,"y":10},{"x":-5,"y":10}],"outline":"blue","fill":"blue","polygon":[{"x":781,"y":2},{"x":801,"y":2},{"x":801,"y":602},{"x":781,"y":602}],"minx":781,"miny":2,"maxx":801,"maxy":602,"stroke":"black"},"scale":2,"subviews":[],"part":{"x":791,"y":582,"heading":0,"type":"Wall","name":"part5","worldx":791,"worldy":302,"outline":"blue","fill":"blue","power":0,"rotated":false,"moveable":false,"resizeFactor":1,"resizeFactorHeight":29,"resizeFactorWidth":1}},{"x":10,"y":582,"heading":0,"view":{"x":10,"y":582,"heading":0,"scale":2,"points":[{"x":-5,"y":-300},{"x":5,"y":-300},{"x":5,"y":10},{"x":-5,"y":10}],"outline":"blue","fill":"blue","polygon":[{"x":0,"y":-18},{"x":20,"y":-18},{"x":20,"y":602},{"x":0,"y":602}],"minx":0,"miny":-18,"maxx":20,"maxy":602,"stroke":"black"},"scale":2,"subviews":[],"part":{"x":10,"y":582,"heading":0,"type":"Wall","name":"part4","worldx":10,"worldy":292,"outline":"blue","fill":"blue","power":0,"rotated":false,"moveable":false,"resizeFactor":1,"resizeFactorHeight":30,"resizeFactorWidth":1}},{"x":19,"y":9,"heading":0,"view":{"x":19,"y":9,"heading":0,"scale":2,"points":[{"x":-10,"y":-5},{"x":390,"y":-5},{"x":390,"y":5},{"x":-10,"y":5}],"outline":"blue","fill":"blue","polygon":[{"x":-1,"y":-1},{"x":799,"y":-1},{"x":799,"y":19},{"x":-1,"y":19}],"minx":-1,"miny":-1,"maxx":799,"maxy":19,"stroke":"black"},"scale":2,"subviews":[],"part":{"x":19,"y":9,"heading":0,"type":"Wall","name":"part7","worldx":399,"worldy":9,"outline":"blue","fill":"blue","power":0,"rotated":1,"moveable":false,"resizeFactor":1,"resizeFactorHeight":39,"resizeFactorWidth":1}},{"x":390,"y":40,"heading":0,"view":{"x":390,"y":40,"heading":0,"scale":2,"points":[{"x":-5,"y":-20},{"x":15,"y":-20},{"x":15,"y":10},{"x":-5,"y":10}],"outline":"blue","fill":"blue","polygon":[{"x":380,"y":0},{"x":420,"y":0},{"x":420,"y":60},{"x":380,"y":60}],"minx":380,"miny":0,"maxx":420,"maxy":60,"stroke":"black"},"scale":2,"subviews":[],"part":{"x":390,"y":40,"heading":0,"type":"Wall","name":"part8","worldx":400,"worldy":30,"outline":"blue","fill":"blue","power":0,"rotated":0,"resizeFactor":1,"resizeFactorHeight":2,"resizeFactorWidth":3}},{"x":390,"y":581,"heading":0,"view":{"x":390,"y":581,"heading":0,"scale":2,"points":[{"x":-5,"y":-20},{"x":15,"y":-20},{"x":15,"y":10},{"x":-5,"y":10}],"outline":"blue","fill":"blue","polygon":[{"x":380,"y":541},{"x":420,"y":541},{"x":420,"y":601},{"x":380,"y":601}],"minx":380,"miny":541,"maxx":420,"maxy":601,"stroke":"black"},"scale":2,"subviews":[],"part":{"x":390,"y":581,"heading":0,"type":"Wall","name":"part10","worldx":400,"worldy":571,"outline":"blue","fill":"blue","power":0,"rotated":false,"moveable":false,"resizeFactor":1,"resizeFactorHeight":2,"resizeFactorWidth":3}},{"x":141,"y":112,"heading":0,"view":{"x":141,"y":112,"heading":0,"scale":2,"points":[{"x":-10,"y":-5},{"x":20,"y":-5},{"x":20,"y":5},{"x":-10,"y":5}],"outline":"blue","fill":"blue","polygon":[{"x":121,"y":102},{"x":181,"y":102},{"x":181,"y":122},{"x":121,"y":122}],"minx":121,"miny":102,"maxx":181,"maxy":122,"stroke":"black"},"scale":2,"subviews":[],"part":{"x":141,"y":112,"heading":0,"type":"Wall","name":"part12","worldx":151,"worldy":112,"outline":"blue","fill":"blue","power":0,"rotated":1,"moveable":false,"resizeFactor":1,"resizeFactorHeight":2,"resizeFactorWidth":1}},{"x":141,"y":491,"heading":0,"view":{"x":141,"y":491,"heading":0,"scale":2,"points":[{"x":-10,"y":-5},{"x":20,"y":-5},{"x":20,"y":5},{"x":-10,"y":5}],"outline":"blue","fill":"blue","polygon":[{"x":121,"y":481},{"x":181,"y":481},{"x":181,"y":501},{"x":121,"y":501}],"minx":121,"miny":481,"maxx":181,"maxy":501,"stroke":"black"},"scale":2,"subviews":[],"part":{"x":141,"y":491,"heading":0,"type":"Wall","name":"part15","worldx":151,"worldy":491,"outline":"blue","fill":"blue","power":0,"rotated":1,"moveable":false,"resizeFactor":1,"resizeFactorHeight":2,"resizeFactorWidth":1}},{"x":639,"y":491,"heading":0,"view":{"x":639,"y":491,"heading":0,"scale":2,"points":[{"x":-10,"y":-5},{"x":20,"y":-5},{"x":20,"y":5},{"x":-10,"y":5}],"outline":"blue","fill":"blue","polygon":[{"x":619,"y":481},{"x":679,"y":481},{"x":679,"y":501},{"x":619,"y":501}],"minx":619,"miny":481,"maxx":679,"maxy":501,"stroke":"black"},"scale":2,"subviews":[],"part":{"x":639,"y":491,"heading":0,"type":"Wall","name":"part14","worldx":649,"worldy":491,"outline":"blue","fill":"blue","power":0,"rotated":1,"moveable":false,"resizeFactor":1,"resizeFactorHeight":2,"resizeFactorWidth":1}},{"x":640,"y":110,"heading":0,"view":{"x":640,"y":110,"heading":0,"scale":2,"points":[{"x":-10,"y":-5},{"x":20,"y":-5},{"x":20,"y":5},{"x":-10,"y":5}],"outline":"blue","fill":"blue","polygon":[{"x":620,"y":100},{"x":680,"y":100},{"x":680,"y":120},{"x":620,"y":120}],"minx":620,"miny":100,"maxx":680,"maxy":120,"stroke":"black"},"scale":2,"subviews":[],"part":{"x":640,"y":110,"heading":0,"type":"Wall","name":"part16","worldx":650,"worldy":110,"outline":"blue","fill":"blue","power":0,"rotated":1,"moveable":false,"resizeFactor":1,"resizeFactorHeight":2,"resizeFactorWidth":1}},{"x":671,"y":381,"heading":0,"view":{"x":671,"y":381,"heading":0,"scale":2,"points":[{"x":-5,"y":-90},{"x":5,"y":-90},{"x":5,"y":10},{"x":-5,"y":10}],"outline":"lightblue","fill":"blue","polygon":[{"x":661,"y":201},{"x":681,"y":201},{"x":681,"y":401},{"x":661,"y":401}],"minx":661,"miny":201,"maxx":681,"maxy":401,"stroke":"black"},"scale":2,"subviews":[],"part":{"x":671,"y":381,"heading":0,"type":"Wall","name":"part18","worldx":671,"worldy":301,"outline":"lightblue","fill":"blue","power":0,"rotated":false,"moveable":false,"resizeFactor":1,"resizeFactorHeight":9,"resizeFactorWidth":1}},{"x":682,"y":211,"heading":0,"view":{"x":682,"y":211,"heading":0,"scale":2,"points":[{"x":-10,"y":-5},{"x":10,"y":-5},{"x":10,"y":5},{"x":-10,"y":5}],"outline":"blue","fill":"blue","polygon":[{"x":662,"y":201},{"x":702,"y":201},{"x":702,"y":221},{"x":662,"y":221}],"minx":662,"miny":201,"maxx":702,"maxy":221,"stroke":"black"},"scale":2,"subviews":[],"part":{"x":682,"y":211,"heading":0,"type":"Wall","name":"part20","worldx":682,"worldy":211,"outline":"blue","fill":"blue","power":0,"rotated":1,"moveable":false,"resizeFactor":1,"resizeFactorHeight":1,"resizeFactorWidth":1}},{"x":682,"y":391,"heading":0,"view":{"x":682,"y":391,"heading":0,"scale":2,"points":[{"x":-10,"y":-5},{"x":10,"y":-5},{"x":10,"y":5},{"x":-10,"y":5}],"outline":"blue","fill":"blue","polygon":[{"x":662,"y":381},{"x":702,"y":381},{"x":702,"y":401},{"x":662,"y":401}],"minx":662,"miny":381,"maxx":702,"maxy":401,"stroke":"black"},"scale":2,"subviews":[],"part":{"x":682,"y":391,"heading":0,"type":"Wall","name":"part21","worldx":682,"worldy":391,"outline":"blue","fill":"blue","power":0,"rotated":1,"moveable":false,"resizeFactor":1,"resizeFactorHeight":1,"resizeFactorWidth":1}},{"x":121,"y":392,"heading":0,"view":{"x":121,"y":392,"heading":0,"scale":2,"points":[{"x":-10,"y":-5},{"x":10,"y":-5},{"x":10,"y":5},{"x":-10,"y":5}],"outline":"blue","fill":"blue","polygon":[{"x":101,"y":382},{"x":141,"y":382},{"x":141,"y":402},{"x":101,"y":402}],"minx":101,"miny":382,"maxx":141,"maxy":402,"stroke":"black"},"scale":2,"subviews":[],"part":{"x":121,"y":392,"heading":0,"type":"Wall","name":"part25","worldx":121,"worldy":392,"outline":"blue","fill":"blue","power":0,"rotated":1,"moveable":false,"resizeFactor":1,"resizeFactorHeight":1,"resizeFactorWidth":1}},{"x":131,"y":380,"heading":0,"view":{"x":131,"y":380,"heading":0,"scale":2,"points":[{"x":-5,"y":-90},{"x":5,"y":-90},{"x":5,"y":10},{"x":-5,"y":10}],"outline":"blue","fill":"blue","polygon":[{"x":121,"y":200},{"x":141,"y":200},{"x":141,"y":400},{"x":121,"y":400}],"minx":121,"miny":200,"maxx":141,"maxy":400,"stroke":"black"},"scale":2,"subviews":[],"part":{"x":131,"y":380,"heading":0,"type":"Wall","name":"part27","worldx":131,"worldy":300,"outline":"blue","fill":"blue","power":0,"rotated":false,"moveable":false,"resizeFactor":1,"resizeFactorHeight":9,"resizeFactorWidth":1}},{"x":510,"y":179,"heading":0,"view":{"x":510,"y":179,"heading":0,"scale":2,"points":[{"x":-5,"y":-10},{"x":5,"y":-10},{"x":5,"y":10},{"x":-5,"y":10}],"outline":"blue","fill":"blue","polygon":[{"x":500,"y":159},{"x":520,"y":159},{"x":520,"y":199},{"x":500,"y":199}],"minx":500,"miny":159,"maxx":520,"maxy":199,"stroke":"black"},"scale":2,"subviews":[],"part":{"x":510,"y":179,"heading":0,"type":"Wall","name":"part22","worldx":510,"worldy":179,"outline":"blue","fill":"blue","power":0,"rotated":false,"moveable":false,"resizeFactor":1,"resizeFactorHeight":1,"resizeFactorWidth":1}},{"x":511,"y":421,"heading":0,"view":{"x":511,"y":421,"heading":0,"scale":2,"points":[{"x":-5,"y":-10},{"x":5,"y":-10},{"x":5,"y":10},{"x":-5,"y":10}],"outline":"blue","fill":"blue","polygon":[{"x":501,"y":401},{"x":521,"y":401},{"x":521,"y":441},{"x":501,"y":441}],"minx":501,"miny":401,"maxx":521,"maxy":441,"stroke":"black"},"scale":2,"subviews":[],"part":{"x":511,"y":421,"heading":0,"type":"Wall","name":"part25","worldx":511,"worldy":421,"outline":"blue","fill":"blue","power":0,"rotated":false,"moveable":false,"resizeFactor":1,"resizeFactorHeight":1,"resizeFactorWidth":1}},{"x":479,"y":169,"heading":0,"view":{"x":479,"y":169,"heading":0,"scale":2,"points":[{"x":-10,"y":-5},{"x":10,"y":-5},{"x":10,"y":5},{"x":-10,"y":5}],"outline":"blue","fill":"blue","polygon":[{"x":459,"y":159},{"x":499,"y":159},{"x":499,"y":179},{"x":459,"y":179}],"minx":459,"miny":159,"maxx":499,"maxy":179,"stroke":"black"},"scale":2,"subviews":[],"part":{"x":479,"y":169,"heading":0,"type":"Wall","name":"part23","worldx":489,"worldy":169,"outline":"blue","fill":"blue","power":0,"rotated":1,"moveable":false,"resizeFactor":1,"resizeFactorHeight":2,"resizeFactorWidth":1}},{"x":479,"y":169,"heading":0,"view":{"x":479,"y":169,"heading":0,"scale":2,"points":[{"x":-10,"y":-5},{"x":20,"y":-5},{"x":20,"y":5},{"x":-10,"y":5}],"outline":"blue","fill":"blue","polygon":[{"x":459,"y":159},{"x":519,"y":159},{"x":519,"y":179},{"x":459,"y":179}],"minx":459,"miny":159,"maxx":519,"maxy":179,"stroke":"black"},"scale":2,"subviews":[],"part":{"x":479,"y":169,"heading":0,"type":"Wall","name":"part23","worldx":489,"worldy":169,"outline":"blue","fill":"blue","power":0,"rotated":1,"moveable":false,"resizeFactor":1,"resizeFactorHeight":2,"resizeFactorWidth":1}},{"x":550,"y":300,"heading":0,"view":{"x":550,"y":300,"heading":0,"scale":2,"points":[{"x":-5,"y":-10},{"x":15,"y":-10},{"x":15,"y":10},{"x":-5,"y":10}],"outline":"blue","fill":"blue","polygon":[{"x":540,"y":280},{"x":580,"y":280},{"x":580,"y":320},{"x":540,"y":320}],"minx":540,"miny":280,"maxx":580,"maxy":320,"stroke":"black"},"scale":2,"subviews":[],"part":{"x":550,"y":300,"heading":0,"type":"Wall","name":"part24","worldx":560,"worldy":300,"outline":"blue","fill":"blue","power":0,"rotated":false,"moveable":false,"resizeFactor":1,"resizeFactorHeight":1,"resizeFactorWidth":3}},{"x":480,"y":431,"heading":0,"view":{"x":480,"y":431,"heading":0,"scale":2,"points":[{"x":-10,"y":-5},{"x":10,"y":-5},{"x":10,"y":5},{"x":-10,"y":5}],"outline":"blue","fill":"blue","polygon":[{"x":460,"y":421},{"x":500,"y":421},{"x":500,"y":441},{"x":460,"y":441}],"minx":460,"miny":421,"maxx":500,"maxy":441,"stroke":"black"},"scale":2,"subviews":[],"part":{"x":480,"y":431,"heading":0,"type":"Wall","name":"part26","worldx":490,"worldy":431,"outline":"blue","fill":"blue","power":0,"rotated":1,"moveable":false,"resizeFactor":1,"resizeFactorHeight":2,"resizeFactorWidth":1}},{"x":480,"y":431,"heading":0,"view":{"x":480,"y":431,"heading":0,"scale":2,"points":[{"x":-10,"y":-5},{"x":20,"y":-5},{"x":20,"y":5},{"x":-10,"y":5}],"outline":"blue","fill":"blue","polygon":[{"x":460,"y":421},{"x":520,"y":421},{"x":520,"y":441},{"x":460,"y":441}],"minx":460,"miny":421,"maxx":520,"maxy":441,"stroke":"black"},"scale":2,"subviews":[],"part":{"x":480,"y":431,"heading":0,"type":"Wall","name":"part26","worldx":490,"worldy":431,"outline":"blue","fill":"blue","power":0,"rotated":1,"moveable":false,"resizeFactor":1,"resizeFactorHeight":2,"resizeFactorWidth":1}},{"x":230,"y":301,"heading":0,"view":{"x":230,"y":301,"heading":0,"scale":2,"points":[{"x":-5,"y":-10},{"x":10,"y":-10},{"x":10,"y":10},{"x":-5,"y":10}],"outline":"blue","fill":"blue","polygon":[{"x":220,"y":281},{"x":250,"y":281},{"x":250,"y":321},{"x":220,"y":321}],"minx":220,"miny":281,"maxx":250,"maxy":321,"stroke":"black"},"scale":2,"subviews":[],"part":{"x":230,"y":301,"heading":0,"type":"Wall","name":"part28","worldx":240,"worldy":301,"outline":"blue","fill":"blue","power":0,"rotated":false,"moveable":false,"resizeFactor":1,"resizeFactorHeight":1,"resizeFactorWidth":3}},{"x":230,"y":301,"heading":0,"view":{"x":230,"y":301,"heading":0,"scale":2,"points":[{"x":-5,"y":-10},{"x":15,"y":-10},{"x":15,"y":10},{"x":-5,"y":10}],"outline":"blue","fill":"blue","polygon":[{"x":220,"y":281},{"x":260,"y":281},{"x":260,"y":321},{"x":220,"y":321}],"minx":220,"miny":281,"maxx":260,"maxy":321,"stroke":"black"},"scale":2,"subviews":[],"part":{"x":230,"y":301,"heading":0,"type":"Wall","name":"part28","worldx":240,"worldy":301,"outline":"blue","fill":"blue","power":0,"rotated":false,"moveable":false,"resizeFactor":1,"resizeFactorHeight":1,"resizeFactorWidth":3}},{"x":291,"y":180,"heading":0,"view":{"x":291,"y":180,"heading":0,"scale":2,"points":[{"x":-5,"y":-10},{"x":5,"y":-10},{"x":5,"y":10},{"x":-5,"y":10}],"outline":"blue","fill":"blue","polygon":[{"x":281,"y":160},{"x":301,"y":160},{"x":301,"y":200},{"x":281,"y":200}],"minx":281,"miny":160,"maxx":301,"maxy":200,"stroke":"black"},"scale":2,"subviews":[],"part":{"x":291,"y":180,"heading":0,"type":"Wall","name":"part29","worldx":291,"worldy":180,"outline":"blue","fill":"blue","power":0,"rotated":false,"moveable":false,"resizeFactor":1,"resizeFactorHeight":1,"resizeFactorWidth":1}},{"x":289,"y":421,"heading":0,"view":{"x":289,"y":421,"heading":0,"scale":2,"points":[{"x":-5,"y":-10},{"x":5,"y":-10},{"x":5,"y":10},{"x":-5,"y":10}],"outline":"blue","fill":"blue","polygon":[{"x":279,"y":401},{"x":299,"y":401},{"x":299,"y":441},{"x":279,"y":441}],"minx":279,"miny":401,"maxx":299,"maxy":441,"stroke":"black"},"scale":2,"subviews":[],"part":{"x":289,"y":421,"heading":0,"type":"Wall","name":"part31","worldx":289,"worldy":421,"outline":"blue","fill":"blue","power":0,"rotated":false,"moveable":false,"resizeFactor":1,"resizeFactorHeight":1,"resizeFactorWidth":1}},{"x":121,"y":209,"heading":0,"view":{"x":121,"y":209,"heading":0,"scale":2,"points":[{"x":-10,"y":-5},{"x":10,"y":-5},{"x":10,"y":5},{"x":-10,"y":5}],"outline":"blue","fill":"blue","polygon":[{"x":101,"y":199},{"x":141,"y":199},{"x":141,"y":219},{"x":101,"y":219}],"minx":101,"miny":199,"maxx":141,"maxy":219,"stroke":"black"},"scale":2,"subviews":[],"part":{"x":121,"y":209,"heading":0,"type":"Wall","name":"part27","worldx":121,"worldy":209,"outline":"blue","fill":"blue","power":0,"rotated":1,"moveable":false,"resizeFactor":1,"resizeFactorHeight":1,"resizeFactorWidth":1}},{"x":301,"y":169,"heading":0,"view":{"x":301,"y":169,"heading":0,"scale":2,"points":[{"x":-10,"y":-5},{"x":20,"y":-5},{"x":20,"y":5},{"x":-10,"y":5}],"outline":"blue","fill":"blue","polygon":[{"x":281,"y":159},{"x":341,"y":159},{"x":341,"y":179},{"x":281,"y":179}],"minx":281,"miny":159,"maxx":341,"maxy":179,"stroke":"black"},"scale":2,"subviews":[],"part":{"x":301,"y":169,"heading":0,"type":"Wall","name":"part30","worldx":311,"worldy":169,"outline":"blue","fill":"blue","power":0,"rotated":1,"moveable":false,"resizeFactor":1,"resizeFactorHeight":2,"resizeFactorWidth":1}},{"x":299,"y":432,"heading":0,"view":{"x":299,"y":432,"heading":0,"scale":2,"points":[{"x":-10,"y":-5},{"x":20,"y":-5},{"x":20,"y":5},{"x":-10,"y":5}],"outline":"blue","fill":"blue","polygon":[{"x":279,"y":422},{"x":339,"y":422},{"x":339,"y":442},{"x":279,"y":442}],"minx":279,"miny":422,"maxx":339,"maxy":442,"stroke":"black"},"scale":2,"subviews":[],"part":{"x":299,"y":432,"heading":0,"type":"Wall","name":"part32","worldx":309,"worldy":432,"outline":"blue","fill":"blue","power":0,"rotated":1,"moveable":false,"resizeFactor":1,"resizeFactorHeight":2,"resizeFactorWidth":1}}],"editTarget":null,"editOriginalOutline":"blue","running":false,"width":0,"height":0,"rotated":false}'
    if(opponent){
        opponent.moveTo(simState.opponentCombatStartX,simState.opponentCombatStartY);
    }
    robotStartingLocation();    //GAVIN CHANGED 04/05/2023
    removePacmanPoints();       //Added By GAVIN 04/06/2023
    loadWorld(combatWorld);
}

function loadMazeWorld() {
    simState.combatWorldLoaded = false;
    simState.mazeWorldLoaded = true;
    simState.pacmanWorldLoaded = false;
    simState.worldObjects= []; //Clears world objects
    var mazeWorld = '{"prefix":"sim","dragMode":0,"dragTarget":{"x":520,"y":110,"heading":0,"view":{"x":520,"y":110,"heading":0,"scale":2,"points":[{"x":-10,"y":-5},{"x":70,"y":-5},{"x":70,"y":15},{"x":-10,"y":15}],"outline":"coral","fill":"blue","polygon":[{"x":500,"y":100},{"x":660,"y":100},{"x":660,"y":140},{"x":500,"y":140}],"minx":500,"miny":100,"maxx":660,"maxy":140,"stroke":"black"},"scale":2,"subviews":[],"part":{"x":520,"y":110,"heading":0,"type":"Wall","name":"part44","worldx":590,"worldy":120,"outline":"blue","fill":"blue","power":0,"rotated":1,"moveable":false,"resizeFactor":1,"resizeFactorHeight":8,"resizeFactorWidth":3}},"lastX":521,"lastY":125,"robotStartX":100,"robotStartY":100,"robotStartHeading":0,"opponentStartX":700,"opponentStartY":500,"opponentStartHeading":3.141592653589793,"timer":67,"prevTab":"Simulate","robotThread":null,"opponentThread":null,"worldObjects":[{"x":10,"y":580,"heading":0,"view":{"x":10,"y":580,"heading":0,"scale":2,"points":[{"x":-5,"y":-290},{"x":5,"y":-290},{"x":5,"y":10},{"x":-5,"y":10}],"outline":"blue","fill":"blue","polygon":[{"x":0,"y":0},{"x":20,"y":0},{"x":20,"y":600},{"x":0,"y":600}],"minx":0,"miny":0,"maxx":20,"maxy":600,"stroke":"black"},"scale":2,"subviews":[],"part":{"x":10,"y":580,"heading":0,"type":"Wall","name":"LeftSide","worldx":10,"worldy":300,"outline":"blue","fill":"blue","power":0,"rotated":false,"moveable":false,"moveable":false,"resizeFactor":1,"resizeFactorHeight":29,"resizeFactorWidth":1}},{"x":791,"y":575,"heading":0,"view":{"x":791,"y":575,"heading":0,"scale":2,"points":[{"x":-5,"y":-290},{"x":5,"y":-290},{"x":5,"y":10},{"x":-5,"y":10}],"outline":"lightblue","fill":"blue","polygon":[{"x":781,"y":-5},{"x":801,"y":-5},{"x":801,"y":595},{"x":781,"y":595}],"minx":781,"miny":-5,"maxx":801,"maxy":595,"stroke":"black"},"scale":2,"subviews":[],"part":{"x":791,"y":575,"heading":0,"type":"Wall","name":"part7","worldx":791,"worldy":295,"outline":"lightblue","fill":"blue","power":0,"rotated":false,"moveable":false,"moveable":false,"resizeFactor":1,"resizeFactorHeight":29,"resizeFactorWidth":1}},{"x":4,"y":591,"heading":0,"view":{"x":4,"y":591,"heading":0,"scale":2,"points":[{"x":-10,"y":-5},{"x":400,"y":-5},{"x":400,"y":5},{"x":-10,"y":5}],"outline":"blue","fill":"blue","polygon":[{"x":-16,"y":581},{"x":804,"y":581},{"x":804,"y":601},{"x":-16,"y":601}],"minx":-16,"miny":581,"maxx":804,"maxy":601,"stroke":"black"},"scale":2,"subviews":[],"part":{"x":4,"y":591,"heading":0,"type":"Wall","name":"BottomWall","worldx":394,"worldy":591,"outline":"blue","fill":"blue","power":0,"rotated":1,"moveable":false,"moveable":false,"resizeFactor":1,"resizeFactorHeight":40,"resizeFactorWidth":1}},{"x":20,"y":10,"heading":0,"view":{"x":20,"y":10,"heading":0,"scale":2,"points":[{"x":-10,"y":-5},{"x":390,"y":-5},{"x":390,"y":5},{"x":-10,"y":5}],"outline":"blue","fill":"blue","polygon":[{"x":0,"y":0},{"x":800,"y":0},{"x":800,"y":20},{"x":0,"y":20}],"minx":0,"miny":0,"maxx":800,"maxy":20,"stroke":"black"},"scale":2,"subviews":[],"part":{"x":20,"y":10,"heading":0,"type":"Wall","name":"TopWall","worldx":400,"worldy":10,"outline":"blue","fill":"blue","power":0,"rotated":1,"moveable":false,"moveable":false,"resizeFactor":1,"resizeFactorHeight":39,"resizeFactorWidth":1}},{"x":744,"y":535,"heading":0,"view":{"x":744,"y":535,"heading":0,"scale":2,"points":[{"x":-8,"y":-8},{"x":8,"y":-8},{"x":8,"y":8},{"x":-8,"y":8}],"outline":"blue","fill":"lightblue","polygon":[{"x":728,"y":519},{"x":760,"y":519},{"x":760,"y":551},{"x":728,"y":551}],"minx":728,"miny":519,"maxx":760,"maxy":551,"stroke":"black"},"scale":2,"subviews":[],"part":{"x":744,"y":535,"heading":0,"type":"Box","name":"FinalBox","worldx":744,"worldy":535,"outline":"blue","fill":"lightblue","power":0,"resizeFactor":1}},{"x":110,"y":480,"heading":0,"view":{"x":110,"y":480,"heading":0,"scale":2,"points":[{"x":-5,"y":-110},{"x":15,"y":-110},{"x":15,"y":10},{"x":-5,"y":10}],"outline":"blue","fill":"blue","polygon":[{"x":100,"y":260},{"x":140,"y":260},{"x":140,"y":500},{"x":100,"y":500}],"minx":100,"miny":260,"maxx":140,"maxy":500,"stroke":"black"},"scale":2,"subviews":[],"part":{"x":110,"y":480,"heading":0,"type":"Wall","name":"part11","worldx":120,"worldy":380,"outline":"blue","fill":"blue","power":0,"rotated":false,"moveable":false,"resizeFactor":1,"resizeFactorHeight":11,"resizeFactorWidth":3}},{"x":110,"y":160,"heading":0,"view":{"x":110,"y":160,"heading":0,"scale":2,"points":[{"x":-5,"y":-30},{"x":35,"y":-30},{"x":35,"y":10},{"x":-5,"y":10}],"outline":"lightblue","fill":"blue","polygon":[{"x":100,"y":100},{"x":180,"y":100},{"x":180,"y":180},{"x":100,"y":180}],"minx":100,"miny":100,"maxx":180,"maxy":180,"stroke":"black"},"scale":2,"subviews":[],"part":{"x":110,"y":160,"heading":0,"type":"Wall","name":"part10","worldx":140,"worldy":140,"outline":"lightblue","fill":"blue","power":0,"rotated":false,"moveable":false,"resizeFactor":1,"resizeFactorHeight":3,"resizeFactorWidth":7}},{"x":120,"y":269,"heading":0,"view":{"x":120,"y":269,"heading":0,"scale":2,"points":[{"x":-10,"y":-5},{"x":70,"y":-5},{"x":70,"y":15},{"x":-10,"y":15}],"outline":"blue","fill":"blue","polygon":[{"x":100,"y":259},{"x":260,"y":259},{"x":260,"y":299},{"x":100,"y":299}],"minx":100,"miny":259,"maxx":260,"maxy":299,"stroke":"black"},"scale":2,"subviews":[],"part":{"x":120,"y":269,"heading":0,"type":"Wall","name":"part13","worldx":180,"worldy":279,"outline":"blue","fill":"blue","power":0,"rotated":1,"moveable":false,"resizeFactor":1,"resizeFactorHeight":7,"resizeFactorWidth":3}},{"x":269,"y":160,"heading":0,"view":{"x":269,"y":160,"heading":0,"scale":2,"points":[{"x":-5,"y":-70},{"x":15,"y":-70},{"x":15,"y":10},{"x":-5,"y":10}],"outline":"blue","fill":"blue","polygon":[{"x":259,"y":20},{"x":299,"y":20},{"x":299,"y":180},{"x":259,"y":180}],"minx":259,"miny":20,"maxx":299,"maxy":180,"stroke":"black"},"scale":2,"subviews":[],"part":{"x":269,"y":160,"heading":0,"type":"Wall","name":"part14","worldx":279,"worldy":100,"outline":"blue","fill":"blue","power":0,"rotated":false,"moveable":false,"resizeFactor":1,"resizeFactorHeight":7,"resizeFactorWidth":3}},{"x":232,"y":479,"heading":0,"view":{"x":232,"y":479,"heading":0,"scale":2,"points":[{"x":-5,"y":-30},{"x":15,"y":-30},{"x":15,"y":10},{"x":-5,"y":10}],"outline":"blue","fill":"blue","polygon":[{"x":222,"y":419},{"x":262,"y":419},{"x":262,"y":499},{"x":222,"y":499}],"minx":222,"miny":419,"maxx":262,"maxy":499,"stroke":"black"},"scale":2,"subviews":[],"part":{"x":232,"y":479,"heading":0,"type":"Wall","name":"part16","worldx":242,"worldy":459,"outline":"blue","fill":"blue","power":0,"rotated":false,"moveable":false,"resizeFactor":1,"resizeFactorHeight":3,"resizeFactorWidth":3}},{"x":140,"y":390,"heading":0,"view":{"x":140,"y":390,"heading":0,"scale":2,"points":[{"x":-10,"y":-5},{"x":100,"y":-5},{"x":100,"y":15},{"x":-10,"y":15}],"outline":"blue","fill":"blue","polygon":[{"x":120,"y":380},{"x":340,"y":380},{"x":340,"y":420},{"x":120,"y":420}],"minx":120,"miny":380,"maxx":340,"maxy":420,"stroke":"black"},"scale":2,"subviews":[],"part":{"x":140,"y":390,"heading":0,"type":"Wall","name":"part15","worldx":230,"worldy":400,"outline":"blue","fill":"blue","power":0,"rotated":1,"moveable":false,"resizeFactor":1,"resizeFactorHeight":10,"resizeFactorWidth":3}},{"x":350,"y":400,"heading":0,"view":{"x":350,"y":400,"heading":0,"scale":2,"points":[{"x":-5,"y":-70},{"x":15,"y":-70},{"x":15,"y":10},{"x":-5,"y":10}],"outline":"blue","fill":"blue","polygon":[{"x":340,"y":260},{"x":380,"y":260},{"x":380,"y":420},{"x":340,"y":420}],"minx":340,"miny":260,"maxx":380,"maxy":420,"stroke":"black"},"scale":2,"subviews":[],"part":{"x":350,"y":400,"heading":0,"type":"Wall","name":"part17","worldx":360,"worldy":340,"outline":"blue","fill":"blue","power":0,"rotated":false,"moveable":false,"resizeFactor":1,"resizeFactorHeight":7,"resizeFactorWidth":3}},{"x":369,"y":280,"heading":0,"view":{"x":369,"y":280,"heading":0,"scale":2,"points":[{"x":-5,"y":-10},{"x":25,"y":-10},{"x":25,"y":10},{"x":-5,"y":10}],"outline":"blue","fill":"blue","polygon":[{"x":359,"y":260},{"x":419,"y":260},{"x":419,"y":300},{"x":359,"y":300}],"minx":359,"miny":260,"maxx":419,"maxy":300,"stroke":"black"},"scale":2,"subviews":[],"part":{"x":369,"y":280,"heading":0,"type":"Wall","name":"part18","worldx":389,"worldy":280,"outline":"blue","fill":"blue","power":0,"rotated":false,"moveable":false,"resizeFactor":1,"resizeFactorHeight":1,"resizeFactorWidth":5}},{"x":389,"y":240,"heading":0,"view":{"x":389,"y":240,"heading":0,"scale":2,"points":[{"x":-5,"y":-70},{"x":15,"y":-70},{"x":15,"y":10},{"x":-5,"y":10}],"outline":"blue","fill":"blue","polygon":[{"x":379,"y":100},{"x":419,"y":100},{"x":419,"y":260},{"x":379,"y":260}],"minx":379,"miny":100,"maxx":419,"maxy":260,"stroke":"black"},"scale":2,"subviews":[],"part":{"x":389,"y":240,"heading":0,"type":"Wall","name":"part19","worldx":399,"worldy":180,"outline":"blue","fill":"blue","power":0,"rotated":false,"moveable":false,"resizeFactor":1,"resizeFactorHeight":7,"resizeFactorWidth":3}},{"x":349,"y":561,"heading":0,"view":{"x":349,"y":561,"heading":0,"scale":2,"points":[{"x":-5,"y":-30},{"x":15,"y":-30},{"x":15,"y":10},{"x":-5,"y":10}],"outline":"blue","fill":"blue","polygon":[{"x":339,"y":501},{"x":379,"y":501},{"x":379,"y":581},{"x":339,"y":581}],"minx":339,"miny":501,"maxx":379,"maxy":581,"stroke":"black"},"scale":2,"subviews":[],"part":{"x":349,"y":561,"heading":0,"type":"Wall","name":"part41","worldx":359,"worldy":541,"outline":"blue","fill":"blue","power":0,"rotated":false,"moveable":false,"resizeFactor":1,"resizeFactorHeight":3,"resizeFactorWidth":3}},{"x":480,"y":471,"heading":0,"view":{"x":480,"y":471,"heading":0,"scale":2,"points":[{"x":-10,"y":-5},{"x":110,"y":-5},{"x":110,"y":15},{"x":-10,"y":15}],"outline":"lightblue","fill":"blue","polygon":[{"x":460,"y":461},{"x":700,"y":461},{"x":700,"y":501},{"x":460,"y":501}],"minx":460,"miny":461,"maxx":700,"maxy":501,"stroke":"black"},"scale":2,"subviews":[],"part":{"x":480,"y":471,"heading":0,"type":"Wall","name":"part35","worldx":580,"worldy":481,"outline":"lightblue","fill":"blue","power":0,"rotated":1,"moveable":false,"resizeFactor":1,"resizeFactorHeight":11,"resizeFactorWidth":3}},{"x":669,"y":570,"heading":0,"view":{"x":669,"y":570,"heading":0,"scale":2,"points":[{"x":-5,"y":-40},{"x":15,"y":-40},{"x":15,"y":10},{"x":-5,"y":10}],"outline":"blue","fill":"blue","polygon":[{"x":659,"y":490},{"x":699,"y":490},{"x":699,"y":590},{"x":659,"y":590}],"minx":659,"miny":490,"maxx":699,"maxy":590,"stroke":"black"},"scale":2,"subviews":[],"part":{"x":669,"y":570,"heading":0,"type":"Wall","name":"part36","worldx":679,"worldy":540,"outline":"blue","fill":"blue","power":0,"rotated":false,"moveable":false,"resizeFactor":1,"resizeFactorHeight":4,"resizeFactorWidth":3}},{"x":469,"y":480,"heading":0,"view":{"x":469,"y":480,"heading":0,"scale":2,"points":[{"x":-5,"y":-50},{"x":15,"y":-50},{"x":15,"y":10},{"x":-5,"y":10}],"outline":"blue","fill":"blue","polygon":[{"x":459,"y":380},{"x":499,"y":380},{"x":499,"y":500},{"x":459,"y":500}],"minx":459,"miny":380,"maxx":499,"maxy":500,"stroke":"black"},"scale":2,"subviews":[],"part":{"x":469,"y":480,"heading":0,"type":"Wall","name":"part37","worldx":479,"worldy":440,"outline":"blue","fill":"blue","power":0,"rotated":false,"moveable":false,"resizeFactor":1,"resizeFactorHeight":5,"resizeFactorWidth":3}},{"x":519,"y":230,"heading":0,"view":{"x":519,"y":230,"heading":0,"scale":2,"points":[{"x":-10,"y":-5},{"x":90,"y":-5},{"x":90,"y":15},{"x":-10,"y":15}],"outline":"blue","fill":"blue","polygon":[{"x":499,"y":220},{"x":699,"y":220},{"x":699,"y":260},{"x":499,"y":260}],"minx":499,"miny":220,"maxx":699,"maxy":260,"stroke":"black"},"scale":2,"subviews":[],"part":{"x":519,"y":230,"heading":0,"type":"Wall","name":"part38","worldx":599,"worldy":240,"outline":"blue","fill":"blue","power":0,"rotated":1,"moveable":false,"resizeFactor":1,"resizeFactorHeight":9,"resizeFactorWidth":3}},{"x":720,"y":350,"heading":0,"view":{"x":720,"y":350,"heading":0,"scale":2,"points":[{"x":-10,"y":-5},{"x":40,"y":-5},{"x":40,"y":15},{"x":-10,"y":15}],"outline":"blue","fill":"blue","polygon":[{"x":700,"y":340},{"x":800,"y":340},{"x":800,"y":380},{"x":700,"y":380}],"minx":700,"miny":340,"maxx":800,"maxy":380,"stroke":"black"},"scale":2,"subviews":[],"part":{"x":720,"y":350,"heading":0,"type":"Wall","name":"part39","worldx":750,"worldy":360,"outline":"blue","fill":"blue","power":0,"rotated":1,"moveable":false,"resizeFactor":1,"resizeFactorHeight":4,"resizeFactorWidth":3}},{"x":589,"y":442,"heading":0,"view":{"x":589,"y":442,"heading":0,"scale":2,"points":[{"x":-5,"y":-100},{"x":15,"y":-100},{"x":15,"y":10},{"x":-5,"y":10}],"outline":"blue","fill":"blue","polygon":[{"x":579,"y":242},{"x":619,"y":242},{"x":619,"y":462},{"x":579,"y":462}],"minx":579,"miny":242,"maxx":619,"maxy":462,"stroke":"black"},"scale":2,"subviews":[],"part":{"x":589,"y":442,"heading":0,"type":"Wall","name":"part43","worldx":599,"worldy":352,"outline":"blue","fill":"blue","power":0,"rotated":false,"moveable":false,"resizeFactor":1,"resizeFactorHeight":10,"resizeFactorWidth":3}},{"x":670,"y":240,"heading":0,"view":{"x":670,"y":240,"heading":0,"scale":2,"points":[{"x":-5,"y":-70},{"x":15,"y":-70},{"x":15,"y":10},{"x":-5,"y":10}],"outline":"blue","fill":"blue","polygon":[{"x":660,"y":100},{"x":700,"y":100},{"x":700,"y":260},{"x":660,"y":260}],"minx":660,"miny":100,"maxx":700,"maxy":260,"stroke":"black"},"scale":2,"subviews":[],"part":{"x":670,"y":240,"heading":0,"type":"Wall","name":"part42","worldx":680,"worldy":180,"outline":"blue","fill":"blue","power":0,"rotated":false,"moveable":false,"resizeFactor":1,"resizeFactorHeight":7,"resizeFactorWidth":3}},{"x":520,"y":110,"heading":0,"view":{"x":520,"y":110,"heading":0,"scale":2,"points":[{"x":-10,"y":-5},{"x":80,"y":-5},{"x":80,"y":15},{"x":-10,"y":15}],"outline":"blue","fill":"blue","polygon":[{"x":500,"y":100},{"x":680,"y":100},{"x":680,"y":140},{"x":500,"y":140}],"minx":500,"miny":100,"maxx":680,"maxy":140,"stroke":"black"},"scale":2,"subviews":[],"part":{"x":520,"y":110,"heading":0,"type":"Wall","name":"part44","worldx":590,"worldy":120,"outline":"blue","fill":"blue","power":0,"rotated":1,"moveable":false,"resizeFactor":1,"resizeFactorHeight":8,"resizeFactorWidth":3}}],"editTarget":null,"editOriginalOutline":"lightblue","running":false,"width":800,"height":600,"rotated":false}'
    document.getElementById("simRemoveOpponent").click(); //GAVIN ADDED 04/03/2023
    robotStartingLocation();    //GAVIN CHANGED 04/05/2023
    removePacmanPoints();       //Added By GAVIN 04/06/2023
    loadWorld(mazeWorld);

}
//LOADPACMAN UPDATED 03/22/2023 GAVIN
function loadPacmanWorld() {
    simState.combatWorldLoaded = false;
    simState.mazeWorldLoaded = false;
    simState.pacmanWorldLoaded = true;
    simState.worldObjects= []; //Clears world objects
    var pacmanWorld = '{"prefix":"sim","dragMode":0,"dragTarget":{"x":399,"y":287,"heading":0,"view":{"x":399,"y":287,"heading":0,"scale":2,"points":[{"x":0,"y":0}],"outline":"black","fill":"yellow","polygon":[{"x":399,"y":287}],"minx":393,"miny":281,"maxx":405,"maxy":293,"radius":3},"scale":2,"subviews":[],"part":{"x":399,"y":287,"heading":0,"type":"Light","name":"part333","worldx":399,"worldy":287,"outline":"black","fill":"yellow","power":0,"radius":3,"moveable":true}},"lastX":399,"lastY":287,"robotStartX":100,"robotStartY":100,"robotMazeStartX":60,"robotMazeStartY":60,"robotCombatStartX":60,"robotCombatStartY":300,"robotPacmanStartX":400,"robotPacmanStartY":300,"robotStartHeading":0,"pacmanStartHeading":4.71239,"opponentStartX":700,"opponentStartY":500,"opponentStartHeading":3.141592653589793,"timer":null,"prevTab":"Simulate","robotThread":null,"opponentThread":null,"worldObjects":[{"x":11,"y":580,"heading":0,"view":{"x":11,"y":580,"heading":0,"scale":2,"points":[{"x":-5,"y":-120},{"x":5,"y":-120},{"x":5,"y":10},{"x":-5,"y":10}],"outline":"blue","fill":"blue","polygon":[{"x":1,"y":340},{"x":21,"y":340},{"x":21,"y":600},{"x":1,"y":600}],"minx":1,"miny":340,"maxx":21,"maxy":600,"stroke":"black"},"scale":2,"subviews":[],"part":{"x":11,"y":580,"heading":0,"type":"Wall","name":"part15","worldx":11,"worldy":470,"outline":"blue","fill":"blue","power":0,"rotated":false,"moveable":false,"resizeFactor":1,"resizeFactorHeight":12,"resizeFactorWidth":1}},{"x":10,"y":241,"heading":0,"view":{"x":10,"y":241,"heading":0,"scale":2,"points":[{"x":-5,"y":-120},{"x":5,"y":-120},{"x":5,"y":10},{"x":-5,"y":10}],"outline":"blue","fill":"blue","polygon":[{"x":0,"y":1},{"x":20,"y":1},{"x":20,"y":261},{"x":0,"y":261}],"minx":0,"miny":1,"maxx":20,"maxy":261,"stroke":"black"},"scale":2,"subviews":[],"part":{"x":10,"y":241,"heading":0,"type":"Wall","name":"part16","worldx":10,"worldy":131,"outline":"blue","fill":"blue","power":0,"rotated":false,"moveable":false,"resizeFactor":1,"resizeFactorHeight":12,"resizeFactorWidth":1}},{"x":792,"y":239,"heading":0,"view":{"x":792,"y":239,"heading":0,"scale":2,"points":[{"x":-5,"y":-120},{"x":5,"y":-120},{"x":5,"y":10},{"x":-5,"y":10}],"outline":"blue","fill":"blue","polygon":[{"x":782,"y":-1},{"x":802,"y":-1},{"x":802,"y":259},{"x":782,"y":259}],"minx":782,"miny":-1,"maxx":802,"maxy":259,"stroke":"black"},"scale":2,"subviews":[],"part":{"x":792,"y":239,"heading":0,"type":"Wall","name":"part17","worldx":792,"worldy":129,"outline":"blue","fill":"blue","power":0,"rotated":false,"moveable":false,"resizeFactor":1,"resizeFactorHeight":12,"resizeFactorWidth":1}},{"x":790,"y":581,"heading":0,"view":{"x":790,"y":581,"heading":0,"scale":2,"points":[{"x":-5,"y":-120},{"x":5,"y":-120},{"x":5,"y":10},{"x":-5,"y":10}],"outline":"blue","fill":"blue","polygon":[{"x":780,"y":341},{"x":800,"y":341},{"x":800,"y":601},{"x":780,"y":601}],"minx":780,"miny":341,"maxx":800,"maxy":601,"stroke":"black"},"scale":2,"subviews":[],"part":{"x":790,"y":581,"heading":0,"type":"Wall","name":"part18","worldx":790,"worldy":471,"outline":"blue","fill":"blue","power":0,"rotated":false,"moveable":false,"resizeFactor":1,"resizeFactorHeight":12,"resizeFactorWidth":1}},{"x":20,"y":590,"heading":0,"view":{"x":20,"y":590,"heading":0,"scale":2,"points":[{"x":-10,"y":-5},{"x":390,"y":-5},{"x":390,"y":5},{"x":-10,"y":5}],"outline":"blue","fill":"blue","polygon":[{"x":0,"y":580},{"x":800,"y":580},{"x":800,"y":600},{"x":0,"y":600}],"minx":0,"miny":580,"maxx":800,"maxy":600,"stroke":"black"},"scale":2,"subviews":[],"part":{"x":20,"y":590,"heading":0,"type":"Wall","name":"part20","worldx":400,"worldy":590,"outline":"blue","fill":"blue","power":0,"rotated":1,"moveable":false,"resizeFactor":1,"resizeFactorHeight":39,"resizeFactorWidth":1}},{"x":18,"y":9,"heading":0,"view":{"x":18,"y":9,"heading":0,"scale":2,"points":[{"x":-10,"y":-5},{"x":390,"y":-5},{"x":390,"y":5},{"x":-10,"y":5}],"outline":"blue","fill":"blue","polygon":[{"x":-2,"y":-1},{"x":798,"y":-1},{"x":798,"y":19},{"x":-2,"y":19}],"minx":-2,"miny":-1,"maxx":798,"maxy":19,"stroke":"black"},"scale":2,"subviews":[],"part":{"x":18,"y":9,"heading":0,"type":"Wall","name":"part19","worldx":398,"worldy":9,"outline":"blue","fill":"blue","power":0,"rotated":1,"moveable":false,"resizeFactor":1,"resizeFactorHeight":39,"resizeFactorWidth":1}},{"x":500,"y":91,"heading":0,"view":{"x":500,"y":91,"heading":0,"scale":2,"points":[{"x":-10,"y":-5},{"x":50,"y":-5},{"x":50,"y":15},{"x":-10,"y":15}],"outline":"blue","fill":"blue","polygon":[{"x":480,"y":81},{"x":600,"y":81},{"x":600,"y":121},{"x":480,"y":121}],"minx":480,"miny":81,"maxx":600,"maxy":121,"stroke":"black"},"scale":2,"subviews":[],"part":{"x":500,"y":91,"heading":0,"type":"Wall","name":"part27","worldx":540,"worldy":101,"outline":"blue","fill":"blue","power":0,"rotated":1,"moveable":false,"resizeFactor":1,"resizeFactorHeight":5,"resizeFactorWidth":3}},{"x":390,"y":99,"heading":0,"view":{"x":390,"y":99,"heading":0,"scale":2,"points":[{"x":-5,"y":-50},{"x":15,"y":-50},{"x":15,"y":10},{"x":-5,"y":10}],"outline":"blue","fill":"blue","polygon":[{"x":380,"y":-1},{"x":420,"y":-1},{"x":420,"y":119},{"x":380,"y":119}],"minx":380,"miny":-1,"maxx":420,"maxy":119,"stroke":"black"},"scale":2,"subviews":[],"part":{"x":390,"y":99,"heading":0,"type":"Wall","name":"part24","worldx":400,"worldy":59,"outline":"blue","fill":"blue","power":0,"rotated":false,"moveable":false,"resizeFactor":1,"resizeFactorHeight":5,"resizeFactorWidth":3}},{"x":681,"y":90,"heading":0,"view":{"x":681,"y":90,"heading":0,"scale":2,"points":[{"x":-10,"y":-5},{"x":20,"y":-5},{"x":20,"y":15},{"x":-10,"y":15}],"outline":"blue","fill":"blue","polygon":[{"x":661,"y":80},{"x":721,"y":80},{"x":721,"y":120},{"x":661,"y":120}],"minx":661,"miny":80,"maxx":721,"maxy":120,"stroke":"black"},"scale":2,"subviews":[],"part":{"x":681,"y":90,"heading":0,"type":"Wall","name":"part25","worldx":691,"worldy":100,"outline":"blue","fill":"blue","power":0,"rotated":1,"moveable":false,"resizeFactor":1,"resizeFactorHeight":2,"resizeFactorWidth":3}},{"x":681,"y":191,"heading":0,"view":{"x":681,"y":191,"heading":0,"scale":2,"points":[{"x":-10,"y":-5},{"x":20,"y":-5},{"x":20,"y":5},{"x":-10,"y":5}],"outline":"blue","fill":"blue","polygon":[{"x":661,"y":181},{"x":721,"y":181},{"x":721,"y":201},{"x":661,"y":201}],"minx":661,"miny":181,"maxx":721,"maxy":201,"stroke":"black"},"scale":2,"subviews":[],"part":{"x":681,"y":191,"heading":0,"type":"Wall","name":"part26","worldx":691,"worldy":191,"outline":"blue","fill":"blue","power":0,"rotated":1,"moveable":false,"resizeFactor":1,"resizeFactorHeight":2,"resizeFactorWidth":1}},{"x":99,"y":90,"heading":0,"view":{"x":99,"y":90,"heading":0,"scale":2,"points":[{"x":-10,"y":-5},{"x":20,"y":-5},{"x":20,"y":15},{"x":-10,"y":15}],"outline":"blue","fill":"blue","polygon":[{"x":79,"y":80},{"x":139,"y":80},{"x":139,"y":120},{"x":79,"y":120}],"minx":79,"miny":80,"maxx":139,"maxy":120,"stroke":"black"},"scale":2,"subviews":[],"part":{"x":99,"y":90,"heading":0,"type":"Wall","name":"part28","worldx":109,"worldy":100,"outline":"blue","fill":"blue","power":0,"rotated":1,"moveable":false,"resizeFactor":1,"resizeFactorHeight":2,"resizeFactorWidth":3}},{"x":219,"y":90,"heading":0,"view":{"x":219,"y":90,"heading":0,"scale":2,"points":[{"x":-10,"y":-5},{"x":50,"y":-5},{"x":50,"y":15},{"x":-10,"y":15}],"outline":"blue","fill":"blue","polygon":[{"x":199,"y":80},{"x":319,"y":80},{"x":319,"y":120},{"x":199,"y":120}],"minx":199,"miny":80,"maxx":319,"maxy":120,"stroke":"black"},"scale":2,"subviews":[],"part":{"x":219,"y":90,"heading":0,"type":"Wall","name":"part39","worldx":259,"worldy":100,"outline":"blue","fill":"blue","power":0,"rotated":1,"moveable":false,"resizeFactor":1,"resizeFactorHeight":5,"resizeFactorWidth":3}},{"x":99,"y":191,"heading":0,"view":{"x":99,"y":191,"heading":0,"scale":2,"points":[{"x":-10,"y":-5},{"x":20,"y":-5},{"x":20,"y":5},{"x":-10,"y":5}],"outline":"blue","fill":"blue","polygon":[{"x":79,"y":181},{"x":139,"y":181},{"x":139,"y":201},{"x":79,"y":201}],"minx":79,"miny":181,"maxx":139,"maxy":201,"stroke":"black"},"scale":2,"subviews":[],"part":{"x":99,"y":191,"heading":0,"type":"Wall","name":"part30","worldx":109,"worldy":191,"outline":"blue","fill":"blue","power":0,"rotated":1,"moveable":false,"resizeFactor":1,"resizeFactorHeight":2,"resizeFactorWidth":1}},{"x":320,"y":191,"heading":0,"view":{"x":320,"y":191,"heading":0,"scale":2,"points":[{"x":-10,"y":-5},{"x":90,"y":-5},{"x":90,"y":5},{"x":-10,"y":5}],"outline":"blue","fill":"blue","polygon":[{"x":300,"y":181},{"x":500,"y":181},{"x":500,"y":201},{"x":300,"y":201}],"minx":300,"miny":181,"maxx":500,"maxy":201,"stroke":"black"},"scale":2,"subviews":[],"part":{"x":320,"y":191,"heading":0,"type":"Wall","name":"part32","worldx":400,"worldy":191,"outline":"blue","fill":"blue","power":0,"rotated":1,"moveable":false,"resizeFactor":1,"resizeFactorHeight":9,"resizeFactorWidth":1}},{"x":18,"y":269,"heading":0,"view":{"x":18,"y":269,"heading":0,"scale":2,"points":[{"x":-10,"y":-5},{"x":60,"y":-5},{"x":60,"y":5},{"x":-10,"y":5}],"outline":"blue","fill":"blue","polygon":[{"x":-2,"y":259},{"x":138,"y":259},{"x":138,"y":279},{"x":-2,"y":279}],"minx":-2,"miny":259,"maxx":138,"maxy":279,"stroke":"black"},"scale":2,"subviews":[],"part":{"x":18,"y":269,"heading":0,"type":"Wall","name":"part45","worldx":68,"worldy":269,"outline":"blue","fill":"blue","power":0,"rotated":1,"moveable":false,"resizeFactor":1,"resizeFactorHeight":6,"resizeFactorWidth":1}},{"x":19,"y":349,"heading":0,"view":{"x":19,"y":349,"heading":0,"scale":2,"points":[{"x":-10,"y":-5},{"x":60,"y":-5},{"x":60,"y":5},{"x":-10,"y":5}],"outline":"blue","fill":"blue","polygon":[{"x":-1,"y":339},{"x":139,"y":339},{"x":139,"y":359},{"x":-1,"y":359}],"minx":-1,"miny":339,"maxx":139,"maxy":359,"stroke":"black"},"scale":2,"subviews":[],"part":{"x":19,"y":349,"heading":0,"type":"Wall","name":"part34","worldx":69,"worldy":349,"outline":"blue","fill":"blue","power":0,"rotated":1,"moveable":false,"resizeFactor":1,"resizeFactorHeight":6,"resizeFactorWidth":1}},{"x":100,"y":509,"heading":0,"view":{"x":100,"y":509,"heading":0,"scale":2,"points":[{"x":-10,"y":-5},{"x":110,"y":-5},{"x":110,"y":5},{"x":-10,"y":5}],"outline":"blue","fill":"blue","polygon":[{"x":80,"y":499},{"x":320,"y":499},{"x":320,"y":519},{"x":80,"y":519}],"minx":80,"miny":499,"maxx":320,"maxy":519,"stroke":"black"},"scale":2,"subviews":[],"part":{"x":100,"y":509,"heading":0,"type":"Wall","name":"part46","worldx":200,"worldy":509,"outline":"blue","fill":"blue","power":0,"rotated":1,"moveable":false,"resizeFactor":1,"resizeFactorHeight":11,"resizeFactorWidth":1}},{"x":390,"y":500,"heading":0,"view":{"x":390,"y":500,"heading":0,"scale":2,"points":[{"x":-5,"y":-30},{"x":15,"y":-30},{"x":15,"y":10},{"x":-5,"y":10}],"outline":"blue","fill":"blue","polygon":[{"x":380,"y":440},{"x":420,"y":440},{"x":420,"y":520},{"x":380,"y":520}],"minx":380,"miny":440,"maxx":420,"maxy":520,"stroke":"black"},"scale":2,"subviews":[],"part":{"x":390,"y":500,"heading":0,"type":"Wall","name":"part38","worldx":400,"worldy":480,"outline":"blue","fill":"blue","power":0,"rotated":0,"moveable":false,"resizeFactor":1,"resizeFactorHeight":3,"resizeFactorWidth":3}},{"x":99,"y":431,"heading":0,"view":{"x":99,"y":431,"heading":0,"scale":2,"points":[{"x":-10,"y":-5},{"x":70,"y":-5},{"x":70,"y":5},{"x":-10,"y":5}],"outline":"blue","fill":"blue","polygon":[{"x":79,"y":421},{"x":239,"y":421},{"x":239,"y":441},{"x":79,"y":441}],"minx":79,"miny":421,"maxx":239,"maxy":441,"stroke":"black"},"scale":2,"subviews":[],"part":{"x":99,"y":431,"heading":0,"type":"Wall","name":"part31","worldx":159,"worldy":431,"outline":"blue","fill":"blue","power":0,"rotated":1,"moveable":false,"resizeFactor":1,"resizeFactorHeight":7,"resizeFactorWidth":1}},{"x":210,"y":420,"heading":0,"view":{"x":210,"y":420,"heading":0,"scale":2,"points":[{"x":-5,"y":-40},{"x":15,"y":-40},{"x":15,"y":10},{"x":-5,"y":10}],"outline":"blue","fill":"blue","polygon":[{"x":200,"y":340},{"x":240,"y":340},{"x":240,"y":440},{"x":200,"y":440}],"minx":200,"miny":340,"maxx":240,"maxy":440,"stroke":"black"},"scale":2,"subviews":[],"part":{"x":210,"y":420,"heading":0,"type":"Wall","name":"part33","worldx":220,"worldy":390,"outline":"blue","fill":"blue","power":0,"rotated":false,"moveable":false,"resizeFactor":1,"resizeFactorHeight":4,"resizeFactorWidth":3}},{"x":680,"y":269,"heading":0,"view":{"x":680,"y":269,"heading":0,"scale":2,"points":[{"x":-10,"y":-5},{"x":60,"y":-5},{"x":60,"y":5},{"x":-10,"y":5}],"outline":"blue","fill":"blue","polygon":[{"x":660,"y":259},{"x":800,"y":259},{"x":800,"y":279},{"x":660,"y":279}],"minx":660,"miny":259,"maxx":800,"maxy":279,"stroke":"black"},"scale":2,"subviews":[],"part":{"x":680,"y":269,"heading":0,"type":"Wall","name":"part42","worldx":730,"worldy":269,"outline":"blue","fill":"blue","power":0,"rotated":1,"moveable":false,"resizeFactor":1,"resizeFactorHeight":6,"resizeFactorWidth":1}},{"x":680,"y":349,"heading":0,"view":{"x":680,"y":349,"heading":0,"scale":2,"points":[{"x":-10,"y":-5},{"x":60,"y":-5},{"x":60,"y":5},{"x":-10,"y":5}],"outline":"blue","fill":"blue","polygon":[{"x":660,"y":339},{"x":800,"y":339},{"x":800,"y":359},{"x":660,"y":359}],"minx":660,"miny":339,"maxx":800,"maxy":359,"stroke":"black"},"scale":2,"subviews":[],"part":{"x":680,"y":349,"heading":0,"type":"Wall","name":"part44","worldx":730,"worldy":349,"outline":"blue","fill":"blue","power":0,"rotated":1,"moveable":false,"resizeFactor":1,"resizeFactorHeight":6,"resizeFactorWidth":1}},{"x":320,"y":430,"heading":0,"view":{"x":320,"y":430,"heading":0,"scale":2,"points":[{"x":-10,"y":-5},{"x":90,"y":-5},{"x":90,"y":5},{"x":-10,"y":5}],"outline":"blue","fill":"blue","polygon":[{"x":300,"y":420},{"x":500,"y":420},{"x":500,"y":440},{"x":300,"y":440}],"minx":300,"miny":420,"maxx":500,"maxy":440,"stroke":"black"},"scale":2,"subviews":[],"part":{"x":320,"y":430,"heading":0,"type":"Wall","name":"part40","worldx":400,"worldy":430,"outline":"blue","fill":"blue","power":0,"rotated":1,"moveable":false,"resizeFactor":1,"resizeFactorHeight":9,"resizeFactorWidth":1}},{"x":310,"y":340,"heading":0,"view":{"x":310,"y":340,"heading":0,"scale":2,"points":[{"x":-5,"y":-40},{"x":5,"y":-40},{"x":5,"y":10},{"x":-5,"y":10}],"outline":"blue","fill":"blue","polygon":[{"x":300,"y":260},{"x":320,"y":260},{"x":320,"y":360},{"x":300,"y":360}],"minx":300,"miny":260,"maxx":320,"maxy":360,"stroke":"black"},"scale":2,"subviews":[],"part":{"x":310,"y":340,"heading":0,"type":"Wall","name":"part35","worldx":310,"worldy":310,"outline":"blue","fill":"blue","power":0,"rotated":false,"moveable":false,"resizeFactor":1,"resizeFactorHeight":4,"resizeFactorWidth":1}},{"x":320,"y":350,"heading":0,"view":{"x":320,"y":350,"heading":0,"scale":2,"points":[{"x":-10,"y":-5},{"x":90,"y":-5},{"x":90,"y":5},{"x":-10,"y":5}],"outline":"blue","fill":"blue","polygon":[{"x":300,"y":340},{"x":500,"y":340},{"x":500,"y":360},{"x":300,"y":360}],"minx":300,"miny":340,"maxx":500,"maxy":360,"stroke":"black"},"scale":2,"subviews":[],"part":{"x":320,"y":350,"heading":0,"type":"Wall","name":"part37","worldx":400,"worldy":350,"outline":"blue","fill":"blue","power":0,"rotated":1,"moveable":false,"resizeFactor":1,"resizeFactorHeight":9,"resizeFactorWidth":1}},{"x":491,"y":340,"heading":0,"view":{"x":491,"y":340,"heading":0,"scale":2,"points":[{"x":-5,"y":-40},{"x":5,"y":-40},{"x":5,"y":10},{"x":-5,"y":10}],"outline":"blue","fill":"blue","polygon":[{"x":481,"y":260},{"x":501,"y":260},{"x":501,"y":360},{"x":481,"y":360}],"minx":481,"miny":260,"maxx":501,"maxy":360,"stroke":"black"},"scale":2,"subviews":[],"part":{"x":491,"y":340,"heading":0,"type":"Wall","name":"part43","worldx":491,"worldy":310,"outline":"blue","fill":"blue","power":0,"rotated":false,"moveable":false,"resizeFactor":1,"resizeFactorHeight":4,"resizeFactorWidth":1}},{"x":501,"y":510,"heading":0,"view":{"x":501,"y":510,"heading":0,"scale":2,"points":[{"x":-10,"y":-5},{"x":110,"y":-5},{"x":110,"y":5},{"x":-10,"y":5}],"outline":"blue","fill":"blue","polygon":[{"x":481,"y":500},{"x":721,"y":500},{"x":721,"y":520},{"x":481,"y":520}],"minx":481,"miny":500,"maxx":721,"maxy":520,"stroke":"black"},"scale":2,"subviews":[],"part":{"x":501,"y":510,"heading":0,"type":"Wall","name":"part47","worldx":601,"worldy":510,"outline":"blue","fill":"blue","power":0,"rotated":1,"moveable":false,"resizeFactor":1,"resizeFactorHeight":11,"resizeFactorWidth":1}},{"x":581,"y":431,"heading":0,"view":{"x":581,"y":431,"heading":0,"scale":2,"points":[{"x":-10,"y":-5},{"x":70,"y":-5},{"x":70,"y":5},{"x":-10,"y":5}],"outline":"blue","fill":"blue","polygon":[{"x":561,"y":421},{"x":721,"y":421},{"x":721,"y":441},{"x":561,"y":441}],"minx":561,"miny":421,"maxx":721,"maxy":441,"stroke":"black"},"scale":2,"subviews":[],"part":{"x":581,"y":431,"heading":0,"type":"Wall","name":"part48","worldx":641,"worldy":431,"outline":"blue","fill":"blue","power":0,"rotated":1,"moveable":false,"resizeFactor":1,"resizeFactorHeight":7,"resizeFactorWidth":1}},{"x":570,"y":421,"heading":0,"view":{"x":570,"y":421,"heading":0,"scale":2,"points":[{"x":-5,"y":-40},{"x":15,"y":-40},{"x":15,"y":10},{"x":-5,"y":10}],"outline":"blue","fill":"blue","polygon":[{"x":560,"y":341},{"x":600,"y":341},{"x":600,"y":441},{"x":560,"y":441}],"minx":560,"miny":341,"maxx":600,"maxy":441,"stroke":"black"},"scale":2,"subviews":[],"part":{"x":570,"y":421,"heading":0,"type":"Wall","name":"part49","worldx":580,"worldy":391,"outline":"blue","fill":"blue","power":0,"rotated":false,"moveable":false,"resizeFactor":1,"resizeFactorHeight":4,"resizeFactorWidth":3}},{"x":319,"y":269,"heading":0,"view":{"x":319,"y":269,"heading":0,"scale":2,"points":[{"x":-10,"y":-5},{"x":20,"y":-5},{"x":20,"y":45},{"x":-10,"y":45}],"outline":"blue","fill":"blue","polygon":[{"x":299,"y":259},{"x":359,"y":259},{"x":359,"y":359},{"x":299,"y":359}],"minx":299,"miny":259,"maxx":359,"maxy":359,"stroke":"black"},"scale":2,"subviews":[],"part":{"x":319,"y":269,"heading":0,"type":"Wall","name":"part50","worldx":329,"worldy":309,"outline":"blue","fill":"blue","power":0,"rotated":1,"moveable":false,"resizeFactor":1,"resizeFactorHeight":2,"resizeFactorWidth":9}},{"x":461,"y":269,"heading":0,"view":{"x":461,"y":269,"heading":0,"scale":2,"points":[{"x":-10,"y":-5},{"x":20,"y":-5},{"x":20,"y":45},{"x":-10,"y":45}],"outline":"blue","fill":"blue","polygon":[{"x":441,"y":259},{"x":501,"y":259},{"x":501,"y":359},{"x":441,"y":359}],"minx":441,"miny":259,"maxx":501,"maxy":359,"stroke":"black"},"scale":2,"subviews":[],"part":{"x":461,"y":269,"heading":0,"type":"Wall","name":"part51","worldx":471,"worldy":309,"outline":"blue","fill":"blue","power":0,"rotated":1,"moveable":false,"resizeFactor":1,"resizeFactorHeight":2,"resizeFactorWidth":9}},{"x":391,"y":230,"heading":0,"view":{"x":391,"y":230,"heading":0,"scale":2,"points":[{"x":0,"y":0}],"outline":"black","fill":"yellow","polygon":[{"x":391,"y":230}],"minx":385,"miny":224,"maxx":397,"maxy":236,"radius":3},"scale":2,"subviews":[],"part":{"x":391,"y":230,"heading":0,"type":"Light","name":"part39","worldx":391,"worldy":230,"outline":"black","fill":"yellow","power":0,"radius":3,"moveable":true}},{"x":410,"y":230,"heading":0,"view":{"x":410,"y":230,"heading":0,"scale":2,"points":[{"x":0,"y":0}],"outline":"black","fill":"yellow","polygon":[{"x":410,"y":230}],"minx":404,"miny":224,"maxx":416,"maxy":236,"radius":3},"scale":2,"subviews":[],"part":{"x":410,"y":230,"heading":0,"type":"Light","name":"part40","worldx":410,"worldy":230,"outline":"black","fill":"yellow","power":0,"radius":3,"moveable":true}},{"x":370,"y":230,"heading":0,"view":{"x":370,"y":230,"heading":0,"scale":2,"points":[{"x":0,"y":0}],"outline":"black","fill":"yellow","polygon":[{"x":370,"y":230}],"minx":364,"miny":224,"maxx":376,"maxy":236,"radius":3},"scale":2,"subviews":[],"part":{"x":370,"y":230,"heading":0,"type":"Light","name":"part41","worldx":370,"worldy":230,"outline":"black","fill":"yellow","power":0,"radius":3,"moveable":true}},{"x":350,"y":230,"heading":0,"view":{"x":350,"y":230,"heading":0,"scale":2,"points":[{"x":0,"y":0}],"outline":"black","fill":"yellow","polygon":[{"x":350,"y":230}],"minx":344,"miny":224,"maxx":356,"maxy":236,"radius":3},"scale":2,"subviews":[],"part":{"x":350,"y":230,"heading":0,"type":"Light","name":"part42","worldx":350,"worldy":230,"outline":"black","fill":"yellow","power":0,"radius":3,"moveable":true}},{"x":330,"y":230,"heading":0,"view":{"x":330,"y":230,"heading":0,"scale":2,"points":[{"x":0,"y":0}],"outline":"black","fill":"yellow","polygon":[{"x":330,"y":230}],"minx":324,"miny":224,"maxx":336,"maxy":236,"radius":3},"scale":2,"subviews":[],"part":{"x":330,"y":230,"heading":0,"type":"Light","name":"part43","worldx":330,"worldy":230,"outline":"black","fill":"yellow","power":0,"radius":3,"moveable":true}},{"x":310,"y":230,"heading":0,"view":{"x":310,"y":230,"heading":0,"scale":2,"points":[{"x":0,"y":0}],"outline":"black","fill":"yellow","polygon":[{"x":310,"y":230}],"minx":304,"miny":224,"maxx":316,"maxy":236,"radius":3},"scale":2,"subviews":[],"part":{"x":310,"y":230,"heading":0,"type":"Light","name":"part44","worldx":310,"worldy":230,"outline":"black","fill":"yellow","power":0,"radius":3,"moveable":true}},{"x":290,"y":230,"heading":0,"view":{"x":290,"y":230,"heading":0,"scale":2,"points":[{"x":0,"y":0}],"outline":"black","fill":"yellow","polygon":[{"x":290,"y":230}],"minx":284,"miny":224,"maxx":296,"maxy":236,"radius":3},"scale":2,"subviews":[],"part":{"x":290,"y":230,"heading":0,"type":"Light","name":"part45","worldx":290,"worldy":230,"outline":"black","fill":"yellow","power":0,"radius":3,"moveable":true}},{"x":430,"y":230,"heading":0,"view":{"x":430,"y":230,"heading":0,"scale":2,"points":[{"x":0,"y":0}],"outline":"black","fill":"yellow","polygon":[{"x":430,"y":230}],"minx":424,"miny":224,"maxx":436,"maxy":236,"radius":3},"scale":2,"subviews":[],"part":{"x":430,"y":230,"heading":0,"type":"Light","name":"part46","worldx":430,"worldy":230,"outline":"black","fill":"yellow","power":0,"radius":3,"moveable":true}},{"x":450,"y":230,"heading":0,"view":{"x":450,"y":230,"heading":0,"scale":2,"points":[{"x":0,"y":0}],"outline":"black","fill":"yellow","polygon":[{"x":450,"y":230}],"minx":444,"miny":224,"maxx":456,"maxy":236,"radius":3},"scale":2,"subviews":[],"part":{"x":450,"y":230,"heading":0,"type":"Light","name":"part47","worldx":450,"worldy":230,"outline":"black","fill":"yellow","power":0,"radius":3,"moveable":true}},{"x":470,"y":230,"heading":0,"view":{"x":470,"y":230,"heading":0,"scale":2,"points":[{"x":0,"y":0}],"outline":"black","fill":"yellow","polygon":[{"x":470,"y":230}],"minx":464,"miny":224,"maxx":476,"maxy":236,"radius":3},"scale":2,"subviews":[],"part":{"x":470,"y":230,"heading":0,"type":"Light","name":"part48","worldx":470,"worldy":230,"outline":"black","fill":"yellow","power":0,"radius":3,"moveable":true}},{"x":490,"y":230,"heading":0,"view":{"x":490,"y":230,"heading":0,"scale":2,"points":[{"x":0,"y":0}],"outline":"black","fill":"yellow","polygon":[{"x":490,"y":230}],"minx":484,"miny":224,"maxx":496,"maxy":236,"radius":3},"scale":2,"subviews":[],"part":{"x":490,"y":230,"heading":0,"type":"Light","name":"part49","worldx":490,"worldy":230,"outline":"black","fill":"yellow","power":0,"radius":3,"moveable":true}},{"x":510,"y":230,"heading":0,"view":{"x":510,"y":230,"heading":0,"scale":2,"points":[{"x":0,"y":0}],"outline":"black","fill":"yellow","polygon":[{"x":510,"y":230}],"minx":504,"miny":224,"maxx":516,"maxy":236,"radius":3},"scale":2,"subviews":[],"part":{"x":510,"y":230,"heading":0,"type":"Light","name":"part50","worldx":510,"worldy":230,"outline":"black","fill":"yellow","power":0,"radius":3,"moveable":true}},{"x":530,"y":230,"heading":0,"view":{"x":530,"y":230,"heading":0,"scale":2,"points":[{"x":0,"y":0}],"outline":"black","fill":"yellow","polygon":[{"x":530,"y":230}],"minx":524,"miny":224,"maxx":536,"maxy":236,"radius":3},"scale":2,"subviews":[],"part":{"x":530,"y":230,"heading":0,"type":"Light","name":"part51","worldx":530,"worldy":230,"outline":"black","fill":"yellow","power":0,"radius":3,"moveable":true}},{"x":530,"y":250,"heading":0,"view":{"x":530,"y":250,"heading":0,"scale":2,"points":[{"x":0,"y":0}],"outline":"black","fill":"yellow","polygon":[{"x":530,"y":250}],"minx":524,"miny":244,"maxx":536,"maxy":256,"radius":3},"scale":2,"subviews":[],"part":{"x":530,"y":250,"heading":0,"type":"Light","name":"part52","worldx":530,"worldy":250,"outline":"black","fill":"yellow","power":0,"radius":3,"moveable":true}},{"x":530,"y":270,"heading":0,"view":{"x":530,"y":270,"heading":0,"scale":2,"points":[{"x":0,"y":0}],"outline":"black","fill":"yellow","polygon":[{"x":530,"y":270}],"minx":524,"miny":264,"maxx":536,"maxy":276,"radius":3},"scale":2,"subviews":[],"part":{"x":530,"y":270,"heading":0,"type":"Light","name":"part53","worldx":530,"worldy":270,"outline":"black","fill":"yellow","power":0,"radius":3,"moveable":true}},{"x":530,"y":290,"heading":0,"view":{"x":530,"y":290,"heading":0,"scale":2,"points":[{"x":0,"y":0}],"outline":"black","fill":"yellow","polygon":[{"x":530,"y":290}],"minx":524,"miny":284,"maxx":536,"maxy":296,"radius":3},"scale":2,"subviews":[],"part":{"x":530,"y":290,"heading":0,"type":"Light","name":"part54","worldx":530,"worldy":290,"outline":"black","fill":"yellow","power":0,"radius":3,"moveable":true}},{"x":530,"y":310,"heading":0,"view":{"x":530,"y":310,"heading":0,"scale":2,"points":[{"x":0,"y":0}],"outline":"black","fill":"yellow","polygon":[{"x":530,"y":310}],"minx":524,"miny":304,"maxx":536,"maxy":316,"radius":3},"scale":2,"subviews":[],"part":{"x":530,"y":310,"heading":0,"type":"Light","name":"part55","worldx":530,"worldy":310,"outline":"black","fill":"yellow","power":0,"radius":3,"moveable":true}},{"x":530,"y":330,"heading":0,"view":{"x":530,"y":330,"heading":0,"scale":2,"points":[{"x":0,"y":0}],"outline":"black","fill":"yellow","polygon":[{"x":530,"y":330}],"minx":524,"miny":324,"maxx":536,"maxy":336,"radius":3},"scale":2,"subviews":[],"part":{"x":530,"y":330,"heading":0,"type":"Light","name":"part56","worldx":530,"worldy":330,"outline":"black","fill":"yellow","power":0,"radius":3,"moveable":true}},{"x":530,"y":350,"heading":0,"view":{"x":530,"y":350,"heading":0,"scale":2,"points":[{"x":0,"y":0}],"outline":"black","fill":"yellow","polygon":[{"x":530,"y":350}],"minx":524,"miny":344,"maxx":536,"maxy":356,"radius":3},"scale":2,"subviews":[],"part":{"x":530,"y":350,"heading":0,"type":"Light","name":"part57","worldx":530,"worldy":350,"outline":"black","fill":"yellow","power":0,"radius":3,"moveable":true}},{"x":530,"y":370,"heading":0,"view":{"x":530,"y":370,"heading":0,"scale":2,"points":[{"x":0,"y":0}],"outline":"black","fill":"yellow","polygon":[{"x":530,"y":370}],"minx":524,"miny":364,"maxx":536,"maxy":376,"radius":3},"scale":2,"subviews":[],"part":{"x":530,"y":370,"heading":0,"type":"Light","name":"part58","worldx":530,"worldy":370,"outline":"black","fill":"yellow","power":0,"radius":3,"moveable":true}},{"x":270,"y":230,"heading":0,"view":{"x":270,"y":230,"heading":0,"scale":2,"points":[{"x":0,"y":0}],"outline":"black","fill":"yellow","polygon":[{"x":270,"y":230}],"minx":264,"miny":224,"maxx":276,"maxy":236,"radius":3},"scale":2,"subviews":[],"part":{"x":270,"y":230,"heading":0,"type":"Light","name":"part59","worldx":270,"worldy":230,"outline":"black","fill":"yellow","power":0,"radius":3,"moveable":true}},{"x":270,"y":250,"heading":0,"view":{"x":270,"y":250,"heading":0,"scale":2,"points":[{"x":0,"y":0}],"outline":"black","fill":"yellow","polygon":[{"x":270,"y":250}],"minx":264,"miny":244,"maxx":276,"maxy":256,"radius":3},"scale":2,"subviews":[],"part":{"x":270,"y":250,"heading":0,"type":"Light","name":"part60","worldx":270,"worldy":250,"outline":"black","fill":"yellow","power":0,"radius":3,"moveable":true}},{"x":270,"y":270,"heading":0,"view":{"x":270,"y":270,"heading":0,"scale":2,"points":[{"x":0,"y":0}],"outline":"black","fill":"yellow","polygon":[{"x":270,"y":270}],"minx":264,"miny":264,"maxx":276,"maxy":276,"radius":3},"scale":2,"subviews":[],"part":{"x":270,"y":270,"heading":0,"type":"Light","name":"part61","worldx":270,"worldy":270,"outline":"black","fill":"yellow","power":0,"radius":3,"moveable":true}},{"x":270,"y":290,"heading":0,"view":{"x":270,"y":290,"heading":0,"scale":2,"points":[{"x":0,"y":0}],"outline":"black","fill":"yellow","polygon":[{"x":270,"y":290}],"minx":264,"miny":284,"maxx":276,"maxy":296,"radius":3},"scale":2,"subviews":[],"part":{"x":270,"y":290,"heading":0,"type":"Light","name":"part62","worldx":270,"worldy":290,"outline":"black","fill":"yellow","power":0,"radius":3,"moveable":true}},{"x":270,"y":310,"heading":0,"view":{"x":270,"y":310,"heading":0,"scale":2,"points":[{"x":0,"y":0}],"outline":"black","fill":"yellow","polygon":[{"x":270,"y":310}],"minx":264,"miny":304,"maxx":276,"maxy":316,"radius":3},"scale":2,"subviews":[],"part":{"x":270,"y":310,"heading":0,"type":"Light","name":"part63","worldx":270,"worldy":310,"outline":"black","fill":"yellow","power":0,"radius":3,"moveable":true}},{"x":270,"y":330,"heading":0,"view":{"x":270,"y":330,"heading":0,"scale":2,"points":[{"x":0,"y":0}],"outline":"black","fill":"yellow","polygon":[{"x":270,"y":330}],"minx":264,"miny":324,"maxx":276,"maxy":336,"radius":3},"scale":2,"subviews":[],"part":{"x":270,"y":330,"heading":0,"type":"Light","name":"part64","worldx":270,"worldy":330,"outline":"black","fill":"yellow","power":0,"radius":3,"moveable":true}},{"x":270,"y":350,"heading":0,"view":{"x":270,"y":350,"heading":0,"scale":2,"points":[{"x":0,"y":0}],"outline":"black","fill":"yellow","polygon":[{"x":270,"y":350}],"minx":264,"miny":344,"maxx":276,"maxy":356,"radius":3},"scale":2,"subviews":[],"part":{"x":270,"y":350,"heading":0,"type":"Light","name":"part65","worldx":270,"worldy":350,"outline":"black","fill":"yellow","power":0,"radius":3,"moveable":true}},{"x":270,"y":370,"heading":0,"view":{"x":270,"y":370,"heading":0,"scale":2,"points":[{"x":0,"y":0}],"outline":"black","fill":"yellow","polygon":[{"x":270,"y":370}],"minx":264,"miny":364,"maxx":276,"maxy":376,"radius":3},"scale":2,"subviews":[],"part":{"x":270,"y":370,"heading":0,"type":"Light","name":"part66","worldx":270,"worldy":370,"outline":"black","fill":"yellow","power":0,"radius":3,"moveable":true}},{"x":270,"y":390,"heading":0,"view":{"x":270,"y":390,"heading":0,"scale":2,"points":[{"x":0,"y":0}],"outline":"black","fill":"yellow","polygon":[{"x":270,"y":390}],"minx":264,"miny":384,"maxx":276,"maxy":396,"radius":3},"scale":2,"subviews":[],"part":{"x":270,"y":390,"heading":0,"type":"Light","name":"part67","worldx":270,"worldy":390,"outline":"black","fill":"yellow","power":0,"radius":3,"moveable":true}},{"x":290,"y":390,"heading":0,"view":{"x":290,"y":390,"heading":0,"scale":2,"points":[{"x":0,"y":0}],"outline":"black","fill":"yellow","polygon":[{"x":290,"y":390}],"minx":284,"miny":384,"maxx":296,"maxy":396,"radius":3},"scale":2,"subviews":[],"part":{"x":290,"y":390,"heading":0,"type":"Light","name":"part68","worldx":290,"worldy":390,"outline":"black","fill":"yellow","power":0,"radius":3,"moveable":true}},{"x":310,"y":390,"heading":0,"view":{"x":310,"y":390,"heading":0,"scale":2,"points":[{"x":0,"y":0}],"outline":"black","fill":"yellow","polygon":[{"x":310,"y":390}],"minx":304,"miny":384,"maxx":316,"maxy":396,"radius":3},"scale":2,"subviews":[],"part":{"x":310,"y":390,"heading":0,"type":"Light","name":"part69","worldx":310,"worldy":390,"outline":"black","fill":"yellow","power":0,"radius":3,"moveable":true}},{"x":330,"y":390,"heading":0,"view":{"x":330,"y":390,"heading":0,"scale":2,"points":[{"x":0,"y":0}],"outline":"black","fill":"yellow","polygon":[{"x":330,"y":390}],"minx":324,"miny":384,"maxx":336,"maxy":396,"radius":3},"scale":2,"subviews":[],"part":{"x":330,"y":390,"heading":0,"type":"Light","name":"part70","worldx":330,"worldy":390,"outline":"black","fill":"yellow","power":0,"radius":3,"moveable":true}},{"x":350,"y":390,"heading":0,"view":{"x":350,"y":390,"heading":0,"scale":2,"points":[{"x":0,"y":0}],"outline":"black","fill":"yellow","polygon":[{"x":350,"y":390}],"minx":344,"miny":384,"maxx":356,"maxy":396,"radius":3},"scale":2,"subviews":[],"part":{"x":350,"y":390,"heading":0,"type":"Light","name":"part71","worldx":350,"worldy":390,"outline":"black","fill":"yellow","power":0,"radius":3,"moveable":true}},{"x":370,"y":390,"heading":0,"view":{"x":370,"y":390,"heading":0,"scale":2,"points":[{"x":0,"y":0}],"outline":"black","fill":"yellow","polygon":[{"x":370,"y":390}],"minx":364,"miny":384,"maxx":376,"maxy":396,"radius":3},"scale":2,"subviews":[],"part":{"x":370,"y":390,"heading":0,"type":"Light","name":"part72","worldx":370,"worldy":390,"outline":"black","fill":"yellow","power":0,"radius":3,"moveable":true}},{"x":390,"y":390,"heading":0,"view":{"x":390,"y":390,"heading":0,"scale":2,"points":[{"x":0,"y":0}],"outline":"black","fill":"yellow","polygon":[{"x":390,"y":390}],"minx":384,"miny":384,"maxx":396,"maxy":396,"radius":3},"scale":2,"subviews":[],"part":{"x":390,"y":390,"heading":0,"type":"Light","name":"part73","worldx":390,"worldy":390,"outline":"black","fill":"yellow","power":0,"radius":3,"moveable":true}},{"x":410,"y":390,"heading":0,"view":{"x":410,"y":390,"heading":0,"scale":2,"points":[{"x":0,"y":0}],"outline":"black","fill":"yellow","polygon":[{"x":410,"y":390}],"minx":404,"miny":384,"maxx":416,"maxy":396,"radius":3},"scale":2,"subviews":[],"part":{"x":410,"y":390,"heading":0,"type":"Light","name":"part74","worldx":410,"worldy":390,"outline":"black","fill":"yellow","power":0,"radius":3,"moveable":true}},{"x":430,"y":390,"heading":0,"view":{"x":430,"y":390,"heading":0,"scale":2,"points":[{"x":0,"y":0}],"outline":"black","fill":"yellow","polygon":[{"x":430,"y":390}],"minx":424,"miny":384,"maxx":436,"maxy":396,"radius":3},"scale":2,"subviews":[],"part":{"x":430,"y":390,"heading":0,"type":"Light","name":"part75","worldx":430,"worldy":390,"outline":"black","fill":"yellow","power":0,"radius":3,"moveable":true}},{"x":450,"y":390,"heading":0,"view":{"x":450,"y":390,"heading":0,"scale":2,"points":[{"x":0,"y":0}],"outline":"black","fill":"yellow","polygon":[{"x":450,"y":390}],"minx":444,"miny":384,"maxx":456,"maxy":396,"radius":3},"scale":2,"subviews":[],"part":{"x":450,"y":390,"heading":0,"type":"Light","name":"part76","worldx":450,"worldy":390,"outline":"black","fill":"yellow","power":0,"radius":3,"moveable":true}},{"x":530,"y":390,"heading":0,"view":{"x":530,"y":390,"heading":0,"scale":2,"points":[{"x":0,"y":0}],"outline":"black","fill":"yellow","polygon":[{"x":530,"y":390}],"minx":524,"miny":384,"maxx":536,"maxy":396,"radius":3},"scale":2,"subviews":[],"part":{"x":530,"y":390,"heading":0,"type":"Light","name":"part77","worldx":530,"worldy":390,"outline":"black","fill":"yellow","power":0,"radius":3,"moveable":true}},{"x":510,"y":390,"heading":0,"view":{"x":510,"y":390,"heading":0,"scale":2,"points":[{"x":0,"y":0}],"outline":"black","fill":"yellow","polygon":[{"x":510,"y":390}],"minx":504,"miny":384,"maxx":516,"maxy":396,"radius":3},"scale":2,"subviews":[],"part":{"x":510,"y":390,"heading":0,"type":"Light","name":"part78","worldx":510,"worldy":390,"outline":"black","fill":"yellow","power":0,"radius":3,"moveable":true}},{"x":490,"y":390,"heading":0,"view":{"x":490,"y":390,"heading":0,"scale":2,"points":[{"x":0,"y":0}],"outline":"black","fill":"yellow","polygon":[{"x":490,"y":390}],"minx":484,"miny":384,"maxx":496,"maxy":396,"radius":3},"scale":2,"subviews":[],"part":{"x":490,"y":390,"heading":0,"type":"Light","name":"part79","worldx":490,"worldy":390,"outline":"black","fill":"yellow","power":0,"radius":3,"moveable":true}},{"x":470,"y":390,"heading":0,"view":{"x":470,"y":390,"heading":0,"scale":2,"points":[{"x":0,"y":0}],"outline":"black","fill":"yellow","polygon":[{"x":470,"y":390}],"minx":464,"miny":384,"maxx":476,"maxy":396,"radius":3},"scale":2,"subviews":[],"part":{"x":470,"y":390,"heading":0,"type":"Light","name":"part80","worldx":470,"worldy":390,"outline":"black","fill":"yellow","power":0,"radius":3,"moveable":true}},{"x":270,"y":210,"heading":0,"view":{"x":270,"y":210,"heading":0,"scale":2,"points":[{"x":0,"y":0}],"outline":"black","fill":"yellow","polygon":[{"x":270,"y":210}],"minx":264,"miny":204,"maxx":276,"maxy":216,"radius":3},"scale":2,"subviews":[],"part":{"x":270,"y":210,"heading":0,"type":"Light","name":"part81","worldx":270,"worldy":210,"outline":"black","fill":"yellow","power":0,"radius":3,"moveable":true}},{"x":270,"y":190,"heading":0,"view":{"x":270,"y":190,"heading":0,"scale":2,"points":[{"x":0,"y":0}],"outline":"black","fill":"yellow","polygon":[{"x":270,"y":190}],"minx":264,"miny":184,"maxx":276,"maxy":196,"radius":3},"scale":2,"subviews":[],"part":{"x":270,"y":190,"heading":0,"type":"Light","name":"part82","worldx":270,"worldy":190,"outline":"black","fill":"yellow","power":0,"radius":3,"moveable":true}},{"x":270,"y":170,"heading":0,"view":{"x":270,"y":170,"heading":0,"scale":2,"points":[{"x":0,"y":0}],"outline":"black","fill":"yellow","polygon":[{"x":270,"y":170}],"minx":264,"miny":164,"maxx":276,"maxy":176,"radius":3},"scale":2,"subviews":[],"part":{"x":270,"y":170,"heading":0,"type":"Light","name":"part83","worldx":270,"worldy":170,"outline":"black","fill":"yellow","power":0,"radius":3,"moveable":true}},{"x":270,"y":150,"heading":0,"view":{"x":270,"y":150,"heading":0,"scale":2,"points":[{"x":0,"y":0}],"outline":"black","fill":"yellow","polygon":[{"x":270,"y":150}],"minx":264,"miny":144,"maxx":276,"maxy":156,"radius":3},"scale":2,"subviews":[],"part":{"x":270,"y":150,"heading":0,"type":"Light","name":"part84","worldx":270,"worldy":150,"outline":"black","fill":"yellow","power":0,"radius":3,"moveable":true}},{"x":290,"y":150,"heading":0,"view":{"x":290,"y":150,"heading":0,"scale":2,"points":[{"x":0,"y":0}],"outline":"black","fill":"yellow","polygon":[{"x":290,"y":150}],"minx":284,"miny":144,"maxx":296,"maxy":156,"radius":3},"scale":2,"subviews":[],"part":{"x":290,"y":150,"heading":0,"type":"Light","name":"part85","worldx":290,"worldy":150,"outline":"black","fill":"yellow","power":0,"radius":3,"moveable":true}},{"x":310,"y":150,"heading":0,"view":{"x":310,"y":150,"heading":0,"scale":2,"points":[{"x":0,"y":0}],"outline":"black","fill":"yellow","polygon":[{"x":310,"y":150}],"minx":304,"miny":144,"maxx":316,"maxy":156,"radius":3},"scale":2,"subviews":[],"part":{"x":310,"y":150,"heading":0,"type":"Light","name":"part86","worldx":310,"worldy":150,"outline":"black","fill":"yellow","power":0,"radius":3,"moveable":true}},{"x":330,"y":150,"heading":0,"view":{"x":330,"y":150,"heading":0,"scale":2,"points":[{"x":0,"y":0}],"outline":"black","fill":"yellow","polygon":[{"x":330,"y":150}],"minx":324,"miny":144,"maxx":336,"maxy":156,"radius":3},"scale":2,"subviews":[],"part":{"x":330,"y":150,"heading":0,"type":"Light","name":"part87","worldx":330,"worldy":150,"outline":"black","fill":"yellow","power":0,"radius":3,"moveable":true}},{"x":350,"y":150,"heading":0,"view":{"x":350,"y":150,"heading":0,"scale":2,"points":[{"x":0,"y":0}],"outline":"black","fill":"yellow","polygon":[{"x":350,"y":150}],"minx":344,"miny":144,"maxx":356,"maxy":156,"radius":3},"scale":2,"subviews":[],"part":{"x":350,"y":150,"heading":0,"type":"Light","name":"part88","worldx":350,"worldy":150,"outline":"black","fill":"yellow","power":0,"radius":3,"moveable":true}},{"x":370,"y":150,"heading":0,"view":{"x":370,"y":150,"heading":0,"scale":2,"points":[{"x":0,"y":0}],"outline":"black","fill":"yellow","polygon":[{"x":370,"y":150}],"minx":364,"miny":144,"maxx":376,"maxy":156,"radius":3},"scale":2,"subviews":[],"part":{"x":370,"y":150,"heading":0,"type":"Light","name":"part89","worldx":370,"worldy":150,"outline":"black","fill":"yellow","power":0,"radius":3,"moveable":true}},{"x":390,"y":150,"heading":0,"view":{"x":390,"y":150,"heading":0,"scale":2,"points":[{"x":0,"y":0}],"outline":"black","fill":"yellow","polygon":[{"x":390,"y":150}],"minx":384,"miny":144,"maxx":396,"maxy":156,"radius":3},"scale":2,"subviews":[],"part":{"x":390,"y":150,"heading":0,"type":"Light","name":"part90","worldx":390,"worldy":150,"outline":"black","fill":"yellow","power":0,"radius":3,"moveable":true}},{"x":410,"y":150,"heading":0,"view":{"x":410,"y":150,"heading":0,"scale":2,"points":[{"x":0,"y":0}],"outline":"black","fill":"yellow","polygon":[{"x":410,"y":150}],"minx":404,"miny":144,"maxx":416,"maxy":156,"radius":3},"scale":2,"subviews":[],"part":{"x":410,"y":150,"heading":0,"type":"Light","name":"part91","worldx":410,"worldy":150,"outline":"black","fill":"yellow","power":0,"radius":3,"moveable":true}},{"x":430,"y":150,"heading":0,"view":{"x":430,"y":150,"heading":0,"scale":2,"points":[{"x":0,"y":0}],"outline":"black","fill":"yellow","polygon":[{"x":430,"y":150}],"minx":424,"miny":144,"maxx":436,"maxy":156,"radius":3},"scale":2,"subviews":[],"part":{"x":430,"y":150,"heading":0,"type":"Light","name":"part92","worldx":430,"worldy":150,"outline":"black","fill":"yellow","power":0,"radius":3,"moveable":true}},{"x":450,"y":150,"heading":0,"view":{"x":450,"y":150,"heading":0,"scale":2,"points":[{"x":0,"y":0}],"outline":"black","fill":"yellow","polygon":[{"x":450,"y":150}],"minx":444,"miny":144,"maxx":456,"maxy":156,"radius":3},"scale":2,"subviews":[],"part":{"x":450,"y":150,"heading":0,"type":"Light","name":"part93","worldx":450,"worldy":150,"outline":"black","fill":"yellow","power":0,"radius":3,"moveable":true}},{"x":450,"y":130,"heading":0,"view":{"x":450,"y":130,"heading":0,"scale":2,"points":[{"x":0,"y":0}],"outline":"black","fill":"yellow","polygon":[{"x":450,"y":130}],"minx":444,"miny":124,"maxx":456,"maxy":136,"radius":3},"scale":2,"subviews":[],"part":{"x":450,"y":130,"heading":0,"type":"Light","name":"part94","worldx":450,"worldy":130,"outline":"black","fill":"yellow","power":0,"radius":3,"moveable":true}},{"x":470,"y":150,"heading":0,"view":{"x":470,"y":150,"heading":0,"scale":2,"points":[{"x":0,"y":0}],"outline":"black","fill":"yellow","polygon":[{"x":470,"y":150}],"minx":464,"miny":144,"maxx":476,"maxy":156,"radius":3},"scale":2,"subviews":[],"part":{"x":470,"y":150,"heading":0,"type":"Light","name":"part95","worldx":470,"worldy":150,"outline":"black","fill":"yellow","power":0,"radius":3,"moveable":true}},{"x":490,"y":150,"heading":0,"view":{"x":490,"y":150,"heading":0,"scale":2,"points":[{"x":0,"y":0}],"outline":"black","fill":"yellow","polygon":[{"x":490,"y":150}],"minx":484,"miny":144,"maxx":496,"maxy":156,"radius":3},"scale":2,"subviews":[],"part":{"x":490,"y":150,"heading":0,"type":"Light","name":"part96","worldx":490,"worldy":150,"outline":"black","fill":"yellow","power":0,"radius":3,"moveable":true}},{"x":510,"y":150,"heading":0,"view":{"x":510,"y":150,"heading":0,"scale":2,"points":[{"x":0,"y":0}],"outline":"black","fill":"yellow","polygon":[{"x":510,"y":150}],"minx":504,"miny":144,"maxx":516,"maxy":156,"radius":3},"scale":2,"subviews":[],"part":{"x":510,"y":150,"heading":0,"type":"Light","name":"part97","worldx":510,"worldy":150,"outline":"black","fill":"yellow","power":0,"radius":3,"moveable":true}},{"x":530,"y":150,"heading":0,"view":{"x":530,"y":150,"heading":0,"scale":2,"points":[{"x":0,"y":0}],"outline":"black","fill":"yellow","polygon":[{"x":530,"y":150}],"minx":524,"miny":144,"maxx":536,"maxy":156,"radius":3},"scale":2,"subviews":[],"part":{"x":530,"y":150,"heading":0,"type":"Light","name":"part98","worldx":530,"worldy":150,"outline":"black","fill":"yellow","power":0,"radius":3,"moveable":true}},{"x":530,"y":170,"heading":0,"view":{"x":530,"y":170,"heading":0,"scale":2,"points":[{"x":0,"y":0}],"outline":"black","fill":"yellow","polygon":[{"x":530,"y":170}],"minx":524,"miny":164,"maxx":536,"maxy":176,"radius":3},"scale":2,"subviews":[],"part":{"x":530,"y":170,"heading":0,"type":"Light","name":"part99","worldx":530,"worldy":170,"outline":"black","fill":"yellow","power":0,"radius":3,"moveable":true}},{"x":530,"y":190,"heading":0,"view":{"x":530,"y":190,"heading":0,"scale":2,"points":[{"x":0,"y":0}],"outline":"black","fill":"yellow","polygon":[{"x":530,"y":190}],"minx":524,"miny":184,"maxx":536,"maxy":196,"radius":3},"scale":2,"subviews":[],"part":{"x":530,"y":190,"heading":0,"type":"Light","name":"part100","worldx":530,"worldy":190,"outline":"black","fill":"yellow","power":0,"radius":3,"moveable":true}},{"x":530,"y":210,"heading":0,"view":{"x":530,"y":210,"heading":0,"scale":2,"points":[{"x":0,"y":0}],"outline":"black","fill":"yellow","polygon":[{"x":530,"y":210}],"minx":524,"miny":204,"maxx":536,"maxy":216,"radius":3},"scale":2,"subviews":[],"part":{"x":530,"y":210,"heading":0,"type":"Light","name":"part101","worldx":530,"worldy":210,"outline":"black","fill":"yellow","power":0,"radius":3,"moveable":true}},{"x":450,"y":110,"heading":0,"view":{"x":450,"y":110,"heading":0,"scale":2,"points":[{"x":0,"y":0}],"outline":"black","fill":"yellow","polygon":[{"x":450,"y":110}],"minx":444,"miny":104,"maxx":456,"maxy":116,"radius":3},"scale":2,"subviews":[],"part":{"x":450,"y":110,"heading":0,"type":"Light","name":"part102","worldx":450,"worldy":110,"outline":"black","fill":"yellow","power":0,"radius":3,"moveable":true}},{"x":450,"y":90,"heading":0,"view":{"x":450,"y":90,"heading":0,"scale":2,"points":[{"x":0,"y":0}],"outline":"black","fill":"yellow","polygon":[{"x":450,"y":90}],"minx":444,"miny":84,"maxx":456,"maxy":96,"radius":3},"scale":2,"subviews":[],"part":{"x":450,"y":90,"heading":0,"type":"Light","name":"part103","worldx":450,"worldy":90,"outline":"black","fill":"yellow","power":0,"radius":3,"moveable":true}},{"x":550,"y":150,"heading":0,"view":{"x":550,"y":150,"heading":0,"scale":2,"points":[{"x":0,"y":0}],"outline":"black","fill":"yellow","polygon":[{"x":550,"y":150}],"minx":544,"miny":144,"maxx":556,"maxy":156,"radius":3},"scale":2,"subviews":[],"part":{"x":550,"y":150,"heading":0,"type":"Light","name":"part104","worldx":550,"worldy":150,"outline":"black","fill":"yellow","power":0,"radius":3,"moveable":true}},{"x":350,"y":130,"heading":0,"view":{"x":350,"y":130,"heading":0,"scale":2,"points":[{"x":0,"y":0}],"outline":"black","fill":"yellow","polygon":[{"x":350,"y":130}],"minx":344,"miny":124,"maxx":356,"maxy":136,"radius":3},"scale":2,"subviews":[],"part":{"x":350,"y":130,"heading":0,"type":"Light","name":"part105","worldx":350,"worldy":130,"outline":"black","fill":"yellow","power":0,"radius":3,"moveable":true}},{"x":350,"y":110,"heading":0,"view":{"x":350,"y":110,"heading":0,"scale":2,"points":[{"x":0,"y":0}],"outline":"black","fill":"yellow","polygon":[{"x":350,"y":110}],"minx":344,"miny":104,"maxx":356,"maxy":116,"radius":3},"scale":2,"subviews":[],"part":{"x":350,"y":110,"heading":0,"type":"Light","name":"part106","worldx":350,"worldy":110,"outline":"black","fill":"yellow","power":0,"radius":3,"moveable":true}},{"x":450,"y":70,"heading":0,"view":{"x":450,"y":70,"heading":0,"scale":2,"points":[{"x":0,"y":0}],"outline":"black","fill":"yellow","polygon":[{"x":450,"y":70}],"minx":444,"miny":64,"maxx":456,"maxy":76,"radius":3},"scale":2,"subviews":[],"part":{"x":450,"y":70,"heading":0,"type":"Light","name":"part107","worldx":450,"worldy":70,"outline":"black","fill":"yellow","power":0,"radius":3,"moveable":true}},{"x":450,"y":50,"heading":0,"view":{"x":450,"y":50,"heading":0,"scale":2,"points":[{"x":0,"y":0}],"outline":"black","fill":"yellow","polygon":[{"x":450,"y":50}],"minx":444,"miny":44,"maxx":456,"maxy":56,"radius":3},"scale":2,"subviews":[],"part":{"x":450,"y":50,"heading":0,"type":"Light","name":"part108","worldx":450,"worldy":50,"outline":"black","fill":"yellow","power":0,"radius":3,"moveable":true}},{"x":470,"y":50,"heading":0,"view":{"x":470,"y":50,"heading":0,"scale":2,"points":[{"x":0,"y":0}],"outline":"black","fill":"yellow","polygon":[{"x":470,"y":50}],"minx":464,"miny":44,"maxx":476,"maxy":56,"radius":3},"scale":2,"subviews":[],"part":{"x":470,"y":50,"heading":0,"type":"Light","name":"part109","worldx":470,"worldy":50,"outline":"black","fill":"yellow","power":0,"radius":3,"moveable":true}},{"x":350,"y":90,"heading":0,"view":{"x":350,"y":90,"heading":0,"scale":2,"points":[{"x":0,"y":0}],"outline":"black","fill":"yellow","polygon":[{"x":350,"y":90}],"minx":344,"miny":84,"maxx":356,"maxy":96,"radius":3},"scale":2,"subviews":[],"part":{"x":350,"y":90,"heading":0,"type":"Light","name":"part110","worldx":350,"worldy":90,"outline":"black","fill":"yellow","power":0,"radius":3,"moveable":true}},{"x":350,"y":70,"heading":0,"view":{"x":350,"y":70,"heading":0,"scale":2,"points":[{"x":0,"y":0}],"outline":"black","fill":"yellow","polygon":[{"x":350,"y":70}],"minx":344,"miny":64,"maxx":356,"maxy":76,"radius":3},"scale":2,"subviews":[],"part":{"x":350,"y":70,"heading":0,"type":"Light","name":"part111","worldx":350,"worldy":70,"outline":"black","fill":"yellow","power":0,"radius":3,"moveable":true}},{"x":350,"y":50,"heading":0,"view":{"x":350,"y":50,"heading":0,"scale":2,"points":[{"x":0,"y":0}],"outline":"black","fill":"yellow","polygon":[{"x":350,"y":50}],"minx":344,"miny":44,"maxx":356,"maxy":56,"radius":3},"scale":2,"subviews":[],"part":{"x":350,"y":50,"heading":0,"type":"Light","name":"part112","worldx":350,"worldy":50,"outline":"black","fill":"yellow","power":0,"radius":3,"moveable":true}},{"x":330,"y":50,"heading":0,"view":{"x":330,"y":50,"heading":0,"scale":2,"points":[{"x":0,"y":0}],"outline":"black","fill":"yellow","polygon":[{"x":330,"y":50}],"minx":324,"miny":44,"maxx":336,"maxy":56,"radius":3},"scale":2,"subviews":[],"part":{"x":330,"y":50,"heading":0,"type":"Light","name":"part113","worldx":330,"worldy":50,"outline":"black","fill":"yellow","power":0,"radius":3,"moveable":true}},{"x":310,"y":50,"heading":0,"view":{"x":310,"y":50,"heading":0,"scale":2,"points":[{"x":0,"y":0}],"outline":"black","fill":"yellow","polygon":[{"x":310,"y":50}],"minx":304,"miny":44,"maxx":316,"maxy":56,"radius":3},"scale":2,"subviews":[],"part":{"x":310,"y":50,"heading":0,"type":"Light","name":"part114","worldx":310,"worldy":50,"outline":"black","fill":"yellow","power":0,"radius":3,"moveable":true}},{"x":290,"y":50,"heading":0,"view":{"x":290,"y":50,"heading":0,"scale":2,"points":[{"x":0,"y":0}],"outline":"black","fill":"yellow","polygon":[{"x":290,"y":50}],"minx":284,"miny":44,"maxx":296,"maxy":56,"radius":3},"scale":2,"subviews":[],"part":{"x":290,"y":50,"heading":0,"type":"Light","name":"part115","worldx":290,"worldy":50,"outline":"black","fill":"yellow","power":0,"radius":3,"moveable":true}},{"x":270,"y":50,"heading":0,"view":{"x":270,"y":50,"heading":0,"scale":2,"points":[{"x":0,"y":0}],"outline":"black","fill":"yellow","polygon":[{"x":270,"y":50}],"minx":264,"miny":44,"maxx":276,"maxy":56,"radius":3},"scale":2,"subviews":[],"part":{"x":270,"y":50,"heading":0,"type":"Light","name":"part116","worldx":270,"worldy":50,"outline":"black","fill":"yellow","power":0,"radius":3,"moveable":true}},{"x":250,"y":50,"heading":0,"view":{"x":250,"y":50,"heading":0,"scale":2,"points":[{"x":0,"y":0}],"outline":"black","fill":"yellow","polygon":[{"x":250,"y":50}],"minx":244,"miny":44,"maxx":256,"maxy":56,"radius":3},"scale":2,"subviews":[],"part":{"x":250,"y":50,"heading":0,"type":"Light","name":"part117","worldx":250,"worldy":50,"outline":"black","fill":"yellow","power":0,"radius":3,"moveable":true}},{"x":250,"y":150,"heading":0,"view":{"x":250,"y":150,"heading":0,"scale":2,"points":[{"x":0,"y":0}],"outline":"black","fill":"yellow","polygon":[{"x":250,"y":150}],"minx":244,"miny":144,"maxx":256,"maxy":156,"radius":3},"scale":2,"subviews":[],"part":{"x":250,"y":150,"heading":0,"type":"Light","name":"part118","worldx":250,"worldy":150,"outline":"black","fill":"yellow","power":0,"radius":3,"moveable":true}},{"x":230,"y":150,"heading":0,"view":{"x":230,"y":150,"heading":0,"scale":2,"points":[{"x":0,"y":0}],"outline":"black","fill":"yellow","polygon":[{"x":230,"y":150}],"minx":224,"miny":144,"maxx":236,"maxy":156,"radius":3},"scale":2,"subviews":[],"part":{"x":230,"y":150,"heading":0,"type":"Light","name":"part119","worldx":230,"worldy":150,"outline":"black","fill":"yellow","power":0,"radius":3,"moveable":true}},{"x":210,"y":150,"heading":0,"view":{"x":210,"y":150,"heading":0,"scale":2,"points":[{"x":0,"y":0}],"outline":"black","fill":"yellow","polygon":[{"x":210,"y":150}],"minx":204,"miny":144,"maxx":216,"maxy":156,"radius":3},"scale":2,"subviews":[],"part":{"x":210,"y":150,"heading":0,"type":"Light","name":"part120","worldx":210,"worldy":150,"outline":"black","fill":"yellow","power":0,"radius":3,"moveable":true}},{"x":190,"y":150,"heading":0,"view":{"x":190,"y":150,"heading":0,"scale":2,"points":[{"x":0,"y":0}],"outline":"black","fill":"yellow","polygon":[{"x":190,"y":150}],"minx":184,"miny":144,"maxx":196,"maxy":156,"radius":3},"scale":2,"subviews":[],"part":{"x":190,"y":150,"heading":0,"type":"Light","name":"part121","worldx":190,"worldy":150,"outline":"black","fill":"yellow","power":0,"radius":3,"moveable":true}},{"x":170,"y":150,"heading":0,"view":{"x":170,"y":150,"heading":0,"scale":2,"points":[{"x":0,"y":0}],"outline":"black","fill":"yellow","polygon":[{"x":170,"y":150}],"minx":164,"miny":144,"maxx":176,"maxy":156,"radius":3},"scale":2,"subviews":[],"part":{"x":170,"y":150,"heading":0,"type":"Light","name":"part122","worldx":170,"worldy":150,"outline":"black","fill":"yellow","power":0,"radius":3,"moveable":true}},{"x":170,"y":130,"heading":0,"view":{"x":170,"y":130,"heading":0,"scale":2,"points":[{"x":0,"y":0}],"outline":"black","fill":"yellow","polygon":[{"x":170,"y":130}],"minx":164,"miny":124,"maxx":176,"maxy":136,"radius":3},"scale":2,"subviews":[],"part":{"x":170,"y":130,"heading":0,"type":"Light","name":"part123","worldx":170,"worldy":130,"outline":"black","fill":"yellow","power":0,"radius":3,"moveable":true}},{"x":170,"y":110,"heading":0,"view":{"x":170,"y":110,"heading":0,"scale":2,"points":[{"x":0,"y":0}],"outline":"black","fill":"yellow","polygon":[{"x":170,"y":110}],"minx":164,"miny":104,"maxx":176,"maxy":116,"radius":3},"scale":2,"subviews":[],"part":{"x":170,"y":110,"heading":0,"type":"Light","name":"part124","worldx":170,"worldy":110,"outline":"black","fill":"yellow","power":0,"radius":3,"moveable":true}},{"x":170,"y":90,"heading":0,"view":{"x":170,"y":90,"heading":0,"scale":2,"points":[{"x":0,"y":0}],"outline":"black","fill":"yellow","polygon":[{"x":170,"y":90}],"minx":164,"miny":84,"maxx":176,"maxy":96,"radius":3},"scale":2,"subviews":[],"part":{"x":170,"y":90,"heading":0,"type":"Light","name":"part125","worldx":170,"worldy":90,"outline":"black","fill":"yellow","power":0,"radius":3,"moveable":true}},{"x":170,"y":70,"heading":0,"view":{"x":170,"y":70,"heading":0,"scale":2,"points":[{"x":0,"y":0}],"outline":"black","fill":"yellow","polygon":[{"x":170,"y":70}],"minx":164,"miny":64,"maxx":176,"maxy":76,"radius":3},"scale":2,"subviews":[],"part":{"x":170,"y":70,"heading":0,"type":"Light","name":"part126","worldx":170,"worldy":70,"outline":"black","fill":"yellow","power":0,"radius":3,"moveable":true}},{"x":170,"y":50,"heading":0,"view":{"x":170,"y":50,"heading":0,"scale":2,"points":[{"x":0,"y":0}],"outline":"black","fill":"yellow","polygon":[{"x":170,"y":50}],"minx":164,"miny":44,"maxx":176,"maxy":56,"radius":3},"scale":2,"subviews":[],"part":{"x":170,"y":50,"heading":0,"type":"Light","name":"part127","worldx":170,"worldy":50,"outline":"black","fill":"yellow","power":0,"radius":3,"moveable":true}},{"x":190,"y":50,"heading":0,"view":{"x":190,"y":50,"heading":0,"scale":2,"points":[{"x":0,"y":0}],"outline":"black","fill":"yellow","polygon":[{"x":190,"y":50}],"minx":184,"miny":44,"maxx":196,"maxy":56,"radius":3},"scale":2,"subviews":[],"part":{"x":190,"y":50,"heading":0,"type":"Light","name":"part128","worldx":190,"worldy":50,"outline":"black","fill":"yellow","power":0,"radius":3,"moveable":true}},{"x":210,"y":50,"heading":0,"view":{"x":210,"y":50,"heading":0,"scale":2,"points":[{"x":0,"y":0}],"outline":"black","fill":"yellow","polygon":[{"x":210,"y":50}],"minx":204,"miny":44,"maxx":216,"maxy":56,"radius":3},"scale":2,"subviews":[],"part":{"x":210,"y":50,"heading":0,"type":"Light","name":"part129","worldx":210,"worldy":50,"outline":"black","fill":"yellow","power":0,"radius":3,"moveable":true}},{"x":230,"y":50,"heading":0,"view":{"x":230,"y":50,"heading":0,"scale":2,"points":[{"x":0,"y":0}],"outline":"black","fill":"yellow","polygon":[{"x":230,"y":50}],"minx":224,"miny":44,"maxx":236,"maxy":56,"radius":3},"scale":2,"subviews":[],"part":{"x":230,"y":50,"heading":0,"type":"Light","name":"part130","worldx":230,"worldy":50,"outline":"black","fill":"yellow","power":0,"radius":3,"moveable":true}},{"x":150,"y":50,"heading":0,"view":{"x":150,"y":50,"heading":0,"scale":2,"points":[{"x":0,"y":0}],"outline":"black","fill":"yellow","polygon":[{"x":150,"y":50}],"minx":144,"miny":44,"maxx":156,"maxy":56,"radius":3},"scale":2,"subviews":[],"part":{"x":150,"y":50,"heading":0,"type":"Light","name":"part131","worldx":150,"worldy":50,"outline":"black","fill":"yellow","power":0,"radius":3,"moveable":true}},{"x":130,"y":50,"heading":0,"view":{"x":130,"y":50,"heading":0,"scale":2,"points":[{"x":0,"y":0}],"outline":"black","fill":"yellow","polygon":[{"x":130,"y":50}],"minx":124,"miny":44,"maxx":136,"maxy":56,"radius":3},"scale":2,"subviews":[],"part":{"x":130,"y":50,"heading":0,"type":"Light","name":"part132","worldx":130,"worldy":50,"outline":"black","fill":"yellow","power":0,"radius":3,"moveable":true}},{"x":110,"y":50,"heading":0,"view":{"x":110,"y":50,"heading":0,"scale":2,"points":[{"x":0,"y":0}],"outline":"black","fill":"yellow","polygon":[{"x":110,"y":50}],"minx":104,"miny":44,"maxx":116,"maxy":56,"radius":3},"scale":2,"subviews":[],"part":{"x":110,"y":50,"heading":0,"type":"Light","name":"part133","worldx":110,"worldy":50,"outline":"black","fill":"yellow","power":0,"radius":3,"moveable":true}},{"x":90,"y":50,"heading":0,"view":{"x":90,"y":50,"heading":0,"scale":2,"points":[{"x":0,"y":0}],"outline":"black","fill":"yellow","polygon":[{"x":90,"y":50}],"minx":84,"miny":44,"maxx":96,"maxy":56,"radius":3},"scale":2,"subviews":[],"part":{"x":90,"y":50,"heading":0,"type":"Light","name":"part134","worldx":90,"worldy":50,"outline":"black","fill":"yellow","power":0,"radius":3,"moveable":true}},{"x":70,"y":50,"heading":0,"view":{"x":70,"y":50,"heading":0,"scale":2,"points":[{"x":0,"y":0}],"outline":"black","fill":"yellow","polygon":[{"x":70,"y":50}],"minx":64,"miny":44,"maxx":76,"maxy":56,"radius":3},"scale":2,"subviews":[],"part":{"x":70,"y":50,"heading":0,"type":"Light","name":"part135","worldx":70,"worldy":50,"outline":"black","fill":"yellow","power":0,"radius":3,"moveable":true}},{"x":50,"y":50,"heading":0,"view":{"x":50,"y":50,"heading":0,"scale":2,"points":[{"x":0,"y":0}],"outline":"black","fill":"yellow","polygon":[{"x":50,"y":50}],"minx":44,"miny":44,"maxx":56,"maxy":56,"radius":3},"scale":2,"subviews":[],"part":{"x":50,"y":50,"heading":0,"type":"Light","name":"part136","worldx":50,"worldy":50,"outline":"black","fill":"yellow","power":0,"radius":3,"moveable":true}},{"x":50,"y":70,"heading":0,"view":{"x":50,"y":70,"heading":0,"scale":2,"points":[{"x":0,"y":0}],"outline":"black","fill":"yellow","polygon":[{"x":50,"y":70}],"minx":44,"miny":64,"maxx":56,"maxy":76,"radius":3},"scale":2,"subviews":[],"part":{"x":50,"y":70,"heading":0,"type":"Light","name":"part137","worldx":50,"worldy":70,"outline":"black","fill":"yellow","power":0,"radius":3,"moveable":true}},{"x":50,"y":90,"heading":0,"view":{"x":50,"y":90,"heading":0,"scale":2,"points":[{"x":0,"y":0}],"outline":"black","fill":"yellow","polygon":[{"x":50,"y":90}],"minx":44,"miny":84,"maxx":56,"maxy":96,"radius":3},"scale":2,"subviews":[],"part":{"x":50,"y":90,"heading":0,"type":"Light","name":"part138","worldx":50,"worldy":90,"outline":"black","fill":"yellow","power":0,"radius":3,"moveable":true}},{"x":50,"y":110,"heading":0,"view":{"x":50,"y":110,"heading":0,"scale":2,"points":[{"x":0,"y":0}],"outline":"black","fill":"yellow","polygon":[{"x":50,"y":110}],"minx":44,"miny":104,"maxx":56,"maxy":116,"radius":3},"scale":2,"subviews":[],"part":{"x":50,"y":110,"heading":0,"type":"Light","name":"part139","worldx":50,"worldy":110,"outline":"black","fill":"yellow","power":0,"radius":3,"moveable":true}},{"x":50,"y":130,"heading":0,"view":{"x":50,"y":130,"heading":0,"scale":2,"points":[{"x":0,"y":0}],"outline":"black","fill":"yellow","polygon":[{"x":50,"y":130}],"minx":44,"miny":124,"maxx":56,"maxy":136,"radius":3},"scale":2,"subviews":[],"part":{"x":50,"y":130,"heading":0,"type":"Light","name":"part140","worldx":50,"worldy":130,"outline":"black","fill":"yellow","power":0,"radius":3,"moveable":true}},{"x":50,"y":150,"heading":0,"view":{"x":50,"y":150,"heading":0,"scale":2,"points":[{"x":0,"y":0}],"outline":"black","fill":"yellow","polygon":[{"x":50,"y":150}],"minx":44,"miny":144,"maxx":56,"maxy":156,"radius":3},"scale":2,"subviews":[],"part":{"x":50,"y":150,"heading":0,"type":"Light","name":"part141","worldx":50,"worldy":150,"outline":"black","fill":"yellow","power":0,"radius":3,"moveable":true}},{"x":70,"y":150,"heading":0,"view":{"x":70,"y":150,"heading":0,"scale":2,"points":[{"x":0,"y":0}],"outline":"black","fill":"yellow","polygon":[{"x":70,"y":150}],"minx":64,"miny":144,"maxx":76,"maxy":156,"radius":3},"scale":2,"subviews":[],"part":{"x":70,"y":150,"heading":0,"type":"Light","name":"part142","worldx":70,"worldy":150,"outline":"black","fill":"yellow","power":0,"radius":3,"moveable":true}},{"x":90,"y":150,"heading":0,"view":{"x":90,"y":150,"heading":0,"scale":2,"points":[{"x":0,"y":0}],"outline":"black","fill":"yellow","polygon":[{"x":90,"y":150}],"minx":84,"miny":144,"maxx":96,"maxy":156,"radius":3},"scale":2,"subviews":[],"part":{"x":90,"y":150,"heading":0,"type":"Light","name":"part143","worldx":90,"worldy":150,"outline":"black","fill":"yellow","power":0,"radius":3,"moveable":true}},{"x":110,"y":150,"heading":0,"view":{"x":110,"y":150,"heading":0,"scale":2,"points":[{"x":0,"y":0}],"outline":"black","fill":"yellow","polygon":[{"x":110,"y":150}],"minx":104,"miny":144,"maxx":116,"maxy":156,"radius":3},"scale":2,"subviews":[],"part":{"x":110,"y":150,"heading":0,"type":"Light","name":"part144","worldx":110,"worldy":150,"outline":"black","fill":"yellow","power":0,"radius":3,"moveable":true}},{"x":130,"y":150,"heading":0,"view":{"x":130,"y":150,"heading":0,"scale":2,"points":[{"x":0,"y":0}],"outline":"black","fill":"yellow","polygon":[{"x":130,"y":150}],"minx":124,"miny":144,"maxx":136,"maxy":156,"radius":3},"scale":2,"subviews":[],"part":{"x":130,"y":150,"heading":0,"type":"Light","name":"part145","worldx":130,"worldy":150,"outline":"black","fill":"yellow","power":0,"radius":3,"moveable":true}},{"x":150,"y":150,"heading":0,"view":{"x":150,"y":150,"heading":0,"scale":2,"points":[{"x":0,"y":0}],"outline":"black","fill":"yellow","polygon":[{"x":150,"y":150}],"minx":144,"miny":144,"maxx":156,"maxy":156,"radius":3},"scale":2,"subviews":[],"part":{"x":150,"y":150,"heading":0,"type":"Light","name":"part146","worldx":150,"worldy":150,"outline":"black","fill":"yellow","power":0,"radius":3,"moveable":true}},{"x":170,"y":170,"heading":0,"view":{"x":170,"y":170,"heading":0,"scale":2,"points":[{"x":0,"y":0}],"outline":"black","fill":"yellow","polygon":[{"x":170,"y":170}],"minx":164,"miny":164,"maxx":176,"maxy":176,"radius":3},"scale":2,"subviews":[],"part":{"x":170,"y":170,"heading":0,"type":"Light","name":"part147","worldx":170,"worldy":170,"outline":"black","fill":"yellow","power":0,"radius":3,"moveable":true}},{"x":170,"y":190,"heading":0,"view":{"x":170,"y":190,"heading":0,"scale":2,"points":[{"x":0,"y":0}],"outline":"black","fill":"yellow","polygon":[{"x":170,"y":190}],"minx":164,"miny":184,"maxx":176,"maxy":196,"radius":3},"scale":2,"subviews":[],"part":{"x":170,"y":190,"heading":0,"type":"Light","name":"part148","worldx":170,"worldy":190,"outline":"black","fill":"yellow","power":0,"radius":3,"moveable":true}},{"x":170,"y":210,"heading":0,"view":{"x":170,"y":210,"heading":0,"scale":2,"points":[{"x":0,"y":0}],"outline":"black","fill":"yellow","polygon":[{"x":170,"y":210}],"minx":164,"miny":204,"maxx":176,"maxy":216,"radius":3},"scale":2,"subviews":[],"part":{"x":170,"y":210,"heading":0,"type":"Light","name":"part149","worldx":170,"worldy":210,"outline":"black","fill":"yellow","power":0,"radius":3,"moveable":true}},{"x":170,"y":230,"heading":0,"view":{"x":170,"y":230,"heading":0,"scale":2,"points":[{"x":0,"y":0}],"outline":"black","fill":"yellow","polygon":[{"x":170,"y":230}],"minx":164,"miny":224,"maxx":176,"maxy":236,"radius":3},"scale":2,"subviews":[],"part":{"x":170,"y":230,"heading":0,"type":"Light","name":"part150","worldx":170,"worldy":230,"outline":"black","fill":"yellow","power":0,"radius":3,"moveable":true}},{"x":150,"y":230,"heading":0,"view":{"x":150,"y":230,"heading":0,"scale":2,"points":[{"x":0,"y":0}],"outline":"black","fill":"yellow","polygon":[{"x":150,"y":230}],"minx":144,"miny":224,"maxx":156,"maxy":236,"radius":3},"scale":2,"subviews":[],"part":{"x":150,"y":230,"heading":0,"type":"Light","name":"part151","worldx":150,"worldy":230,"outline":"black","fill":"yellow","power":0,"radius":3,"moveable":true}},{"x":130,"y":230,"heading":0,"view":{"x":130,"y":230,"heading":0,"scale":2,"points":[{"x":0,"y":0}],"outline":"black","fill":"yellow","polygon":[{"x":130,"y":230}],"minx":124,"miny":224,"maxx":136,"maxy":236,"radius":3},"scale":2,"subviews":[],"part":{"x":130,"y":230,"heading":0,"type":"Light","name":"part152","worldx":130,"worldy":230,"outline":"black","fill":"yellow","power":0,"radius":3,"moveable":true}},{"x":110,"y":230,"heading":0,"view":{"x":110,"y":230,"heading":0,"scale":2,"points":[{"x":0,"y":0}],"outline":"black","fill":"yellow","polygon":[{"x":110,"y":230}],"minx":104,"miny":224,"maxx":116,"maxy":236,"radius":3},"scale":2,"subviews":[],"part":{"x":110,"y":230,"heading":0,"type":"Light","name":"part153","worldx":110,"worldy":230,"outline":"black","fill":"yellow","power":0,"radius":3,"moveable":true}},{"x":90,"y":230,"heading":0,"view":{"x":90,"y":230,"heading":0,"scale":2,"points":[{"x":0,"y":0}],"outline":"black","fill":"yellow","polygon":[{"x":90,"y":230}],"minx":84,"miny":224,"maxx":96,"maxy":236,"radius":3},"scale":2,"subviews":[],"part":{"x":90,"y":230,"heading":0,"type":"Light","name":"part154","worldx":90,"worldy":230,"outline":"black","fill":"yellow","power":0,"radius":3,"moveable":true}},{"x":70,"y":230,"heading":0,"view":{"x":70,"y":230,"heading":0,"scale":2,"points":[{"x":0,"y":0}],"outline":"black","fill":"yellow","polygon":[{"x":70,"y":230}],"minx":64,"miny":224,"maxx":76,"maxy":236,"radius":3},"scale":2,"subviews":[],"part":{"x":70,"y":230,"heading":0,"type":"Light","name":"part155","worldx":70,"worldy":230,"outline":"black","fill":"yellow","power":0,"radius":3,"moveable":true}},{"x":50,"y":230,"heading":0,"view":{"x":50,"y":230,"heading":0,"scale":2,"points":[{"x":0,"y":0}],"outline":"black","fill":"yellow","polygon":[{"x":50,"y":230}],"minx":44,"miny":224,"maxx":56,"maxy":236,"radius":3},"scale":2,"subviews":[],"part":{"x":50,"y":230,"heading":0,"type":"Light","name":"part156","worldx":50,"worldy":230,"outline":"black","fill":"yellow","power":0,"radius":3,"moveable":true}},{"x":50,"y":210,"heading":0,"view":{"x":50,"y":210,"heading":0,"scale":2,"points":[{"x":0,"y":0}],"outline":"black","fill":"yellow","polygon":[{"x":50,"y":210}],"minx":44,"miny":204,"maxx":56,"maxy":216,"radius":3},"scale":2,"subviews":[],"part":{"x":50,"y":210,"heading":0,"type":"Light","name":"part157","worldx":50,"worldy":210,"outline":"black","fill":"yellow","power":0,"radius":3,"moveable":true}},{"x":50,"y":190,"heading":0,"view":{"x":50,"y":190,"heading":0,"scale":2,"points":[{"x":0,"y":0}],"outline":"black","fill":"yellow","polygon":[{"x":50,"y":190}],"minx":44,"miny":184,"maxx":56,"maxy":196,"radius":3},"scale":2,"subviews":[],"part":{"x":50,"y":190,"heading":0,"type":"Light","name":"part158","worldx":50,"worldy":190,"outline":"black","fill":"yellow","power":0,"radius":3,"moveable":true}},{"x":50,"y":170,"heading":0,"view":{"x":50,"y":170,"heading":0,"scale":2,"points":[{"x":0,"y":0}],"outline":"black","fill":"yellow","polygon":[{"x":50,"y":170}],"minx":44,"miny":164,"maxx":56,"maxy":176,"radius":3},"scale":2,"subviews":[],"part":{"x":50,"y":170,"heading":0,"type":"Light","name":"part159","worldx":50,"worldy":170,"outline":"black","fill":"yellow","power":0,"radius":3,"moveable":true}},{"x":250,"y":310,"heading":0,"view":{"x":250,"y":310,"heading":0,"scale":2,"points":[{"x":0,"y":0}],"outline":"black","fill":"yellow","polygon":[{"x":250,"y":310}],"minx":244,"miny":304,"maxx":256,"maxy":316,"radius":3},"scale":2,"subviews":[],"part":{"x":250,"y":310,"heading":0,"type":"Light","name":"part160","worldx":250,"worldy":310,"outline":"black","fill":"yellow","power":0,"radius":3,"moveable":true}},{"x":230,"y":310,"heading":0,"view":{"x":230,"y":310,"heading":0,"scale":2,"points":[{"x":0,"y":0}],"outline":"black","fill":"yellow","polygon":[{"x":230,"y":310}],"minx":224,"miny":304,"maxx":236,"maxy":316,"radius":3},"scale":2,"subviews":[],"part":{"x":230,"y":310,"heading":0,"type":"Light","name":"part161","worldx":230,"worldy":310,"outline":"black","fill":"yellow","power":0,"radius":3,"moveable":true}},{"x":210,"y":310,"heading":0,"view":{"x":210,"y":310,"heading":0,"scale":2,"points":[{"x":0,"y":0}],"outline":"black","fill":"yellow","polygon":[{"x":210,"y":310}],"minx":204,"miny":304,"maxx":216,"maxy":316,"radius":3},"scale":2,"subviews":[],"part":{"x":210,"y":310,"heading":0,"type":"Light","name":"part162","worldx":210,"worldy":310,"outline":"black","fill":"yellow","power":0,"radius":3,"moveable":true}},{"x":190,"y":310,"heading":0,"view":{"x":190,"y":310,"heading":0,"scale":2,"points":[{"x":0,"y":0}],"outline":"black","fill":"yellow","polygon":[{"x":190,"y":310}],"minx":184,"miny":304,"maxx":196,"maxy":316,"radius":3},"scale":2,"subviews":[],"part":{"x":190,"y":310,"heading":0,"type":"Light","name":"part163","worldx":190,"worldy":310,"outline":"black","fill":"yellow","power":0,"radius":3,"moveable":true}},{"x":170,"y":290,"heading":0,"view":{"x":170,"y":290,"heading":0,"scale":2,"points":[{"x":0,"y":0}],"outline":"black","fill":"yellow","polygon":[{"x":170,"y":290}],"minx":164,"miny":284,"maxx":176,"maxy":296,"radius":3},"scale":2,"subviews":[],"part":{"x":170,"y":290,"heading":0,"type":"Light","name":"part164","worldx":170,"worldy":290,"outline":"black","fill":"yellow","power":0,"radius":3,"moveable":true}},{"x":170,"y":270,"heading":0,"view":{"x":170,"y":270,"heading":0,"scale":2,"points":[{"x":0,"y":0}],"outline":"black","fill":"yellow","polygon":[{"x":170,"y":270}],"minx":164,"miny":264,"maxx":176,"maxy":276,"radius":3},"scale":2,"subviews":[],"part":{"x":170,"y":270,"heading":0,"type":"Light","name":"part165","worldx":170,"worldy":270,"outline":"black","fill":"yellow","power":0,"radius":3,"moveable":true}},{"x":170,"y":250,"heading":0,"view":{"x":170,"y":250,"heading":0,"scale":2,"points":[{"x":0,"y":0}],"outline":"black","fill":"yellow","polygon":[{"x":170,"y":250}],"minx":164,"miny":244,"maxx":176,"maxy":256,"radius":3},"scale":2,"subviews":[],"part":{"x":170,"y":250,"heading":0,"type":"Light","name":"part166","worldx":170,"worldy":250,"outline":"black","fill":"yellow","power":0,"radius":3,"moveable":true}},{"x":170,"y":310,"heading":0,"view":{"x":170,"y":310,"heading":0,"scale":2,"points":[{"x":0,"y":0}],"outline":"black","fill":"yellow","polygon":[{"x":170,"y":310}],"minx":164,"miny":304,"maxx":176,"maxy":316,"radius":3},"scale":2,"subviews":[],"part":{"x":170,"y":310,"heading":0,"type":"Light","name":"part167","worldx":170,"worldy":310,"outline":"black","fill":"yellow","power":0,"radius":3,"moveable":true}},{"x":170,"y":330,"heading":0,"view":{"x":170,"y":330,"heading":0,"scale":2,"points":[{"x":0,"y":0}],"outline":"black","fill":"yellow","polygon":[{"x":170,"y":330}],"minx":164,"miny":324,"maxx":176,"maxy":336,"radius":3},"scale":2,"subviews":[],"part":{"x":170,"y":330,"heading":0,"type":"Light","name":"part168","worldx":170,"worldy":330,"outline":"black","fill":"yellow","power":0,"radius":3,"moveable":true}},{"x":170,"y":350,"heading":0,"view":{"x":170,"y":350,"heading":0,"scale":2,"points":[{"x":0,"y":0}],"outline":"black","fill":"yellow","polygon":[{"x":170,"y":350}],"minx":164,"miny":344,"maxx":176,"maxy":356,"radius":3},"scale":2,"subviews":[],"part":{"x":170,"y":350,"heading":0,"type":"Light","name":"part169","worldx":170,"worldy":350,"outline":"black","fill":"yellow","power":0,"radius":3,"moveable":true}},{"x":170,"y":370,"heading":0,"view":{"x":170,"y":370,"heading":0,"scale":2,"points":[{"x":0,"y":0}],"outline":"black","fill":"yellow","polygon":[{"x":170,"y":370}],"minx":164,"miny":364,"maxx":176,"maxy":376,"radius":3},"scale":2,"subviews":[],"part":{"x":170,"y":370,"heading":0,"type":"Light","name":"part170","worldx":170,"worldy":370,"outline":"black","fill":"yellow","power":0,"radius":3,"moveable":true}},{"x":170,"y":390,"heading":0,"view":{"x":170,"y":390,"heading":0,"scale":2,"points":[{"x":0,"y":0}],"outline":"black","fill":"yellow","polygon":[{"x":170,"y":390}],"minx":164,"miny":384,"maxx":176,"maxy":396,"radius":3},"scale":2,"subviews":[],"part":{"x":170,"y":390,"heading":0,"type":"Light","name":"part171","worldx":170,"worldy":390,"outline":"black","fill":"yellow","power":0,"radius":3,"moveable":true}},{"x":150,"y":390,"heading":0,"view":{"x":150,"y":390,"heading":0,"scale":2,"points":[{"x":0,"y":0}],"outline":"black","fill":"yellow","polygon":[{"x":150,"y":390}],"minx":144,"miny":384,"maxx":156,"maxy":396,"radius":3},"scale":2,"subviews":[],"part":{"x":150,"y":390,"heading":0,"type":"Light","name":"part172","worldx":150,"worldy":390,"outline":"black","fill":"yellow","power":0,"radius":3,"moveable":true}},{"x":130,"y":390,"heading":0,"view":{"x":130,"y":390,"heading":0,"scale":2,"points":[{"x":0,"y":0}],"outline":"black","fill":"yellow","polygon":[{"x":130,"y":390}],"minx":124,"miny":384,"maxx":136,"maxy":396,"radius":3},"scale":2,"subviews":[],"part":{"x":130,"y":390,"heading":0,"type":"Light","name":"part173","worldx":130,"worldy":390,"outline":"black","fill":"yellow","power":0,"radius":3,"moveable":true}},{"x":270,"y":410,"heading":0,"view":{"x":270,"y":410,"heading":0,"scale":2,"points":[{"x":0,"y":0}],"outline":"black","fill":"yellow","polygon":[{"x":270,"y":410}],"minx":264,"miny":404,"maxx":276,"maxy":416,"radius":3},"scale":2,"subviews":[],"part":{"x":270,"y":410,"heading":0,"type":"Light","name":"part174","worldx":270,"worldy":410,"outline":"black","fill":"yellow","power":0,"radius":3,"moveable":true}},{"x":270,"y":430,"heading":0,"view":{"x":270,"y":430,"heading":0,"scale":2,"points":[{"x":0,"y":0}],"outline":"black","fill":"yellow","polygon":[{"x":270,"y":430}],"minx":264,"miny":424,"maxx":276,"maxy":436,"radius":3},"scale":2,"subviews":[],"part":{"x":270,"y":430,"heading":0,"type":"Light","name":"part175","worldx":270,"worldy":430,"outline":"black","fill":"yellow","power":0,"radius":3,"moveable":true}},{"x":530,"y":410,"heading":0,"view":{"x":530,"y":410,"heading":0,"scale":2,"points":[{"x":0,"y":0}],"outline":"black","fill":"yellow","polygon":[{"x":530,"y":410}],"minx":524,"miny":404,"maxx":536,"maxy":416,"radius":3},"scale":2,"subviews":[],"part":{"x":530,"y":410,"heading":0,"type":"Light","name":"part176","worldx":530,"worldy":410,"outline":"black","fill":"yellow","power":0,"radius":3,"moveable":true}},{"x":530,"y":430,"heading":0,"view":{"x":530,"y":430,"heading":0,"scale":2,"points":[{"x":0,"y":0}],"outline":"black","fill":"yellow","polygon":[{"x":530,"y":430}],"minx":524,"miny":424,"maxx":536,"maxy":436,"radius":3},"scale":2,"subviews":[],"part":{"x":530,"y":430,"heading":0,"type":"Light","name":"part177","worldx":530,"worldy":430,"outline":"black","fill":"yellow","power":0,"radius":3,"moveable":true}},{"x":550,"y":310,"heading":0,"view":{"x":550,"y":310,"heading":0,"scale":2,"points":[{"x":0,"y":0}],"outline":"black","fill":"yellow","polygon":[{"x":550,"y":310}],"minx":544,"miny":304,"maxx":556,"maxy":316,"radius":3},"scale":2,"subviews":[],"part":{"x":550,"y":310,"heading":0,"type":"Light","name":"part178","worldx":550,"worldy":310,"outline":"black","fill":"yellow","power":0,"radius":3,"moveable":true}},{"x":490,"y":50,"heading":0,"view":{"x":490,"y":50,"heading":0,"scale":2,"points":[{"x":0,"y":0}],"outline":"black","fill":"yellow","polygon":[{"x":490,"y":50}],"minx":484,"miny":44,"maxx":496,"maxy":56,"radius":3},"scale":2,"subviews":[],"part":{"x":490,"y":50,"heading":0,"type":"Light","name":"part179","worldx":490,"worldy":50,"outline":"black","fill":"yellow","power":0,"radius":3,"moveable":true}},{"x":510,"y":50,"heading":0,"view":{"x":510,"y":50,"heading":0,"scale":2,"points":[{"x":0,"y":0}],"outline":"black","fill":"yellow","polygon":[{"x":510,"y":50}],"minx":504,"miny":44,"maxx":516,"maxy":56,"radius":3},"scale":2,"subviews":[],"part":{"x":510,"y":50,"heading":0,"type":"Light","name":"part180","worldx":510,"worldy":50,"outline":"black","fill":"yellow","power":0,"radius":3,"moveable":true}},{"x":530,"y":50,"heading":0,"view":{"x":530,"y":50,"heading":0,"scale":2,"points":[{"x":0,"y":0}],"outline":"black","fill":"yellow","polygon":[{"x":530,"y":50}],"minx":524,"miny":44,"maxx":536,"maxy":56,"radius":3},"scale":2,"subviews":[],"part":{"x":530,"y":50,"heading":0,"type":"Light","name":"part181","worldx":530,"worldy":50,"outline":"black","fill":"yellow","power":0,"radius":3,"moveable":true}},{"x":550,"y":50,"heading":0,"view":{"x":550,"y":50,"heading":0,"scale":2,"points":[{"x":0,"y":0}],"outline":"black","fill":"yellow","polygon":[{"x":550,"y":50}],"minx":544,"miny":44,"maxx":556,"maxy":56,"radius":3},"scale":2,"subviews":[],"part":{"x":550,"y":50,"heading":0,"type":"Light","name":"part182","worldx":550,"worldy":50,"outline":"black","fill":"yellow","power":0,"radius":3,"moveable":true}},{"x":570,"y":50,"heading":0,"view":{"x":570,"y":50,"heading":0,"scale":2,"points":[{"x":0,"y":0}],"outline":"black","fill":"yellow","polygon":[{"x":570,"y":50}],"minx":564,"miny":44,"maxx":576,"maxy":56,"radius":3},"scale":2,"subviews":[],"part":{"x":570,"y":50,"heading":0,"type":"Light","name":"part183","worldx":570,"worldy":50,"outline":"black","fill":"yellow","power":0,"radius":3,"moveable":true}},{"x":590,"y":50,"heading":0,"view":{"x":590,"y":50,"heading":0,"scale":2,"points":[{"x":0,"y":0}],"outline":"black","fill":"yellow","polygon":[{"x":590,"y":50}],"minx":584,"miny":44,"maxx":596,"maxy":56,"radius":3},"scale":2,"subviews":[],"part":{"x":590,"y":50,"heading":0,"type":"Light","name":"part184","worldx":590,"worldy":50,"outline":"black","fill":"yellow","power":0,"radius":3,"moveable":true}},{"x":610,"y":50,"heading":0,"view":{"x":610,"y":50,"heading":0,"scale":2,"points":[{"x":0,"y":0}],"outline":"black","fill":"yellow","polygon":[{"x":610,"y":50}],"minx":604,"miny":44,"maxx":616,"maxy":56,"radius":3},"scale":2,"subviews":[],"part":{"x":610,"y":50,"heading":0,"type":"Light","name":"part185","worldx":610,"worldy":50,"outline":"black","fill":"yellow","power":0,"radius":3,"moveable":true}},{"x":630,"y":50,"heading":0,"view":{"x":630,"y":50,"heading":0,"scale":2,"points":[{"x":0,"y":0}],"outline":"black","fill":"yellow","polygon":[{"x":630,"y":50}],"minx":624,"miny":44,"maxx":636,"maxy":56,"radius":3},"scale":2,"subviews":[],"part":{"x":630,"y":50,"heading":0,"type":"Light","name":"part186","worldx":630,"worldy":50,"outline":"black","fill":"yellow","power":0,"radius":3,"moveable":true}},{"x":650,"y":50,"heading":0,"view":{"x":650,"y":50,"heading":0,"scale":2,"points":[{"x":0,"y":0}],"outline":"black","fill":"yellow","polygon":[{"x":650,"y":50}],"minx":644,"miny":44,"maxx":656,"maxy":56,"radius":3},"scale":2,"subviews":[],"part":{"x":650,"y":50,"heading":0,"type":"Light","name":"part187","worldx":650,"worldy":50,"outline":"black","fill":"yellow","power":0,"radius":3,"moveable":true}},{"x":670,"y":50,"heading":0,"view":{"x":670,"y":50,"heading":0,"scale":2,"points":[{"x":0,"y":0}],"outline":"black","fill":"yellow","polygon":[{"x":670,"y":50}],"minx":664,"miny":44,"maxx":676,"maxy":56,"radius":3},"scale":2,"subviews":[],"part":{"x":670,"y":50,"heading":0,"type":"Light","name":"part188","worldx":670,"worldy":50,"outline":"black","fill":"yellow","power":0,"radius":3,"moveable":true}},{"x":630,"y":70,"heading":0,"view":{"x":630,"y":70,"heading":0,"scale":2,"points":[{"x":0,"y":0}],"outline":"black","fill":"yellow","polygon":[{"x":630,"y":70}],"minx":624,"miny":64,"maxx":636,"maxy":76,"radius":3},"scale":2,"subviews":[],"part":{"x":630,"y":70,"heading":0,"type":"Light","name":"part189","worldx":630,"worldy":70,"outline":"black","fill":"yellow","power":0,"radius":3,"moveable":true}},{"x":690,"y":50,"heading":0,"view":{"x":690,"y":50,"heading":0,"scale":2,"points":[{"x":0,"y":0}],"outline":"black","fill":"yellow","polygon":[{"x":690,"y":50}],"minx":684,"miny":44,"maxx":696,"maxy":56,"radius":3},"scale":2,"subviews":[],"part":{"x":690,"y":50,"heading":0,"type":"Light","name":"part190","worldx":690,"worldy":50,"outline":"black","fill":"yellow","power":0,"radius":3,"moveable":true}},{"x":710,"y":50,"heading":0,"view":{"x":710,"y":50,"heading":0,"scale":2,"points":[{"x":0,"y":0}],"outline":"black","fill":"yellow","polygon":[{"x":710,"y":50}],"minx":704,"miny":44,"maxx":716,"maxy":56,"radius":3},"scale":2,"subviews":[],"part":{"x":710,"y":50,"heading":0,"type":"Light","name":"part191","worldx":710,"worldy":50,"outline":"black","fill":"yellow","power":0,"radius":3,"moveable":true}},{"x":730,"y":50,"heading":0,"view":{"x":730,"y":50,"heading":0,"scale":2,"points":[{"x":0,"y":0}],"outline":"black","fill":"yellow","polygon":[{"x":730,"y":50}],"minx":724,"miny":44,"maxx":736,"maxy":56,"radius":3},"scale":2,"subviews":[],"part":{"x":730,"y":50,"heading":0,"type":"Light","name":"part192","worldx":730,"worldy":50,"outline":"black","fill":"yellow","power":0,"radius":3,"moveable":true}},{"x":750,"y":50,"heading":0,"view":{"x":750,"y":50,"heading":0,"scale":2,"points":[{"x":0,"y":0}],"outline":"black","fill":"yellow","polygon":[{"x":750,"y":50}],"minx":744,"miny":44,"maxx":756,"maxy":56,"radius":3},"scale":2,"subviews":[],"part":{"x":750,"y":50,"heading":0,"type":"Light","name":"part193","worldx":750,"worldy":50,"outline":"black","fill":"yellow","power":0,"radius":3,"moveable":true}},{"x":630,"y":90,"heading":0,"view":{"x":630,"y":90,"heading":0,"scale":2,"points":[{"x":0,"y":0}],"outline":"black","fill":"yellow","polygon":[{"x":630,"y":90}],"minx":624,"miny":84,"maxx":636,"maxy":96,"radius":3},"scale":2,"subviews":[],"part":{"x":630,"y":90,"heading":0,"type":"Light","name":"part194","worldx":630,"worldy":90,"outline":"black","fill":"yellow","power":0,"radius":3,"moveable":true}},{"x":630,"y":110,"heading":0,"view":{"x":630,"y":110,"heading":0,"scale":2,"points":[{"x":0,"y":0}],"outline":"black","fill":"yellow","polygon":[{"x":630,"y":110}],"minx":624,"miny":104,"maxx":636,"maxy":116,"radius":3},"scale":2,"subviews":[],"part":{"x":630,"y":110,"heading":0,"type":"Light","name":"part195","worldx":630,"worldy":110,"outline":"black","fill":"yellow","power":0,"radius":3,"moveable":true}},{"x":630,"y":130,"heading":0,"view":{"x":630,"y":130,"heading":0,"scale":2,"points":[{"x":0,"y":0}],"outline":"black","fill":"yellow","polygon":[{"x":630,"y":130}],"minx":624,"miny":124,"maxx":636,"maxy":136,"radius":3},"scale":2,"subviews":[],"part":{"x":630,"y":130,"heading":0,"type":"Light","name":"part196","worldx":630,"worldy":130,"outline":"black","fill":"yellow","power":0,"radius":3,"moveable":true}},{"x":630,"y":150,"heading":0,"view":{"x":630,"y":150,"heading":0,"scale":2,"points":[{"x":0,"y":0}],"outline":"black","fill":"yellow","polygon":[{"x":630,"y":150}],"minx":624,"miny":144,"maxx":636,"maxy":156,"radius":3},"scale":2,"subviews":[],"part":{"x":630,"y":150,"heading":0,"type":"Light","name":"part197","worldx":630,"worldy":150,"outline":"black","fill":"yellow","power":0,"radius":3,"moveable":true}},{"x":610,"y":150,"heading":0,"view":{"x":610,"y":150,"heading":0,"scale":2,"points":[{"x":0,"y":0}],"outline":"black","fill":"yellow","polygon":[{"x":610,"y":150}],"minx":604,"miny":144,"maxx":616,"maxy":156,"radius":3},"scale":2,"subviews":[],"part":{"x":610,"y":150,"heading":0,"type":"Light","name":"part198","worldx":610,"worldy":150,"outline":"black","fill":"yellow","power":0,"radius":3,"moveable":true}},{"x":570,"y":150,"heading":0,"view":{"x":570,"y":150,"heading":0,"scale":2,"points":[{"x":0,"y":0}],"outline":"black","fill":"yellow","polygon":[{"x":570,"y":150}],"minx":564,"miny":144,"maxx":576,"maxy":156,"radius":3},"scale":2,"subviews":[],"part":{"x":570,"y":150,"heading":0,"type":"Light","name":"part199","worldx":570,"worldy":150,"outline":"black","fill":"yellow","power":0,"radius":3,"moveable":true}},{"x":590,"y":150,"heading":0,"view":{"x":590,"y":150,"heading":0,"scale":2,"points":[{"x":0,"y":0}],"outline":"black","fill":"yellow","polygon":[{"x":590,"y":150}],"minx":584,"miny":144,"maxx":596,"maxy":156,"radius":3},"scale":2,"subviews":[],"part":{"x":590,"y":150,"heading":0,"type":"Light","name":"part200","worldx":590,"worldy":150,"outline":"black","fill":"yellow","power":0,"radius":3,"moveable":true}},{"x":630,"y":170,"heading":0,"view":{"x":630,"y":170,"heading":0,"scale":2,"points":[{"x":0,"y":0}],"outline":"black","fill":"yellow","polygon":[{"x":630,"y":170}],"minx":624,"miny":164,"maxx":636,"maxy":176,"radius":3},"scale":2,"subviews":[],"part":{"x":630,"y":170,"heading":0,"type":"Light","name":"part201","worldx":630,"worldy":170,"outline":"black","fill":"yellow","power":0,"radius":3,"moveable":true}},{"x":630,"y":190,"heading":0,"view":{"x":630,"y":190,"heading":0,"scale":2,"points":[{"x":0,"y":0}],"outline":"black","fill":"yellow","polygon":[{"x":630,"y":190}],"minx":624,"miny":184,"maxx":636,"maxy":196,"radius":3},"scale":2,"subviews":[],"part":{"x":630,"y":190,"heading":0,"type":"Light","name":"part202","worldx":630,"worldy":190,"outline":"black","fill":"yellow","power":0,"radius":3,"moveable":true}},{"x":650,"y":150,"heading":0,"view":{"x":650,"y":150,"heading":0,"scale":2,"points":[{"x":0,"y":0}],"outline":"black","fill":"yellow","polygon":[{"x":650,"y":150}],"minx":644,"miny":144,"maxx":656,"maxy":156,"radius":3},"scale":2,"subviews":[],"part":{"x":650,"y":150,"heading":0,"type":"Light","name":"part203","worldx":650,"worldy":150,"outline":"black","fill":"yellow","power":0,"radius":3,"moveable":true}},{"x":670,"y":150,"heading":0,"view":{"x":670,"y":150,"heading":0,"scale":2,"points":[{"x":0,"y":0}],"outline":"black","fill":"yellow","polygon":[{"x":670,"y":150}],"minx":664,"miny":144,"maxx":676,"maxy":156,"radius":3},"scale":2,"subviews":[],"part":{"x":670,"y":150,"heading":0,"type":"Light","name":"part204","worldx":670,"worldy":150,"outline":"black","fill":"yellow","power":0,"radius":3,"moveable":true}},{"x":690,"y":150,"heading":0,"view":{"x":690,"y":150,"heading":0,"scale":2,"points":[{"x":0,"y":0}],"outline":"black","fill":"yellow","polygon":[{"x":690,"y":150}],"minx":684,"miny":144,"maxx":696,"maxy":156,"radius":3},"scale":2,"subviews":[],"part":{"x":690,"y":150,"heading":0,"type":"Light","name":"part205","worldx":690,"worldy":150,"outline":"black","fill":"yellow","power":0,"radius":3,"moveable":true}},{"x":710,"y":150,"heading":0,"view":{"x":710,"y":150,"heading":0,"scale":2,"points":[{"x":0,"y":0}],"outline":"black","fill":"yellow","polygon":[{"x":710,"y":150}],"minx":704,"miny":144,"maxx":716,"maxy":156,"radius":3},"scale":2,"subviews":[],"part":{"x":710,"y":150,"heading":0,"type":"Light","name":"part206","worldx":710,"worldy":150,"outline":"black","fill":"yellow","power":0,"radius":3,"moveable":true}},{"x":730,"y":150,"heading":0,"view":{"x":730,"y":150,"heading":0,"scale":2,"points":[{"x":0,"y":0}],"outline":"black","fill":"yellow","polygon":[{"x":730,"y":150}],"minx":724,"miny":144,"maxx":736,"maxy":156,"radius":3},"scale":2,"subviews":[],"part":{"x":730,"y":150,"heading":0,"type":"Light","name":"part207","worldx":730,"worldy":150,"outline":"black","fill":"yellow","power":0,"radius":3,"moveable":true}},{"x":750,"y":150,"heading":0,"view":{"x":750,"y":150,"heading":0,"scale":2,"points":[{"x":0,"y":0}],"outline":"black","fill":"yellow","polygon":[{"x":750,"y":150}],"minx":744,"miny":144,"maxx":756,"maxy":156,"radius":3},"scale":2,"subviews":[],"part":{"x":750,"y":150,"heading":0,"type":"Light","name":"part208","worldx":750,"worldy":150,"outline":"black","fill":"yellow","power":0,"radius":3,"moveable":true}},{"x":750,"y":130,"heading":0,"view":{"x":750,"y":130,"heading":0,"scale":2,"points":[{"x":0,"y":0}],"outline":"black","fill":"yellow","polygon":[{"x":750,"y":130}],"minx":744,"miny":124,"maxx":756,"maxy":136,"radius":3},"scale":2,"subviews":[],"part":{"x":750,"y":130,"heading":0,"type":"Light","name":"part209","worldx":750,"worldy":130,"outline":"black","fill":"yellow","power":0,"radius":3,"moveable":true}},{"x":750,"y":110,"heading":0,"view":{"x":750,"y":110,"heading":0,"scale":2,"points":[{"x":0,"y":0}],"outline":"black","fill":"yellow","polygon":[{"x":750,"y":110}],"minx":744,"miny":104,"maxx":756,"maxy":116,"radius":3},"scale":2,"subviews":[],"part":{"x":750,"y":110,"heading":0,"type":"Light","name":"part210","worldx":750,"worldy":110,"outline":"black","fill":"yellow","power":0,"radius":3,"moveable":true}},{"x":750,"y":90,"heading":0,"view":{"x":750,"y":90,"heading":0,"scale":2,"points":[{"x":0,"y":0}],"outline":"black","fill":"yellow","polygon":[{"x":750,"y":90}],"minx":744,"miny":84,"maxx":756,"maxy":96,"radius":3},"scale":2,"subviews":[],"part":{"x":750,"y":90,"heading":0,"type":"Light","name":"part211","worldx":750,"worldy":90,"outline":"black","fill":"yellow","power":0,"radius":3,"moveable":true}},{"x":750,"y":70,"heading":0,"view":{"x":750,"y":70,"heading":0,"scale":2,"points":[{"x":0,"y":0}],"outline":"black","fill":"yellow","polygon":[{"x":750,"y":70}],"minx":744,"miny":64,"maxx":756,"maxy":76,"radius":3},"scale":2,"subviews":[],"part":{"x":750,"y":70,"heading":0,"type":"Light","name":"part212","worldx":750,"worldy":70,"outline":"black","fill":"yellow","power":0,"radius":3,"moveable":true}},{"x":630,"y":210,"heading":0,"view":{"x":630,"y":210,"heading":0,"scale":2,"points":[{"x":0,"y":0}],"outline":"black","fill":"yellow","polygon":[{"x":630,"y":210}],"minx":624,"miny":204,"maxx":636,"maxy":216,"radius":3},"scale":2,"subviews":[],"part":{"x":630,"y":210,"heading":0,"type":"Light","name":"part213","worldx":630,"worldy":210,"outline":"black","fill":"yellow","power":0,"radius":3,"moveable":true}},{"x":630,"y":230,"heading":0,"view":{"x":630,"y":230,"heading":0,"scale":2,"points":[{"x":0,"y":0}],"outline":"black","fill":"yellow","polygon":[{"x":630,"y":230}],"minx":624,"miny":224,"maxx":636,"maxy":236,"radius":3},"scale":2,"subviews":[],"part":{"x":630,"y":230,"heading":0,"type":"Light","name":"part214","worldx":630,"worldy":230,"outline":"black","fill":"yellow","power":0,"radius":3,"moveable":true}},{"x":650,"y":230,"heading":0,"view":{"x":650,"y":230,"heading":0,"scale":2,"points":[{"x":0,"y":0}],"outline":"black","fill":"yellow","polygon":[{"x":650,"y":230}],"minx":644,"miny":224,"maxx":656,"maxy":236,"radius":3},"scale":2,"subviews":[],"part":{"x":650,"y":230,"heading":0,"type":"Light","name":"part215","worldx":650,"worldy":230,"outline":"black","fill":"yellow","power":0,"radius":3,"moveable":true}},{"x":670,"y":230,"heading":0,"view":{"x":670,"y":230,"heading":0,"scale":2,"points":[{"x":0,"y":0}],"outline":"black","fill":"yellow","polygon":[{"x":670,"y":230}],"minx":664,"miny":224,"maxx":676,"maxy":236,"radius":3},"scale":2,"subviews":[],"part":{"x":670,"y":230,"heading":0,"type":"Light","name":"part216","worldx":670,"worldy":230,"outline":"black","fill":"yellow","power":0,"radius":3,"moveable":true}},{"x":210,"y":261,"heading":0,"view":{"x":210,"y":261,"heading":0,"scale":2,"points":[{"x":-5,"y":-40},{"x":15,"y":-40},{"x":15,"y":10},{"x":-5,"y":10}],"outline":"blue","fill":"blue","polygon":[{"x":200,"y":181},{"x":240,"y":181},{"x":240,"y":281},{"x":200,"y":281}],"minx":200,"miny":181,"maxx":240,"maxy":281,"stroke":"black"},"scale":2,"subviews":[],"part":{"x":210,"y":261,"heading":0,"type":"Wall","name":"part41","worldx":220,"worldy":231,"outline":"blue","fill":"blue","power":0,"rotated":false,"moveable":false,"resizeFactor":1,"resizeFactorHeight":4,"resizeFactorWidth":3}},{"x":571,"y":259,"heading":0,"view":{"x":571,"y":259,"heading":0,"scale":2,"points":[{"x":-5,"y":-40},{"x":15,"y":-40},{"x":15,"y":10},{"x":-5,"y":10}],"outline":"blue","fill":"blue","polygon":[{"x":561,"y":179},{"x":601,"y":179},{"x":601,"y":279},{"x":561,"y":279}],"minx":561,"miny":179,"maxx":601,"maxy":279,"stroke":"black"},"scale":2,"subviews":[],"part":{"x":571,"y":259,"heading":0,"type":"Wall","name":"part29","worldx":581,"worldy":229,"outline":"blue","fill":"blue","power":0,"rotated":false,"moveable":false,"resizeFactor":1,"resizeFactorHeight":4,"resizeFactorWidth":3}},{"x":690,"y":230,"heading":0,"view":{"x":690,"y":230,"heading":0,"scale":2,"points":[{"x":0,"y":0}],"outline":"black","fill":"yellow","polygon":[{"x":690,"y":230}],"minx":684,"miny":224,"maxx":696,"maxy":236,"radius":3},"scale":2,"subviews":[],"part":{"x":690,"y":230,"heading":0,"type":"Light","name":"part217","worldx":690,"worldy":230,"outline":"black","fill":"yellow","power":0,"radius":3,"moveable":true}},{"x":570,"y":310,"heading":0,"view":{"x":570,"y":310,"heading":0,"scale":2,"points":[{"x":0,"y":0}],"outline":"black","fill":"yellow","polygon":[{"x":570,"y":310}],"minx":564,"miny":304,"maxx":576,"maxy":316,"radius":3},"scale":2,"subviews":[],"part":{"x":570,"y":310,"heading":0,"type":"Light","name":"part218","worldx":570,"worldy":310,"outline":"black","fill":"yellow","power":0,"radius":3,"moveable":true}},{"x":590,"y":310,"heading":0,"view":{"x":590,"y":310,"heading":0,"scale":2,"points":[{"x":0,"y":0}],"outline":"black","fill":"yellow","polygon":[{"x":590,"y":310}],"minx":584,"miny":304,"maxx":596,"maxy":316,"radius":3},"scale":2,"subviews":[],"part":{"x":590,"y":310,"heading":0,"type":"Light","name":"part219","worldx":590,"worldy":310,"outline":"black","fill":"yellow","power":0,"radius":3,"moveable":true}},{"x":610,"y":310,"heading":0,"view":{"x":610,"y":310,"heading":0,"scale":2,"points":[{"x":0,"y":0}],"outline":"black","fill":"yellow","polygon":[{"x":610,"y":310}],"minx":604,"miny":304,"maxx":616,"maxy":316,"radius":3},"scale":2,"subviews":[],"part":{"x":610,"y":310,"heading":0,"type":"Light","name":"part220","worldx":610,"worldy":310,"outline":"black","fill":"yellow","power":0,"radius":3,"moveable":true}},{"x":630,"y":310,"heading":0,"view":{"x":630,"y":310,"heading":0,"scale":2,"points":[{"x":0,"y":0}],"outline":"black","fill":"yellow","polygon":[{"x":630,"y":310}],"minx":624,"miny":304,"maxx":636,"maxy":316,"radius":3},"scale":2,"subviews":[],"part":{"x":630,"y":310,"heading":0,"type":"Light","name":"part221","worldx":630,"worldy":310,"outline":"black","fill":"yellow","power":0,"radius":3,"moveable":true}},{"x":630,"y":290,"heading":0,"view":{"x":630,"y":290,"heading":0,"scale":2,"points":[{"x":0,"y":0}],"outline":"black","fill":"yellow","polygon":[{"x":630,"y":290}],"minx":624,"miny":284,"maxx":636,"maxy":296,"radius":3},"scale":2,"subviews":[],"part":{"x":630,"y":290,"heading":0,"type":"Light","name":"part222","worldx":630,"worldy":290,"outline":"black","fill":"yellow","power":0,"radius":3,"moveable":true}},{"x":630,"y":270,"heading":0,"view":{"x":630,"y":270,"heading":0,"scale":2,"points":[{"x":0,"y":0}],"outline":"black","fill":"yellow","polygon":[{"x":630,"y":270}],"minx":624,"miny":264,"maxx":636,"maxy":276,"radius":3},"scale":2,"subviews":[],"part":{"x":630,"y":270,"heading":0,"type":"Light","name":"part223","worldx":630,"worldy":270,"outline":"black","fill":"yellow","power":0,"radius":3,"moveable":true}},{"x":630,"y":250,"heading":0,"view":{"x":630,"y":250,"heading":0,"scale":2,"points":[{"x":0,"y":0}],"outline":"black","fill":"yellow","polygon":[{"x":630,"y":250}],"minx":624,"miny":244,"maxx":636,"maxy":256,"radius":3},"scale":2,"subviews":[],"part":{"x":630,"y":250,"heading":0,"type":"Light","name":"part224","worldx":630,"worldy":250,"outline":"black","fill":"yellow","power":0,"radius":3,"moveable":true}},{"x":710,"y":230,"heading":0,"view":{"x":710,"y":230,"heading":0,"scale":2,"points":[{"x":0,"y":0}],"outline":"black","fill":"yellow","polygon":[{"x":710,"y":230}],"minx":704,"miny":224,"maxx":716,"maxy":236,"radius":3},"scale":2,"subviews":[],"part":{"x":710,"y":230,"heading":0,"type":"Light","name":"part225","worldx":710,"worldy":230,"outline":"black","fill":"yellow","power":0,"radius":3,"moveable":true}},{"x":730,"y":230,"heading":0,"view":{"x":730,"y":230,"heading":0,"scale":2,"points":[{"x":0,"y":0}],"outline":"black","fill":"yellow","polygon":[{"x":730,"y":230}],"minx":724,"miny":224,"maxx":736,"maxy":236,"radius":3},"scale":2,"subviews":[],"part":{"x":730,"y":230,"heading":0,"type":"Light","name":"part226","worldx":730,"worldy":230,"outline":"black","fill":"yellow","power":0,"radius":3,"moveable":true}},{"x":750,"y":230,"heading":0,"view":{"x":750,"y":230,"heading":0,"scale":2,"points":[{"x":0,"y":0}],"outline":"black","fill":"yellow","polygon":[{"x":750,"y":230}],"minx":744,"miny":224,"maxx":756,"maxy":236,"radius":3},"scale":2,"subviews":[],"part":{"x":750,"y":230,"heading":0,"type":"Light","name":"part227","worldx":750,"worldy":230,"outline":"black","fill":"yellow","power":0,"radius":3,"moveable":true}},{"x":750,"y":210,"heading":0,"view":{"x":750,"y":210,"heading":0,"scale":2,"points":[{"x":0,"y":0}],"outline":"black","fill":"yellow","polygon":[{"x":750,"y":210}],"minx":744,"miny":204,"maxx":756,"maxy":216,"radius":3},"scale":2,"subviews":[],"part":{"x":750,"y":210,"heading":0,"type":"Light","name":"part228","worldx":750,"worldy":210,"outline":"black","fill":"yellow","power":0,"radius":3,"moveable":true}},{"x":750,"y":190,"heading":0,"view":{"x":750,"y":190,"heading":0,"scale":2,"points":[{"x":0,"y":0}],"outline":"black","fill":"yellow","polygon":[{"x":750,"y":190}],"minx":744,"miny":184,"maxx":756,"maxy":196,"radius":3},"scale":2,"subviews":[],"part":{"x":750,"y":190,"heading":0,"type":"Light","name":"part229","worldx":750,"worldy":190,"outline":"black","fill":"yellow","power":0,"radius":3,"moveable":true}},{"x":750,"y":170,"heading":0,"view":{"x":750,"y":170,"heading":0,"scale":2,"points":[{"x":0,"y":0}],"outline":"black","fill":"yellow","polygon":[{"x":750,"y":170}],"minx":744,"miny":164,"maxx":756,"maxy":176,"radius":3},"scale":2,"subviews":[],"part":{"x":750,"y":170,"heading":0,"type":"Light","name":"part230","worldx":750,"worldy":170,"outline":"black","fill":"yellow","power":0,"radius":3,"moveable":true}},{"x":630,"y":330,"heading":0,"view":{"x":630,"y":330,"heading":0,"scale":2,"points":[{"x":0,"y":0}],"outline":"black","fill":"yellow","polygon":[{"x":630,"y":330}],"minx":624,"miny":324,"maxx":636,"maxy":336,"radius":3},"scale":2,"subviews":[],"part":{"x":630,"y":330,"heading":0,"type":"Light","name":"part231","worldx":630,"worldy":330,"outline":"black","fill":"yellow","power":0,"radius":3,"moveable":true}},{"x":630,"y":350,"heading":0,"view":{"x":630,"y":350,"heading":0,"scale":2,"points":[{"x":0,"y":0}],"outline":"black","fill":"yellow","polygon":[{"x":630,"y":350}],"minx":624,"miny":344,"maxx":636,"maxy":356,"radius":3},"scale":2,"subviews":[],"part":{"x":630,"y":350,"heading":0,"type":"Light","name":"part232","worldx":630,"worldy":350,"outline":"black","fill":"yellow","power":0,"radius":3,"moveable":true}},{"x":630,"y":370,"heading":0,"view":{"x":630,"y":370,"heading":0,"scale":2,"points":[{"x":0,"y":0}],"outline":"black","fill":"yellow","polygon":[{"x":630,"y":370}],"minx":624,"miny":364,"maxx":636,"maxy":376,"radius":3},"scale":2,"subviews":[],"part":{"x":630,"y":370,"heading":0,"type":"Light","name":"part233","worldx":630,"worldy":370,"outline":"black","fill":"yellow","power":0,"radius":3,"moveable":true}},{"x":630,"y":390,"heading":0,"view":{"x":630,"y":390,"heading":0,"scale":2,"points":[{"x":0,"y":0}],"outline":"black","fill":"yellow","polygon":[{"x":630,"y":390}],"minx":624,"miny":384,"maxx":636,"maxy":396,"radius":3},"scale":2,"subviews":[],"part":{"x":630,"y":390,"heading":0,"type":"Light","name":"part234","worldx":630,"worldy":390,"outline":"black","fill":"yellow","power":0,"radius":3,"moveable":true}},{"x":650,"y":390,"heading":0,"view":{"x":650,"y":390,"heading":0,"scale":2,"points":[{"x":0,"y":0}],"outline":"black","fill":"yellow","polygon":[{"x":650,"y":390}],"minx":644,"miny":384,"maxx":656,"maxy":396,"radius":3},"scale":2,"subviews":[],"part":{"x":650,"y":390,"heading":0,"type":"Light","name":"part235","worldx":650,"worldy":390,"outline":"black","fill":"yellow","power":0,"radius":3,"moveable":true}},{"x":670,"y":390,"heading":0,"view":{"x":670,"y":390,"heading":0,"scale":2,"points":[{"x":0,"y":0}],"outline":"black","fill":"yellow","polygon":[{"x":670,"y":390}],"minx":664,"miny":384,"maxx":676,"maxy":396,"radius":3},"scale":2,"subviews":[],"part":{"x":670,"y":390,"heading":0,"type":"Light","name":"part236","worldx":670,"worldy":390,"outline":"black","fill":"yellow","power":0,"radius":3,"moveable":true}},{"x":690,"y":390,"heading":0,"view":{"x":690,"y":390,"heading":0,"scale":2,"points":[{"x":0,"y":0}],"outline":"black","fill":"yellow","polygon":[{"x":690,"y":390}],"minx":684,"miny":384,"maxx":696,"maxy":396,"radius":3},"scale":2,"subviews":[],"part":{"x":690,"y":390,"heading":0,"type":"Light","name":"part237","worldx":690,"worldy":390,"outline":"black","fill":"yellow","power":0,"radius":3,"moveable":true}},{"x":710,"y":390,"heading":0,"view":{"x":710,"y":390,"heading":0,"scale":2,"points":[{"x":0,"y":0}],"outline":"black","fill":"yellow","polygon":[{"x":710,"y":390}],"minx":704,"miny":384,"maxx":716,"maxy":396,"radius":3},"scale":2,"subviews":[],"part":{"x":710,"y":390,"heading":0,"type":"Light","name":"part238","worldx":710,"worldy":390,"outline":"black","fill":"yellow","power":0,"radius":3,"moveable":true}},{"x":730,"y":390,"heading":0,"view":{"x":730,"y":390,"heading":0,"scale":2,"points":[{"x":0,"y":0}],"outline":"black","fill":"yellow","polygon":[{"x":730,"y":390}],"minx":724,"miny":384,"maxx":736,"maxy":396,"radius":3},"scale":2,"subviews":[],"part":{"x":730,"y":390,"heading":0,"type":"Light","name":"part239","worldx":730,"worldy":390,"outline":"black","fill":"yellow","power":0,"radius":3,"moveable":true}},{"x":750,"y":390,"heading":0,"view":{"x":750,"y":390,"heading":0,"scale":2,"points":[{"x":0,"y":0}],"outline":"black","fill":"yellow","polygon":[{"x":750,"y":390}],"minx":744,"miny":384,"maxx":756,"maxy":396,"radius":3},"scale":2,"subviews":[],"part":{"x":750,"y":390,"heading":0,"type":"Light","name":"part240","worldx":750,"worldy":390,"outline":"black","fill":"yellow","power":0,"radius":3,"moveable":true}},{"x":750,"y":410,"heading":0,"view":{"x":750,"y":410,"heading":0,"scale":2,"points":[{"x":0,"y":0}],"outline":"black","fill":"yellow","polygon":[{"x":750,"y":410}],"minx":744,"miny":404,"maxx":756,"maxy":416,"radius":3},"scale":2,"subviews":[],"part":{"x":750,"y":410,"heading":0,"type":"Light","name":"part241","worldx":750,"worldy":410,"outline":"black","fill":"yellow","power":0,"radius":3,"moveable":true}},{"x":750,"y":430,"heading":0,"view":{"x":750,"y":430,"heading":0,"scale":2,"points":[{"x":0,"y":0}],"outline":"black","fill":"yellow","polygon":[{"x":750,"y":430}],"minx":744,"miny":424,"maxx":756,"maxy":436,"radius":3},"scale":2,"subviews":[],"part":{"x":750,"y":430,"heading":0,"type":"Light","name":"part242","worldx":750,"worldy":430,"outline":"black","fill":"yellow","power":0,"radius":3,"moveable":true}},{"x":530,"y":450,"heading":0,"view":{"x":530,"y":450,"heading":0,"scale":2,"points":[{"x":0,"y":0}],"outline":"black","fill":"yellow","polygon":[{"x":530,"y":450}],"minx":524,"miny":444,"maxx":536,"maxy":456,"radius":3},"scale":2,"subviews":[],"part":{"x":530,"y":450,"heading":0,"type":"Light","name":"part243","worldx":530,"worldy":450,"outline":"black","fill":"yellow","power":0,"radius":3,"moveable":true}},{"x":530,"y":470,"heading":0,"view":{"x":530,"y":470,"heading":0,"scale":2,"points":[{"x":0,"y":0}],"outline":"black","fill":"yellow","polygon":[{"x":530,"y":470}],"minx":524,"miny":464,"maxx":536,"maxy":476,"radius":3},"scale":2,"subviews":[],"part":{"x":530,"y":470,"heading":0,"type":"Light","name":"part244","worldx":530,"worldy":470,"outline":"black","fill":"yellow","power":0,"radius":3,"moveable":true}},{"x":550,"y":470,"heading":0,"view":{"x":550,"y":470,"heading":0,"scale":2,"points":[{"x":0,"y":0}],"outline":"black","fill":"yellow","polygon":[{"x":550,"y":470}],"minx":544,"miny":464,"maxx":556,"maxy":476,"radius":3},"scale":2,"subviews":[],"part":{"x":550,"y":470,"heading":0,"type":"Light","name":"part245","worldx":550,"worldy":470,"outline":"black","fill":"yellow","power":0,"radius":3,"moveable":true}},{"x":510,"y":470,"heading":0,"view":{"x":510,"y":470,"heading":0,"scale":2,"points":[{"x":0,"y":0}],"outline":"black","fill":"yellow","polygon":[{"x":510,"y":470}],"minx":504,"miny":464,"maxx":516,"maxy":476,"radius":3},"scale":2,"subviews":[],"part":{"x":510,"y":470,"heading":0,"type":"Light","name":"part246","worldx":510,"worldy":470,"outline":"black","fill":"yellow","power":0,"radius":3,"moveable":true}},{"x":490,"y":470,"heading":0,"view":{"x":490,"y":470,"heading":0,"scale":2,"points":[{"x":0,"y":0}],"outline":"black","fill":"yellow","polygon":[{"x":490,"y":470}],"minx":484,"miny":464,"maxx":496,"maxy":476,"radius":3},"scale":2,"subviews":[],"part":{"x":490,"y":470,"heading":0,"type":"Light","name":"part247","worldx":490,"worldy":470,"outline":"black","fill":"yellow","power":0,"radius":3,"moveable":true}},{"x":470,"y":470,"heading":0,"view":{"x":470,"y":470,"heading":0,"scale":2,"points":[{"x":0,"y":0}],"outline":"black","fill":"yellow","polygon":[{"x":470,"y":470}],"minx":464,"miny":464,"maxx":476,"maxy":476,"radius":3},"scale":2,"subviews":[],"part":{"x":470,"y":470,"heading":0,"type":"Light","name":"part248","worldx":470,"worldy":470,"outline":"black","fill":"yellow","power":0,"radius":3,"moveable":true}},{"x":450,"y":470,"heading":0,"view":{"x":450,"y":470,"heading":0,"scale":2,"points":[{"x":0,"y":0}],"outline":"black","fill":"yellow","polygon":[{"x":450,"y":470}],"minx":444,"miny":464,"maxx":456,"maxy":476,"radius":3},"scale":2,"subviews":[],"part":{"x":450,"y":470,"heading":0,"type":"Light","name":"part249","worldx":450,"worldy":470,"outline":"black","fill":"yellow","power":0,"radius":3,"moveable":true}},{"x":450,"y":490,"heading":0,"view":{"x":450,"y":490,"heading":0,"scale":2,"points":[{"x":0,"y":0}],"outline":"black","fill":"yellow","polygon":[{"x":450,"y":490}],"minx":444,"miny":484,"maxx":456,"maxy":496,"radius":3},"scale":2,"subviews":[],"part":{"x":450,"y":490,"heading":0,"type":"Light","name":"part250","worldx":450,"worldy":490,"outline":"black","fill":"yellow","power":0,"radius":3,"moveable":true}},{"x":570,"y":470,"heading":0,"view":{"x":570,"y":470,"heading":0,"scale":2,"points":[{"x":0,"y":0}],"outline":"black","fill":"yellow","polygon":[{"x":570,"y":470}],"minx":564,"miny":464,"maxx":576,"maxy":476,"radius":3},"scale":2,"subviews":[],"part":{"x":570,"y":470,"heading":0,"type":"Light","name":"part251","worldx":570,"worldy":470,"outline":"black","fill":"yellow","power":0,"radius":3,"moveable":true}},{"x":590,"y":470,"heading":0,"view":{"x":590,"y":470,"heading":0,"scale":2,"points":[{"x":0,"y":0}],"outline":"black","fill":"yellow","polygon":[{"x":590,"y":470}],"minx":584,"miny":464,"maxx":596,"maxy":476,"radius":3},"scale":2,"subviews":[],"part":{"x":590,"y":470,"heading":0,"type":"Light","name":"part252","worldx":590,"worldy":470,"outline":"black","fill":"yellow","power":0,"radius":3,"moveable":true}},{"x":610,"y":470,"heading":0,"view":{"x":610,"y":470,"heading":0,"scale":2,"points":[{"x":0,"y":0}],"outline":"black","fill":"yellow","polygon":[{"x":610,"y":470}],"minx":604,"miny":464,"maxx":616,"maxy":476,"radius":3},"scale":2,"subviews":[],"part":{"x":610,"y":470,"heading":0,"type":"Light","name":"part253","worldx":610,"worldy":470,"outline":"black","fill":"yellow","power":0,"radius":3,"moveable":true}},{"x":630,"y":470,"heading":0,"view":{"x":630,"y":470,"heading":0,"scale":2,"points":[{"x":0,"y":0}],"outline":"black","fill":"yellow","polygon":[{"x":630,"y":470}],"minx":624,"miny":464,"maxx":636,"maxy":476,"radius":3},"scale":2,"subviews":[],"part":{"x":630,"y":470,"heading":0,"type":"Light","name":"part254","worldx":630,"worldy":470,"outline":"black","fill":"yellow","power":0,"radius":3,"moveable":true}},{"x":650,"y":470,"heading":0,"view":{"x":650,"y":470,"heading":0,"scale":2,"points":[{"x":0,"y":0}],"outline":"black","fill":"yellow","polygon":[{"x":650,"y":470}],"minx":644,"miny":464,"maxx":656,"maxy":476,"radius":3},"scale":2,"subviews":[],"part":{"x":650,"y":470,"heading":0,"type":"Light","name":"part255","worldx":650,"worldy":470,"outline":"black","fill":"yellow","power":0,"radius":3,"moveable":true}},{"x":670,"y":470,"heading":0,"view":{"x":670,"y":470,"heading":0,"scale":2,"points":[{"x":0,"y":0}],"outline":"black","fill":"yellow","polygon":[{"x":670,"y":470}],"minx":664,"miny":464,"maxx":676,"maxy":476,"radius":3},"scale":2,"subviews":[],"part":{"x":670,"y":470,"heading":0,"type":"Light","name":"part256","worldx":670,"worldy":470,"outline":"black","fill":"yellow","power":0,"radius":3,"moveable":true}},{"x":690,"y":470,"heading":0,"view":{"x":690,"y":470,"heading":0,"scale":2,"points":[{"x":0,"y":0}],"outline":"black","fill":"yellow","polygon":[{"x":690,"y":470}],"minx":684,"miny":464,"maxx":696,"maxy":476,"radius":3},"scale":2,"subviews":[],"part":{"x":690,"y":470,"heading":0,"type":"Light","name":"part257","worldx":690,"worldy":470,"outline":"black","fill":"yellow","power":0,"radius":3,"moveable":true}},{"x":710,"y":470,"heading":0,"view":{"x":710,"y":470,"heading":0,"scale":2,"points":[{"x":0,"y":0}],"outline":"black","fill":"yellow","polygon":[{"x":710,"y":470}],"minx":704,"miny":464,"maxx":716,"maxy":476,"radius":3},"scale":2,"subviews":[],"part":{"x":710,"y":470,"heading":0,"type":"Light","name":"part258","worldx":710,"worldy":470,"outline":"black","fill":"yellow","power":0,"radius":3,"moveable":true}},{"x":730,"y":470,"heading":0,"view":{"x":730,"y":470,"heading":0,"scale":2,"points":[{"x":0,"y":0}],"outline":"black","fill":"yellow","polygon":[{"x":730,"y":470}],"minx":724,"miny":464,"maxx":736,"maxy":476,"radius":3},"scale":2,"subviews":[],"part":{"x":730,"y":470,"heading":0,"type":"Light","name":"part259","worldx":730,"worldy":470,"outline":"black","fill":"yellow","power":0,"radius":3,"moveable":true}},{"x":750,"y":470,"heading":0,"view":{"x":750,"y":470,"heading":0,"scale":2,"points":[{"x":0,"y":0}],"outline":"black","fill":"yellow","polygon":[{"x":750,"y":470}],"minx":744,"miny":464,"maxx":756,"maxy":476,"radius":3},"scale":2,"subviews":[],"part":{"x":750,"y":470,"heading":0,"type":"Light","name":"part260","worldx":750,"worldy":470,"outline":"black","fill":"yellow","power":0,"radius":3,"moveable":true}},{"x":750,"y":450,"heading":0,"view":{"x":750,"y":450,"heading":0,"scale":2,"points":[{"x":0,"y":0}],"outline":"black","fill":"yellow","polygon":[{"x":750,"y":450}],"minx":744,"miny":444,"maxx":756,"maxy":456,"radius":3},"scale":2,"subviews":[],"part":{"x":750,"y":450,"heading":0,"type":"Light","name":"part261","worldx":750,"worldy":450,"outline":"black","fill":"yellow","power":0,"radius":3,"moveable":true}},{"x":750,"y":490,"heading":0,"view":{"x":750,"y":490,"heading":0,"scale":2,"points":[{"x":0,"y":0}],"outline":"black","fill":"yellow","polygon":[{"x":750,"y":490}],"minx":744,"miny":484,"maxx":756,"maxy":496,"radius":3},"scale":2,"subviews":[],"part":{"x":750,"y":490,"heading":0,"type":"Light","name":"part262","worldx":750,"worldy":490,"outline":"black","fill":"yellow","power":0,"radius":3,"moveable":true}},{"x":450,"y":510,"heading":0,"view":{"x":450,"y":510,"heading":0,"scale":2,"points":[{"x":0,"y":0}],"outline":"black","fill":"yellow","polygon":[{"x":450,"y":510}],"minx":444,"miny":504,"maxx":456,"maxy":516,"radius":3},"scale":2,"subviews":[],"part":{"x":450,"y":510,"heading":0,"type":"Light","name":"part263","worldx":450,"worldy":510,"outline":"black","fill":"yellow","power":0,"radius":3,"moveable":true}},{"x":450,"y":530,"heading":0,"view":{"x":450,"y":530,"heading":0,"scale":2,"points":[{"x":0,"y":0}],"outline":"black","fill":"yellow","polygon":[{"x":450,"y":530}],"minx":444,"miny":524,"maxx":456,"maxy":536,"radius":3},"scale":2,"subviews":[],"part":{"x":450,"y":530,"heading":0,"type":"Light","name":"part264","worldx":450,"worldy":530,"outline":"black","fill":"yellow","power":0,"radius":3,"moveable":true}},{"x":450,"y":550,"heading":0,"view":{"x":450,"y":550,"heading":0,"scale":2,"points":[{"x":0,"y":0}],"outline":"black","fill":"yellow","polygon":[{"x":450,"y":550}],"minx":444,"miny":544,"maxx":456,"maxy":556,"radius":3},"scale":2,"subviews":[],"part":{"x":450,"y":550,"heading":0,"type":"Light","name":"part265","worldx":450,"worldy":550,"outline":"black","fill":"yellow","power":0,"radius":3,"moveable":true}},{"x":470,"y":550,"heading":0,"view":{"x":470,"y":550,"heading":0,"scale":2,"points":[{"x":0,"y":0}],"outline":"black","fill":"yellow","polygon":[{"x":470,"y":550}],"minx":464,"miny":544,"maxx":476,"maxy":556,"radius":3},"scale":2,"subviews":[],"part":{"x":470,"y":550,"heading":0,"type":"Light","name":"part266","worldx":470,"worldy":550,"outline":"black","fill":"yellow","power":0,"radius":3,"moveable":true}},{"x":490,"y":550,"heading":0,"view":{"x":490,"y":550,"heading":0,"scale":2,"points":[{"x":0,"y":0}],"outline":"black","fill":"yellow","polygon":[{"x":490,"y":550}],"minx":484,"miny":544,"maxx":496,"maxy":556,"radius":3},"scale":2,"subviews":[],"part":{"x":490,"y":550,"heading":0,"type":"Light","name":"part267","worldx":490,"worldy":550,"outline":"black","fill":"yellow","power":0,"radius":3,"moveable":true}},{"x":510,"y":550,"heading":0,"view":{"x":510,"y":550,"heading":0,"scale":2,"points":[{"x":0,"y":0}],"outline":"black","fill":"yellow","polygon":[{"x":510,"y":550}],"minx":504,"miny":544,"maxx":516,"maxy":556,"radius":3},"scale":2,"subviews":[],"part":{"x":510,"y":550,"heading":0,"type":"Light","name":"part268","worldx":510,"worldy":550,"outline":"black","fill":"yellow","power":0,"radius":3,"moveable":true}},{"x":530,"y":550,"heading":0,"view":{"x":530,"y":550,"heading":0,"scale":2,"points":[{"x":0,"y":0}],"outline":"black","fill":"yellow","polygon":[{"x":530,"y":550}],"minx":524,"miny":544,"maxx":536,"maxy":556,"radius":3},"scale":2,"subviews":[],"part":{"x":530,"y":550,"heading":0,"type":"Light","name":"part269","worldx":530,"worldy":550,"outline":"black","fill":"yellow","power":0,"radius":3,"moveable":true}},{"x":550,"y":550,"heading":0,"view":{"x":550,"y":550,"heading":0,"scale":2,"points":[{"x":0,"y":0}],"outline":"black","fill":"yellow","polygon":[{"x":550,"y":550}],"minx":544,"miny":544,"maxx":556,"maxy":556,"radius":3},"scale":2,"subviews":[],"part":{"x":550,"y":550,"heading":0,"type":"Light","name":"part270","worldx":550,"worldy":550,"outline":"black","fill":"yellow","power":0,"radius":3,"moveable":true}},{"x":570,"y":550,"heading":0,"view":{"x":570,"y":550,"heading":0,"scale":2,"points":[{"x":0,"y":0}],"outline":"black","fill":"yellow","polygon":[{"x":570,"y":550}],"minx":564,"miny":544,"maxx":576,"maxy":556,"radius":3},"scale":2,"subviews":[],"part":{"x":570,"y":550,"heading":0,"type":"Light","name":"part271","worldx":570,"worldy":550,"outline":"black","fill":"yellow","power":0,"radius":3,"moveable":true}},{"x":590,"y":550,"heading":0,"view":{"x":590,"y":550,"heading":0,"scale":2,"points":[{"x":0,"y":0}],"outline":"black","fill":"yellow","polygon":[{"x":590,"y":550}],"minx":584,"miny":544,"maxx":596,"maxy":556,"radius":3},"scale":2,"subviews":[],"part":{"x":590,"y":550,"heading":0,"type":"Light","name":"part272","worldx":590,"worldy":550,"outline":"black","fill":"yellow","power":0,"radius":3,"moveable":true}},{"x":610,"y":550,"heading":0,"view":{"x":610,"y":550,"heading":0,"scale":2,"points":[{"x":0,"y":0}],"outline":"black","fill":"yellow","polygon":[{"x":610,"y":550}],"minx":604,"miny":544,"maxx":616,"maxy":556,"radius":3},"scale":2,"subviews":[],"part":{"x":610,"y":550,"heading":0,"type":"Light","name":"part273","worldx":610,"worldy":550,"outline":"black","fill":"yellow","power":0,"radius":3,"moveable":true}},{"x":630,"y":550,"heading":0,"view":{"x":630,"y":550,"heading":0,"scale":2,"points":[{"x":0,"y":0}],"outline":"black","fill":"yellow","polygon":[{"x":630,"y":550}],"minx":624,"miny":544,"maxx":636,"maxy":556,"radius":3},"scale":2,"subviews":[],"part":{"x":630,"y":550,"heading":0,"type":"Light","name":"part274","worldx":630,"worldy":550,"outline":"black","fill":"yellow","power":0,"radius":3,"moveable":true}},{"x":650,"y":550,"heading":0,"view":{"x":650,"y":550,"heading":0,"scale":2,"points":[{"x":0,"y":0}],"outline":"black","fill":"yellow","polygon":[{"x":650,"y":550}],"minx":644,"miny":544,"maxx":656,"maxy":556,"radius":3},"scale":2,"subviews":[],"part":{"x":650,"y":550,"heading":0,"type":"Light","name":"part275","worldx":650,"worldy":550,"outline":"black","fill":"yellow","power":0,"radius":3,"moveable":true}},{"x":670,"y":550,"heading":0,"view":{"x":670,"y":550,"heading":0,"scale":2,"points":[{"x":0,"y":0}],"outline":"black","fill":"yellow","polygon":[{"x":670,"y":550}],"minx":664,"miny":544,"maxx":676,"maxy":556,"radius":3},"scale":2,"subviews":[],"part":{"x":670,"y":550,"heading":0,"type":"Light","name":"part276","worldx":670,"worldy":550,"outline":"black","fill":"yellow","power":0,"radius":3,"moveable":true}},{"x":690,"y":550,"heading":0,"view":{"x":690,"y":550,"heading":0,"scale":2,"points":[{"x":0,"y":0}],"outline":"black","fill":"yellow","polygon":[{"x":690,"y":550}],"minx":684,"miny":544,"maxx":696,"maxy":556,"radius":3},"scale":2,"subviews":[],"part":{"x":690,"y":550,"heading":0,"type":"Light","name":"part277","worldx":690,"worldy":550,"outline":"black","fill":"yellow","power":0,"radius":3,"moveable":true}},{"x":710,"y":550,"heading":0,"view":{"x":710,"y":550,"heading":0,"scale":2,"points":[{"x":0,"y":0}],"outline":"black","fill":"yellow","polygon":[{"x":710,"y":550}],"minx":704,"miny":544,"maxx":716,"maxy":556,"radius":3},"scale":2,"subviews":[],"part":{"x":710,"y":550,"heading":0,"type":"Light","name":"part278","worldx":710,"worldy":550,"outline":"black","fill":"yellow","power":0,"radius":3,"moveable":true}},{"x":730,"y":550,"heading":0,"view":{"x":730,"y":550,"heading":0,"scale":2,"points":[{"x":0,"y":0}],"outline":"black","fill":"yellow","polygon":[{"x":730,"y":550}],"minx":724,"miny":544,"maxx":736,"maxy":556,"radius":3},"scale":2,"subviews":[],"part":{"x":730,"y":550,"heading":0,"type":"Light","name":"part279","worldx":730,"worldy":550,"outline":"black","fill":"yellow","power":0,"radius":3,"moveable":true}},{"x":750,"y":550,"heading":0,"view":{"x":750,"y":550,"heading":0,"scale":2,"points":[{"x":0,"y":0}],"outline":"black","fill":"yellow","polygon":[{"x":750,"y":550}],"minx":744,"miny":544,"maxx":756,"maxy":556,"radius":3},"scale":2,"subviews":[],"part":{"x":750,"y":550,"heading":0,"type":"Light","name":"part280","worldx":750,"worldy":550,"outline":"black","fill":"yellow","power":0,"radius":3,"moveable":true}},{"x":750,"y":530,"heading":0,"view":{"x":750,"y":530,"heading":0,"scale":2,"points":[{"x":0,"y":0}],"outline":"black","fill":"yellow","polygon":[{"x":750,"y":530}],"minx":744,"miny":524,"maxx":756,"maxy":536,"radius":3},"scale":2,"subviews":[],"part":{"x":750,"y":530,"heading":0,"type":"Light","name":"part281","worldx":750,"worldy":530,"outline":"black","fill":"yellow","power":0,"radius":3,"moveable":true}},{"x":750,"y":510,"heading":0,"view":{"x":750,"y":510,"heading":0,"scale":2,"points":[{"x":0,"y":0}],"outline":"black","fill":"yellow","polygon":[{"x":750,"y":510}],"minx":744,"miny":504,"maxx":756,"maxy":516,"radius":3},"scale":2,"subviews":[],"part":{"x":750,"y":510,"heading":0,"type":"Light","name":"part282","worldx":750,"worldy":510,"outline":"black","fill":"yellow","power":0,"radius":3,"moveable":true}},{"x":430,"y":550,"heading":0,"view":{"x":430,"y":550,"heading":0,"scale":2,"points":[{"x":0,"y":0}],"outline":"black","fill":"yellow","polygon":[{"x":430,"y":550}],"minx":424,"miny":544,"maxx":436,"maxy":556,"radius":3},"scale":2,"subviews":[],"part":{"x":430,"y":550,"heading":0,"type":"Light","name":"part283","worldx":430,"worldy":550,"outline":"black","fill":"yellow","power":0,"radius":3,"moveable":true}},{"x":350,"y":470,"heading":0,"view":{"x":350,"y":470,"heading":0,"scale":2,"points":[{"x":0,"y":0}],"outline":"black","fill":"yellow","polygon":[{"x":350,"y":470}],"minx":344,"miny":464,"maxx":356,"maxy":476,"radius":3},"scale":2,"subviews":[],"part":{"x":350,"y":470,"heading":0,"type":"Light","name":"part284","worldx":350,"worldy":470,"outline":"black","fill":"yellow","power":0,"radius":3,"moveable":true}},{"x":330,"y":470,"heading":0,"view":{"x":330,"y":470,"heading":0,"scale":2,"points":[{"x":0,"y":0}],"outline":"black","fill":"yellow","polygon":[{"x":330,"y":470}],"minx":324,"miny":464,"maxx":336,"maxy":476,"radius":3},"scale":2,"subviews":[],"part":{"x":330,"y":470,"heading":0,"type":"Light","name":"part285","worldx":330,"worldy":470,"outline":"black","fill":"yellow","power":0,"radius":3,"moveable":true}},{"x":350,"y":490,"heading":0,"view":{"x":350,"y":490,"heading":0,"scale":2,"points":[{"x":0,"y":0}],"outline":"black","fill":"yellow","polygon":[{"x":350,"y":490}],"minx":344,"miny":484,"maxx":356,"maxy":496,"radius":3},"scale":2,"subviews":[],"part":{"x":350,"y":490,"heading":0,"type":"Light","name":"part286","worldx":350,"worldy":490,"outline":"black","fill":"yellow","power":0,"radius":3,"moveable":true}},{"x":270,"y":450,"heading":0,"view":{"x":270,"y":450,"heading":0,"scale":2,"points":[{"x":0,"y":0}],"outline":"black","fill":"yellow","polygon":[{"x":270,"y":450}],"minx":264,"miny":444,"maxx":276,"maxy":456,"radius":3},"scale":2,"subviews":[],"part":{"x":270,"y":450,"heading":0,"type":"Light","name":"part287","worldx":270,"worldy":450,"outline":"black","fill":"yellow","power":0,"radius":3,"moveable":true}},{"x":270,"y":470,"heading":0,"view":{"x":270,"y":470,"heading":0,"scale":2,"points":[{"x":0,"y":0}],"outline":"black","fill":"yellow","polygon":[{"x":270,"y":470}],"minx":264,"miny":464,"maxx":276,"maxy":476,"radius":3},"scale":2,"subviews":[],"part":{"x":270,"y":470,"heading":0,"type":"Light","name":"part288","worldx":270,"worldy":470,"outline":"black","fill":"yellow","power":0,"radius":3,"moveable":true}},{"x":290,"y":470,"heading":0,"view":{"x":290,"y":470,"heading":0,"scale":2,"points":[{"x":0,"y":0}],"outline":"black","fill":"yellow","polygon":[{"x":290,"y":470}],"minx":284,"miny":464,"maxx":296,"maxy":476,"radius":3},"scale":2,"subviews":[],"part":{"x":290,"y":470,"heading":0,"type":"Light","name":"part289","worldx":290,"worldy":470,"outline":"black","fill":"yellow","power":0,"radius":3,"moveable":true}},{"x":310,"y":470,"heading":0,"view":{"x":310,"y":470,"heading":0,"scale":2,"points":[{"x":0,"y":0}],"outline":"black","fill":"yellow","polygon":[{"x":310,"y":470}],"minx":304,"miny":464,"maxx":316,"maxy":476,"radius":3},"scale":2,"subviews":[],"part":{"x":310,"y":470,"heading":0,"type":"Light","name":"part290","worldx":310,"worldy":470,"outline":"black","fill":"yellow","power":0,"radius":3,"moveable":true}},{"x":350,"y":510,"heading":0,"view":{"x":350,"y":510,"heading":0,"scale":2,"points":[{"x":0,"y":0}],"outline":"black","fill":"yellow","polygon":[{"x":350,"y":510}],"minx":344,"miny":504,"maxx":356,"maxy":516,"radius":3},"scale":2,"subviews":[],"part":{"x":350,"y":510,"heading":0,"type":"Light","name":"part291","worldx":350,"worldy":510,"outline":"black","fill":"yellow","power":0,"radius":3,"moveable":true}},{"x":350,"y":530,"heading":0,"view":{"x":350,"y":530,"heading":0,"scale":2,"points":[{"x":0,"y":0}],"outline":"black","fill":"yellow","polygon":[{"x":350,"y":530}],"minx":344,"miny":524,"maxx":356,"maxy":536,"radius":3},"scale":2,"subviews":[],"part":{"x":350,"y":530,"heading":0,"type":"Light","name":"part292","worldx":350,"worldy":530,"outline":"black","fill":"yellow","power":0,"radius":3,"moveable":true}},{"x":350,"y":550,"heading":0,"view":{"x":350,"y":550,"heading":0,"scale":2,"points":[{"x":0,"y":0}],"outline":"black","fill":"yellow","polygon":[{"x":350,"y":550}],"minx":344,"miny":544,"maxx":356,"maxy":556,"radius":3},"scale":2,"subviews":[],"part":{"x":350,"y":550,"heading":0,"type":"Light","name":"part293","worldx":350,"worldy":550,"outline":"black","fill":"yellow","power":0,"radius":3,"moveable":true}},{"x":370,"y":550,"heading":0,"view":{"x":370,"y":550,"heading":0,"scale":2,"points":[{"x":0,"y":0}],"outline":"black","fill":"yellow","polygon":[{"x":370,"y":550}],"minx":364,"miny":544,"maxx":376,"maxy":556,"radius":3},"scale":2,"subviews":[],"part":{"x":370,"y":550,"heading":0,"type":"Light","name":"part294","worldx":370,"worldy":550,"outline":"black","fill":"yellow","power":0,"radius":3,"moveable":true}},{"x":390,"y":550,"heading":0,"view":{"x":390,"y":550,"heading":0,"scale":2,"points":[{"x":0,"y":0}],"outline":"black","fill":"yellow","polygon":[{"x":390,"y":550}],"minx":384,"miny":544,"maxx":396,"maxy":556,"radius":3},"scale":2,"subviews":[],"part":{"x":390,"y":550,"heading":0,"type":"Light","name":"part295","worldx":390,"worldy":550,"outline":"black","fill":"yellow","power":0,"radius":3,"moveable":true}},{"x":410,"y":550,"heading":0,"view":{"x":410,"y":550,"heading":0,"scale":2,"points":[{"x":0,"y":0}],"outline":"black","fill":"yellow","polygon":[{"x":410,"y":550}],"minx":404,"miny":544,"maxx":416,"maxy":556,"radius":3},"scale":2,"subviews":[],"part":{"x":410,"y":550,"heading":0,"type":"Light","name":"part296","worldx":410,"worldy":550,"outline":"black","fill":"yellow","power":0,"radius":3,"moveable":true}},{"x":250,"y":470,"heading":0,"view":{"x":250,"y":470,"heading":0,"scale":2,"points":[{"x":0,"y":0}],"outline":"black","fill":"yellow","polygon":[{"x":250,"y":470}],"minx":244,"miny":464,"maxx":256,"maxy":476,"radius":3},"scale":2,"subviews":[],"part":{"x":250,"y":470,"heading":0,"type":"Light","name":"part297","worldx":250,"worldy":470,"outline":"black","fill":"yellow","power":0,"radius":3,"moveable":true}},{"x":230,"y":470,"heading":0,"view":{"x":230,"y":470,"heading":0,"scale":2,"points":[{"x":0,"y":0}],"outline":"black","fill":"yellow","polygon":[{"x":230,"y":470}],"minx":224,"miny":464,"maxx":236,"maxy":476,"radius":3},"scale":2,"subviews":[],"part":{"x":230,"y":470,"heading":0,"type":"Light","name":"part298","worldx":230,"worldy":470,"outline":"black","fill":"yellow","power":0,"radius":3,"moveable":true}},{"x":330,"y":550,"heading":0,"view":{"x":330,"y":550,"heading":0,"scale":2,"points":[{"x":0,"y":0}],"outline":"black","fill":"yellow","polygon":[{"x":330,"y":550}],"minx":324,"miny":544,"maxx":336,"maxy":556,"radius":3},"scale":2,"subviews":[],"part":{"x":330,"y":550,"heading":0,"type":"Light","name":"part299","worldx":330,"worldy":550,"outline":"black","fill":"yellow","power":0,"radius":3,"moveable":true}},{"x":310,"y":550,"heading":0,"view":{"x":310,"y":550,"heading":0,"scale":2,"points":[{"x":0,"y":0}],"outline":"black","fill":"yellow","polygon":[{"x":310,"y":550}],"minx":304,"miny":544,"maxx":316,"maxy":556,"radius":3},"scale":2,"subviews":[],"part":{"x":310,"y":550,"heading":0,"type":"Light","name":"part300","worldx":310,"worldy":550,"outline":"black","fill":"yellow","power":0,"radius":3,"moveable":true}},{"x":290,"y":550,"heading":0,"view":{"x":290,"y":550,"heading":0,"scale":2,"points":[{"x":0,"y":0}],"outline":"black","fill":"yellow","polygon":[{"x":290,"y":550}],"minx":284,"miny":544,"maxx":296,"maxy":556,"radius":3},"scale":2,"subviews":[],"part":{"x":290,"y":550,"heading":0,"type":"Light","name":"part301","worldx":290,"worldy":550,"outline":"black","fill":"yellow","power":0,"radius":3,"moveable":true}},{"x":270,"y":550,"heading":0,"view":{"x":270,"y":550,"heading":0,"scale":2,"points":[{"x":0,"y":0}],"outline":"black","fill":"yellow","polygon":[{"x":270,"y":550}],"minx":264,"miny":544,"maxx":276,"maxy":556,"radius":3},"scale":2,"subviews":[],"part":{"x":270,"y":550,"heading":0,"type":"Light","name":"part302","worldx":270,"worldy":550,"outline":"black","fill":"yellow","power":0,"radius":3,"moveable":true}},{"x":250,"y":550,"heading":0,"view":{"x":250,"y":550,"heading":0,"scale":2,"points":[{"x":0,"y":0}],"outline":"black","fill":"yellow","polygon":[{"x":250,"y":550}],"minx":244,"miny":544,"maxx":256,"maxy":556,"radius":3},"scale":2,"subviews":[],"part":{"x":250,"y":550,"heading":0,"type":"Light","name":"part303","worldx":250,"worldy":550,"outline":"black","fill":"yellow","power":0,"radius":3,"moveable":true}},{"x":230,"y":550,"heading":0,"view":{"x":230,"y":550,"heading":0,"scale":2,"points":[{"x":0,"y":0}],"outline":"black","fill":"yellow","polygon":[{"x":230,"y":550}],"minx":224,"miny":544,"maxx":236,"maxy":556,"radius":3},"scale":2,"subviews":[],"part":{"x":230,"y":550,"heading":0,"type":"Light","name":"part304","worldx":230,"worldy":550,"outline":"black","fill":"yellow","power":0,"radius":3,"moveable":true}},{"x":210,"y":470,"heading":0,"view":{"x":210,"y":470,"heading":0,"scale":2,"points":[{"x":0,"y":0}],"outline":"black","fill":"yellow","polygon":[{"x":210,"y":470}],"minx":204,"miny":464,"maxx":216,"maxy":476,"radius":3},"scale":2,"subviews":[],"part":{"x":210,"y":470,"heading":0,"type":"Light","name":"part305","worldx":210,"worldy":470,"outline":"black","fill":"yellow","power":0,"radius":3,"moveable":true}},{"x":190,"y":470,"heading":0,"view":{"x":190,"y":470,"heading":0,"scale":2,"points":[{"x":0,"y":0}],"outline":"black","fill":"yellow","polygon":[{"x":190,"y":470}],"minx":184,"miny":464,"maxx":196,"maxy":476,"radius":3},"scale":2,"subviews":[],"part":{"x":190,"y":470,"heading":0,"type":"Light","name":"part306","worldx":190,"worldy":470,"outline":"black","fill":"yellow","power":0,"radius":3,"moveable":true}},{"x":170,"y":470,"heading":0,"view":{"x":170,"y":470,"heading":0,"scale":2,"points":[{"x":0,"y":0}],"outline":"black","fill":"yellow","polygon":[{"x":170,"y":470}],"minx":164,"miny":464,"maxx":176,"maxy":476,"radius":3},"scale":2,"subviews":[],"part":{"x":170,"y":470,"heading":0,"type":"Light","name":"part307","worldx":170,"worldy":470,"outline":"black","fill":"yellow","power":0,"radius":3,"moveable":true}},{"x":150,"y":470,"heading":0,"view":{"x":150,"y":470,"heading":0,"scale":2,"points":[{"x":0,"y":0}],"outline":"black","fill":"yellow","polygon":[{"x":150,"y":470}],"minx":144,"miny":464,"maxx":156,"maxy":476,"radius":3},"scale":2,"subviews":[],"part":{"x":150,"y":470,"heading":0,"type":"Light","name":"part308","worldx":150,"worldy":470,"outline":"black","fill":"yellow","power":0,"radius":3,"moveable":true}},{"x":130,"y":470,"heading":0,"view":{"x":130,"y":470,"heading":0,"scale":2,"points":[{"x":0,"y":0}],"outline":"black","fill":"yellow","polygon":[{"x":130,"y":470}],"minx":124,"miny":464,"maxx":136,"maxy":476,"radius":3},"scale":2,"subviews":[],"part":{"x":130,"y":470,"heading":0,"type":"Light","name":"part309","worldx":130,"worldy":470,"outline":"black","fill":"yellow","power":0,"radius":3,"moveable":true}},{"x":110,"y":470,"heading":0,"view":{"x":110,"y":470,"heading":0,"scale":2,"points":[{"x":0,"y":0}],"outline":"black","fill":"yellow","polygon":[{"x":110,"y":470}],"minx":104,"miny":464,"maxx":116,"maxy":476,"radius":3},"scale":2,"subviews":[],"part":{"x":110,"y":470,"heading":0,"type":"Light","name":"part310","worldx":110,"worldy":470,"outline":"black","fill":"yellow","power":0,"radius":3,"moveable":true}},{"x":110,"y":390,"heading":0,"view":{"x":110,"y":390,"heading":0,"scale":2,"points":[{"x":0,"y":0}],"outline":"black","fill":"yellow","polygon":[{"x":110,"y":390}],"minx":104,"miny":384,"maxx":116,"maxy":396,"radius":3},"scale":2,"subviews":[],"part":{"x":110,"y":390,"heading":0,"type":"Light","name":"part311","worldx":110,"worldy":390,"outline":"black","fill":"yellow","power":0,"radius":3,"moveable":true}},{"x":90,"y":390,"heading":0,"view":{"x":90,"y":390,"heading":0,"scale":2,"points":[{"x":0,"y":0}],"outline":"black","fill":"yellow","polygon":[{"x":90,"y":390}],"minx":84,"miny":384,"maxx":96,"maxy":396,"radius":3},"scale":2,"subviews":[],"part":{"x":90,"y":390,"heading":0,"type":"Light","name":"part312","worldx":90,"worldy":390,"outline":"black","fill":"yellow","power":0,"radius":3,"moveable":true}},{"x":70,"y":390,"heading":0,"view":{"x":70,"y":390,"heading":0,"scale":2,"points":[{"x":0,"y":0}],"outline":"black","fill":"yellow","polygon":[{"x":70,"y":390}],"minx":64,"miny":384,"maxx":76,"maxy":396,"radius":3},"scale":2,"subviews":[],"part":{"x":70,"y":390,"heading":0,"type":"Light","name":"part313","worldx":70,"worldy":390,"outline":"black","fill":"yellow","power":0,"radius":3,"moveable":true}},{"x":90,"y":470,"heading":0,"view":{"x":90,"y":470,"heading":0,"scale":2,"points":[{"x":0,"y":0}],"outline":"black","fill":"yellow","polygon":[{"x":90,"y":470}],"minx":84,"miny":464,"maxx":96,"maxy":476,"radius":3},"scale":2,"subviews":[],"part":{"x":90,"y":470,"heading":0,"type":"Light","name":"part314","worldx":90,"worldy":470,"outline":"black","fill":"yellow","power":0,"radius":3,"moveable":true}},{"x":70,"y":470,"heading":0,"view":{"x":70,"y":470,"heading":0,"scale":2,"points":[{"x":0,"y":0}],"outline":"black","fill":"yellow","polygon":[{"x":70,"y":470}],"minx":64,"miny":464,"maxx":76,"maxy":476,"radius":3},"scale":2,"subviews":[],"part":{"x":70,"y":470,"heading":0,"type":"Light","name":"part315","worldx":70,"worldy":470,"outline":"black","fill":"yellow","power":0,"radius":3,"moveable":true}},{"x":50,"y":470,"heading":0,"view":{"x":50,"y":470,"heading":0,"scale":2,"points":[{"x":0,"y":0}],"outline":"black","fill":"yellow","polygon":[{"x":50,"y":470}],"minx":44,"miny":464,"maxx":56,"maxy":476,"radius":3},"scale":2,"subviews":[],"part":{"x":50,"y":470,"heading":0,"type":"Light","name":"part316","worldx":50,"worldy":470,"outline":"black","fill":"yellow","power":0,"radius":3,"moveable":true}},{"x":50,"y":450,"heading":0,"view":{"x":50,"y":450,"heading":0,"scale":2,"points":[{"x":0,"y":0}],"outline":"black","fill":"yellow","polygon":[{"x":50,"y":450}],"minx":44,"miny":444,"maxx":56,"maxy":456,"radius":3},"scale":2,"subviews":[],"part":{"x":50,"y":450,"heading":0,"type":"Light","name":"part317","worldx":50,"worldy":450,"outline":"black","fill":"yellow","power":0,"radius":3,"moveable":true}},{"x":50,"y":390,"heading":0,"view":{"x":50,"y":390,"heading":0,"scale":2,"points":[{"x":0,"y":0}],"outline":"black","fill":"yellow","polygon":[{"x":50,"y":390}],"minx":44,"miny":384,"maxx":56,"maxy":396,"radius":3},"scale":2,"subviews":[],"part":{"x":50,"y":390,"heading":0,"type":"Light","name":"part318","worldx":50,"worldy":390,"outline":"black","fill":"yellow","power":0,"radius":3,"moveable":true}},{"x":50,"y":410,"heading":0,"view":{"x":50,"y":410,"heading":0,"scale":2,"points":[{"x":0,"y":0}],"outline":"black","fill":"yellow","polygon":[{"x":50,"y":410}],"minx":44,"miny":404,"maxx":56,"maxy":416,"radius":3},"scale":2,"subviews":[],"part":{"x":50,"y":410,"heading":0,"type":"Light","name":"part319","worldx":50,"worldy":410,"outline":"black","fill":"yellow","power":0,"radius":3,"moveable":true}},{"x":50,"y":430,"heading":0,"view":{"x":50,"y":430,"heading":0,"scale":2,"points":[{"x":0,"y":0}],"outline":"black","fill":"yellow","polygon":[{"x":50,"y":430}],"minx":44,"miny":424,"maxx":56,"maxy":436,"radius":3},"scale":2,"subviews":[],"part":{"x":50,"y":430,"heading":0,"type":"Light","name":"part320","worldx":50,"worldy":430,"outline":"black","fill":"yellow","power":0,"radius":3,"moveable":true}},{"x":50,"y":490,"heading":0,"view":{"x":50,"y":490,"heading":0,"scale":2,"points":[{"x":0,"y":0}],"outline":"black","fill":"yellow","polygon":[{"x":50,"y":490}],"minx":44,"miny":484,"maxx":56,"maxy":496,"radius":3},"scale":2,"subviews":[],"part":{"x":50,"y":490,"heading":0,"type":"Light","name":"part321","worldx":50,"worldy":490,"outline":"black","fill":"yellow","power":0,"radius":3,"moveable":true}},{"x":50,"y":510,"heading":0,"view":{"x":50,"y":510,"heading":0,"scale":2,"points":[{"x":0,"y":0}],"outline":"black","fill":"yellow","polygon":[{"x":50,"y":510}],"minx":44,"miny":504,"maxx":56,"maxy":516,"radius":3},"scale":2,"subviews":[],"part":{"x":50,"y":510,"heading":0,"type":"Light","name":"part322","worldx":50,"worldy":510,"outline":"black","fill":"yellow","power":0,"radius":3,"moveable":true}},{"x":50,"y":530,"heading":0,"view":{"x":50,"y":530,"heading":0,"scale":2,"points":[{"x":0,"y":0}],"outline":"black","fill":"yellow","polygon":[{"x":50,"y":530}],"minx":44,"miny":524,"maxx":56,"maxy":536,"radius":3},"scale":2,"subviews":[],"part":{"x":50,"y":530,"heading":0,"type":"Light","name":"part323","worldx":50,"worldy":530,"outline":"black","fill":"yellow","power":0,"radius":3,"moveable":true}},{"x":50,"y":550,"heading":0,"view":{"x":50,"y":550,"heading":0,"scale":2,"points":[{"x":0,"y":0}],"outline":"black","fill":"yellow","polygon":[{"x":50,"y":550}],"minx":44,"miny":544,"maxx":56,"maxy":556,"radius":3},"scale":2,"subviews":[],"part":{"x":50,"y":550,"heading":0,"type":"Light","name":"part324","worldx":50,"worldy":550,"outline":"black","fill":"yellow","power":0,"radius":3,"moveable":true}},{"x":70,"y":550,"heading":0,"view":{"x":70,"y":550,"heading":0,"scale":2,"points":[{"x":0,"y":0}],"outline":"black","fill":"yellow","polygon":[{"x":70,"y":550}],"minx":64,"miny":544,"maxx":76,"maxy":556,"radius":3},"scale":2,"subviews":[],"part":{"x":70,"y":550,"heading":0,"type":"Light","name":"part325","worldx":70,"worldy":550,"outline":"black","fill":"yellow","power":0,"radius":3,"moveable":true}},{"x":90,"y":550,"heading":0,"view":{"x":90,"y":550,"heading":0,"scale":2,"points":[{"x":0,"y":0}],"outline":"black","fill":"yellow","polygon":[{"x":90,"y":550}],"minx":84,"miny":544,"maxx":96,"maxy":556,"radius":3},"scale":2,"subviews":[],"part":{"x":90,"y":550,"heading":0,"type":"Light","name":"part326","worldx":90,"worldy":550,"outline":"black","fill":"yellow","power":0,"radius":3,"moveable":true}},{"x":110,"y":550,"heading":0,"view":{"x":110,"y":550,"heading":0,"scale":2,"points":[{"x":0,"y":0}],"outline":"black","fill":"yellow","polygon":[{"x":110,"y":550}],"minx":104,"miny":544,"maxx":116,"maxy":556,"radius":3},"scale":2,"subviews":[],"part":{"x":110,"y":550,"heading":0,"type":"Light","name":"part327","worldx":110,"worldy":550,"outline":"black","fill":"yellow","power":0,"radius":3,"moveable":true}},{"x":130,"y":550,"heading":0,"view":{"x":130,"y":550,"heading":0,"scale":2,"points":[{"x":0,"y":0}],"outline":"black","fill":"yellow","polygon":[{"x":130,"y":550}],"minx":124,"miny":544,"maxx":136,"maxy":556,"radius":3},"scale":2,"subviews":[],"part":{"x":130,"y":550,"heading":0,"type":"Light","name":"part328","worldx":130,"worldy":550,"outline":"black","fill":"yellow","power":0,"radius":3,"moveable":true}},{"x":150,"y":550,"heading":0,"view":{"x":150,"y":550,"heading":0,"scale":2,"points":[{"x":0,"y":0}],"outline":"black","fill":"yellow","polygon":[{"x":150,"y":550}],"minx":144,"miny":544,"maxx":156,"maxy":556,"radius":3},"scale":2,"subviews":[],"part":{"x":150,"y":550,"heading":0,"type":"Light","name":"part329","worldx":150,"worldy":550,"outline":"black","fill":"yellow","power":0,"radius":3,"moveable":true}},{"x":170,"y":550,"heading":0,"view":{"x":170,"y":550,"heading":0,"scale":2,"points":[{"x":0,"y":0}],"outline":"black","fill":"yellow","polygon":[{"x":170,"y":550}],"minx":164,"miny":544,"maxx":176,"maxy":556,"radius":3},"scale":2,"subviews":[],"part":{"x":170,"y":550,"heading":0,"type":"Light","name":"part330","worldx":170,"worldy":550,"outline":"black","fill":"yellow","power":0,"radius":3,"moveable":true}},{"x":190,"y":550,"heading":0,"view":{"x":190,"y":550,"heading":0,"scale":2,"points":[{"x":0,"y":0}],"outline":"black","fill":"yellow","polygon":[{"x":190,"y":550}],"minx":184,"miny":544,"maxx":196,"maxy":556,"radius":3},"scale":2,"subviews":[],"part":{"x":190,"y":550,"heading":0,"type":"Light","name":"part331","worldx":190,"worldy":550,"outline":"black","fill":"yellow","power":0,"radius":3,"moveable":true}},{"x":210,"y":550,"heading":0,"view":{"x":210,"y":550,"heading":0,"scale":2,"points":[{"x":0,"y":0}],"outline":"black","fill":"yellow","polygon":[{"x":210,"y":550}],"minx":204,"miny":544,"maxx":216,"maxy":556,"radius":3},"scale":2,"subviews":[],"part":{"x":210,"y":550,"heading":0,"type":"Light","name":"part332","worldx":210,"worldy":550,"outline":"black","fill":"yellow","power":0,"radius":3,"moveable":true}}],"editTarget":null,"editOriginalOutline":"black","running":false,"width":0,"height":0,"rotated":false,"mazeWorldLoaded":false,"combatWorldLoaded":false,"pacmanWorldLoaded":true,"pacmanPoints":0}'
    robotStartingLocation();    //GAVIN CHANGED 04/05/2023
    document.getElementById("simRemoveOpponent").click(); //GAVIN ADDED 04/03/2023
    loadWorld(pacmanWorld);
    simState.pacmanPoints = 0;  //GAVIN ADDED 04/06/2023
    displayPacmanPoints();
    startPacmanSound.play();  //GAVIN UNCOMMENT WHEN DONE

}
//End of edited by Gavin 03/20/2023


function loadWorld(worldString) {

    if(!worldString) worldString = localStorage.getItem("world");
    if(!worldString) return;

    var obj = JSON.parse(worldString);

    /* grab the attributes */
    for(var attr in obj) {
        if(attr == "part") { continue; }
        robot[attr] = obj[attr];
    }

    /* handle the parts */

    //simState.worldObjects = [];
    for(var i=0; i<obj.worldObjects.length; i++) {
        //Check for Wall
        if(obj.worldObjects[i].part.type == "Wall"){
        var wall = new Wall(null, obj.worldObjects[i].x, obj.worldObjects[i].y);
        wall.rotated = obj.worldObjects[i].part.rotated;
        wall.resizeFactor = obj.worldObjects[i].part.resizeFactor;
        wall.resizeFactorHeight = obj.worldObjects[i].part.resizeFactorHeight;
        wall.resizeFactorWidth = obj.worldObjects[i].part.resizeFactorWidth;
        wall.name = obj.worldObjects[i].part.name;
        wall.outline = obj.worldObjects[i].part.outline;
        wall.fill = obj.worldObjects[i].part.fill;
        wall.moveable = obj.worldObjects[i].part.moveable;      //Added by Gavin 03/21/2023
        simState.worldObjects.push(constructView(wall));
        }

        //Check for Light
        if(obj.worldObjects[i].part.type == "Light"){
        var light = new Light(null, obj.worldObjects[i].x, obj.worldObjects[i].y);
        light.name = obj.worldObjects[i].part.name;
        light.outline = obj.worldObjects[i].part.outline;
        light.fill = obj.worldObjects[i].part.fill;
        light.moveable = obj.worldObjects[i].part.moveable;     //Added by Gavin 03/21/2023
        simState.worldObjects.push(constructView(light));
        }

        //Check for Box
        if(obj.worldObjects[i].part.type == "Box"){
        var box = new Box(null, obj.worldObjects[i].x, obj.worldObjects[i].y);
        box.name = obj.worldObjects[i].part.name;
        box.outline = obj.worldObjects[i].part.outline;
        box.fill = obj.worldObjects[i].part.fill;
        box.moveable = obj.worldObjects[i].part.moveable;   //Added by Gavin 03/21/2023
        simState.worldObjects.push(constructView(box));
        }

        drawSim();
    }
}

//START OF EDITED BY GAVIN 03/20/2023
function sound(src) {
    this.sound = document.createElement("audio");
    this.sound.src = src;
    this.sound.setAttribute("preload", "auto");
    this.sound.setAttribute("controls", "none");
    this.sound.style.display = "none";
    document.body.appendChild(this.sound);
    this.play = function(){
        this.sound.play();
    }
    this.stop = function(){
        this.sound.pause();
    }    
}

var startPacmanSound = new sound("sounds_startMusic.mp3");
var pacmanEatingSound = new sound("sounds_eatingDot.mp3");

//ADDED BY GAVIN 04/06/2023
//END OF ADDED BY GAVIN 04/06/2023
function displayPacmanPoints(){
    var displayPacmanPoints = document.getElementById("pacmanScore");
    displayPacmanPoints.innerHTML = "Score: " + simState.pacmanPoints;  
}

function removePacmanPoints(){
    var displayPacmanPoints = document.getElementById("pacmanScore");
    displayPacmanPoints.innerHTML = "";
}

// create a variable to hold the start time
var startTime;

// create a function to start the stopwatch
function startStopwatch() {
  startTime = new Date().getTime();
}

// create a function to stop the stopwatch and return the elapsed time
function stopStopwatch() {
  const endTime = new Date().getTime();
  const elapsedTime = endTime - startTime;
  return elapsedTime;
}
//ADDED BY GAVIN 04/05/2023
//ALERTS
function notAvailablePopup() {
    message = window.alert("Sorry, this feature is not available inside of this premade world");
}
//END OF ADDED BY GAVIN 04/05/2023
//END OF EDITED BY GAVIN 03/20/2023


//Dark mode

// document.querySelector('[data-switch-dark]').addEventListener('click', function() {
//     document.body.classList.toggle('dark-class');
//   })

//document.onkeypress = function (e) {
//    e = e || window.event;
//
//    if (e.keyCode === 13) {
//        document.documentElement.classList.toggle('dark-mode');
//    }}
 // Press enter key to enter dark mode.

//  const btn = document.querySelector(".btn-toggle");
//  const theme = document.querySelector("#theme-link");
//  btn.addEventListener("click", function() {
//    if (theme.getAttribute("href") === "light-theme.css") {
//      theme.href = "dark-theme.css";
//    } else {
//      theme.href = "light-theme.css";
//    }
//  });
