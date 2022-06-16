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
    return view1.minx <= view2.maxx && 
           view1.maxx >= view2.minx &&
           view1.miny <= view2.maxy &&
           view1.maxy >= view2.miny;
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
          doc: 'Set the power of the motor from -100 to 100',
          params: Array( { name: 'p', doc: 'power setting'})
        }
    );
    this.doc.vars = Array(
        {name: 'power', doc: 'The current power setting of the motor'}
    );


    // handle speed of the motor
    this.speed = 0;  // motor speed in radians per second
    this.update = function() {
        //we are basing this on the sparkfun hobby motors which spin at 65 RPM (max)
        //This maximum speed is roughly 6.81 radians per second
        this.speed = 6.81 * this.power / 100;
    }


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
        { name: 'penDown', doc: 'Begin drawing.', params: Array()},
        { name: 'penUp', doc: 'Stop drawing.', params: Array()},
        { name: 'setColor', doc: 'Change drawing color.', params: Array(
            {name: 'c', doc: 'color'}
        )}
    );
    this.doc.vars = Array(
        { name: 'color', doc: 'The current color of the marker' },
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


        //update all the sub parts
        for(var i in this.parts) {
            var p = this.parts[i];
            p.update();
        }

        //update the motors
        this.left.update();
        this.right.update();

        //compute our forward translation and yaw speeds
        var r = .065; // 65mm diameter wheels
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
        this.laserBattery -= amount;
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

    this.doc.functions = Array(
        { name: 'setColor', doc: 'Change light color.', params: Array(
            {name: 'c', doc: 'color'}
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
    this.outline="lightblue";
    this.fill = "blue";
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
        {name: 'distance', doc: 'distance to the nearest object'}
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
        {name: 'intensity', doc: 'intensity of the sensed light'}
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

    var speed = 30;

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
        {name: 'fire', doc: 'Fire the laser beam.', params: Array()}
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
          doc: 'Wait for a period of time to pass',
          params: Array({name: 'ms', doc: 'delay in milliseconds'})}
    );
}



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
function WallView(part) {
    if (!part.resizeFactor){
        part.resizeFactor = 1;
    }
    PartView.call(this, part);

    //create my vector view
    var points = [
        {x: -5 * part.resizeFactor, y: -10 * part.resizeFactor},
        {x: 5 * part.resizeFactor, y: -10 * part.resizeFactor},
        {x: 5 * part.resizeFactor, y: 10 * part.resizeFactor},
        {x: -5 * part.resizeFactor, y: 10 * part.resizeFactor},
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
        {x: 10, y: -1},
        {x: 10, y: 1},
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
const DRAG_RESIZE=3;

//state of the simulator ui
var simState = {
    prefix: "sim",
    dragMode: DRAG_NONE,
    dragTarget: null,
    lastX: 0,
    lastY: 0,
    robotStartX: 100,
    robotStartY: 100,
    robotStartHeading: 0,
    timer: null,
    prevTab: null,
    robotThread: null,
    opponentThread: null,
    worldObjects: [],
    editTarget: null,
    editOriginalOutline: null,
    running : false,
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
            if(obj.view.encloses(event.offsetX, event.offsetY)) {
                simState.dragTarget = obj;
                break;
            }
        }
    }
    
    if(!simState.dragTarget) {
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
    } else if(document.getElementById('dragResize').checked) {
        simState.dragMode = DRAG_RESIZE;
    } else {
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
    // one last move (if that is what we are up to!)
    if(simState.dragMode == DRAG_MOVE) {
        simState.dragTarget.part.moveTo(event.offsetX, event.offsetY);
    } else if (simState.dragMode == DRAG_RESIZE && simState.dragTarget.part.type == 'Wall'){
        if (simState.dragTarget.part.resizeFactor < 5){
            simState.dragTarget.part.resizeFactor = simState.dragTarget.part.resizeFactor + 1;
        }else{
            simState.dragTarget.part.resizeFactor = 1;
        }
        for (var i=0; simState.worldObjects.length; i++){
            if (simState.worldObjects[i].part.name == simState.dragTarget.part.name){
                simState.worldObjects.splice(i, 1);
                break;
            };
        }
        simState.worldObjects.push(new WallView(simState.dragTarget.part));
    }

    // end the drag mode
    simState.dragMode = DRAG_NONE;

    //if this is not the robot, select it
    if(simState.dragTarget && simState.dragTarget.part.type != "Chassis") {
        selectPart(simState.dragTarget, simState);
    }

    //refresh the canvas
    drawSim();
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
        simState.dragTarget.part.rotate((event.offsetY-simState.lastY) * 0.1);
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
    var canvas = document.getElementById("simfg");
    var wall = new Wall(null, canvas.width/2, canvas.height/2);

    simState.worldObjects.push(constructView(wall));
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
}


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


/**
 * Apply the editor to the given state.
 */
function applyEditor(state) {
    //get the color from the editor
    var fill = document.getElementById(state.prefix + "PartFillColor").value;
    var outline = document.getElementById(state.prefix + "PartOutlineColor").value;

    //get the part name
    var name = document.getElementById(state.prefix + "PartName").value;

    //get the part we are editing
    var part = state.editTarget.part;

    //deselect the part
    deselectPart(state);

    //set the fields
    part.fill = fill;
    part.outline = outline;
    part.name = name;

    // redraw the canvas
    if(state.prefix == "build") {
        drawBuild();
    } else if(state.prefix == "sim") {
        drawSim();
    }
}


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
 * Handle adding a marker. 
 * @param {*} event 
 */
function buildAddMarker(event) {
    var marker = new Marker(robot);
    robot.addPart(marker);
    buildView.addPart(marker);
    drawBuild();
}


/**
 * Handle adding a light.
 * @param {*} event
 */
function buildAddLight(event) {
    var light = new Light(robot);
    light.radius = 1;
    robot.addPart(light);
    buildView.addPart(light);
    drawBuild();
}


/**
 * Handle adding a light sensor.
 * @param {*} event
 */
function buildAddLightSensor(event) {
    var sensor = new LightSensor(robot);
    robot.addPart(sensor);
    buildView.addPart(sensor);
    drawBuild();
}


/**
 * Handle adding a range sensor.
 * @param {*} event
 */
function buildAddRangeSensor(event) {
    var sensor = new RangeSensor(robot);
    robot.addPart(sensor);
    buildView.addPart(sensor);
    drawBuild();
}


/**
 * Handle adding a laser.
 * @param {*} event
 */
function buildAddLaser(event) {
    var laser = new Laser(robot);
    robot.addPart(laser);
    buildView.addPart(laser);
    drawBuild();
}


/**
 * Handle the simulation go button.
 * @param {*} event 
 */
function simulationGo(event) {
    var text = event.target.innerHTML;

    if(text == "Start") {
        event.target.innerHTML = "Pause";
        
        //preserve the starting pose of the robot
        simState.robotStartX = robot.x;
        simState.robotStartY = robot.y;
        simState.robotStartHeading = robot.heading;
        simulationStart();
    } else if(text == "Pause") {
        event.target.innerHTML = "Resume";
        simulationStop();
    } else if(text == "Resume") {
        event.target.innerHTML = "Pause";
        simulationStart();
    }
}


function simulationReset(event) {
    document.getElementById("simGo").innerHTML = "Start";
    var canvas = document.getElementById("simfg");
    //put the robot back in its original position and heading
    robot.moveTo(simState.robotStartX, simState.robotStartY);
    robot.face(simState.robotStartHeading);

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
    if(opponent) {
        opponent.x = 700;
        opponent.y = 500;
        opponent.heading = Math.PI;
    }

    //clear lights and walls
    if(simState.worldObjects.length != 0) {
        if(confirm("Would you like to remove the world objects?")) {
            simState.worldObjects= [];
        }
    }

    //redraw 
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
 * Initialize the gradbot interface.
 */
function gradbotInit() {
    //fill the simulation background with graph paper
    graphPaperFill('simbg');

    //create the robot
    robot = new Chassis(100, 100, 0, "chassis");
    loadRobot(robot);
    simState.robotStartX = robot.x;
    simState.robotStartY = robot.y;
    simState.robotStartHeading = robot.heading;

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

    //set up the sim buttons
    document.getElementById('simGo').onclick = simulationGo;
    document.getElementById('simReset').onclick = simulationReset;
    document.getElementById('simClear').onclick = simulationClear;

    //set up the object buttons
    document.getElementById('simAddLight').onclick = simAddLight;
    document.getElementById('simAddWall').onclick = simAddWall;

    //set up the build mouse events
    canvas = document.getElementById("buildCanvas");
    canvas.onmousedown = buildMouseDown;
    canvas.onmouseup = buildMouseUp;
    canvas.onmousemove = buildMouseMove;

    //set up the build's form buttons
    document.getElementById("buildPartApply").onclick = buildApply;
    document.getElementById("buildPartCancel").onclick = buildCancel;
    document.getElementById("buildPartDelete").onclick = buildDeletePart;

    //set up the sim's form buttons
    document.getElementById("simPartApply").onclick = simApply;
    document.getElementById("simPartCancel").onclick = simCancel;
    document.getElementById("simPartDelete").onclick = simDeletePart;

    //select the simulation tab
    document.getElementById('simButton').click();

    //set up file handlers
    document.getElementById("buildOpen").onclick = function() {
        deselectPart(buildState);
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

    //set up opponent handlers
    document.getElementById("simUpload").onchange = openOpponentFile;
    document.getElementById("simOpenOpponent").onclick = function() {
        document.getElementById("simClear").click();
        document.getElementById("simUpload").click();
    };
    document.getElementById("simRemoveOpponent").onclick = function() {
        opponent = null;
        if(simView.opponentThread) {
            simView.opponentThread.terminate();
        }
        simView.opponentThread = null;
        opponentView = null;
        drawSim();
    };

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

    // mark the simulation as running
    simState.running = true;

    // clear the timer, if there is one
    if(simState.timer) {
        clearInterval(simState.timer);
    }

    //reset the bots
    robot.left.setPower(0);
    robot.right.setPower(0);
    robot.resetLaserBattery();
    robot.hp = 3;
    if(opponent) {
        opponent.left.setPower(0);
        opponent.right.setPower(0);
        opnent.resetLaserBattery();
        opponent.hp = 3;
    }

    //start the robot thread
    simState.robotThread = new Worker("userbot.js");
    simState.robotThread.onerror = gradbotError;
    simState.robotThread.onmessage = simulationReceiveMessage;
    simState.robotThread.postMessage({type: "start", robot: robot.sendable()});
    robot.thread = simState.robotThread;


    //start the opponent thread (if there is one)
    if(opponent) {
        simState.opponentThread = new Worker("userbot.js");
        simState.opponentThread.onerror = gradbotError;
        simState.opponentThread.onmessage = opponentReceiveMessage;
        simState.opponentThread.postMessage({type: "start", robot: opponent.sendable()});
        opponent.thread = simState.opponentThread;
    }


    //set the timer going!
    simState.timer = setInterval(simulationUpdate, 1000/30);
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

    //update all the world objects
    var toVanish = [];
    for(var i=0; i < simState.worldObjects.length; i++) {
        var obj = simState.worldObjects[i];
        obj.part.update();

        //check for laser blast collisions
        if(obj.part.type == "LaserBlast") {
            for(var j=0; j < botViews.length; j++) {
                if(bots[j] !== obj.part.firedBy && collision(botViews[j].view, obj.view)) {
                    bots[j].hp--;
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

        // check for robot collisions
        //if(obj.part.type == "Wall") {
            //for(var j=0; j < botViews.length; j++) {
                //if(collision(botViews[j].view, obj.view)) {
                    //bots[j].hp--;
                    //bots[j].left.setPower(0);
                    //bots[j].right.setPower(0);
                //}
            //}
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
function saveWorld(world){
	localStorage.setItem("world", JSON.stringify(world));
}

function saveWorldFile() {
    var file = new Blob([JSON.stringify(world)]);
    var a = document.getElementById('worldDownload');
    a.href = URL.createObjectURL(file, {type: "text/plain"});
    a.download = "world";
    a.click();
    
    URL.revokeObjectURL(a.href);
}

function openWorldFile() {
    var reader = new FileReader();
    reader.onload = function() {
        loadWorld(world, reader.result);

        //rebuild the world for simulation
        simView = new ChassisView(world);
        buildView = new ChassisBuildView(world);

        //redraw
        graphPaperFill("simbg");
        drawSim();
        drawBuild();
    };

    reader.readAsText(this.files[0]);

}

function loadWorld(world, worldString) {

    if(!worldString) worldString = localStorage.getItem("world");
    if(!worldString) return;


    var obj = JSON.parse(worldString);
}

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
    var file = new Blob([JSON.stringify(robot)]);
    var a = document.getElementById('buildDownload');
    a.href = URL.createObjectURL(file, {type: "text/plain"});
    a.download = "robot";
    a.click();
    
    URL.revokeObjectURL(a.href);
}


function openRobotFile() {
    var reader = new FileReader();
    reader.onload = function() {
        loadRobot(robot, reader.result);

        //rebuild the robot views
        simView = new ChassisView(robot);
        buildView = new ChassisBuildView(robot);

        //redraw
        graphPaperFill("simbg");
        drawSim();
        drawBuild();
    };

    reader.readAsText(this.files[0]);

}


function openOpponentFile() {
    var reader = new FileReader();
    reader.onload = function() {
        opponent = new Chassis();
        loadRobot(opponent, reader.result);
        opponent.x = 700;
        opponent.y = 500;
        opponent.heading = Math.PI;

        //rebuild the robot view
        opponentView = new ChassisView(opponent);

        //redraw
        graphPaperFill("simbg");
        drawSim();
    };

    reader.readAsText(this.files[0]);

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
    robot.left = finishPart(obj.left);
    robot.right = finishPart(obj.right);
    robot.left.parent = robot;
    robot.right.parent = robot;

    /* handle the parts */
    robot.parts = [];
    for(var i=0; i<obj.parts.length; i++) {
        robot.addPart(finishPart(obj.parts[i]));
        robot.parts[i].parent = robot;
    }
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

    //rebuild the robot views
    simView = new ChassisView(robot);
    buildView = new ChassisBuildView(robot);

    //redraw
    graphPaperFill("simbg");
    drawSim();
    drawBuild();
}

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
