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
    Positionable.call(this, x, y, heading);
    this.width = width != undefined ? width : 0;
    this.height = height != undefined ? height : 0;

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
        this.power = power;
    };


    /**
     * Update the part's state.
     */
    this.update = function() { };


}


function Motor(parent, x, y, heading)
{
    //construct the part
    Part.call(this, parent, x, y, heading, 6, 2);


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
    Part.call(this, null, x, y, heading, 20, 12)

    //handle the subparts of the chassis
    this.parts = Array();
    this.addPart = function(p) {
        this.parts.push(p);
    };

    // create the left and right motors
    this.left = new Motor(this, -7, -7, 0);
    this.right = new Motor(this, -7, 7, Math.PI);

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
    this.addSubview(new MotorView(part.left));
    this.addSubview(new MotorView(part.right));

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
    this.partDraw = this.draw;

    /**
     * Update for the movement of the model and then draw.
     * @param {*} canvas 
     * @param {*} context 
     */
    this.draw = function(canvas, context) {
        //copy the chassis pose
        this.x = this.part.x;
        this.y = this.part.y;
        this.heading = this.part.heading;

        //draw like normal
        this.partDraw(canvas, context); 
    }
}


function ChassisBuildView(part) {
    ChassisView.call(this, part);

    // This view has a fixed position and heading
    this.x = 400;
    this.y = 300;
    this.scale=30; //it's also big!
    this.heading = -Math.PI/2;


    //We won't move this.
    this.draw = this.partDraw;
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


/****************************************** 
 * USER INTERFACE
 ******************************************/
var robot;
var simView;
var buildView;

// dragmodes
const DRAG_NONE= 0;
const DRAG_MOVE=1;
const DRAG_ROTATE=2;

//state of the simulator ui
var simState = {
    dragMode: DRAG_NONE,
    dragTarget: null,
    lastX: 0,
    lastY: 0 
};


//state of the build ui
var buildState = {
    dragMode: DRAG_NONE,
    dragTarget: null,
    lastX: 0,
    lastY: 0,
    editTarget: null,
    editOriginalOutline: null
}



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
        deselectPart();
        drawSim();
    } else if(tabId == "Build") {
        drawBuild();
    }
}


/**
 * Draw the simulator canvas.
 */
function drawSim() {
    var canvas = document.getElementById('simfg');
    var context = canvas.getContext("2d");

    // clear the frame
    context.clearRect(0, 0, canvas.width, canvas.height);

    //draw the robot
    simView.draw(canvas, context);
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
        simState.dragTarget = simView.part;
    } else {
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
        simState.dragTarget.moveTo(event.offsetX, event.offsetY);
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
        simState.dragTarget.moveTo(event.offsetX, event.offsetY);
    }

    // end the drag mode
    simState.dragMode = DRAG_NONE;

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
        simState.dragTarget.moveTo(event.offsetX, event.offsetY);
    }

    //process rotation
    if(simState.dragMode == DRAG_ROTATE) {
        simState.dragTarget.rotate((event.offsetY-simState.lastY) * 0.1);
    }

    //record this position
    simState.lastX = event.offsetX;
    simState.lastY = event.offsetY;

    // refresh the canvas
    drawSim();

    return true;
}



/*
 * Build Events
 */

/**
 * Show the editor for the part specified by "view".
 * @param {*} view 
 */
function showPartEditor(view) {
    //get the colors populated
    document.getElementById("partOutlineColor").value = view.part.outline;
    document.getElementById("partFillColor").value = view.part.fill;

    //show the editor pane
    document.getElementById("partColorEditor").style.display="block";
}


/**
 * Hide the part editor.
 */
function hidePartEditor() {
    //hide the editor pane
    document.getElementById("partColorEditor").style.display="none";
}


/**
 * Select the partview in the editor.
 * @param {*} view 
 */
function selectPart(view) {
    // deselect (if needed) 
    if(buildState.editTarget != null) {
        deselectPart();
    }

    // grab the original color 
    buildState.editOriginalOutline = view.part.outline;

    // set up the target
    buildState.editTarget = view;

    //show the editor
    showPartEditor(view);

    // color it coral
    view.part.outline = "coral";


    // redraw the canvas
    drawBuild();
}


/**
 * Deselect the selected partview (if there is one)
 */
function deselectPart() {
    // do nothing if there is no selected part
    if(buildState.editTarget == null) {
        return;
    }

    // retore the original color
    buildState.editTarget.part.outline = buildState.editOriginalOutline;

    // remove the selection
    buildState.editTarget = null;
    hidePartEditor();

    // redraw the canvas
    drawBuild();
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
        selectPart(buildState.dragTarget);
    }

    // clear the drag target
    buildState.dragTarget = null;
    buildState.dragMode = DRAG_NONE;
}


/**
 * Handle the apply button for the part editor.
 * @param {*} event 
 */
function buildApply(event) {

    //get the color from the editor
    var fill = document.getElementById("partFillColor").value;
    var outline = document.getElementById("partOutlineColor").value;

    //get the part we are editing
    var part = buildState.editTarget.part;

    //deselect the part
    deselectPart();

    //set the colors
    part.fill = fill;
    part.outline = outline;

    //refresh the canvas
    drawBuild();
}


/**
 * Handle the cancel button
 * @param {*} event 
 */
function buildCancel(event) {
    deselectPart();
}


/**
 * Initialize the gradbot interface.
 */
function gradbotInit() {
    //select the simulation tab
    document.getElementById('simButton').click();

    //fill the simulation background with graph paper
    graphPaperFill('simbg');

    //create the robot
    robot = new Chassis(100, 100, 0);

    //put the robot on the foreground of the simulator
    simView = new ChassisView(robot);
    simView.scale=2;
    drawSim();

    //build the robot builder view
    buildView = new ChassisBuildView(robot);
    drawBuild();

    //set up the sim mouse events
    var canvas = document.getElementById('simfg');
    canvas.onmousedown = simMouseDown;
    canvas.onmouseup = simMouseUp;
    canvas.onmousemove = simMouseMove;

    //set up the build mouse events
    canvas = document.getElementById("buildCanvas");
    canvas.onmousedown = buildMouseDown;
    canvas.onmouseup = buildMouseUp;
    canvas.onmousemove = buildMouseMove;

    //set up the build's form buttons
    document.getElementById("partApply").onclick = buildApply;
    document.getElementById("partCancel").onclick = buildCancel;
}