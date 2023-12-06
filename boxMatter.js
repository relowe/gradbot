function BoxMatter(x, y, size, isStatic) {
    if (isStatic) {
        this.body = Bodies.rectangle(x, y, size, size);
    }
    else {
        this.body = Bodies.rectangle(x, y, size, size);
    }
    Part.call(this, parent, x, y);
    this.type = "BoxMatter";
    this.stroke = "blue";
    this.fill = "lightblue";
    this.size = size;
    this.moveable = true;
    Composite.add(worldMatter, this.body);


    this.show = function (context) {
        var pos = this.body.position;
        context.fillStyle = this.fill;
        context.fillRect(pos.x, pos.y, size, size);
        context.strokeStyle = this.stroke;
        context.strokeRect(pos.x, pos.y, size, size);
    }
}