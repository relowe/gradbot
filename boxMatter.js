function BoxMatter(x, y, w, h, isStatic) {
    if (isStatic) {
        this.body = Bodies.rectangle(x, y, w, h, { isStatic: true });
    }
    else {
        this.body = Bodies.rectangle(x, y, w, h);
    }

    this.w = w;
    this.h = h;
    Composite.add(worldMatter, this.body);


    this.show = function (context) {
        var pos = this.body.position;
        console.log(pos.x);
        console.log(pos.y);
        console.log(w);
        console.log(h);
        context.fillRect(pos.x, pos.y, w, h);
    }
}