function boxMatter(x, y, w, h) {
    this.body = Bodies.rectangle(x, y, w, h);
    this.w = w;
    this.h = h;
    Composite.add(worldMatter, this.body);


    this.show = function () {
        var pos = this.body.position;

        push();

        translate(pos.x, pos.y);
        rect(0, 0, this.w, this.h);

        pop();
    }
}