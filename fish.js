let fish;
function setup() {
  createCanvas(800, 500);
  fish = new Fish();
  fish.addMain(new Main(width/2, height/2));
  for(let i = 0; i < 10; i++){
    fish.addSmall(new Small(width/2, height/2));
  }
}
function draw(){
  background(102,178,255);
  fish.run();
}
function Fish(){
  this.main = [];
  this.small = [];
}
Fish.prototype.run = function(){
  for (let i = 0; i < this.main.length; i++){
    this.main[i].run(this.main);
  }
  for (let i = 0; i < this.small.length; i++) {
    this.small[i].run(this.small, this.main);  // Passing the entire list of boids to each boid individually
  }
}
Fish.prototype.addMain = function(b) {
  this.main.push(b);
}
Fish.prototype.addSmall = function(b) {
  this.small.push(b);
}
function Main(x, y){
  this.acceleration = createVector(0, 0);
  this.velocity = createVector(random(-1, 1), random(-1, 1));
  this.position = createVector(x, y);
  this.r = 3.0;
  this.maxspeed = 2;
  this.maxforce = 0.05;
}

Main.prototype.run = function(main) {
  this.flock(main);
  this.update();
  this.render();
}
Main.prototype.applyForce = function(force) {
  // We could add mass here if we want A = F / M
  this.acceleration.add(force);
}
Main.prototype.flock = function(main){
  let ctr = this.control(main);
  
  ctr.mult(1.0);

  this.applyForce(ctr)
}
Main.prototype.update = function(){
  this.velocity.add(this.acceleration);
  this.velocity.limit(this.maxspeed);
  this.position.add(this.velocity);
  // Reset accelertion to 0 each cycle
  this.acceleration.mult(0);
}
Main.prototype.render = function(){
  fill(102,51,0);
  push();
  translate(this.position.x, this.position.y);
  circle(0, 0, 20);
  pop();
}

Main.prototype.seek = function(target) {
  let desired = p5.Vector.sub(target,this.position);  // A vector pointing from the location to the target
  // Normalize desired and scale to maximum speed
  desired.normalize();
  desired.mult(this.maxspeed);
  // Steering = Desired minus Velocity
  let steer = p5.Vector.sub(desired,this.velocity);
  steer.limit(this.maxforce);  // Limit to maximum steering force
  return steer;
}

Main.prototype.control = function(main) {
  return this.seek(createVector(mouseX, mouseY));  // Steer towards the location
}




function Small(x, y){
  this.acceleration = createVector(0, 0);
  this.velocity = createVector(random(-1, 1), random(-1, 1));
  this.position = createVector(x, y);
  this.r = 3.0;
  this.maxspeed = 2;
  this.maxforce = 0.05;
}

Small.prototype.run = function(small, main) {
  this.flock(small, main);
  this.update();
  this.render();
}
Small.prototype.applyForce = function(force) {
  // We could add mass here if we want A = F / M
  this.acceleration.add(force);
}
Small.prototype.update = function() {
  // Update velocity
  this.velocity.add(this.acceleration);
  // Limit speed
  this.velocity.limit(this.maxspeed);
  this.position.add(this.velocity);
  // Reset accelertion to 0 each cycle
  this.acceleration.mult(0);
}
Small.prototype.render = function() {
  // Draw a triangle rotated in the direction of velocity
  let theta = this.velocity.heading() + radians(90);
  fill(255,255,0); 
  stroke(200);
  push();
  translate(this.position.x, this.position.y);
  rotate(theta);
  ellipse(0, this.r, 10, 20);
  noStroke();
  beginShape();
  vertex(0, -this.r * 2);
  vertex(-this.r*1.5, this.r * 6);
  vertex(this.r*1.5, this.r * 6);
  endShape(CLOSE);
  fill(0,0,0);
  stroke(200);
  circle(0, -this.r, 2);
  line(-this.r*1.5, this.r*4, 0, this.r*4);
  pop();
}
Small.prototype.flock = function(small, main){
  let sep = this.separate(small);  // Separation
  let sep2 = this.separate2(small, main); 
  let ali = this.align(small);      // Alignment
  let coh = this.cohesion(small);   // Cohesion
  let avo = this.avoid(small);     // Avoid walls
  // Arbitrarily weight these forces
  sep.mult(6.0);
  sep2.mult(6.0);
  ali.mult(2.0);
  coh.mult(1.0);
  avo.mult(3.0);
  // Add the force vectors to acceleration
  this.applyForce(sep);
  this.applyForce(sep2);
  this.applyForce(ali);
  this.applyForce(coh);
  this.applyForce(avo);
}
Small.prototype.seek = function(target) {
  let desired = p5.Vector.sub(target,this.position);  // A vector pointing from the location to the target
  // Normalize desired and scale to maximum speed
  desired.normalize();
  desired.mult(this.maxspeed);
  // Steering = Desired minus Velocity
  let steer = p5.Vector.sub(desired,this.velocity);
  steer.limit(this.maxforce);  // Limit to maximum steering force
  return steer;
}
Small.prototype.separate2 = function(small, main) {
  let desiredseparation = 25.0;
  let steer = createVector(0, 0);
  let count = 0;
  // For every boid in the system, check if it's too close
  for (let i = 0; i < main.length; i++) {
    let d = p5.Vector.dist(this.position,main[i].position);
    // If the distance is greater than 0 and less than an arbitrary amount (0 when you are yourself)
    if ((d > 0) && (d < desiredseparation)) {
      // Calculate vector pointing away from neighbor
      let diff = p5.Vector.sub(this.position, main[i].position);
      diff.normalize();
      diff.div(d);        // Weight by distance
      steer.add(diff);
      count++;            // Keep track of how many
    }
  }
  // Average -- divide by how many
  if (count > 0) {
    steer.div(count);
  }

  // As long as the vector is greater than 0
  if (steer.mag() > 0) {
    // Implement Reynolds: Steering = Desired - Velocity
    steer.normalize();
    steer.mult(this.maxspeed);
    steer.sub(this.velocity);
    steer.limit(this.maxforce);
  }
  return steer;
}
Small.prototype.separate = function(small) {
  let desiredseparation = 25.0;
  let steer = createVector(0, 0);
  let count = 0;
  // For every boid in the system, check if it's too close
  for (let i = 0; i < small.length; i++) {
    let d = p5.Vector.dist(this.position,small[i].position);
    // If the distance is greater than 0 and less than an arbitrary amount (0 when you are yourself)
    if ((d > 0) && (d < desiredseparation)) {
      // Calculate vector pointing away from neighbor
      let diff = p5.Vector.sub(this.position, small[i].position);
      diff.normalize();
      diff.div(d);        // Weight by distance
      steer.add(diff);
      count++;            // Keep track of how many
    }
  }
  // Average -- divide by how many
  if (count > 0) {
    steer.div(count);
  }

  // As long as the vector is greater than 0
  if (steer.mag() > 0) {
    // Implement Reynolds: Steering = Desired - Velocity
    steer.normalize();
    steer.mult(this.maxspeed);
    steer.sub(this.velocity);
    steer.limit(this.maxforce);
  }
  return steer;
}
Small.prototype.cohesion = function(small) {
  let neighbordist = 50;
  let sum = createVector(0, 0);   // Start with empty vector to accumulate all locations
  let count = 0;
  for (let i = 0; i < small.length; i++) {
    let d = p5.Vector.dist(this.position,small[i].position);
    if ((d > 0) && (d < neighbordist)) {
      sum.add(small[i].position); // Add location
      count++;
    }
  }
  if (count > 0) {
    sum.div(count);
    return this.seek(sum);  // Steer towards the location
  } else {
    return createVector(0, 0);
  }
}
Small.prototype.align = function(small) {
  let neighbordist = 50;
  let sum = createVector(0,0);
  let count = 0;
  for (let i = 0; i < small.length; i++) {
    let d = p5.Vector.dist(this.position,small[i].position);
    if ((d > 0) && (d < neighbordist)) {
      sum.add(small[i].velocity);
      count++;
    }
  }
  if (count > 0) {
    sum.div(count);
    sum.normalize();
    sum.mult(this.maxspeed);
    let steer = p5.Vector.sub(sum, this.velocity);
    steer.limit(this.maxforce);
    return steer;
  } else {
    return createVector(0, 0);
  }
}
Small.prototype.avoid = function(small) {
  let steer = createVector(0, 0);
  if (this.position.x <= 0) {
    steer.add(createVector(1, 0));
  }
  if (this.position.x > width) { // width of canvas
    steer.add(createVector(-1, 0));
  }
  if (this.position.y <= 0) {
    steer.add(createVector(0, 1));
  }
  if (this.position.y > height) { // height of canvas
    steer.add(createVector(0, -1));
  }
  return steer;
}