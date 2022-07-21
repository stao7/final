let points;
let record = 0;
let fish;
let num = 0;
let evo = 0;
let count_small_fish = 20;
let count_bomb = 0;
let limit_bomb = 0;
let count_big_fish = 3;
let flag = false;
let bombF = false;
let x = 0;
var chomp;
var eat;
var levelup;
var die;
var exp1;
let p;
let v;
function preload(){
  chomp = new Audio('./assets/Eat.mp3');
  eat = new Audio('./assets/Eating.mp3');
  levelup = new Audio('./assets/levelup.wav');
  die = new Audio('./assets/dropitem.wav');
  exp1 = new Audio('./assets/explode.wav');
}
function setup() {
  num = 0;
  evo = 0;
  count_small_fish = 20;
  count_big_fish = 3;
  count_bomb = 0;
  limit_bomb = 0;
  flag = false;
  bombF = false;
  x = 0;
  points = 0;
  createCanvas(1000, 700);
  v = createP("Score " + points);
  p = createP("Be careful of the big fish who may eat you and the game will restart!");
  createP("Highest Record " + record);
  
  fish = new Fish();
  for(let i = 0; i < 20; i++){
    fish.addBubble(new Bubble(random(0, width), height));
  }
  fish.addMain(new Main(width/2, height/2));
  for(let i = 0; i < 20; i++){
    fish.addSmall(new Small(width, height));
  }
  fish.addBig(new Big(0, 0));
  
}
function draw(){
  background(102,178,255);
  fish.run();
  //small
  v.remove();
  v = createP("Score " + points);
  if(count_small_fish < 20){
    let i = random();
    if(i > 0 && i < 0.25){
      fish.addSmall(new Small(width, height));
    }
    else if(i > 0.25 && i <0.5){
      fish.addSmall(new Small(0, height));
    }
    else if(i > 0.5 && i <0.75){
      fish.addSmall(new Small(width, 0));
    }
    else{
      fish.addSmall(new Small(0, 0));
    }
    count_small_fish++;
  }
  if(num > 5 && flag == false){
    count_big_fish = 1;
    flag = true;
  }
  if(evo >= 5){
    if(num < 15){
      evo = 0;
      x += 1;
      levelup.play();
      if(x == 2){
        p.remove();
        createP("Now you can eat the Big fish!");
        createP("They worth more points but do not get hit by the bomb.");
      }
      if(x > 7){
        limit_bomb = x* 2
      }
      limit_bomb = x;
    }
    else{
      evo = -num;
      x += 1;
      levelup.play();
      if(x > 7){
        limit_bomb = x* 2
      }
      limit_bomb = x;
    }
    points += 10;
  }
  //big
  if(count_big_fish < 3){
    let i = random();
    if(i > 0 && i < 0.25){
      fish.addBig(new Big(width, height));
    }
    else if(i > 0.25 && i <0.5){
      fish.addBig(new Big(0, height));
    }
    else if(i > 0.5 && i <0.75){
      fish.addBig(new Big(width, 0));
    }
    else{
      fish.addBig(new Big(0, 0));
    }
    count_big_fish++;
  }
  if(x > 2){
    bombF = true;
    limit_bomb = x;
  }
  if(count_bomb < limit_bomb && bombF == true){
    let i = random();
    if(i > 0 && i < 0.25){
      fish.addBomb(new Bomb(width, height/ 2));
    }
    else if(i > 0.25 && i <0.5){
      fish.addBomb(new Bomb(0, height/ 2));
    }
    else if(i > 0.5 && i <0.75){
      fish.addBomb(new Bomb(width/2, 0));
    }
    else{
      fish.addBomb(new Bomb(width/2, height));
    }
    count_bomb++;
  }


}
function Fish(){
  this.bomb = [];
  this.main = [];
  this.small = [];
  this.big = [];
  this.bubble = [];
  
}
Fish.prototype.run = function(){
  for (let i = 0; i < this.bubble.length; i++) {
    this.bubble[i].run(this.bubble);  
  }
  for (let i = 0; i < this.bomb.length; i++){
    this.bomb[i].run(this.main, this.small, this.big, this.bomb);
  }
  for (let i = 0; i < this.main.length; i++){
    this.main[i].run(this.main, this.small, this.big, this.bomb);
  }
  for (let i = 0; i < this.small.length; i++) {
    this.small[i].run(this.main, this.small, this.big, this.bomb);  
  }
  for (let i = 0; i < this.big.length; i++) {
    this.big[i].run(this.main, this.small, this.big, this.bomb);  
  }
}
Fish.prototype.addBubble = function(b) {
  this.bubble.push(b);
}
Fish.prototype.addMain = function(b) {
  this.main.push(b);
}
Fish.prototype.addSmall = function(b) {
  this.small.push(b);
}
Fish.prototype.addBig = function(b) {
  this.big.push(b);
}
Fish.prototype.addBomb = function(b) {
  this.bomb.push(b);
}
//bomb
function Bomb(x, y){
  this.acceleration = createVector(0, 0);
  this.velocity = createVector(random(-1, 1), random(-1, 1));
  this.position = createVector(x, y);
  this.r = 3.0;
  this.maxspeed = 1;
  this.maxforce = 0.05;
}
Bomb.prototype.applyForce = function(force) {
  // We could add mass here if we want A = F / M
  this.acceleration.add(force);
}
Bomb.prototype.run = function(main, small, big, bomb) {
  this.render(main, small, big, bomb);
  this.flock(main, small, big, bomb);
  this.update();
}
Bomb.prototype.flock = function(main, small, big, bomb){
  let avo = this.avoid(main, small, big, bomb);
  avo.mult(1.0);
  this.applyForce(avo);
}
Bomb.prototype.render = function(main, small, big, bomb){
  push();
  translate(this.position.x, this.position.y);
  noStroke();
  fill(153, 153, 0);
  beginShape();
  vertex(-5, -this.r- 10);
  vertex(0, 0);
  vertex(-35, -this.r - 35);
  vertex(-25, -this.r - 43);
  endShape(CLOSE);
  fill(0,0,0);
  circle(-15, -this.r - 18, 20);
  circle(0, -this.r, 50);
  fill(204,229,255);
  circle(-10, -this.r - 10, 10);
  stroke(255,51,51);
  fill(255,255,51);
  beginShape();
  vertex(-25, -this.r- 35);
  vertex(-35, -this.r- 45);
  vertex(-25, -this.r- 30);
  vertex(-30, -this.r - 45);
  vertex(-27, -this.r- 37);
  vertex(-35, -this.r - 30);
  vertex(-18, -this.r - 30);
  endShape(CLOSE);
  pop();
}
Bomb.prototype.update = function(){
  this.velocity.add(this.acceleration);
  this.velocity.limit(this.maxspeed);
  this.position.add(this.velocity);
  // Reset accelertion to 0 each cycle
  this.acceleration.mult(0);
}
Bomb.prototype.avoid = function(main, small, big, bomb) {
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
//bubble
function Bubble(x, y){
  this.acceleration = createVector(0, 0);
  this.velocity = createVector(random(-1, 1), random(-1, 1));
  this.position = createVector(x, y);
  this.r = 3.0;
  this.maxspeed = 2;
  this.maxforce = 0.05;
}
Bubble.prototype.run = function(bubble) {
  this.render(bubble);
  this.update();
  this.avoid();
}
Bubble.prototype.render = function(bubble){
  push();
  translate(this.position.x, this.position.y);
  fill(102,178,255);
  stroke(204, 255, 255);
  circle(0, -this.r, 50);
  noStroke();
  fill(204,229,255);
  circle(10, -this.r - 10, 10);
  pop();
}
Bubble.prototype.update = function() {
  // Update velocity
  this.velocity.add(this.acceleration);
  // Limit speed
  this.velocity.limit(this.maxspeed);
  this.position.add(this.velocity);
  // Reset accelertion to 0 each cycle
  this.acceleration.mult(0);
}
Bubble.prototype.applyForce = function(force) {
  // We could add mass here if we want A = F / M
  this.acceleration.add(force);
}
Bubble.prototype.avoid = function() {
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
  this.applyForce(steer);
}



function Main(x, y){
  this.acceleration = createVector(0, 0);
  this.velocity = createVector(random(-1, 1), random(-1, 1));
  this.position = createVector(x, y);
  this.r = 3.0;
  this.maxspeed = 2;
  this.maxforce = 0.05;
}
Main.prototype.run = function(main, small, big, bomb) {
  this.flock(main, small, big, bomb);
  this.update(main, small, big);
  this.render(main, small, big, bomb);
}
Main.prototype.applyForce = function(force) {
  // We could add mass here if we want A = F / M
  this.acceleration.add(force);
}
Main.prototype.flock = function(main, small, big, bomb){
  let ctr = this.control(main, small, big);
  let avo = this.avoid(main, small, big);
  ctr.mult(1.0);
  avo.mult(1.0);
  this.applyForce(ctr);
  this.applyForce(avo);
}
Main.prototype.update = function(main, small, big){
  this.velocity.add(this.acceleration);
  this.velocity.limit(this.maxspeed - 0.1*x);
  this.position.add(this.velocity);
  // Reset accelertion to 0 each cycle
  this.acceleration.mult(0);
}
Main.prototype.render = function(main, small, big, bomb){
  for(let i = 0; i < bomb.length; i++){
    let d = p5.Vector.dist(this.position,bomb[i].position);
    if(d < (15 + 10 * x)){
      removeElements();
      createP("The bomb exploded!");
      exp1.play();
      die.play(); 
      if(points > record){
        record = points;
      }
      setup();
    }
  }
  
  let theta = this.velocity.heading() + radians(90);
  push();
  fill(255,128,0);
  noStroke();
  translate(this.position.x, this.position.y);
  rotate(theta);
  ellipse(0, this.r, 20 + x*10, 2*(20 + x*10));
  noStroke();
  beginShape();
  vertex(0, this.r * 2 + 9*x);
  vertex(-this.r*(3 + 2*x), this.r * 12 + 12*x);
  vertex(this.r*(3 + 2*x), this.r * 12 + 12*x);
  endShape(CLOSE);
  fill(0,0,0);
  stroke(200);
  circle(0, -this.r - 9 * x, 4 + x);
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
Main.prototype.control = function(main, small, big) {
  return this.seek(createVector(mouseX, mouseY));  // Steer towards the location
}
Main.prototype.avoid = function(main, small, big) {
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



function Small(x, y){
  this.acceleration = createVector(0, 0);
  this.velocity = createVector(random(-1, 1), random(-1, 1));
  this.position = createVector(x, y);
  this.r = 3.0;
  this.maxspeed = 3;
  this.maxforce = 0.05;
}

Small.prototype.run = function(main, small, big, bomb) {
  this.flock(main, small, big);
  this.update();
  this.render(main, small, big, bomb);
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
Small.prototype.render = function(main, small, big, bomb) {
  let neighbordist = 23 + x*10;
  let neighbordistbig = 38;
  let neighbordistbomb = 15;
  let d = p5.Vector.dist(this.position,main[0].position);
  
  if(d < neighbordist){
    for(let i = 0; i < small.length; i++){
      if(small[i] == this){
        small.splice(i, 1);
        num++;
        count_small_fish--;
        evo++;
        chomp.play();
        points += 3;
      }
    }
  }
  else{
    for(let i = 0; i < big.length; i++){
      let db = p5.Vector.dist(this.position,big[i].position);
      if(db < neighbordistbig){
        for(let i = 0; i < small.length; i++){
          if(small[i] == this){
            small.splice(i, 1);
            count_small_fish--;
            eat.play();   
            return;
          }
        }
      }
    }
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
  
}
  
Small.prototype.flock = function(main, small, big, bomb){
  let sep = this.separate(main, small, big);  // Separation
  let sep2 = this.separate2(main, small, big); 
  let ali = this.align(main, small, big);      // Alignment
  let coh = this.cohesion(main, small, big);   // Cohesion
  let avo = this.avoid(main, small, big);   
  let dod = this.avoid(main, small, big);  // Avoid walls
  // Arbitrarily weight these forces
  sep.mult(6.0);
  sep2.mult(10.0);
  ali.mult(2.0);
  coh.mult(1.0);
  avo.mult(2.0);
  dod.mult(5.0)
  // Add the force vectors to acceleration
  this.applyForce(sep);
  this.applyForce(sep2);
  this.applyForce(ali);
  this.applyForce(coh);
  this.applyForce(avo);
  this.applyForce(dod);
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
Small.prototype.separate2 = function(main, small, big) {
  let desiredseparation = 35 + x * 10;
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
Small.prototype.separate = function(main, small, big) {
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
Small.prototype.cohesion = function(main, small, big) {
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
Small.prototype.align = function(main, small, big) {
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
Small.prototype.avoid = function(main, small, big) {
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
Small.prototype.dodge = function(main, small, big) {
  let desiredseparation = 70.0 + 8*x;
  let steer = createVector(0, 0);
  let count = 0;
  // For every boid in the system, check if it's too close
  for (let i = 0; i < big.length; i++) {
    let d = p5.Vector.dist(this.position,big[i].position);
    // If the distance is greater than 0 and less than an arbitrary amount (0 when you are yourself)
    if ((d > 0) && (d < desiredseparation)) {
      // Calculate vector pointing away from neighbor
      let diff = p5.Vector.sub(this.position, big[i].position);
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



function Big(x, y){
  this.acceleration = createVector(0, 0);
  this.velocity = createVector(random(-1, 1), random(-1, 1));
  this.position = createVector(x, y);
  this.r = 3.0;
  this.maxspeed = 1.5;
  this.maxforce = 0.05;
}
Big.prototype.run = function(main, small, big, bomb) {
  this.flock(main, small, big, bomb);
  this.update();
  this.render(main, small, big, bomb);
}
Big.prototype.applyForce = function(force) {
  // We could add mass here if we want A = F / M
  this.acceleration.add(force);
}
Big.prototype.update = function() {
  // Update velocity
  this.velocity.add(this.acceleration);
  // Limit speed
  this.velocity.limit(this.maxspeed);
  this.position.add(this.velocity);
  // Reset accelertion to 0 each cycle
  this.acceleration.mult(0);
}
Big.prototype.render = function(main, small, big, bomb) {
  let neighbordist = 25 + x * 10;
  let d = p5.Vector.dist(this.position,main[0].position);
  if (d > neighbordist) {
    let theta = this.velocity.heading() + radians(90);
    fill(0,0,253); 
    push();
    translate(this.position.x, this.position.y);
    noStroke();
    rotate(theta);
    ellipse(0, this.r, 40, 80);
    beginShape();
    vertex(0, -this.r * 4);
    vertex(-this.r*8, this.r * 18);
    vertex(this.r*8, this.r * 18);
    endShape(CLOSE);
    fill(0,0,0);
    stroke(200);
    circle(0, -this.r- 10, 5);
    pop();
  }
  else{
    if(x < 2){
      removeElements();
      createP("Big fish ate you!");
      eat.play();
      die.play();
      if(points > record){
        record = points;
      }
      setup();
      
    }else{
      for(let i = 0; i < big.length; i++){
        if(big[i] == this){
          big.splice(i, 1);
          num++;
          evo += 4;
          chomp.play();
          count_big_fish--;
          points += 10;
        }
      }
    }
  }
}
Big.prototype.flock = function(main, small, big, bomb){
  let sep = this.separate(main, small, big);  // Separation 
  let avo = this.avoid(main, small, big);     // Avoid walls
  let atr = this.attract(main, small, big);
  if(x < 2){
    let atm = this.attractMain(main, small, big);
    atm.mult(2.0);
    this.applyForce(atm);
  }
  else if(x > 3){
    let dod = this.avoid(main, small, big);  
    dod.mult(4.0);
    this.applyForce(dod);
  }
  
  // Arbitrarily weight these forces
  sep.mult(2.0);
  avo.mult(3.0);
  atr.mult(3.0);

  
  // Add the force vectors to acceleration
  this.applyForce(sep);
  this.applyForce(avo);
  this.applyForce(atr);
  
  
}
Big.prototype.seek = function(target) {
  let desired = p5.Vector.sub(target,this.position);  // A vector pointing from the location to the target
  // Normalize desired and scale to maximum speed
  desired.normalize();
  desired.mult(this.maxspeed);
  // Steering = Desired minus Velocity
  let steer = p5.Vector.sub(desired,this.velocity);
  steer.limit(this.maxforce);  // Limit to maximum steering force
  return steer;
}
Big.prototype.separate = function(main, small, big) {
  let desiredseparation = 50.0;
  let steer = createVector(0, 0);
  let count = 0;
  // For every boid in the system, check if it's too close
  for (let i = 0; i < big.length; i++) {
    let d = p5.Vector.dist(this.position,big[i].position);
    // If the distance is greater than 0 and less than an arbitrary amount (0 when you are yourself)
    if ((d > 0) && (d < desiredseparation)) {
      // Calculate vector pointing away from neighbor
      let diff = p5.Vector.sub(this.position, big[i].position);
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
Big.prototype.avoid = function(main, small, big) {
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
Big.prototype.attract = function(main, small, big) {
  let neighbordist = 100;
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

Big.prototype.attractMain = function(main, small, big) {
  let neighbordist = 80;
  let sum = createVector(0, 0);   // Start with empty vector to accumulate all locations
  let count = 0;
  for (let i = 0; i < main.length; i++) {
    let d = p5.Vector.dist(this.position,main[i].position);
    if ((d > 0) && (d < neighbordist)) {
      sum.add(main[i].position); // Add location
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
Big.prototype.dodge = function(main, small, big) {
  let desiredseparation = 75.0+ 8*x;
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

