/*
3D cloth simulation with air drag and object collision

Author: Zachary Wieczorek
*/

//General parameters
float gravity = 50;
float radius = 5;

//Object /obstacle parameters and dimensions
float objRad = 100;
float objX = mouseX;
float objY = mouseY;
//float objX = 550;
//float objY = 300;
//float objZ = 150;
float objZ =0;

//cloth base param.
float clothTopLeftX = 300;
float clothTopLeftY = 50;
float clothTopLeftZ = 0;

float clothSpaceX = 20;
float clothSpaceY = 20;
float clothSpaceZ = 0;  //flat along z axis


float restLen = 20;
float mass = 20; 
float k = 100;    //spring coefficient
float d = 100;   //dampening coefficient safe:60

int numBalls = 30;

Vec3[][][] pos = new Vec3[numBalls][numBalls][1];
Vec3[][][] vel = new Vec3[numBalls][numBalls][1];
Vec3[][][] velNew = new Vec3[numBalls][numBalls][1];  //hold velocity before applying

//best to use (0,0,1)*magnitude since this is normal to the cloth's surface
Vec3 wind = new Vec3(0,0,0);

Camera camera;

void setup() {
  size(800, 800, P3D);
  surface.setTitle("Proj2 Rope Sketch 3");
  scene();
  camera = new Camera();
}

void keyPressed()
{
  camera.HandleKeyPressed();
}

void keyReleased()
{
  camera.HandleKeyReleased();
}

void scene(){
  //populate cloth with sensible initial positions
  for(int i = 0; i < numBalls; i++){
    for(int j = 0; j < numBalls; j++){
      if(j == 0){
        //pos[i][j] = new Vec2(clothTopLeftX + (10*i), clothTopLeftY);
        pos[i][j][0] = new Vec3(clothTopLeftX + (clothSpaceX*(i+1)), clothTopLeftY, clothTopLeftZ);
      }else{
        //scrunched
        //pos[i][j][0] = new Vec3(clothTopLeftX + (clothSpaceX*(i+1)) + random(10)*j, clothTopLeftY + (clothSpaceY*(j)) + random(10), 0);
        //slightly wavy
        //pos[i][j][0] = new Vec3(clothTopLeftX + (clothSpaceX*(i+1)) + 10*j, clothTopLeftY + (clothSpaceY*(j)), 5*j);
        //horizontal (up in z)
        pos[i][j][0] = new Vec3(clothTopLeftX + (clothSpaceX*(i+1)), clothTopLeftY, 15*j);
      }
      vel[i][j][0] = new Vec3(0,0,0);
      velNew[i][j][0] = new Vec3(0,0,0);
    }
  }
}

void update(float dt){
  for(int i = 0; i < numBalls - 1; i++){
    for(int j = 0; j < numBalls; j++){
      velNew[i][j][0] = vel[i][j][0];
    }
 }
 //Horizontal springs
 for(int i = 0; i < numBalls - 1; i++){
    for(int j = 0; j < numBalls; j++){
      
      //Compute spring force
      float len = pos[i][j][0].distanceTo(pos[i+1][j][0]);
      float springF = k*(len - restLen);
      //Compute dampening force
      Vec3 e = (pos[i+1][j][0].minus(pos[i][j][0])).normalized();
      float v1 = dot(e, vel[i][j][0]);
      float v2 = dot(e, vel[i+1][j][0]);
      float dampF = -d*(v1 - v2);
      //Sum forces
      float force = springF + dampF;
      //Update temp velocity (using Midpoint Integration)
      Vec3 temp = e.times(force);
      temp.mul(dt*1.5);
      //velNew[i][j] = new Vec2(0,0);
      velNew[i][j][0].add(temp);
      velNew[i+1][j][0].subtract(temp);
    }
 }
 //Vertical springs
 for(int i = 0; i < numBalls; i++){
    for(int j = 0; j < numBalls -1 ; j++){
      //Compute spring force
      float len = pos[i][j][0].distanceTo(pos[i][j+1][0]);
      float springF = k*(len - restLen);
      //Compute dampening force
      Vec3 e = (pos[i][j+1][0].minus(pos[i][j][0])).normalized();
      float v1 = dot(e, vel[i][j][0]);
      float v2 = dot(e, vel[i][j+1][0]);
      float dampF = -d*(v1 - v2);
      //Sum forces
      float force = springF + dampF;
      //Update temp velocity (using Midpoint Integration)
      Vec3 temp = e.times(force);
      temp.mul(dt*1.5);
      velNew[i][j][0].add( temp );
      velNew[i][j+1][0].subtract(temp);
    }
 }
 
 //Air friction /drag (comment section out to disable drag)
 //apply drag force to velNew and update with the rest
 //  f = (-1/2)*p*c*v^2*a*n
 //    v2an -> single value
 //    (-1/2)pc -> tuning parameter
  Vec3 dragF;
  for(int i = 0; i < numBalls - 1; i++){
    for(int j = 0; j < numBalls - 1; j++){
      Vec3 r1 = pos[i][j][0];
      Vec3 r2 = pos[i+1][j][0];
      Vec3 r3 = pos[i][j+1][0];
      //part of the noraml triangle  formula
      Vec3 n = cross( r2.minus(r1) , r3.minus(r1) );
      //velocity (avg of r1, r2, and r3 velocities)
      Vec3 v1 = vel[i][j][0];
      Vec3 v2 = vel[i+1][j][0];
      Vec3 v3 = vel[i][j+1][0];
      Vec3 s1 = v1.plus(v2);
      Vec3 s2 = s1.plus(v3);
      Vec3 v = s2.times(0.33);
      v = v.minus(wind);
      float vl = v.length();
      float vdotn = dot(v,n);
      float nl = n.length();
      nl = 2*nl;
      float z = (vl*vdotn) / (nl);
      Vec3 v2an = n.times(z);
      //Vec3 v2an =  n.times(  (  (  v.length()* dot(v,n)  )  /  (  2*n.length()  )  )  );     //number stability is a bitch
      float pc = -0.001;
      dragF = v2an.times(pc);
      
      ////apply friction force to velNew
      Vec3 temp = dragF;
      temp.mul(0.33);
      if(i==0 & j == numBalls-2){println("dragF_1 is : " + temp);}
      temp.mul(dt*1.5);
      velNew[i][j][0].add(temp);
      velNew[i+1][j][0].add(temp);
      velNew[i][j+1][0].add(temp);
      
      //and again, for the other triagle in the box
      r1 = pos[i+1][j+1][0];
      //r2 = pos[i+1][j][0];
      //r3 = pos[i][j+1][0];
      
      //part of the noraml triangle  formula
      n = cross( r2.minus(r1) , r3.minus(r1) );
      n.times(-1);
      //velocity (avg of r1, r2, and r3 velocities)
      v1 = vel[i+1][j+1][0];
      //v2 = vel[i+1][j][0];
      //v3 = vel[i][j+1][0];
      
      s1 = v1.plus(v2);
      s2 = s1.plus(v3);
      v = s2.times(0.33);
      v = v.minus(wind);
      
      vl = v.length();
      vdotn = dot(v,n);
      nl = n.length();
      nl = 2*nl;
      z = (vl*vdotn) / (nl);
      v2an = n.times(z);
      //pc = -0.005;
      dragF = v2an.times(pc);
      
      ////apply friction force to velNew
      temp = dragF;
      temp.mul(0.33);
      if(i== 0 & j == numBalls-2 ){println("dragF_2 is : " + temp);}
      temp.mul(dt*1.5);
      velNew[i+1][j+1][0].add(temp);
      velNew[i+1][j][0].add(temp);
      velNew[i][j+1][0].add(temp);
      
    }
  }
 
 //update velocity
  float accG = gravity*(1/mass);
  Vec3 g = new Vec3(0, accG, 0);
  for(int i = 0; i < numBalls; i++){
    for(int j = 0; j < numBalls; j++){
      velNew[i][j][0].add(g);
      if(j == 0){
        Vec3 anchor = new Vec3(0,0,0);  //no movement at the anchor point
        velNew[i][j][0] = anchor;
      }
      vel[i][j][0] = velNew[i][j][0];
      Vec3 temp = vel[i][j][0].times(dt*1.5);
      pos[i][j][0].add(temp);
    }
  }
  
  //Collision handling (comment section out to disable collision)
  for(int i = 0; i < numBalls; i++){
    for(int j = 0; j < numBalls; j++){
      Vec3 objPos = new Vec3(objX, objY, objZ);
      float d = objPos.distanceTo(pos[i][j][0]);
      objX = mouseX;
      objY=  mouseY;
      if(d < objRad + 0.09){
        //get normal vector
        Vec3 n = ((objPos.minus(pos[i][j][0])).times(-1)).normalized();
        //apply dirction to velocity as the bounce vector
        Vec3 bounce = n.times(dot( vel[i][j][0],n));
        bounce.mul(.9);
        //apply bounce to velocity
        vel[i][j][0].subtract(bounce);
        //decollide
        float rx = n.x*(0.1  + objRad - d);
        float ry = n.y*(0.1  + objRad - d);
        float rz = n.z*(0.1  + objRad - d);
        Vec3 decollide = new Vec3(rx,ry,rz);
        pos[i][j][0].add(decollide);
        //println("COLLISION");
      }
    }
  }
}

void draw(){
  background(255,255,255);
  update(0.005);
  camera.Update(1.0/frameRate);
  
  //draw cloth
  fill(0,0,0);
  for(int i = 0; i < numBalls - 1; i++){
    for(int j = 0; j < numBalls; j++){
      if(i == 0){
        pushMatrix();
        translate(pos[i][j][0].x, pos[i][j][0].y, pos[i][j][0].z);
        //sphere(radius);
        popMatrix();
      }
      pushMatrix();
      if(j == numBalls - 1 | j == 0){
        strokeWeight(2);
        line(pos[i][j][0].x,pos[i][j][0].y, pos[i][j][0].z ,pos[i+1][j][0].x,pos[i+1][j][0].y, pos[i+1][j][0].z);
        strokeWeight(1);
      }else{
        line(pos[i][j][0].x,pos[i][j][0].y, pos[i][j][0].z ,pos[i+1][j][0].x,pos[i+1][j][0].y, pos[i+1][j][0].z);
      }
      translate(pos[i+1][j][0].x, pos[i+1][j][0].y, pos[i][j][0].z);
      //sphere(radius);
      popMatrix();
    }
  }
  for(int i = 0; i < numBalls; i++){
    for(int j = 0; j < numBalls-1; j++){
      pushMatrix();
      strokeWeight(2);
      line(pos[i][j][0].x, pos[i][j][0].y, pos[i][j][0].z ,pos[i][j+1][0].x, pos[i][j+1][0].y, pos[i][j+1][0].z);
      strokeWeight(1);
      translate(pos[i][j+1][0].x, pos[i][j+1][0].y, pos[i][j+1][0].z);
      //sphere(radius);
      popMatrix();
    }
  }
  for(int i = 0; i < numBalls-1; i++){
    for(int j = 0; j < numBalls-1; j++){
      pushMatrix();
      //line(pos[i][j][0].x, pos[i+1][j][0].y, pos[i][j][0].z,pos[i+1][j+1][0].x, pos[i][j+1][0].y, pos[i+1][j+1][0].z);
      translate(pos[i][j+1][0].x, pos[i][j+1][0].y, pos[i][j+1][0].z);
      //sphere(radius);
      popMatrix();
    }
  }
  stroke(0,0,255);
  fill(0,0,255);
  pushMatrix();
  //translate(objX, objY, objZ);
  translate(mouseX, mouseY, objZ);
  //sphere(objRad); //Uncomment this to draw the obstacle
  popMatrix();
  stroke(0,0,0);
  fill(0,0,0);
}
