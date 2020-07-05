//Vector Library [2D]
//CSCI 5611 Vector 2 Library [Solution]

//Instructions: Implement all of the following vector operations--

public class Vec2 {
  public float x, y;
  
  public Vec2(float x, float y){
    this.x = x;
    this.y = y;
  }
  
  public String toString(){
    return "(" + x + ", " + y +")";
  }
  
  public float length(){
    return sqrt(x*x + y*y);
  }
  
  public Vec2 plus(Vec2 rhs){
    return new Vec2(x + rhs.x, y + rhs.y);
  }
  
  public void add(Vec2 rhs){
    x += rhs.x;
    y += rhs.y;
  }
  
  public Vec2 minus(Vec2 rhs){
    return new Vec2(x - rhs.x, y - rhs.y);
  }
  
  public void subtract(Vec2 rhs){
    x -= rhs.x;
    y -= rhs.y;
  }
  
  public Vec2 times(float rhs){
    return new Vec2(x*rhs, y*rhs);
  }
  
  public void mul(float rhs){
    x *= rhs;
    y *= rhs;
  }
  
  public void normalize(){
    float mag = sqrt( x*x + y*y );
    x /= mag;
    y /= mag;
  }
  
  public Vec2 normalized(){
    float mag = sqrt( x*x + y*y );
    return new Vec2(x/mag, y/mag);
  }
  
  public float distanceTo(Vec2 rhs){
    float dx = rhs.x - x;
    float dy = rhs.y - y;
    float d = sqrt( dx*dx + dy*dy);
    return d;
  }
  
}

Vec2 interpolate(Vec2 a, Vec2 b, float t){
  float n = interpolate(a.x, b.x, t);
  float m = interpolate(a.y, b.y, t);
  return new Vec2(n, m);
}

float interpolate(float a, float b, float t){
  return a + ((b-a)*t);
}


float dot(Vec2 a, Vec2 b){
  float r = ((a.x * b.x) + (a.y * b.y));
  return r;
}

Vec2 projAB(Vec2 a, Vec2 b){
  return b.times(a.x*b.x + a.y*b.y);
  //return new Vec2(0, 0);
}
