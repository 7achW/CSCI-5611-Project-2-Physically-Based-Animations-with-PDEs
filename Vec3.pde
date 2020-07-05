//Vector Library [2D]
//CSCI 5611 Vector 3 Library [Incomplete]

//Instructions: Add 3D versions of all of the 2D vector functions
//              Vec3 must also support the cross product.
public class Vec3 {
  public float x, y, z;
  
  public Vec3(float x, float y, float z){
    this.x = x;
    this.y = y;
    this.z = z;
  }
  
  public String toString(){
    return "(" + x + "," + y + "," + z + ")";
  }
  
  public float length(){
    return sqrt( x*x + y*y + z*z);
  }
  
  public Vec3 plus(Vec3 rhs){
    return new Vec3(x + rhs.x, y + rhs.y, z + rhs.z);
  }
  
  public void add(Vec3 rhs){
    x += rhs.x;
    y += rhs.y;
    z += rhs.z;
  }
  
  public Vec3 minus(Vec3 rhs){
    return new Vec3(x - rhs.x, y - rhs.y, z - rhs.z);
  }
  
  public void subtract(Vec3 rhs){
    x -= rhs.x;
    y -= rhs.y;
    z -= rhs.z;
  }
  
  public Vec3 times(float rhs){
    return new Vec3(x*rhs, y*rhs, z*rhs);
  }
  
  public void mul(float rhs){
    x *= rhs;
    y *= rhs;
    z *= rhs;
  }
  
  public void normalize(){
    float mag = sqrt(x*x + y*y + z*z);
    x /= mag;
    y /= mag;
    z /= mag;
  }
  
  public Vec3 normalized(){
    float mag = sqrt(x*x + y*y + z*z);
    return new Vec3(x/mag, y/mag, z/mag);
  }
  
  public float distanceTo(Vec3 rhs){
    float dx = rhs.x - x;
    float dy = rhs.y - y;
    float dz = rhs.z - z;
    float d = sqrt(dx*dx + dy*dy + dz*dz);
    return d;
  }
}

//float interpolate(float a, float b, float t){
//  return a + ((b-a)*t);
//}

Vec3 interpolate(Vec3 a, Vec3 b, float t){
  float i = interpolate(a.x, b.x, t);
  float j = interpolate(a.y, b.y, t);
  float k = interpolate(a.z, b.z, t);
  return new Vec3( i, j, k); 
}

float dot(Vec3 a, Vec3 b){
  return a.x*b.x + a.y*b.y + a.z*b.z;
}

Vec3 cross(Vec3 a, Vec3 b){
  float i = a.y*b.z - a.z*b.y;
  float j = a.z*b.x - a.x*b.z;
  float k = a.x*b.y - a.y*b.x;
  return new Vec3(i, j, k);
}

Vec3 projAB(Vec3 a, Vec3 b){
  //return new Vec3(0.0,0.0,0.0);
  return b.times(a.x*b.x + a.y*b.y + a.z*b.z);
}
