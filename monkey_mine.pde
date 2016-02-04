
import javax.vecmath.Quat4f;
import javax.vecmath.Vector3f;
import javax.vecmath.Vector4f;
import javax.vecmath.Matrix3f;
import javax.vecmath.Matrix4f;
import com.bulletphysics.dynamics.constraintsolver.Generic6DofConstraint;
import com.bulletphysics.collision.dispatch.GhostPairCallback;
import com.bulletphysics.collision.dispatch.CollisionFlags;
import com.bulletphysics.collision.dispatch.CollisionObject;
import com.bulletphysics.collision.dispatch.GhostObject;
import com.bulletphysics.linearmath.QuaternionUtil; 
import com.bulletphysics.linearmath.Transform;
import com.bulletphysics.dynamics.RigidBody; 

import peasy.*;
import bRigid.*;
import java.util.*;

import processing.serial.*;

PeasyCam cam;

BPhysics physics;

BBox box1;
BBox box2;
BBox wall1;
BJointHinge hinge;
BJointHinge hinge2;
BObject b1;

BSphere sphere;

//GhostPairCallback ghostPairCallback;

ArrayList<Mine> mines;
ArrayList<Block> obstracles;

//true : Mines are visible
//false: Mines are unvisible
public static final boolean visiblity = true;

int[][] map = {
  {0,0,0,0,0, 0,0,0,0,0, 0,0,0,0,0, 0,0,0,0,0},
  {0,0,0,0,0, 0,0,3,3,3, 3,3,3,0,0, 0,0,0,0,0},
  {0,0,0,0,0, 0,0,3,1,1, 1,1,3,0,0, 0,0,0,0,0},
  {0,0,0,0,0, 0,0,3,1,1, 1,1,3,0,0, 0,0,0,0,0},
  {0,0,0,0,0, 0,0,3,1,1, 1,1,3,0,0, 0,0,0,0,0},
 
  {0,0,0,0,0, 0,0,3,1,1, 1,1,3,0,0, 0,0,0,0,0},
  {0,0,0,0,0, 0,0,0,0,0, 0,0,0,0,0, 0,0,0,0,0},
  {0,0,0,0,0, 0,0,0,0,0, 0,0,0,0,0, 0,0,0,0,0},
  {0,0,0,0,0, 0,0,0,0,1, 1,0,0,0,0, 0,0,0,0,0},
  {0,0,0,0,0, 0,0,0,0,1, 1,0,0,0,0, 0,0,0,0,0},
  
  {0,0,0,0,0, 0,0,0,0,1, 1,0,0,0,0, 0,0,0,0,0},
  {0,0,0,0,0, 0,0,0,0,1, 1,0,0,0,0, 0,0,0,0,0},
  {0,0,0,0,0, 0,0,0,0,1, 1,0,0,0,0, 0,0,0,0,0},
  {0,0,0,0,0, 0,0,0,0,2, 1,0,0,0,0, 0,0,0,0,0},
  {0,0,0,0,0, 0,0,0,0,1, 1,0,0,0,0, 0,0,0,0,0},
  
  {0,0,0,0,0, 0,0,0,0,1, 1,0,0,0,0, 0,0,0,0,0},
  {0,0,0,0,0, 0,3,3,3,1, 1,3,3,3,0, 0,0,0,0,0},
  {0,0,0,0,0, 0,3,1,1,1, 10,1,1,3,0, 0,0,0,0,0},
  {0,0,0,0,0, 0,3,1,1,1, 2,1,1,3,0, 0,0,0,0,0},
  {0,0,0,0,0, 0,3,3,3,3, 3,3,3,3,0, 0,0,0,0,0},
  
  /*{1,1,1,1,1, 1,1,1,1,1, 1,1,1,1,1, 1,1,1,1,1},
  {1,1,1,1,1, 1,1,1,1,1, 1,1,1,1,1, 1,1,1,1,1},
  {1,1,1,1,1, 1,1,0,0,1, 1,1,0,0,1, 1,1,1,1,1},
  {1,1,1,1,1, 1,1,0,0,1, 1,2,0,0,1, 1,1,1,1,1},
  {1,1,1,1,1, 1,1,0,0,1, 1,1,0,0,1, 1,1,1,1,1},
  
  {1,1,1,1,1, 1,1,0,0,1, 1,1,0,0,1, 1,1,1,1,1},
  {1,1,1,1,1, 1,1,0,0,1, 1,1,0,0,1, 1,1,1,1,1},
  {1,1,1,1,1, 1,1,0,0,2, 1,1,0,0,1, 1,1,1,1,1},
  {1,1,1,1,1, 1,1,0,0,1, 1,1,0,0,1, 1,1,1,1,1},
  {1,1,1,1,1, 1,1,0,0,1, 1,1,0,0,1, 1,1,1,1,1},
  
  {1,1,1,1,1, 1,1,0,0,1, 1,1,0,0,1, 1,1,1,1,1},
  {1,1,1,1,1, 1,1,0,0,1, 1,1,0,0,1, 1,1,1,1,1},
  {1,1,1,1,1, 1,1,1,3,1, 1,1,1,1,1, 1,1,1,1,1},
  {1,1,1,1,1, 1,1,1,3,1, 1,1,1,1,1, 1,1,1,1,1},
  {1,1,1,1,1, 1,1,1,3,1, 1,1,1,1,1, 1,1,1,1,1},
  
  {1,1,1,1,1, 1,1,1,1,1, 1,1,1,1,1, 1,1,1,1,1},
  {1,1,1,1,1, 1,1,1,1,1, 1,1,1,1,1, 1,1,1,1,1},
  {1,1,1,1,1, 1,1,1,1,1, 1,1,1,1,1, 1,1,1,1,1},
  {1,1,1,1,1, 1,1,1,1,1, 1,1,1,1,1, 1,1,1,1,1},
  {1,1,1,1,1, 1,1,1,1,1, 1,1,1,1,1, 1,1,1,1,1},*/
};

float angle_x = .0f;
float angle_y = .0f;

Serial serial;
String data;
int x[];
int y[];
int z[];
int count = 0;

boolean serial_able = true;

boolean flag = false;

public void setup() {
  
  mines = new ArrayList<Mine>();
  obstracles = new ArrayList<Block>();
  size(640,480,P3D);
  frameRate(60);
  
  cam = new PeasyCam(this, 0,-200,0, 1000);
  
  Vector3f min = new Vector3f(-1200, -2500, -1200);
  Vector3f max = new Vector3f(1200, 2500, 1200);
  
  physics = new BPhysics(min, max);
  
  physics.world.setGravity(new Vector3f(0,500, 0));
  
  Vector3f pos1 = new Vector3f(0,0,0);
  Vector3f pos2 = new Vector3f(0,-60,0);
  Vector3f pos4 = new Vector3f(0, 0,0);
  
  box1 = new BBox(this,100,1000, 10,1000);
  box2 = new BBox(this,  100, 50, 50, 50);
  wall1 = new BBox(this, 100, 50,100, 50);
  
  //BConvexHull slope = new BConvexHull(this,1,"slope.obj",pos4,true,true);
  //slope.scale(30);
  
  
  
  b1 = new BObject(this,100,box1,pos1,true);
  
  
  //Kinematic Object Settings
  Transform transform = new Transform();
  b1.rigidBody.getMotionState().getWorldTransform(transform);
  b1.rigidBody.setCollisionFlags(b1.rigidBody.getCollisionFlags() | CollisionFlags.KINEMATIC_OBJECT);
  b1.rigidBody.setActivationState(CollisionObject.DISABLE_DEACTIVATION);
  
  Vector3f pos3 = new Vector3f(0,0,0);
  sphere = new BSphere(this,125,20,pos3,true);
  //sphere.setPosition(new Vector3f(0,-500,0));
  //sphere.rigidBody.setFriction(1.0);
  
  //physics.addBody(b1);
  physics.addBody(sphere);

  for (int j=0;j<20;j++) {
    for (int i=0;i<20;i++) {
      if (map[j][i] == 0)continue;
      
      if (map[j][i]==10) {
        sphere.setPosition(new Vector3f(50f*j-475f,-500,50f*i-475f));
        map[j][i]=1;
      }
      
      if (map[j][i] == 1) {
        Vector3f position = new Vector3f();
        BObject obj = new BObject(this,100,box2,position,true);
        
        Transform transform2 = new Transform();
        obj.rigidBody.getMotionState().getWorldTransform(transform2);
        obj.rigidBody.setCollisionFlags(obj.rigidBody.getCollisionFlags() | CollisionFlags.KINEMATIC_OBJECT);
        obj.rigidBody.setActivationState(CollisionObject.DISABLE_DEACTIVATION);
        
        physics.addBody(obj);
        
        Block m2 = new Block(50f*j-475f,-30f,50f*i-475f,obj);
        obstracles.add(m2);
      }
      
      if (map[j][i] == 3) {
        Vector3f position = new Vector3f();
        BObject obj = new BObject(this,100,wall1,position,true);
        
        Transform transform2 = new Transform();
        obj.rigidBody.getMotionState().getWorldTransform(transform2);
        obj.rigidBody.setCollisionFlags(obj.rigidBody.getCollisionFlags() | CollisionFlags.KINEMATIC_OBJECT);
        obj.rigidBody.setActivationState(CollisionObject.DISABLE_DEACTIVATION);
        
        physics.addBody(obj);
        
        Block m = new Block(50*j-475,-80f,50*i-475,obj);
        obstracles.add(m);
      }
      
      /*if (map[j][i] == 4) {
        Vector3f position = new Vector3f();
        BObject obj = new BObject(this,1,slope,position,true);
        
        Transform transform2 = new Transform();
        obj.rigidBody.getMotionState().getWorldTransform(transform2);
        obj.rigidBody.setCollisionFlags(obj.rigidBody.getCollisionFlags() | CollisionFlags.KINEMATIC_OBJECT);
        obj.rigidBody.setActivationState(CollisionObject.DISABLE_DEACTIVATION);
        
        physics.addBody(obj);
        
        Block m = new Block(50*j-475,-80f,50*i-475,obj);
        obstracles.add(m);
      }*/
      
      
      if (map[j][i] == 2) {
        Vector3f position_test = new Vector3f(30,-10,0);
        BObject sensor = new BObject(this,1,box2,position_test,true);
        
        Transform transform2 = new Transform();
        sensor.rigidBody.getMotionState().getWorldTransform(transform2);
        sensor.rigidBody.setCollisionFlags(sensor.rigidBody.getCollisionFlags() | CollisionFlags.KINEMATIC_OBJECT);
        sensor.rigidBody.setActivationState(CollisionObject.DISABLE_DEACTIVATION);
        //physics.addBody(test);
        
        //ghostPairCallback = new GhostPairCallback();
        //physics.world.getPairCache().setInternalGhostPairCallback(ghostPairCallback);
        
        GhostObject ghostObject = new GhostObject();
        ghostObject.setCollisionShape(sensor.collisionShape);
        ghostObject.setCollisionFlags(new CollisionFlags().NO_CONTACT_RESPONSE);
        ghostObject.setWorldTransform(sensor.transform);
        physics.world.addCollisionObject(ghostObject);
        
        Mine m = new Mine(50*j-475,50*i-475,sensor,ghostObject);
        mines.add(m);
        
        Vector3f position = new Vector3f();
        BObject obj = new BObject(this,100,box2,position,true);
        
        Transform transform3 = new Transform();
        obj.rigidBody.getMotionState().getWorldTransform(transform3);
        obj.rigidBody.setCollisionFlags(obj.rigidBody.getCollisionFlags() | CollisionFlags.KINEMATIC_OBJECT);
        obj.rigidBody.setActivationState(CollisionObject.DISABLE_DEACTIVATION);
        
        physics.addBody(obj);
        
        Block m2 = new Block(50f*j-475f,-30f,50f*i-475f,obj);
        obstracles.add(m2);
      }
      
      /*if (map[j][i] == 4) {
        Vector3f position_test = new Vector3f(30,-10,0);
        BObject sensor = new BObject(this,1,slope,position_test,true);
        
        Transform transform2 = new Transform();
        sensor.rigidBody.getMotionState().getWorldTransform(transform2);
        sensor.rigidBody.setCollisionFlags(sensor.rigidBody.getCollisionFlags() | CollisionFlags.KINEMATIC_OBJECT);
        sensor.rigidBody.setActivationState(CollisionObject.DISABLE_DEACTIVATION);
        physics.addBody(sensor);
        
        //ghostPairCallback = new GhostPairCallback();
        //physics.world.getPairCache().setInternalGhostPairCallback(ghostPairCallback);
        
        
        Block m = new Block(50*j-475,-110f,50*i-475,sensor);
        obstracles.add(m);
        
        Vector3f position = new Vector3f();
        BObject obj = new BObject(this,1,box2,position,true);
        
        Transform transform3 = new Transform();
        obj.rigidBody.getMotionState().getWorldTransform(transform3);
        obj.rigidBody.setCollisionFlags(obj.rigidBody.getCollisionFlags() | CollisionFlags.KINEMATIC_OBJECT);
        obj.rigidBody.setActivationState(CollisionObject.DISABLE_DEACTIVATION);
        
        physics.addBody(obj);
        
        Block m2 = new Block(50f*j-475f,-30f,50f*i-475f,obj);
        obstracles.add(m2);
      }*/
      
      
      
    }
    
    
    
  }
  
  String portName = Serial.list()[2];
  
  x = new int[50];
  y = new int[50];
  z = new int[50];
  
  if(portName.startsWith("/dev/cu.usbmodem")){
    serial = new Serial(this, portName, 9600);
    println(portName);
  }else{
    serial_able = false;
  }
  
}  


public void draw() {
  
  background(0);
  lights();
  
  
  if(serial_able){
    getangle();
  }
  else{
    getkey();
  }
  
  updateObstracle();
  updateGhost();
  
  physics.update();
  physics.display();
}


public void getkey() {
  if(keyPressed) {
  if (key == CODED) {
    if (keyCode == UP) {
      
      
      angle_y -= .005f;
      //setRotation(b1.rigidBody,new Vector3f(0,0,1), angle_y);
    }
    else if (keyCode == DOWN) {
      angle_y += .005f;
      //setRotation(b1.rigidBody,new Vector3f(0,0,1), angle_y);
    }
    else if (keyCode == RIGHT) {
      angle_x -= .005f;
      //setRotation(b1.rigidBody,new Vector3f(1,0,0), angle_x);
    }
    else if (keyCode == LEFT) {
      angle_x += .005f;
      //setRotation(b1.rigidBody,new Vector3f(1,0,0), angle_x);
      
      
    }
  }
  
  if(angle_x > .5f) angle_x = .45f;
  if(angle_x < -.5f) angle_x = -.45f;
  if(angle_y > .5f) angle_y = .45f;
  if(angle_y < -.5f) angle_y = -.45f;
  
  
  //Rotation Matrix
  /*Matrix4f bius = new Matrix4f(
    1,0,0,0,
    0,1,0,-200,
    0,0,1,0,
    0,0,0,1
  );*/
  
  
  //Rotation Matrix  
  Matrix3f mat1 = new Matrix3f(
    1,0,0,
    0,cos(angle_y),sin(angle_y),
    0,-sin(angle_y),cos(angle_y)
  );
  
  Matrix3f mat2 = new Matrix3f(
    cos(angle_x),sin(angle_x),0,
    -sin(angle_x) ,cos(angle_x),0,
    0,0,1
    );
    
  mat1.mul(mat2);
  
  //Quat4f qout = new Quat4f();
  
  Transform t= new Transform(mat1);
  //Transform t = new Transform(bius);
  //t.setRotation(t2.getRotation(qout));
  b1.rigidBody.getMotionState().setWorldTransform(t);
  
  }
  
}

public void updateObstracle() {
  for (Block obstracle: obstracles) {
    
    //float bius = 15.f;
    float bius = -obstracle.y * 1.f/4.f;
    
    Vector3f pos_1 = new Vector3f(cos(angle_x),sin(angle_y),0);
    Vector3f pos_2 = new Vector3f(0,-sin(angle_x),cos(angle_y));
    
    
    Vector3f up1 = new Vector3f(pos_1.y,-pos_1.x,0);
    Vector3f up2 = new Vector3f(0,-pos_2.z,+pos_2.y);
    
    pos_1.x *= obstracle.x;
    pos_1.y *= obstracle.z;
    
    pos_2.y *= obstracle.x;
    pos_2.z *= obstracle.z;
    
    //System.out.println(up1.x+","+up1.y+","+up1.z);
    //System.out.println(up2.x+","+up2.y+","+up2.z);
  
    if(up1.lengthSquared() > 0)
      up1.normalize();
    if(up2.lengthSquared() > 0)
      up2.normalize();
    
   //System.out.println(up1.x+","+up1.y+","+up1.z);
   //System.out.println(up2.x+","+up2.y+","+up2.z);
  
    //Vector3f up = new Vector3f(0,0,0);
    
    /*Matrix4f mat0 = new Matrix4f(
      1,0,0,pos_1.x+pos_2.x+bius*(up1.x+up2.x),
      0,1,0,pos_1.y+pos_2.y+bius*(up1.y+up2.y),
      0,0,1,pos_1.z+pos_2.z+bius*(up1.z+up2.z),
      0,0,0,1
    );*/
    
    
    Matrix4f mat1 = new Matrix4f(
      1,0,0                       ,pos_1.x+pos_2.x+bius*(up1.x+up2.x),
      0,cos(angle_y),sin(angle_y) ,pos_1.y+pos_2.y+bius*(up1.y+up2.y),
      0,-sin(angle_y),cos(angle_y),pos_1.z+pos_2.z+bius*(up1.z+up2.z),
      0,0,0,1
    );
    
    Matrix4f mat2 = new Matrix4f(
      cos(angle_x),sin(angle_x),0,0,
      -sin(angle_x),cos(angle_x),0,0,
      0,0,1,0,
      0,0,0,1
    );
    
    mat1.mul(mat2);
   
    Transform t=new Transform(mat1);
    obstracle.obj.rigidBody.getMotionState().setWorldTransform(t);
  }
  
  
}

public void getangle(){
  int a,b,c;
  
  a = b = c = 0;
  
  if (serial.available() > 15) {
    String get = serial.readStringUntil('$');
    //System.out.println(get);
    if (get != null) {
      String ss[] = split(get,",");
      if(ss.length >= 4){
        //System.out.println(ss[0] + ":" + ss[1] + ":" + ss[2] );
        try{
          x[count] = Integer.parseInt(ss[0]);
          y[count] = Integer.parseInt(ss[1]);
          z[count] = Integer.parseInt(ss[2]);
          if(flag){
            for(int i=0; i < 50; i++){
              a += x[i];
              b+= y[i];
              c += z[i];
            }
            a /= 50;
            b /= 50;
            c /= 50;
            angle_x = (float)(atan2(a-519, c-561) / PI * 180.0 / 100);
            angle_y = (float)(atan2(b-531, c-570) / PI * 180.0 / 100);
            System.out.println("X=" + angle_x + " Y=" + angle_y );
           }
          count++;
          if(count >= 50){
            count = 0;
            flag = true;
          }
        }catch (NumberFormatException nfex){
          System.out.println("Data Format Error!");
        }
      }
    }
  }
  
  if(angle_x > .5f) angle_x = .45f;
  if(angle_x < -.5f) angle_x = -.45f;
  if(angle_y > .5f) angle_y = .45f;
  if(angle_y < -.5f) angle_y = -.45f;
  
  //Rotation Matrix  
  Matrix3f mat1 = new Matrix3f(
    1,0,0,
    0,cos(angle_y),sin(angle_y),
    0,-sin(angle_y),cos(angle_y)
  );
  
  Matrix3f mat2 = new Matrix3f(
    cos(angle_x),sin(angle_x),0,
    -sin(angle_x) ,cos(angle_x),0,
    0,0,1
    );
    
  mat1.mul(mat2);
  
  //Quat4f qout = new Quat4f();
  
  Transform t= new Transform(mat1);
  //Transform t = new Transform(bius);
  //t.setRotation(t2.getRotation(qout));
  b1.rigidBody.getMotionState().setWorldTransform(t);
  
}
public void updateGhost() {
  
  for (Mine mine: mines) {
    
    float bius = 30.f;
    
    Vector3f pos_1 = new Vector3f(cos(angle_x),sin(angle_y),0);
    Vector3f pos_2 = new Vector3f(0,-sin(angle_x),cos(angle_y));
    
    
    Vector3f up1 = new Vector3f(pos_1.y,-pos_1.x,0);
    Vector3f up2 = new Vector3f(0,-pos_2.z,+pos_2.y);
    
    pos_1.x *= mine.x;
    pos_1.y *= mine.y;
    
    pos_2.y *= mine.x;
    pos_2.z *= mine.y;
    
    //System.out.println(up1.x+","+up1.y+","+up1.z);
    //System.out.println(up2.x+","+up2.y+","+up2.z);
  
    if(up1.lengthSquared() > 0)
      up1.normalize();
    if(up2.lengthSquared() > 0)
      up2.normalize();
    
   //System.out.println(up1.x+","+up1.y+","+up1.z);
   //System.out.println(up2.x+","+up2.y+","+up2.z);
  
    //Vector3f up = new Vector3f(0,0,0);
    
    Matrix4f mat1 = new Matrix4f(
      1,0,0,                       pos_1.x+pos_2.x+bius*(up1.x+up2.x),
      0,cos(angle_y),sin(angle_y) ,pos_1.y+pos_2.y+bius*(up1.y+up2.y),
      0,-sin(angle_y),cos(angle_y),pos_1.z+pos_2.z+bius*(up1.z+up2.z),
      0,0,0,1
    );
    
    Matrix4f mat2 = new Matrix4f(
      cos(angle_x),sin(angle_x),0,0,
      -sin(angle_x),cos(angle_x),0,0,
      0,0,1,0,
      0,0,0,1
    );
    
    mat1.mul(mat2);
  
    Transform t= new Transform(mat1);
    mine.obj.rigidBody.getMotionState().setWorldTransform(t);
    mine.ghost.setWorldTransform(t);
    
    
    if (visiblity) {
      mine.obj.display();
    }
    
    Matrix4f meMat = new Matrix4f();
    Matrix4f mineMat = new Matrix4f();
    Transform meTransform = new Transform();
    t.getMatrix(meMat);
    sphere.rigidBody.getWorldTransform(meTransform).getMatrix(mineMat);
    
    Vector3f nearest = new Vector3f();
    
    nearest.x = mineMat.m03-meMat.m03;
    nearest.y = mineMat.m13-meMat.m13;
    nearest.z = mineMat.m23-meMat.m23;
    
    //System.out.println(nearest.lengthSquared());
    
    if (nearest.lengthSquared()< 1500) {
      
      Matrix4f vec = new Matrix4f(
        1,            0,           0, 0,
        0,cos(angle_y) ,sin(angle_y),-cos(angle_y),
        0,-sin(angle_y),cos(angle_y), -sin(angle_y),
        0,            0,           0, 1
      );
        
      vec.mul(mat2);
      Vector3f vec2 = new Vector3f(vec.m03,vec.m13,vec.m23);
      vec2.normalize();
      
      vec2.x *= 10000000;
      vec2.y *= 10000000;
      vec2.z *= 10000000;
      
      sphere.rigidBody.applyForce(vec2,sphere.getPosition());
    }
  }
  
}
