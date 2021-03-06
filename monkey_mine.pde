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
import java.util.Collections;
import java.nio.ByteBuffer;

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
ArrayList<Map> maps;
ArrayList<Goal> goals;

ArrayList<Integer> distances_;

int mapIndex;
ArrayList<Block> obstracles;

//true : Mines are visible
//false: Mines are unvisible
public static final boolean visiblity = true;

float angle_x = .0f;
float angle_y = .0f;

Serial serial_sensor;
Serial serial;
String data;
int x[];
int y[];
int z[];
int count = 0;

boolean serial_able = true;

boolean flag = false;

int[] nearest_mines;

class KeyThread extends Thread {
  public void run() {
    for(;;) {
      if(serial_able){
        getangle();
      }
      else{
        getkey();
      }
      try {
        Thread.sleep(1);
      } catch (InterruptedException e) {
      }
    }
  }
}

class LightThread extends Thread {
  public void run() {
    for(;;) {
      light();
      try {
        Thread.sleep(1);
      } catch (InterruptedException e) {
      }
    }
  }
}

public void setup() {

  mines = new ArrayList<Mine>();
  obstracles = new ArrayList<Block>();
  maps = new ArrayList<Map>();
  goals = new ArrayList<Goal>();
  distances_ = new ArrayList<Integer>();
  nearest_mines = new int[3];

  size(640,480,P3D); 
  frameRate(60);
  

  //add Map from MapList.cfg
  String[] lines = loadStrings("Map/MapList.cfg");
  for( int i = 0; i < lines.length; i++ ){
    maps.add( new Map( lines[i] ) );
  } 

  mapIndex = 0;
  
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
  
  b1 = new BObject(this,100,box1,pos1,true);
  
  //Kinematic Object Settings
  Transform transform = new Transform();
  b1.rigidBody.getMotionState().getWorldTransform(transform);
  b1.rigidBody.setCollisionFlags(b1.rigidBody.getCollisionFlags() | CollisionFlags.KINEMATIC_OBJECT);
  b1.rigidBody.setActivationState(CollisionObject.DISABLE_DEACTIVATION);
  
  Vector3f pos3 = new Vector3f(0,0,0);
  sphere = new BSphere(this,125,20,pos3,true);
  sphere.rigidBody.setRestitution(0.0f);
  //sphere.setPosition(new Vector3f(0,-500,0));
  //sphere.rigidBody.setFriction(1.0);
  
  physics.addBody(sphere);

  map_load();

  String portName = "/dev/cu.usbmodem14631";
  
  x = new int[50];
  y = new int[50];
  z = new int[50];
  
  if(portName.startsWith("/dev/cu.usbmodem")){
    serial = new Serial(this, portName, 9600);
    println(portName);
  }else{
    serial_able = false;
  }
  
  String portName_sensor = "/dev/cu.usbmodem14611";
  serial = new Serial(this,portName,9600);
  serial_sensor = new Serial(this, portName_sensor, 9600);

  KeyThread keyThread = new KeyThread();
  keyThread.start();
  LightThread lightThread = new LightThread();
  lightThread.start();

}

public void draw() {
  
  background(0);
  lights();
  
  /*
  if(serial_able){
    getangle();
  }
  else{
    getkey();
  }
  */
  updateObstracle();
  updateGhost();
  updateGoal();
  physics.update();
  physics.display();
}

public void getkey() {
  if(keyPressed) {
  if (key == CODED) {
    if (keyCode == UP) {
      angle_y -= .0005f;
      //setRotation(b1.rigidBody,new Vector3f(0,0,1), angle_y);
    }
    else if (keyCode == DOWN) {
      angle_y += .0005f;
      //setRotation(b1.rigidBody,new Vector3f(0,0,1), angle_y);
    }
    else if (keyCode == RIGHT) {
      angle_x -= .0005f;
      //setRotation(b1.rigidBody,new Vector3f(1,0,0), angle_x);
    }
    else if (keyCode == LEFT) {
      angle_x += .0005f;
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
  distances_.clear();
  distances_ = new ArrayList<Integer>();
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
    
    distances_.add((int)nearest.lengthSquared());
    
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
  Collections.sort(distances_);
  int len = distances_.size();
  System.out.println(len);
  if(len > 0){
  nearest_mines[0] = distances_.get(0);
  }
  if(len > 1){
  nearest_mines[1] = distances_.get(1);
  }
  if(len > 2){
  nearest_mines[2] = distances_.get(2);
  }
}

public void updateGoal() {
  
  for (Goal goal: goals) {
    
    float bius = 30.f;
    
    Vector3f pos_1 = new Vector3f(cos(angle_x),sin(angle_y),0);
    Vector3f pos_2 = new Vector3f(0,-sin(angle_x),cos(angle_y));
    
    
    Vector3f up1 = new Vector3f(pos_1.y,-pos_1.x,0);
    Vector3f up2 = new Vector3f(0,-pos_2.z,+pos_2.y);
    
    pos_1.x *= goal.x;
    pos_1.y *= goal.y;
    
    pos_2.y *= goal.x;
    pos_2.z *= goal.y;
    
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
    goal.obj.rigidBody.getMotionState().setWorldTransform(t);
    goal.ghost.setWorldTransform(t);
    
    goal.obj.display(255,0,0);
    
    
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
      System.out.println("Goal!!");
      map_reload();
    }
  }
  
}

void map_load() {
  for (int j=0;j<maps.get(mapIndex).getYLength();j++) {
    for (int i=0;i<maps.get(mapIndex).getXLength();i++) {
      if (maps.get(mapIndex).board[j][i] == 0)continue;
      
      //start position
      if (maps.get(mapIndex).board[j][i] == 10) {
        sphere.setPosition(new Vector3f(50f*j-475f,-500,50f*i-475f));
        maps.get(mapIndex).board[j][i]=1;
      }
      
      if (maps.get(mapIndex).board[j][i] == 1) {
        Vector3f position = new Vector3f();
        BObject obj = new BObject(this,100,box2,position,true);
        
        //Transform transform2 = new Transform();
        //obj.rigidBody.getMotionState().getWorldTransform(transform2);
        obj.rigidBody.setCollisionFlags(obj.rigidBody.getCollisionFlags() | CollisionFlags.KINEMATIC_OBJECT);
        obj.rigidBody.setActivationState(CollisionObject.DISABLE_DEACTIVATION);
        
        physics.addBody(obj);
        
        Block m2 = new Block(50f*j-475f,-30f,50f*i-475f,obj);
        obstracles.add(m2);
      }
      
      if (maps.get(mapIndex).board[j][i] == 3) {
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
      
      if (maps.get(mapIndex).board[j][i] == 2) {
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
      
      if (maps.get(mapIndex).board[j][i] == 20) {
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
        
        
        Goal m = new Goal(50*j-475,50*i-475,sensor,ghostObject);
        goals.add(m);
        
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
    }
  }
}

void map_reload() {
  mines = new ArrayList<Mine>();
  goals = new ArrayList<Goal>();
  mines = new ArrayList<Mine>();
  
  mapIndex++;
  
  map_load();
  
}

void light() {
  serial_sensor.write('z');
  delay(10);
  for (int i=0;i<3;i++) {
    //System.out.printf("%d\n",nearest_mines[i]);
    ByteBuffer buffer = ByteBuffer.allocate(4);
    
    int format = (nearest_mines[i]-1500)/1500;
    byte[] bytes = buffer.putInt(format).array();
    //System.out.printf("%d\n",bytes[3]);
    serial_sensor.write(bytes[3]);
    delay(10);
  }
}
