import bRigid.*;
import com.bulletphysics.collision.dispatch.GhostObject;

class Block {
  public float x;
  public float y;
  public float z;
  public BObject     block;
  
  Block(float x,float y,BObject block) {
    this.x = x;
    this.y = y;
    this.z = z;
    this.block = block;
  }
}
