 
public class Node {
	Node parent_i, parent_j;
	int x, y;
	public Node getParenti() {
		return parent_i;
	}
	public void setParenti(Node parent) {
		this.parent_i = parent_i;
	}
	public Node getParentj() {
		return parent_j;
	}
	public void setParent(Node parent) {
		this.parent_j = parent_j;
	}
	public int getX() {
		return x;
	}
	public void setX(int x) {
		this.x = x;
	}
	public int getY() {
		return y;
	}
	public void setY(int y) {
		this.y = y;
	}
	public float getF() {
		return f;
	}
	public void setF(float f) {
		this.f = f;
	}
	public float getG() {
		return g;
	}
	public void setG(float g) {
		this.g = g;
	}
	public float getH() {
		return h;
	}
	public void setH(float h) {
		this.h = h;
	}
	float f,g,h;
}
